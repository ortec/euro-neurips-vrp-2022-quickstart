import os
import numpy as np
import torch
import random
import functools
from argparse import Namespace
from torch.nn import functional as F
from typing import Dict, List, Tuple

from buffer import ReplayBuffer
from net import Network
from solver import run_baseline
from environment import VRPEnvironment
from baselines.strategies import STRATEGIES


class DQNAgent:
    """DQN Agent interacting with environment.

    Attribute:
        env (Environment): VRPTW environment
        memory (ReplayBuffer): replay memory to store transitions
        batch_size (int): batch size for sampling
        epsilon (float): parameter for epsilon greedy policy
        epsilon_decay (float): step size to decrease epsilon
        max_epsilon (float): max value of epsilon
        min_epsilon (float): min value of epsilon
        target_update (int): period for target model's hard update
        gamma (float): discount factor
        dqn (Network): model to train and select actions
        dqn_target (Network): target model to update
        optimizer (torch.optim): optimizer for training dqn
        transition (list): transition information including
                           state, action, reward, next_state, done
    """

    def __init__(
        self,
        env,
        train_config: dict,
        validation_config: dict,
        test_config: dict,
        model_kwargs: dict = {}
    ):
        """Initialization.

        Args:
            env (Environment): VRPTW environment
            train_config (dict): object with following keys
                memory_size (int): length of memory
                batch_size (int): batch size for sampling
                steps_per_update (int): number of steps to update model after episode
                target_update (int): period for target model's hard update
                epsilon_decay (float): step size to decrease epsilon
                max_epsilon (float): max value of epsilon
                min_epsilon (float): min value of epsilon
                gamma (float): discount factor
                max_gradient_norm (float): clip gradients to this norm
                optimizer_kwargs (dict): arguments for optimizer
            validation_config (dict): object with config for validation
            test_config (dict): object with config for testing
            model_kwargs (dict): arguments for model initialization
        """
        self.train_config = train_config
        self.validation_config = validation_config
        self.test_config = test_config

        config = self.train_config
        optimizer_kwargs = config['optimizer']

        seed = config.get('seed', 1)
        self.rng = np.random.default_rng(seed)
        torch.manual_seed(seed)

        self.env = env
        NUM_FEATS=self.env.num_of_feats
        self.memory = ReplayBuffer(NUM_FEATS, config['memory_size'], config['batch_size'])
        self.batch_size = config['batch_size']
        self.steps_per_update = config['steps_per_update']
        self.epsilon = config.get('max_epsilon', 1.0)
        self.epsilon_decay = config['epsilon_decay']
        self.max_epsilon = config.get('max_epsilon', 1.0)
        self.min_epsilon = config.get('min_epsilon', 0.1)
        self.target_update = config['target_update']
        self.gamma = config.get('gamma', 0.99)
        self.max_gradient_norm = config.get('max_gradient_norm', 1e9)
        # device: cpu / gpu
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu"
        )

        # networks: dqn, dqn_target
        self.dqn = Network(NUM_FEATS, **model_kwargs).to(self.device)
        self.dqn_target = Network(NUM_FEATS, **model_kwargs).to(self.device)
        self.dqn_target.load_state_dict(self.dqn.state_dict())
        self.dqn_target.eval()

        # optimizer
        self.optimizer = torch.optim.Adam(self.dqn.parameters(), **optimizer_kwargs)

        # transition to store in memory
        self.transition = list()

    def select_action(self, state: np.ndarray) -> np.ndarray:
        """Select an action from the input state."""

        # epsilon greedy policy
        if self.dqn.training and self.epsilon > self.rng.random():
            selected_action = self.rng.integers(0, 2 - 1)
        else:
            selected_action = self.dqn(
                torch.FloatTensor(state).to(self.device),
            ).argmax()
            selected_action = selected_action.detach().cpu().numpy()

        return selected_action

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, np.float64, bool]:
        """Take an action and return the response of the env."""
        next_state, reward, done, _ = self.env.step(action)
        return next_state, reward, done

    def update_model(self) -> torch.Tensor:
        """Update the model by gradient descent."""
        samples = self.memory.sample_batch()

        loss = self._compute_dqn_loss(samples)

        self.optimizer.zero_grad()
        loss.backward()
        for group in self.optimizer.param_groups:
            norm = torch.nn.utils.clip_grad_norm_(group['params'], self.max_gradient_norm)
        self.optimizer.step()

        return loss.item(), norm.item()

    def train(self, train_set: List, validation_set: List, test_set: List):
        """Train the agent."""
        update_cnt = 0
        epsilons = []
        losses = []
        scores = []
        score = 0

        config = self.train_config

        num_instances_per_epoch = len(train_set) if config.get('num_instances_per_epoch', -1) == -1 else config['num_instances_per_epoch']
        repeat_instances = config.get('repeat_instances', 1)
        
        if config.get('validate_at_start', False):
            print("Validating...")
            avg_reward = self.evaluate(validation_set, self.validation_config)
            print(f"Avg validation reward: {avg_reward:.2f}, avg cost: {-avg_reward:.2f}")

        for epoch in range(1, config['num_epochs'] + 1):
            print(f" -------------------------- Epoch {epoch}/{config['num_epochs']}")
            for i, instance in enumerate(self.rng.choice(train_set, num_instances_per_epoch)):
                print(f"Instance {i+1}/{num_instances_per_epoch}")
                for j in range(repeat_instances):
                    if repeat_instances > 1:
                        print(f"Rep {j+1}/{repeat_instances}")
                    try:
                        state, _ = self.env.reset(instance)
                        done = False
                        while not done:
                            action = self.select_action(state)
                            next_state, reward, done, _ = self.env.step(action)
                            self.memory.store(state, action, reward, next_state, done)

                            state = next_state
                            score += reward

                        scores.append(score)
                    except Exception as e:
                        # In case of unexpected failure, skip and continue training
                        print("------------ EXCEPTION -----------")
                        print(e)
                    score = 0

                    # if training is ready
                    if len(self.memory) >= self.batch_size:
                        # Since each episode adds 100s of transitions and takes a long time
                        # perform multiple gradient steps
                        for i in range(self.steps_per_update):
                            loss, gradnorm = self.update_model()
                            if i == 0:
                                print(f"Loss: {loss:.2f}, gradient norm: {gradnorm:.2f}")
                            losses.append(loss)
                            update_cnt += 1

                        # linearly decrease epsilon
                        self.epsilon = max(
                            self.min_epsilon, self.epsilon - (
                                self.max_epsilon - self.min_epsilon
                            ) * self.epsilon_decay
                        )
                        epsilons.append(self.epsilon)

                        # if hard update is needed
                        if update_cnt % self.target_update == 0:
                            self._target_hard_update()

            if config.get('ckpt_dir', None) is not None:
                print(f"Writing checkpoint to {config['ckpt_dir']}")
                torch.save(self.dqn.state_dict(), os.path.join(config['ckpt_dir'], 'model.pth'))

            print("Validating...")
            avg_reward = self.evaluate(validation_set, self.validation_config)
            print(f"Avg reward: {avg_reward:.2f}, avg cost: {-avg_reward:.2f}")

            # plotting
            if config.get('plotting_interval', -1) > 0 and epoch % config['plotting_interval'] == 0:
                self._plot(epoch, scores, losses, epsilons)

        print("Testing...")
        avg_reward = self.evaluate(test_set, self.test_config)
        print(f"Avg reward: {avg_reward:.2f}, avg cost: {-avg_reward:.2f}")

    @torch.no_grad()
    def evaluate(self, dataset: List, config: Dict):
        model = Network(
            num_inputs=self.dqn.num_inputs,
            num_hiddens=self.dqn.num_hiddens,
            k_nearest=self.dqn.k_nearest,
        )
        model.load_state_dict(self.dqn.state_dict())
        model.to('cpu')  # Test on CPU
        model.eval()
        args = Namespace(
            verbose=config.get('verbose', False),
            solver_seed=config.get('solver_seed', 1)
        )
        
        strategy = functools.partial(STRATEGIES['dqn'], net=model)
        rewards = []
        for i, instance in enumerate(dataset):
            print(f"Instance {i+1}/{len(dataset)}")
            env = VRPEnvironment(seed=config.get('instance_seed', 1), instance=instance, epoch_tlim=config.get('epoch_tlim'))
            rewards.append(run_baseline(args, env, strategy=strategy, seed=config.get('strategy_seed', 1)))

        return np.mean(rewards)
        

    def _compute_dqn_loss(self, samples: Dict[str, np.ndarray]) -> torch.Tensor:
        """Return dqn loss."""
        device = self.device  # for shortening the following lines
        state = torch.FloatTensor(samples["obs"]).to(device)
        next_state = torch.FloatTensor(samples["next_obs"]).to(device)
        action = torch.LongTensor(samples["acts"].reshape(-1, 1)).to(device)
        reward = torch.FloatTensor(samples["rews"].reshape(-1, 1)).to(device)
        done = torch.BoolTensor(samples["done"].reshape(-1, 1)).to(device)

        # G_t   = r + gamma * v(s_{t+1})  if state != Terminal
        #       = r                       otherwise
        curr_q_value = self.dqn(state).gather(1, action)
        next_q_value = self.dqn_target(
            next_state
        ).max(dim=1, keepdim=True)[0].detach()
        target = (reward + self.gamma * torch.where(~done, next_q_value, 0)).to(self.device)

        # calculate dqn loss
        loss = F.smooth_l1_loss(curr_q_value, target)

        return loss

    def _target_hard_update(self):
        """Hard update: target <- local."""
        self.dqn_target.load_state_dict(self.dqn.state_dict())

    def _plot(
        self,
        epoch: int,
        scores: List[float],
        losses: List[float],
        epsilons: List[float],
    ):
        from matplotlib import pyplot as plt
        """Plot the training progresses."""
        plt.figure(figsize=(20, 5))
        plt.subplot(131)
        plt.title('epoch %s. score: %s' % (epoch, np.mean(scores[-10:])))
        plt.plot(scores)
        plt.subplot(132)
        plt.title('loss')
        plt.plot(losses)
        plt.subplot(133)
        plt.title('epsilons')
        plt.plot(epsilons)
        plt.show()