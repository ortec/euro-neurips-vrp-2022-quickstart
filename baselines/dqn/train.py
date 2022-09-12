import os
import sys
import argparse
import json
import random

sys.path.insert(0, os.getcwd())
import tools
from agent import DQNAgent
from drl_env import DRLEnv


def shuffle_and_split(data_list, seed=0):
    rng = random.Random(seed)

    rng.shuffle(data_list)

    N = int(len(data_list) * 0.90)
    tr = data_list[:N]
    ts = data_list[N:]

    # M = int(N * 0.90)
    M = -10
    tr, vl = tr[:M], tr[M:]

    return tr, vl, ts


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default='baselines/dqn/config.json')
    args = parser.parse_args()

    with open(args.config, 'r') as f:
        config = json.load(f)

    drl_env = DRLEnv(config)
    agent = DQNAgent(env=drl_env,
                     train_config=config['training'],
                     validation_config=config['validation'],
                     test_config=config.get('test', config['validation']),
                     model_kwargs=config['model'])
    
    params = dict(
        num_inputs=agent.dqn.num_inputs,
        num_hiddens=agent.dqn.num_hiddens,
        k_nearest=agent.dqn.k_nearest,
    )
    ckpt_dir = config['training']['ckpt_dir']
    os.makedirs(ckpt_dir, exist_ok=True)
    with open(os.path.join(ckpt_dir, 'config.json'), 'w') as f:
        json.dump(params, f)

    instances_dir = config['training']['instances_dir']
    print(f"Reading instances from directory '{instances_dir}'")
    instances = [
        tools.read_vrplib(os.path.join(instances_dir, instance_filename))
        for instance_filename in sorted(os.listdir(instances_dir))
        if instance_filename.startswith('ORTEC-VRPTW')
    ]
    print(f"Done reading {len(instances)} instances!")
    train_set, validation_set, test_set = shuffle_and_split(instances, seed=0)

    print("Start training...")
    agent.train(train_set=train_set, validation_set=validation_set, test_set=test_set)

    