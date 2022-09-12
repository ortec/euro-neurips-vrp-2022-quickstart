# Solver for Dynamic VRPTW, baseline strategy is to use the static solver HGS-VRPTW repeatedly
import argparse
import subprocess
import sys
import os
import uuid
import platform
import numpy as np
import functools

import tools
from environment import VRPEnvironment, ControllerEnvironment
from baselines.strategies import STRATEGIES

def solve_static_vrptw(instance, time_limit=3600, tmp_dir="tmp", seed=1, initial_solution=None):

    # Prevent passing empty instances to the static solver, e.g. when
    # strategy decides to not dispatch any requests for the current epoch
    if instance['coords'].shape[0] <= 1:
        yield [], 0
        return

    if instance['coords'].shape[0] <= 2:
        solution = [[1]]
        cost = tools.validate_static_solution(instance, solution)
        yield solution, cost
        return

    os.makedirs(tmp_dir, exist_ok=True)
    instance_filename = os.path.join(tmp_dir, "problem.vrptw")
    tools.write_vrplib(instance_filename, instance, is_vrptw=True)

    executable = os.path.join('baselines', 'hgs_vrptw', 'genvrp')
    # On windows, we may have genvrp.exe
    if platform.system() == 'Windows' and os.path.isfile(executable + '.exe'):
        executable = executable + '.exe'
    assert os.path.isfile(executable), f"HGS executable {executable} does not exist!"
    # Call HGS solver with unlimited number of vehicles allowed and parse outputs
    # Subtract two seconds from the time limit to account for writing of the instance and delay in enforcing the time limit by HGS

    hgs_cmd = [
        executable, instance_filename, str(max(time_limit - 2, 1)),
        '-seed', str(seed), '-veh', '-1', '-useWallClockTime', '1'
    ]
    if initial_solution is None:
        initial_solution = [[i] for i in range(1, instance['coords'].shape[0])]
    if initial_solution is not None:
        hgs_cmd += ['-initialSolution', " ".join(map(str, tools.to_giant_tour(initial_solution)))]
    with subprocess.Popen(hgs_cmd, stdout=subprocess.PIPE, text=True) as p:
        routes = []
        for line in p.stdout:
            line = line.strip()
            # Parse only lines which contain a route
            if line.startswith('Route'):
                label, route = line.split(": ")
                route_nr = int(label.split("#")[-1])
                assert route_nr == len(routes) + 1, "Route number should be strictly increasing"
                routes.append([int(node) for node in route.split(" ")])
            elif line.startswith('Cost'):
                # End of solution
                solution = routes
                cost = int(line.split(" ")[-1].strip())
                check_cost = tools.validate_static_solution(instance, solution)
                assert cost == check_cost, "Cost of HGS VRPTW solution could not be validated"
                yield solution, cost
                # Start next solution
                routes = []
            elif "EXCEPTION" in line:
                raise Exception("HGS failed with exception: " + line)
        assert len(routes) == 0, "HGS has terminated with imcomplete solution (is the line with Cost missing?)"


def run_oracle(args, env):
    # Oracle strategy which looks ahead, this is NOT a feasible strategy but gives a 'bound' on the performance
    # Bound written with quotes because the solution is not optimal so a better solution may exist
    # This oracle can also be used as supervision for training a model to select which requests to dispatch

    # First get hindsight problem (each request will have a release time)
    # As a start solution for the oracle solver, we use the greedy solution
    # This may help the oracle solver to find a good solution more quickly
    log("Running greedy baseline to get start solution and hindsight problem for oracle solver...")
    run_baseline(args, env, strategy='greedy')
    # Get greedy solution as simple list of routes
    greedy_solution = [route for epoch, routes in env.final_solutions.items() for route in routes]
    hindsight_problem = env.get_hindsight_problem()

    # Compute oracle solution (separate time limit since epoch_tlim is used for greedy initial solution)
    log(f"Start computing oracle solution with {len(hindsight_problem['coords'])} requests...")
    oracle_solution = min(solve_static_vrptw(hindsight_problem, time_limit=args.oracle_tlim, tmp_dir=args.tmp_dir, initial_solution=greedy_solution), key=lambda x: x[1])[0]
    oracle_cost = tools.validate_static_solution(hindsight_problem, oracle_solution)
    log(f"Found oracle solution with cost {oracle_cost}")

    # Run oracle solution through environment (note: will reset environment again with same seed)
    total_reward = run_baseline(args, env, oracle_solution=oracle_solution)
    assert -total_reward == oracle_cost, "Oracle solution does not match cost according to environment"
    return total_reward


def run_baseline(args, env, oracle_solution=None, strategy=None, seed=None):

    strategy = strategy or args.strategy
    strategy = STRATEGIES[strategy] if isinstance(strategy, str) else strategy
    seed = seed or args.solver_seed

    rng = np.random.default_rng(seed)

    total_reward = 0
    done = False
    # Note: info contains additional info that can be used by your solver
    observation, static_info = env.reset()
    epoch_tlim = static_info['epoch_tlim']
    num_requests_postponed = 0
    while not done:
        epoch_instance = observation['epoch_instance']

        if args.verbose:
            log(f"Epoch {static_info['start_epoch']} <= {observation['current_epoch']} <= {static_info['end_epoch']}", newline=False)
            num_requests_open = len(epoch_instance['request_idx']) - 1
            num_new_requests = num_requests_open - num_requests_postponed
            log(f" | Requests: +{num_new_requests:3d} = {num_requests_open:3d}, {epoch_instance['must_dispatch'].sum():3d}/{num_requests_open:3d} must-go...", newline=False, flush=True)

        if oracle_solution is not None:
            request_idx = set(epoch_instance['request_idx'])
            epoch_solution = [route for route in oracle_solution if len(request_idx.intersection(route)) == len(route)]
            cost = tools.validate_dynamic_epoch_solution(epoch_instance, epoch_solution)
        else:
            # Select the requests to dispatch using the strategy
            # Note: DQN strategy requires more than just epoch instance, bit hacky for compatibility with other strategies
            epoch_instance_dispatch = strategy({**epoch_instance, 'observation': observation, 'static_info': static_info}, rng)

            # Run HGS with time limit and get last solution (= best solution found)
            # Note we use the same solver_seed in each epoch: this is sufficient as for the static problem
            # we will exactly use the solver_seed whereas in the dynamic problem randomness is in the instance
            solutions = list(solve_static_vrptw(epoch_instance_dispatch, time_limit=epoch_tlim, tmp_dir=args.tmp_dir, seed=args.solver_seed))
            assert len(solutions) > 0, f"No solution found during epoch {observation['current_epoch']}"
            epoch_solution, cost = solutions[-1]

            # Map HGS solution to indices of corresponding requests
            epoch_solution = [epoch_instance_dispatch['request_idx'][route] for route in epoch_solution]

        if args.verbose:
            num_requests_dispatched = sum([len(route) for route in epoch_solution])
            num_requests_open = len(epoch_instance['request_idx']) - 1
            num_requests_postponed = num_requests_open - num_requests_dispatched
            log(f" {num_requests_dispatched:3d}/{num_requests_open:3d} dispatched and {num_requests_postponed:3d}/{num_requests_open:3d} postponed | Routes: {len(epoch_solution):2d} with cost {cost:6d}")

        # Submit solution to environment
        observation, reward, done, info = env.step(epoch_solution)
        assert cost is None or reward == -cost, "Reward should be negative cost of solution"
        assert not info['error'], f"Environment error: {info['error']}"

        total_reward += reward

    if args.verbose:
        log(f"Cost of solution: {-total_reward}")

    return total_reward


def log(obj, newline=True, flush=False):
    # Write logs to stderr since program uses stdout to communicate with controller
    sys.stderr.write(str(obj))
    if newline:
        sys.stderr.write('\n')
    if flush:
        sys.stderr.flush()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--strategy", type=str, default='greedy', help="Baseline strategy used to decide whether to dispatch routes")
    # Note: these arguments are only for convenience during development, during testing you should use controller.py
    parser.add_argument("--instance", help="Instance to solve")
    parser.add_argument("--instance_seed", type=int, default=1, help="Seed to use for the dynamic instance")
    parser.add_argument("--solver_seed", type=int, default=1, help="Seed to use for the solver")
    parser.add_argument("--static", action='store_true', help="Add this flag to solve the static variant of the problem (by default dynamic)")
    parser.add_argument("--epoch_tlim", type=int, default=120, help="Time limit per epoch")
    parser.add_argument("--oracle_tlim", type=int, default=120, help="Time limit for oracle")
    parser.add_argument("--tmp_dir", type=str, default=None, help="Provide a specific directory to use as tmp directory (useful for debugging)")
    parser.add_argument("--model_path", type=str, default=None, help="Provide the path of the machine learning model to be used as strategy (Path must not contain `model.pth`)")
    parser.add_argument("--verbose", action='store_true', help="Show verbose output")
    args = parser.parse_args()

    if args.tmp_dir is None:
        # Generate random tmp directory
        args.tmp_dir = os.path.join("tmp", str(uuid.uuid4()))
        cleanup_tmp_dir = True
    else:
        # If tmp dir is manually provided, don't clean it up (for debugging)
        cleanup_tmp_dir = False

    try:
        if args.instance is not None:
            env = VRPEnvironment(seed=args.instance_seed, instance=tools.read_vrplib(args.instance), epoch_tlim=args.epoch_tlim, is_static=args.static)
        else:
            assert args.strategy != "oracle", "Oracle can not run with external controller"
            # Run within external controller
            env = ControllerEnvironment(sys.stdin, sys.stdout)

        # Make sure these parameters are not used by your solver
        args.instance = None
        args.instance_seed = None
        args.static = None
        args.epoch_tlim = None

        if args.strategy == 'oracle':
            run_oracle(args, env)
        else:
            if args.strategy == 'supervised':
                from baselines.supervised.utils import load_model
                net = load_model(args.model_path, device='cpu')
                strategy = functools.partial(STRATEGIES['supervised'], net=net)
            elif args.strategy == 'dqn':
                from baselines.dqn.utils import load_model
                net = load_model(args.model_path, device='cpu')
                strategy = functools.partial(STRATEGIES['dqn'], net=net)
            else:
                strategy = STRATEGIES[args.strategy]

            run_baseline(args, env, strategy=strategy)

        if args.instance is not None:
            log(tools.json_dumps_np(env.final_solutions))
    finally:
        if cleanup_tmp_dir:
            tools.cleanup_tmp_dir(args.tmp_dir)
