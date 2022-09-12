# Solver for Dynamic VRPTW, baseline strategy is to use the static solver HGS-VRPTW repeatedly
import argparse
import os
import uuid
import numpy as np
import pickle as pkl
import sys

if __name__ == "__main__":
    # Add current working directory to path so we can import
    sys.path.insert(0, os.getcwd())

import tools
from environment import VRPEnvironment
from solver import solve_static_vrptw, run_baseline


def run_oracle(args):

    for seed in args.instance_seed:
        env = VRPEnvironment(seed=seed, instance=tools.read_vrplib(args.instance), epoch_tlim=args.epoch_tlim, is_static=False)

        run_baseline(args, env, strategy='greedy', seed=seed)
        # Get greedy solution as simple list of routes
        greedy_solution = [route for epoch, routes in env.final_solutions.items() for route in routes]
        hindsight_problem = env.get_hindsight_problem()

        oracle_solution = min(solve_static_vrptw(hindsight_problem, time_limit=args.oracle_tlim, tmp_dir=args.tmp_dir, initial_solution=greedy_solution),
                              key=lambda x: x[1])[0]

        observation, static_info = env.reset()

        X = []
        Y = []
        done = False

        while not done:
            epoch_instance = observation['epoch_instance']
            request_idx = set(epoch_instance['request_idx'])
            epoch_solution = [route for route in oracle_solution if len(request_idx.intersection(route)) == len(route)]

            X.append(epoch_instance)
            Y.append(epoch_solution)
            observation, reward, done, info = env.step(epoch_solution)

        os.makedirs(args.data_dir, exist_ok=True)
        with open(os.path.join(args.data_dir, f'{os.path.basename(args.instance)}.{seed}.pkl'), 'wb') as f:
            pkl.dump([X, Y], f)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--instance", help="Instance to solve")
    parser.add_argument("--instance_seed", type=str, default="1,2,3,4,5", help="Seed to use for the dynamic instance (or multiple comma seperated), default 1,2,3,4,5")
    parser.add_argument("--solver_seed", type=int, default=1, help="Seed to use for the solver")
    parser.add_argument("--epoch_tlim", type=int, default=120, help="Time limit per epoch")
    parser.add_argument("--oracle_tlim", type=int, default=120, help="Time limit for oracle")
    parser.add_argument("--tmp_dir", type=str, default=None, help="Provide a specific directory to use as tmp directory (useful for debugging)")
    parser.add_argument("--verbose", action='store_true', help="Show verbose output")
    parser.add_argument("--data_dir", default='baselines/supervised/data')

    args = parser.parse_args()

    if args.tmp_dir is None:
        # Generate random tmp directory
        args.tmp_dir = os.path.join("tmp", str(uuid.uuid4()))
        cleanup_tmp_dir = True
    else:
        # If tmp dir is manually provided, don't clean it up (for debugging)
        cleanup_tmp_dir = False
    
    args.instance_seed = map(int, args.instance_seed.split(","))

    try:
        # Make sure these parameters are not used by your solver
        run_oracle(args)
    finally:
        if cleanup_tmp_dir:
            tools.cleanup_tmp_dir(args.tmp_dir)
