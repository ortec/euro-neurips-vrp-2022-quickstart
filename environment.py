import time
import numpy as np
import tools
import warnings

from typing import Dict, List, Tuple

_BIG_NUMBER = int(1e9)

State = Dict[str, np.ndarray]
Action = List[List[int]]
Info = Dict[str, str]


class Environment:

    def step(self, solution: Action) -> Tuple[State, int, bool, Info]:
        raise NotImplementedError()

    def reset(self, seed: int = None, instance: State = None, epoch_tlim: int = None, is_static: bool = None) -> State:
        raise NotImplementedError()


class ControllerEnvironment(Environment):
    """
    Environment for VRP with Time Windows that wraps interface with controller through the command line
    """

    def __init__(self, read_pipe, write_pipe):
        super().__init__()
        self.read_pipe = read_pipe
        self.write_pipe = write_pipe

    def step(self, solution: Action) -> Tuple[State, int, bool, Info]:
        result = tools.json_loads_np(self._request('step', solution))

        observation = result['observation']
        reward = result['reward']
        done = result['done']
        info = result['info']

        return (observation, reward, done, info)

    def reset(self, seed: int = None, instance: State = None, epoch_tlim: int = None, is_static: bool = None) -> (State):
        assert seed is None, "Argument seed must be None for controller environment as it is decided by controller"
        assert instance is None, "Argument instance must be None for controller environment as it is decided by controller"
        assert epoch_tlim is None, "Argument epoch_tlim must be None for controller environment as it is decided by controller"
        assert is_static is None, "Argument is_static must be None for controller environment as it is decided by controller"
        result = tools.json_loads_np(self._request('reset', None))
        return result['observation'], result['info']

    def _request(self, action: str, data=None):
        self.write_pipe.write(tools.json_dumps_np({
            'action': action,
            'data': data
        }))
        self.write_pipe.write('\n')
        self.write_pipe.flush()
        return self.read_pipe.readline().strip()


class VRPEnvironment(Environment):
    """
    Dynamic environment for VRP with Time Windows (VRPTW) where orders arrive during the day
    An instance is created by resampling customers, time windows and demands from a static VRPTW instance.
    Requests arrive during a number of epochs. At every epoch, a solution can be submitted that dispatches
    a number of vehicles to serve a subset of the open requests. Some requests must be served during the
    current epoch, because the next epoch will be too late to meet there time windows, taking into account
    dispatching time and driving time. Other requests may be served, and may be included in the routes, but
    may also be deferred to the next epoch. During the final epoch, all request must be dispatched.
    The environment has a wall clock time limit to submit solutions. To avoid timing issues, submissions can
    be 'submitted' to the environment early, after which they can still be updated until the time limit.
    A submitted solution is checked and can be accepted or rejected depending on whether it is valid or not.
    To move to the next epoch, the agent should explicitly call the 'next_epoch' function, after which the
    last accepted solution becomes final, and new requests are sampled for the next epoch.
    """

    def __init__(self, seed: int = 1, instance: State = None, epoch_tlim: int = 120, is_static: bool = False):
        super().__init__()
        # DO NOT CHANGE THESE PARAMETERS
        self.MAX_REQUESTS_PER_EPOCH = 100  # Every epoch, we will sample at most 100 new requests
        self.MARGIN_DISPATCH = 3600  # Assume it takes one hour to dispatch the vehicle
        self.EPOCH_DURATION = 3600  # We will dispatch vehicles once an hour
        self.TLIM_GRACE_PERIOD = 2  # 2 seconds wall clock grace time

        # Fill environment with defaults
        self.default_instance = instance
        self.default_seed = seed
        self.default_epoch_tlim = epoch_tlim
        self.default_is_static = is_static

        # Require reset to be called first by marking environment as done
        self.is_done = True
        self.reset_counter = 0

    def reset(self, seed: int = None, instance: State = None, epoch_tlim: int = None, is_static: bool = None) -> State:
        """Resets the environment. Defaults provided during construction can be overridden to reuse the environment."""
        if self.reset_counter > 0 and seed is None:
            warnings.warn("Repeatedly resetting the environment without providing a seed will use the same default seed again")
        self.reset_counter += 1
        self.instance = instance if instance is not None else self.default_instance
        self.seed = seed if seed is not None else self.default_seed
        self.epoch_tlim = epoch_tlim if epoch_tlim is not None else self.default_epoch_tlim
        self.is_static = is_static if is_static is not None else self.default_is_static

        assert self.instance is not None

        if self.is_static:
            self.start_epoch = 0
            self.current_epoch = 0
            self.end_epoch = 0
        else:
            self.rng = np.random.default_rng(self.seed)
            timewi = self.instance['time_windows']
            self.start_epoch = int(max((timewi[1:, 0].min() - self.MARGIN_DISPATCH) // self.EPOCH_DURATION, 0))
            self.end_epoch = int(max((timewi[1:, 0].max() - self.MARGIN_DISPATCH) // self.EPOCH_DURATION, 0))

            self.current_epoch = self.start_epoch

            # Initialize request array with dummy/padding/sentinel request for depot
            self.request_id = np.array([0])
            self.request_customer_index = np.array([0])
            self.request_timewi = self.instance['time_windows'][0:1]
            self.request_service_t = self.instance['service_times'][0:1]
            self.request_demand = self.instance['demands'][0:1]
            self.request_is_dispatched = np.array([False])
            self.request_epoch = np.array([0])
            self.request_must_dispatch = np.array([False])

        self.is_done = False
        obs = self._next_observation()

        self.final_solutions = {}
        self.final_costs = {}
        self.start_time_epoch = time.time()

        info = {
            # For the dynamic problem, requests will be sampled from the static instance
            'dynamic_context': self.instance if not self.is_static else None,
            'is_static': self.is_static,
            'start_epoch': self.start_epoch,
            'end_epoch': self.end_epoch,
            'num_epochs': self.end_epoch - self.start_epoch + 1,
            'epoch_tlim': self.epoch_tlim,
        }

        return obs, info

    def step(self, solution: Action) -> Tuple[State, int, bool, Info]:

        assert not self.is_done, "Environment is finished"

        # Check time limit
        if self.get_elapsed_time_epoch() > self.epoch_tlim + self.TLIM_GRACE_PERIOD:
            return self._fail_episode("Time exceeded")

        # Check if solution is valid
        try:
            driving_duration = tools.validate_dynamic_epoch_solution(self.epoch_instance, solution)
        except AssertionError as e:
            return self._fail_episode(e)

        if not self.is_static:
            # Mark orders of current solution as dispatched
            for route in solution:
                # Route consists of 1 indexed requests
                assert not self.request_is_dispatched[route].any()
                self.request_is_dispatched[route] = True

            # We must not have any undispatched orders that must be dispatched
            assert not (self.request_must_dispatch & ~self.request_is_dispatched).any()

        self.final_solutions[self.current_epoch] = solution
        self.final_costs[self.current_epoch] = driving_duration
        self.current_epoch += 1
        self.is_done = self.current_epoch > self.end_epoch

        observation = self._next_observation() if not self.is_done else None
        reward = -driving_duration

        self.start_time_epoch = time.time()
        return (observation, reward, self.is_done, {'error': None})

    def get_elapsed_time_epoch(self):
        assert self.start_time_epoch is not None
        return time.time() - self.start_time_epoch

    def _fail_episode(self, error):
        self.final_solutions[self.current_epoch] = None
        self.final_costs[self.current_epoch] = max(self.end_epoch - self.current_epoch, 1) * _BIG_NUMBER
        self.is_done = True
        return (None, -_BIG_NUMBER, self.is_done, {'error': str(error)})

    def _next_observation(self):
        assert not self.is_done
        assert self.start_epoch <= self.current_epoch <= self.end_epoch

        if self.is_static:
            # Static instance, don't resample requests

            self.epoch_instance = {
                'is_depot': self.instance['is_depot'],
                'customer_idx': np.arange(len(self.instance['coords'])),
                'request_idx': np.arange(len(self.instance['coords'])),
                'coords': self.instance['coords'],
                'demands': self.instance['demands'],
                'capacity': self.instance['capacity'],
                'time_windows': self.instance['time_windows'],
                'service_times': self.instance['service_times'],
                'duration_matrix': self.instance['duration_matrix'],
                'must_dispatch': ~self.instance['is_depot'],
            }
            return {
                'current_epoch': self.current_epoch,
                'current_time': 0,
                'planning_starttime': 0,
                'epoch_instance': self.epoch_instance
            }

        duration_matrix = self.instance['duration_matrix']

        # Sample new data
        current_time = self.EPOCH_DURATION * self.current_epoch
        planning_starttime = current_time + self.MARGIN_DISPATCH

        # Sample uniformly
        num_customers = len(self.instance['coords']) - 1  # Exclude depot

        # Sample data uniformly from customers (1 to num_customers)
        def sample_from_customers(k=self.MAX_REQUESTS_PER_EPOCH):
            return self.rng.integers(num_customers, size=k) + 1

        cust_idx = sample_from_customers()
        timewi_idx = sample_from_customers()
        demand_idx = sample_from_customers()
        service_t_idx = sample_from_customers()

        new_request_timewi = self.instance['time_windows'][timewi_idx]
        # Filter data that can no longer be delivered
        # Time + margin for dispatch + drive time from depot should not exceed latest arrival
        earliest_arrival = np.maximum(planning_starttime + duration_matrix[0, cust_idx], new_request_timewi[:, 0])
        # Also, return at depot in time must be feasible
        earliest_return_at_depot = earliest_arrival + self.instance['service_times'][service_t_idx] + duration_matrix[cust_idx, 0]
        is_feasible = (earliest_arrival <= new_request_timewi[:, 1]) & (earliest_return_at_depot <= self.instance['time_windows'][0, 1])
        
        if is_feasible.any():
            num_new_requests = is_feasible.sum()
            self.request_id = np.concatenate((self.request_id, np.arange(num_new_requests) + len(self.request_id)))
            self.request_customer_index = np.concatenate((self.request_customer_index, cust_idx[is_feasible]))
            self.request_timewi = np.concatenate((self.request_timewi, new_request_timewi[is_feasible]))
            self.request_service_t = np.concatenate((self.request_service_t, self.instance['service_times'][service_t_idx[is_feasible]]))
            self.request_demand = np.concatenate((self.request_demand, self.instance['demands'][demand_idx[is_feasible]]))
            self.request_is_dispatched = np.pad(self.request_is_dispatched, (0, num_new_requests), mode='constant')
            self.request_epoch = np.concatenate((self.request_epoch, np.full(num_new_requests, self.current_epoch)))

        # Customers must dispatch this epoch if next epoch they will be too late
        if self.current_epoch < self.end_epoch:
            earliest_arrival = np.maximum(
                planning_starttime + self.EPOCH_DURATION + duration_matrix[0, self.request_customer_index],
                self.request_timewi[:, 0]
            )
            earliest_return_at_depot = earliest_arrival + self.request_service_t + duration_matrix[self.request_customer_index, 0]
            self.request_must_dispatch = (
                (earliest_arrival > self.request_timewi[:, 1]) |
                (earliest_return_at_depot > self.instance['time_windows'][0, 1])
            )
        else:
            self.request_must_dispatch = self.request_id > 0
        # Return instance based on customers not yet dispatched
        idx_undispatched = self.request_id[~self.request_is_dispatched]
        customer_idx = self.request_customer_index[idx_undispatched]
        # Return a VRPTW instance with undispatched requests with two additional properties: customer_idx and request_idx
        time_windows = self.request_timewi[idx_undispatched]

        # Renormalize time to start at planning_starttime, and clip time windows in the past (so depot will start at 0)
        time_windows = np.clip(time_windows - planning_starttime, a_min=0, a_max=None)
        self.epoch_instance = {
            'is_depot': self.instance['is_depot'][customer_idx],
            'customer_idx': customer_idx,
            'request_idx': idx_undispatched,
            'coords': self.instance['coords'][customer_idx],
            'demands': self.request_demand[idx_undispatched],
            'capacity': self.instance['capacity'],
            'time_windows': time_windows,
            'service_times': self.request_service_t[idx_undispatched],
            'duration_matrix': self.instance['duration_matrix'][np.ix_(customer_idx, customer_idx)],
            'must_dispatch': self.request_must_dispatch[idx_undispatched],
        }
        return {
            'current_epoch': self.current_epoch,
            'current_time': current_time,
            'planning_starttime': planning_starttime,
            'epoch_instance': self.epoch_instance
        }

    def get_hindsight_problem(self):
        """After the episode is completed, this function can be used to obtain the 'hindsight problem',
        i.e. as if we had future information about all the requests.
        This includes additional info containing the release times of the requests."""
        assert self.is_done

        customer_idx = self.request_customer_index
        # Release times indicate that a route containing this request cannot dispatch before this time
        # This needs to include the margin time for the dispatch
        release_times = self.EPOCH_DURATION * self.request_epoch + self.MARGIN_DISPATCH
        release_times[self.instance['is_depot'][customer_idx]] = 0
        return {
            'is_depot': self.instance['is_depot'][customer_idx],
            'customer_idx': customer_idx,
            'request_idx': self.request_id,
            'coords': self.instance['coords'][customer_idx],
            'demands': self.request_demand,
            'capacity': self.instance['capacity'],
            'time_windows': self.request_timewi,
            'service_times': self.request_service_t,
            'duration_matrix': self.instance['duration_matrix'][np.ix_(customer_idx, customer_idx)],
            # 'must_dispatch': self.request_must_dispatch,
            'release_times': release_times
        }
