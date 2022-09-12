import numpy as np
from environment import State


def _filter_instance(observation: State, mask: np.ndarray):
    res = {}

    for key, value in observation.items():
        if key in ('observation', 'static_info'):
            continue

        if key == 'capacity':
            res[key] = value
            continue

        if key == 'duration_matrix':
            res[key] = value[mask]
            res[key] = res[key][:, mask]
            continue

        res[key] = value[mask]

    return res


def _greedy(observation: State, rng: np.random.Generator):
    mask = np.copy(observation['must_dispatch'])
    mask[:] = True
    return _filter_instance(observation, mask)


def _lazy(observation: State, rng: np.random.Generator):
    mask = np.copy(observation['must_dispatch'])
    mask[0] = True
    return _filter_instance(observation, mask)


def _random(observation: State, rng: np.random.Generator):
    mask = np.copy(observation['must_dispatch'])
    mask = (mask | rng.binomial(1, p=0.5, size=len(mask)).astype(np.bool8))
    mask[0] = True
    return _filter_instance(observation, mask)


def _supervised(observation: State, rng: np.random.Generator, net):
    from baselines.supervised.transform import transform_one
    mask = np.copy(observation['must_dispatch'])
    mask = mask | net(transform_one(observation)).argmax(-1).bool().numpy()
    mask[0] = True
    return _filter_instance(observation, mask)


def _dqn(observation: State, rng: np.random.Generator, net):
    import torch
    from baselines.dqn.utils import get_request_features
    actions = []
    epoch_instance = observation
    observation, static_info = epoch_instance.pop('observation'), epoch_instance.pop('static_info')
    request_features, global_features = get_request_features(observation, static_info, net.k_nearest)
    all_features = torch.cat((request_features, global_features[None, :].repeat(request_features.shape[0], 1)), -1)
    actions = net(all_features).argmax(-1).detach().cpu().tolist()
    mask = epoch_instance['must_dispatch'] | (np.array(actions) == 0)
    mask[0] = True  # Depot always included in scheduling
    return _filter_instance(epoch_instance, mask)


STRATEGIES = dict(
    greedy=_greedy,
    lazy=_lazy,
    random=_random,
    supervised=_supervised,
    dqn=_dqn
)
