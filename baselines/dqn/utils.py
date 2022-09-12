import os
import torch
import json
import numpy as np


def load_model(path, device='cpu'):
    from baselines.dqn.net import Network
    with open(os.path.join(path, 'config.json')) as f:
        config = json.load(f)
    net = Network(**config)
    net.load_state_dict(torch.load(os.path.join(path, 'model.pth'), map_location=device))
    net.eval()
    return net


def get_request_features(instance_observation, instance_static_info, k_nearest=10):
    # Gets features for a node which consist of basic features
    # as well as basic features of k nearest requests (sorted by driving duration)
    # and the avg duration to the k nearest
    # and some global features, e.g. current time and remaining epochs and avg. request features

    epoch_instance = instance_observation['epoch_instance']
    # Compute some properties used for state representation
    d = epoch_instance['duration_matrix']
    # Add large duration to self
    d = d + int(1e9) * np.eye(d.shape[0])
    # Find k nearest
    ind_nearest = np.argpartition(d, min(k_nearest, d.shape[-1]) - 1, -1)[:, :k_nearest]
    # Sort by duration
    row_ind = np.arange(d.shape[0])[:, None]
    ind_nearest_sorted = ind_nearest[row_ind, np.argsort(d[row_ind, ind_nearest], -1)]
    dist_to_nearest = d[row_ind, ind_nearest_sorted]

    # n x d
    basic_request_features = np.concatenate((
        epoch_instance['is_depot'][:, None], 
        epoch_instance['coords'] / 1000, 
        epoch_instance['demands'][:, None] / 10, 
        epoch_instance['time_windows'] / 10000, 
        epoch_instance['service_times'][:, None] / 10000, 
        epoch_instance['must_dispatch'][:, None]), -1)

    # TODO add more, for now only avg duration to nearest n x 1
    advanced_request_features = dist_to_nearest.mean(-1)[:, None]

    # index n x d features by n x k indices gives n x k x d tensor
    # concate n x k x 1 durations feature for n x k x (d+1) matrix
    # then reshape to n x (k x (d + 1))
    nearest_neighb_request_features = np.pad(
            np.concatenate((
            basic_request_features[ind_nearest_sorted],
            dist_to_nearest[:, :, None],
        ), -1),
        ((0, 0), (0, max(k_nearest - d.shape[-1], 0)), (0, 0))
    ).reshape(d.shape[0], -1)

    all_request_features = np.concatenate((
        basic_request_features,
        advanced_request_features,
        nearest_neighb_request_features
    ), -1)

    # global features
    global_features = np.concatenate((
        np.array([
            (instance_static_info['end_epoch'] - instance_observation['current_epoch']) / 10,
            instance_observation['current_time'] / 10000,
            instance_observation['planning_starttime'] / 10000,
            d.shape[0] / 1000,
        ]),
        basic_request_features[:, 1:].mean(0)
    ), -1)

    return torch.tensor(all_request_features, dtype=torch.float32), torch.tensor(global_features, dtype=torch.float32)