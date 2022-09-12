import pickle as pkl
import os
import torch
import numpy as np
from torch_geometric.data import Data


def load_data(data_path):
    "Helper function loading VRPTW instances and Oracle HGS scheduling."
    X = pkl.load(open(f'{data_path}/data.X.pkl', 'rb'))
    Y = pkl.load(open(f'{data_path}/data.Y.pkl', 'rb'))

    return X, Y


def transform_one(x):
    dp = Data()
    dp.num_nodes = x['must_dispatch'].shape[0]
    dp.x = torch.tensor(x['is_depot'] * 1.0).unsqueeze(-1)
    dp.x = torch.cat((dp.x, torch.tensor(x['demands'] / x['demands'].max()).unsqueeze(-1)), dim=1)
    dp.x = torch.cat((dp.x, torch.tensor(x['must_dispatch'] * 1.0).unsqueeze(-1)), dim=1)

    dp.x = dp.x.float()

    edge_index = np.array(x['duration_matrix'].nonzero())
    dp.edge_index = torch.tensor(edge_index, dtype=torch.long)
    dp.edge_attr = torch.tensor(x['duration_matrix'][x['duration_matrix'].nonzero()], dtype=torch.float)
    dp.edge_attr = dp.edge_attr / dp.edge_attr.max()
    dp.edge_attr = dp.edge_attr.float()

    return dp


def transform(X, Y):
    "Helper function transforming VRPTW instances and Oracle scheduling to PyTorch Geometric data type."

    data_list = []
    for x, y in zip(X, Y):
        num_nodes = x['must_dispatch'].shape[0]

        if num_nodes <= 1:
            continue

        dp = transform_one(x)
        dp.y = torch.tensor(y, dtype=torch.long)
        data_list.append(dp)

    return data_list


def load_and_transform(data_path, save_processed=True, verbose=False, processed_dir='baselines/supervised/data/processed'):

    if os.path.exists(processed_dir):
        X = torch.load(os.path.join(processed_dir, 'x.pth'))
        edge_index = torch.load(os.path.join(processed_dir, 'edge_index.pth'))
        edge_attr = torch.load(os.path.join(processed_dir, 'edge_attr.pth'))
        Y = torch.load(os.path.join(processed_dir, 'y.pth'))
        data_list = [Data(x=x, edge_index=e, edge_attr=a, y=y) for x, e, a, y in zip(X, edge_index, edge_attr, Y)]

        return data_list

    X, Y = load_data(data_path)
    data_list = transform(X, Y)

    if save_processed:
        if verbose:
            print(f"Processed data will be dumped to `{processed_dir}`.")
        os.makedirs(name=processed_dir, exist_ok=True)
        torch.save(obj=[dp.x for dp in data_list], f=os.path.join(processed_dir, 'x.pth'))
        torch.save(obj=[dp.edge_index for dp in data_list], f=os.path.join(processed_dir, 'edge_index.pth'))
        torch.save(obj=[dp.edge_attr for dp in data_list], f=os.path.join(processed_dir, 'edge_attr.pth'))
        torch.save(obj=[dp.y for dp in data_list], f=os.path.join(processed_dir, 'y.pth'))

    return data_list
