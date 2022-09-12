import os
import pickle as pkl
import argparse

def load_pkl(filename):
    with open(filename, 'rb') as f:
        return pkl.load(f)

def write_pkl(data, filename):
    with open(filename, 'wb') as f:
        return pkl.dump(data, f)

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--data_dir", default='baselines/supervised/data')
    args = parser.parse_args()

    dataset = [
        load_pkl(os.path.join(args.data_dir, name))
        for name in os.listdir(args.data_dir)
        if name.startswith('ORTEC-VRPTW')
    ]
    X, Y = [], []

    for x, y in dataset:
        X.extend(x)
        Y.extend(y)

    idxs = []
    for i, y in enumerate(Y):
        idxs.append([0] * X[i]['request_idx'].shape[0])
        for sol in y:
            for yi in sol:
                idxs[-1][(X[i]['request_idx'].tolist().index(yi))] = 1

    write_pkl(idxs, os.path.join(args.data_dir, 'data.Y.pkl'))
    write_pkl(X, os.path.join(args.data_dir, 'data.X.pkl'))
