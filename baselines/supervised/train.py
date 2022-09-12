import random
import torch
import functools
import operator
import os
import json
from transform import load_and_transform
from torch_geometric.loader import DataLoader
from net import Net
import argparse


def shuffle_and_split(data_list, seed=0):
    rng = random.Random(seed)

    rng.shuffle(data_list)

    N = int(len(data_list) * 0.90)
    tr = data_list[:N]
    ts = data_list[N:]

    M = int(N * 0.90)
    tr, vl = tr[:M], tr[M:]

    return tr, vl, ts


def train(model, optimizer):
    model.train()
    epoch_loss = 0.0
    for data in tr_loader:
        optimizer.zero_grad()
        out = torch.nn.functional.log_softmax(model(data.to(device)), dim=-1)
        loss = torch.nn.functional.nll_loss(out, data.y)
        epoch_loss += loss.item() / data.num_graphs
        loss.backward()
        optimizer.step()

    return float(epoch_loss)


@torch.no_grad()
def test(model, loader):
    model.eval()
    accs = []
    for data in loader:
        out = model(data.to(device)).argmax(-1)
        acc = (out == data.y).float().mean()
        accs.append((acc, data.num_graphs))

    accuracy = functools.reduce(operator.add,
                                [x[0] * x[1] for x in accs]) / functools.reduce(operator.add,
                                                                                [x[1] for x in accs])
    return accuracy

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--data_dir", default='baselines/supervised/data')
    parser.add_argument("--ckpt_dir", default='baselines/supervised/ckpt')
    parser.add_argument("--num_hiddens", type=int, default=64)
    parser.add_argument("--lr", type=float, default=0.01)
    parser.add_argument("--num_epochs", type=int, default=200)
    args = parser.parse_args()

    tr, vl, ts = shuffle_and_split(
        data_list=load_and_transform(args.data_dir, processed_dir=os.path.join(args.data_dir, 'processed')),
        seed=0
    )

    tr_loader = DataLoader(dataset=tr, batch_size=32, shuffle=True)
    vl_loader = DataLoader(dataset=vl, batch_size=len(vl))
    ts_loader = DataLoader(dataset=ts, batch_size=len(ts))

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    params = dict(
        num_inputs=3,
        num_hiddens=args.num_hiddens,
        num_outputs=2
    )
    os.makedirs(args.ckpt_dir, exist_ok=True)
    with open(os.path.join(args.ckpt_dir, 'config.json'), 'w') as f:
        json.dump(params, f)

    model = Net(**params).to(device)

    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr, weight_decay=5e-3)
    best_acc = 0
    losses = []
    for epoch in range(args.num_epochs):
        loss = train(model, optimizer)
        losses.append(loss)
        train_acc, vl_acc = test(model, tr_loader), test(model, vl_loader)

        if vl_acc > best_acc:
            best_acc = vl_acc
            torch.save(model.state_dict(), os.path.join(args.ckpt_dir, 'model.pth'))

        print(f'Epoch: {epoch+1:03d}, Train: {train_acc:.4f}, Val: {vl_acc:.4f}')

    model.load_state_dict(torch.load(os.path.join(args.ckpt_dir, 'model.pth')))
    ts_acc = test(model, ts_loader)
    print(f'Test: {ts_acc:.4f}')
