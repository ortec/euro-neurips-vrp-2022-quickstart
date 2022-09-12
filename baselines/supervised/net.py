import torch
from torch_geometric.nn import GraphConv


class Net(torch.nn.Module):
    def __init__(self, num_inputs, num_hiddens, num_outputs):
        super().__init__()
        self.conv1 = GraphConv(num_inputs,
                               num_hiddens,
                               aggr='max')
        self.conv2 = GraphConv(num_hiddens,
                               num_hiddens,
                               aggr='max')

        self.lin = torch.nn.Linear(num_hiddens, num_outputs)

    def forward(self, data):
        x, edge_index, edge_attr = data.x, data.edge_index, data.edge_attr
        x = torch.relu(self.conv1(x, edge_index, edge_attr))
        x = torch.relu(self.conv2(x, edge_index, edge_attr))
        x = self.lin(x)
        return x
