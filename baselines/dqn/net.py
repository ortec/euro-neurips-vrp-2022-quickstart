import torch
from torch import nn


class Network(nn.Module):

    def __init__(self, num_inputs: int, num_hiddens: int = 128, k_nearest: int = 10):
        """Initialization."""
        super(Network, self).__init__()

        self.k_nearest = k_nearest

        self.layers = nn.Sequential(
            nn.Linear(num_inputs, num_hiddens),
            nn.ReLU(),
            nn.Linear(num_hiddens, num_hiddens),
            nn.ReLU(),
            nn.Linear(num_hiddens, 2)
        )

    @property
    def num_inputs(self):
        return self.layers[0].in_features

    @property
    def num_hiddens(self):
        return self.layers[0].out_features

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Forward method implementation."""
        return self.layers(x)
