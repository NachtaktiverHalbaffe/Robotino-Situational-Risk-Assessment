from tkinter.constants import N
import gym
import torch as th
from torch import nn

from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable
import math
import numpy as np

class SameExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space: gym.spaces.Box):
        # We do not know features-dim here before going over all the items,
        # so put something dummy for now. PyTorch requires calling
        # nn.Module.__init__ before adding modules
        super(SameExtractor, self).__init__(
            observation_space, features_dim=1)

        self.extractors = nn.Sequential(
            self.layer_init(nn.Conv2d(1, 8, (3, 3))),
            nn.LeakyReLU(),
            nn.BatchNorm2d(8),
            self.layer_init(nn.Conv2d(8, 8, (3, 3))),
            nn.LeakyReLU(),
            nn.BatchNorm2d(8),
            self.layer_init(nn.Conv2d(8, 16, (5, 5), stride=(2, 2))),
            nn.LeakyReLU(),
            nn.BatchNorm2d(16),

            self.layer_init(nn.Conv2d(16, 16, (3, 3))),
            nn.LeakyReLU(),
            nn.BatchNorm2d(16),
            self.layer_init(nn.Conv2d(16, 16, (3, 3))),
            nn.LeakyReLU(),
            nn.BatchNorm2d(16),
            self.layer_init(nn.Conv2d(16, 16, (5, 5), stride=(2, 2))),
            nn.LeakyReLU(),
            nn.BatchNorm2d(16),

            self.layer_init(nn.Conv2d(16, 16, (3, 3))),
            nn.LeakyReLU(),
            nn.BatchNorm2d(16),
            self.layer_init(nn.Conv2d(16, 16, (3, 3))),
            nn.LeakyReLU(),
            nn.BatchNorm2d(16),
            self.layer_init(nn.Conv2d(16, 16, (3, 3), stride=(2, 2))),
            nn.LeakyReLU(),
            nn.BatchNorm2d(16),

            nn.Flatten())

        # Update the features dim manually
        self._features_dim = 16 * 19 * 19

        
    def layer_init(self, layer, std=np.sqrt(2), bias_const=0.0):
        """
        this function initializes the passed layer with an "orthogonal" initialization
        """
        nn.init.orthogonal_(layer.weight, std)
        nn.init.orthogonal_(layer.weight, std)
        nn.init.constant_(layer.bias, bias_const)
        return layer

    def forward(self, observations) -> th.Tensor:
        return self.extractors(observations)