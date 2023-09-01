import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import random_split
import torch.optim as optim
from torch.utils.data import random_split
import os
import numpy as np
from sys import exit
import matplotlib.pyplot as plt

class AdaptNet(nn.Module):
    def __init__(self, adapt_net_params):
        super(AdaptNet, self).__init__()
        self.adapt_net_params = adapt_net_params
        self.del_A_matrix = None
        self.del_B_matrix = None


    def construct_adapt_net(self):
        n = self.adapt_net_params['state_dim']
        m = self.adapt_net_params['ctrl_dim']
        lift_dim = self.adapt_net_params['lift_dim']
        # include constants in lifted states
        first_obs_const = self.adapt_net_params['first_obs_const'] #only one state
        override_C = self.adapt_net_params['override_C']
        # total no of lifted states = states+constants+encoder output
        if override_C:
            self.n_tot = int(first_obs_const) + lift_dim + n
        else:
            self.n_tot = int(first_obs_const) + lift_dim

        # create the linear layer to move the states one step ahead
        self.del_A = nn.Linear(self.n_tot, self.n_tot-first_obs_const, bias = False)
        nn.init.zeros_(self.del_A.weight)
        # actuation
        self.del_B = nn.Linear(m*self.n_tot, self.n_tot-first_obs_const, bias = False)
        nn.init.zeros_(self.del_B.weight)


    def forward(self, data):
        # bilinear states
        # print("data",data.shape)
        Z = data[:,0:self.n_tot]
        # print("z",Z.shape)
        ZU = data[:,self.n_tot:]
        # print("zu",ZU.shape)
        del_A_matrix, del_B_matrix = self.construct_del_matrix_()
        del_z_pred = torch.matmul(Z, torch.transpose(del_A_matrix, 0, 1)) + torch.matmul(ZU, torch.transpose(del_B_matrix, 0, 1))
        return del_z_pred

    def construct_del_matrix_(self):
        n = self.adapt_net_params['state_dim']
        m = self.adapt_net_params['ctrl_dim']
        first_obs_const = int(self.adapt_net_params['first_obs_const'])
        const_obs_dyn_drift = torch.zeros((first_obs_const, self.n_tot)) # drift for the constant term
        self.del_A_matrix = torch.cat((const_obs_dyn_drift, self.del_A.weight), 0)

        const_obs_dyn_act = torch.zeros((first_obs_const, m*self.n_tot)) # actuation for the constant term
        self.del_B_matrix = torch.cat((const_obs_dyn_act, self.del_B.weight), 0)

        return self.del_A_matrix, self.del_B_matrix

    def loss(self, outputs, labels):
        alpha = self.adapt_net_params['l1_reg']
        criterion = nn.MSELoss()
        loss = criterion(outputs, labels) + alpha * torch.norm(self.del_A_matrix, p=1) + alpha * torch.norm(self.del_B_matrix, p=1)
        return loss


    def model_pipeline(self, Z, ZU, del_Z, print_epoch = True):
        self.construct_adapt_net()
        self.set_optimizer_()
        X_train = np.concatenate((Z.T, ZU.T), axis=1)
        # print("x",X_train.shape)
        Y_train = del_Z.T
        X_train_t, y_train_t = torch.from_numpy(X_train).float(), torch.from_numpy(Y_train).float()
        dataset_train = torch.utils.data.TensorDataset(X_train_t, y_train_t)
        self.train_model(dataset_train, print_epoch)

    def train_model(self, dataset_train, print_epoch=True):
        trainloader = torch.utils.data.DataLoader(dataset_train, batch_size = self.adapt_net_params['batch_size'], shuffle = False)

        no_improv_counter = 0
        self.train_loss_hist = []

        for epoch in range(self.adapt_net_params['epochs']):
            running_loss = 0.0
            epoch_steps = 0

            for data in trainloader:
                inputs, labels = data

                self.optimizer.zero_grad()
                output = self(inputs)
                loss = self.loss(output, labels)
                loss.backward()
                self.optimizer.step()

                running_loss += loss.detach()
                epoch_steps += 1

    def set_optimizer_(self):

        if self.adapt_net_params['optimizer'] == 'adam':
            lr = self.adapt_net_params['lr']
            weight_decay = self.adapt_net_params['l2_reg']
            self.optimizer = optim.Adam(self.parameters(), lr=lr, weight_decay=weight_decay)
        elif self.adapt_net_params['optimizer'] == 'adamax':
            lr = self.adapt_net_params['lr']
            weight_decay = self.adapt_net_params['l2_reg']
            self.optimizer = optim.Adam(self.parameters(), lr=lr, weight_decay=weight_decay)

    def plot_adaptation(self):
        train_loss = np.array(self.train_loss_hist)
        iter = np.arange(train_loss.shape[0])
        titles = ["adaptation loss"]
        plt.figure(figsize = (16,5))
        plt.plot(iter, train_loss)
        plt.show()

    def get_del_matrices(self):

        del_A_matrix = np.array(self.del_A_matrix.detach())
        del_B_matrix = np.array(self.del_B_matrix.detach())

        return del_A_matrix, del_B_matrix


