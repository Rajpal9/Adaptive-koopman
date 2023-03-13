import torch
import numpy as np
from core.koopman_core import KoopDNN,KoopmanNet,KoopmanNetCtrl

def model_matricies(file):
    model_koop_dnn = torch.load(file)
    # Koopman model parameters
    A = np.array(model_koop_dnn.A)
    #B = np.array(model_koop_dnn.B).reshape(-1,num_inputs*n_obs)
    B = np.array(model_koop_dnn.B)
    #B_tensor = np.empty((num_inputs,n_obs, n_obs))
    #for ii, b in enumerate(B):
        #B_tensor[ii] = b
    C = np.array(model_koop_dnn.C)

    print(A.shape, B.shape, C.shape)
    return A,B,C


def lift(x,model_koop_dnn, params):
    first_obs_const = params['first_obs_const']
    override_C = params['override_C']
    if first_obs_const == 1:
        if override_C:
            Z = np.concatenate((np.ones((1,)),x,model_koop_dnn.net.encode_forward_(torch.from_numpy(x).float()).detach().numpy()))
        else:
            Z = np.concatenate((np.ones((1,)),model_koop_dnn.net.encode_forward_(torch.from_numpy(x).float()).detach().numpy()))
    else:
        if override_C:
            Z = np.concatenate((x,model_koop_dnn.net.encode_forward_(torch.from_numpy(x).float()).detach().numpy()))
        else:
            Z = (model_koop_dnn.net.encode_forward_(torch.from_numpy(x).float()).detach().numpy())
    return Z