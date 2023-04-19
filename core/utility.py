import numpy as np

def generate_centres(num_centres, data):
    # centres = np.vstack((0.1*(2*np.random.rand(int(num_states/2),num_centres) - 1),
    #                     np.pi*(2*np.random.rand(int(num_states/2),num_centres) - 1)))
    num_states = data.shape[0]
    length = data.shape[1]
    idx = np.random.choice(np.arange(length),num_centres)
    centres = np.zeros((num_states,num_centres))
    j = 0
    for i in idx:
        centres[:,j] = data[:,i]
        j= j+1
    return centres