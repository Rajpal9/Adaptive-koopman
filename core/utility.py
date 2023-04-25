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


def Output_Con(A,B,C):
    for i in range(A.shape[0]):
        if  i == 0:
            A_pow = np.eye(A.shape[0])
            out_con = np.matmul(np.matmul(C,A_pow),B)
        else:
            A_pow = np.matmul(A_pow,A)
            out_con = np.hstack((out_con,np.matmul(np.matmul(C,A_pow),B)))

    
    out_con = np.hstack((out_con,np.zeros((C.shape[0],B.shape[1]))))
    out_con_rank = np.linalg.matrix_rank(out_con)
    
    return out_con, out_con_rank