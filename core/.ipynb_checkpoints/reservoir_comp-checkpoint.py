import numpy as np

## data preparation
def data_prep(data, data_info, system_info):
    x = data['x'];
    u = data['u'];
    
    num_traj = data['num_traj'];
    num_snaps = data_info['num_snaps'];
    num_states = system_info['num_states'];
    num_inputs = system_info['num_inputs']

    
    X = np.zeros((4*num_states,num_snaps*num_traj))
    y = np.zeros((num_inputs,(num_snaps)*num_traj))
    
    
    for i in range(num_traj):
        X[:num_states,i*num_snaps:(i+1)*num_snaps] = x[i,:num_snaps,:num_states].T;
        X[num_states:2*num_states,i*num_snaps:(i+1)*num_snaps] = x[i,1:,:num_states].T;
    
        X[2*num_states:3*num_states,i*num_snaps:(i+1)*num_snaps] = x[i,:num_snaps,num_states:2*num_states].T;
        X[3*num_states:4*num_states,i*num_snaps:(i+1)*num_snaps] = x[i,1:,num_states:2*num_states].T;
    
        y[:,i*num_snaps:(i+1)*num_snaps] = u[i,:num_snaps,:num_inputs].T;
    
    flat_data = {}
    
    flat_data['X'] = X;
    flat_data['Y'] = y;
    
    return flat_data

## function to train reservoir computing network
def train_reservior(data_train,data_info,res_info,system_info):
    W_in = res_info['W_in'];
    res_net = res_info['res_net'];
    alpha = res_info['alpha'];
    kb = res_info['kb'];
    beta = res_info['beta'];
    n = res_info['n'];


    train_length = data_info['num_snaps']*data_info['num_train']; 

    r_train = np.zeros((n,train_length));

    train_flat_data = data_prep(data_train, data_info, system_info);

    train_x = train_flat_data['X'];
    train_y = train_flat_data['Y'];
    
    r_all = np.zeros((n,train_length))
    
    for i in range(train_length-1):
        r_all[:,i+1] = (1-alpha)*r_all[:,i] + alpha*np.tanh(np.matmul(res_net,r_all[:,i])+ np.matmul(W_in,train_x[:,i])+kb*np.ones((n,)))
        
    r_out = r_all
    r_out[1::2,:] = r_out[1::2,:]**2;
    r_end = r_all[:,-1]
    
    r_train = r_out;
    
    Wout = np.matmul(np.matmul(train_y,r_train.T),np.linalg.inv(np.matmul(np.matmul(r_train,r_train.T),beta*np.eye(n))))
    
    return Wout,r_end.reshape(1,-1)

def validate_reservoir(data_info, data_val, res_info, Wout, r_end, system_info):
    ## read parameters
    num_states = system_info['num_states'];
    dt = data_info['dt'];

    W_in = res_info['W_in'];
    res_net = res_info['res_net'];
    alpha = res_info['alpha'];
    kb = res_info['kb'];
    n = res_info['n'];


    ## read data

    r = r_end.reshape(n,1);

    ## generate desired trajectory
    val_flat_data = data_prep( data_val, data_info, system_info);

    val_x = val_flat_data['X'];
    val_y = val_flat_data['Y'];


    ## validation

    val_length = data_val['num_traj']*data_info['num_snaps'];
    u_pred = np.zeros((val_length,system_info['num_inputs']));
    taudt_threshold = np.array([-1,1]);
    tau_pred = val_y;

    # In each step, according to the predicted value, the system evolves
    # according to its inherent rule.
    for t_i in range(val_length-3):
        r = (1-alpha)*r + alpha*np.tanh(np.matmul(res_net,r).reshape(-1,1) + np.matmul(W_in,val_x[:,t_i]).reshape(-1,1)  + kb*np.ones((n,1)));
        r_out = r;
        r_out[1::2,0] = r_out[1::2,0]**2;#even number -> squared; ?????
        predict_value = np.matmul(Wout,r_out);

        if t_i == 1:
            time_li = 1;
        else:
            time_li = t_i - 1;

        for li in range(num_states):
            if predict_value[li] - tau_pred[li, time_li] > taudt_threshold[1]*dt:
                predict_value[li] = tau_pred[li, time_li] + taudt_threshold[1]*dt;
            if predict_value[li] - tau_pred[li, time_li] < taudt_threshold[0]*dt:
                predict_value[li] = tau_pred[li, time_li] + taudt_threshold[0]*dt;

        u_pred[t_i, :] = predict_value.reshape(system_info['num_inputs'],);

    ## output

    control_info = {}
    control_info['u_pred'] = u_pred;
    control_info['u_val_flat'] = val_y.T;
    
    return control_info

def get_control_reservoir(data_info, x, x_ref, res_info, Wout, r_end, system_info):
    ## read parameters
    num_states = system_info['num_states'];
    num_inputs = system_info['num_inputs'];
    dt = data_info['dt'];

    W_in = res_info['W_in'];
    res_net = res_info['res_net'];
    alpha = res_info['alpha'];
    kb = res_info['kb'];
    n = res_info['n'];


    ## read data

    r = r_end.reshape(n,1);

    ## generate desired trajectory

    val_x = np.hstack((np.hstack((np.hstack((x[:num_states],x_ref[:num_states])),x[num_states:])),x_ref[num_states:]))


    ## validation

    taudt_threshold = np.array([-1,1]);

    # In each step, according to the predicted value, the system evolves
    # according to its inherent rule.
    r = (1-alpha)*r + alpha*np.tanh(np.matmul(res_net,r).reshape(-1,1) + np.matmul(W_in,val_x).reshape(-1,1)  + kb*np.ones((n,1)));
    r_out = r;
    r_out[1::2,0] = r_out[1::2,0]**2;#even number -> squared; ?????
    predict_value = np.matmul(Wout,r_out);



    u_pred = predict_value;
    
    return u_pred.reshape(num_inputs,)  
    