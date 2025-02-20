This is the code repository for the paper titled "**Adaptive Koopman Architectures for Control of Complex NonlinearSystems**". The paper can be found at https://arxiv.org/abs/2405.09101.

This study presents an adaptive Koopman algorithm capable of responding to the changes in system dynamics online. The proposed framework initially employs an autoencoder-based neural network which utilizes input-output information from the nominal system to learn the corresponding Koopman embedding offline. Subsequently, we augment this nominal Koopman architecture with a feed-forward neural network that learns to modify the nominal dynamics in response to any deviation between the predicted and observed lifted states, leading to improved generalization and robustness to a wide range of uncertainties and disturbances as compared to contemporary methods.The proposed adaptive Koopman architecture is integrated within a Model Predictive Control (MPC) framework to enable optimal control of the underlying nonlinear system in the presence of uncertainties along with state and input constraints. 

![adaptation_block](https://github.com/Rajpal9/Adaptive-koopman/assets/90927685/9bcaec27-a618-40e6-bb59-6771908c5ec1)

If you use the code in the academic context, please cite:
- R. Singh, C.K. Sah and J. Keshavan, "Adaptive Koopman Embedding for Robust Control of Complex Dynamical Systems.", arXiv e-prints, pp.arXiv-2405, 2024.
