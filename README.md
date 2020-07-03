# Ford_proj_v2

The agent is DDQN and environment is built on Matlab. The RL agent sends the  actions to MATLAB and obtains the observation from the MATLAB.

The Gym-like environment file is located in [Ford_env](Ford_env/) folder:

- [env_ford.py](Ford_env/env_ford.py) file contains the environment file.
- [utils.py](Ford_env/utils.py) file contains the util functions for building PYTHON to MATLAB SIMULINK bridge.
