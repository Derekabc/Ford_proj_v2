import sys
sys.path.append("../")

import gym
import argparse
import configparser
from gym.utils import seeding
from Ford_env.utils import *


def parse_args():
    default_base_dir = 'Data'
    default_config_dir = 'config/config_ford.ini'
    parser = argparse.ArgumentParser()
    parser.add_argument('--base-dir', type=str, required=False,
                        default=default_base_dir, help="experiment base dir")
    parser.add_argument('--config-dir', type=str, required=False,
                        default=default_config_dir, help="experiment config dir")
    parser.add_argument('--is_training', type=str, required=False,
                        default=not True, help="True=train, False=evaluation")
    parser.add_argument('--test-mode', type=str, required=False,
                        default='no_test',
                        help="test mode during training",
                        choices=['no_test', 'in_train_test', 'after_train_test', 'all_test'])

    args = parser.parse_args()
    return args


class FordEnv(gym.Env):
    """
    This is the environment for ford project which is built on Matlab and python.

    Observation:
    Type: Box(7)
    Num	Observation                 Min         Max
    0	VehicleSpd_mph               0          100
    1	Engine_Spd_c__radps         -1e4        1e4
    2	MG1_Spd_radps               -1e4        1e4
    3	MG2_Spd_radps               -1e4        1e4
    4   Acc_pad                      0           1
    5   Dec_pad                      0           1
    6   WheelTqDemand_Nm           -1e4         1e4

    Actions:
        Type: Discrete(discrete_resolution)
        Num	Action
        0	Push cart to the left
        1	Push cart to the right
    """

    def __init__(self, config, discrete=True, rendering=False):
        # Setup gym environment
        # file name of parameters, we need to run it first
        self.rendering = rendering
        self.sample_time = config.getfloat('sample_time')
        self.episode_length = int(config.getfloat('episode_length'))
        self.seed(33)
        # self.fuel_ls = []
        # self.soc_ls = []
        # for compute speed tracking error
        # self.v_mph_ls = []
        # self.target_speed_ls = []
        # self.eng_org_ls = []
        # self.eng_new_ls = []
        # self.tHist = []
        # self.SOC = []

        low = np.array([0, -1e4, -1e4, -1e4, 0, 0, -1e4])
        high = np.array([100, 1e4, 1e4, 1e4, 1, 1, 1e4])

        if discrete is True:
            self.action_space = gym.spaces.Discrete(5)
            self.observation_space = gym.spaces.Box(
                low, high, dtype=np.float32)
        else:
            self.action_space = gym.spaces.Box(np.array([-1, 0]), np.array([1, 1]),
                                               dtype=np.float32)
            self.observation_space = gym.spaces.Box(
                -high, high, dtype=np.float32)

        try:
            # initialize matlab and Ford_env
            self.Matlab_Eng = Matlab_Eng(self.sample_time)
        except Exception as e:
            self.close()
            raise e

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self, ):
        # save reward to compute reward_norm
        # dir = 'Data-1/results/'
        # np.save(dir + '{}'.format('fuel_ls'), self.fuel_ls)
        # np.save(dir + '{}'.format('soc_ls'), self.soc_ls)

        # for compute speed tracking error
        # np.save(dir + '{}'.format('v_mph_ls'), self.v_mph_ls)
        # np.save(dir + '{}'.format('target_speed_ls'), self.target_speed_ls)

        # for see the eng torque changes
        # np.save(dir + '{}'.format('eng_org_ls'), self.eng_org_ls)
        # np.save(dir + '{}'.format('eng_new_ls'), self.eng_new_ls)

        self.steps = 0
        # self.fuel_ls = []
        # self.soc_ls = []
        # for compute speed tracking error
        # self.v_mph_ls = []
        # self.target_speed_ls = []
        # for see the eng torque changes
        # self.eng_org_ls = []
        # self.eng_new_ls = []
        # reset the MATLAB model
        self.obs = self.Matlab_Eng.reset_env(self.rendering)
        return self.obs

    def close(self):
        self.Matlab_Eng.disconnect()

    def render(self, ):
        self.Matlab_Eng.update_fig()

    def step(self, action):
        obs_mean = np.array([4.83868269e+01, 9.26671424e+01, 6.41770269e+02, -3.11372911e+02,
                             6.78844516e-02, 1.27067008e-02, 1.46767778e+02])
        obs_std = np.array([9.87342636e+00, 1.13743122e+02, 1.30954515e+02, 3.87827102e+02,
                            5.11422102e-02, 6.47789938e-02, 2.07343922e+02])

        if action is not None:
            obs_new, self.last_reward, done, _ = self.Matlab_Eng.run_step(
                action)
            obs_new = (obs_new - obs_mean) / obs_std
        if self.steps >= int(self.episode_length / self.sample_time) - 10:
            done = True
            # self.fuel_ls.append(self.Matlab_Eng.x1Hist)
            # self.soc_ls.append(self.Matlab_Eng.x2Hist)
            # for compute speed tracking error
            # self.v_mph_ls.append(self.Matlab_Eng.x1Hist)
            # self.target_speed_ls.append(self.Matlab_Eng.x2Hist)
            # for see the eng torque changes
            # self.eng_org_ls.append(self.Matlab_Eng.x1Hist)
            # self.eng_new_ls.append(self.Matlab_Eng.x2Hist)
            # self.tHist.append(self.Matlab_Eng.tHist)
            # self.SOC .append(self.Matlab_Eng.SOC_C_ls)
            # np.save('Data-1/results/' + '{}'.format('eng_org_ls'), self.eng_org_ls)
            # np.save('Data-1/results/' + '{}'.format('eng_new_ls'), self.eng_new_ls)
            # np.save('Data-1/results/' + '{}'.format('tHist'), self.tHist)
            # np.save('Data-1/results/' + '{}'.format('SOC'), self.SOC)

        self.steps += 1

        return obs_new, self.last_reward, done, _


if __name__ == "__main__":
    args = parse_args()
    config_dir = args.config_dir
    config = configparser.ConfigParser()
    config.read(r'C:\Users\Windows\PycharmProjects\Ford_proj_v2\config\config_ford.ini')
    epoch = 0
    # Example of using FordEnv with sample controller
    env = FordEnv(config['ENV_CONFIG'])
    action_size = env.action_space.n
    print('--------------')
    print("Simulation starting...")
    while True:
        env.reset()
        reward_ls = []
        rewards = 0
        last_reward = 0
        while True:
            # print('--------------')
            # print("steps = ", Ford_env.steps)
            # print("rewards = ", last_reward)
            action = np.random.randint(action_size, size=1)
            # Take an action
            obs, last_reward, done, _ = env.step(action[0])  # action[0], 2
            reward_ls.append(last_reward)
            rewards += last_reward
            if done:
                break
        print('--------------')
        print("steps = ", env.steps)
        print("rewards = ", rewards)
        np.save('{}'.format('reward_ls'), reward_ls)
        epoch += 1
    env.close()
