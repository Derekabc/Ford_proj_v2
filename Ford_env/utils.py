import matlab.engine
import matplotlib.pyplot as plt
import numpy as np


class Matlab_Eng():
    def __init__(self, sample_time):
        self.model_address = r'C:\Users\Windows\Desktop\CX482_IVA_PDP_EncryptedSimulinkModel'
        self.modelName = 'Cx482_IVA_forPDP_wDriverModel_realtime_v27_ProtecModel'
        self.eng = None
        self.sample_time = sample_time

    def reset_env(self, rendering=False):
        self.last_reward = 0
        self.r_fuel = 0
        self.eng_ori = 0  # the initail engine torque
        self.tHist = []
        self.x1Hist = []
        self.x2Hist = []
        # reuse last engine to save loading time
        if self.eng == None:
            print("Starting matlab")
            self.eng = matlab.engine.start_matlab()
        else:
            # reset matlab after one epoch
            self.eng.close("all", nargout=0)
            self.eng.bdclose("all", nargout=0)
            self.eng.clear("classes", nargout=0)
            if rendering:
                self.terminate_fig()

        # go to the model folder
        self.eng.cd(self.model_address, nargout=0)
        # run the simulation configurations (parameters)
        self.eng.Run_Sim(nargout=0)
        # Load the model
        self.eng.eval("model = '{}'".format(self.modelName), nargout=0)
        self.eng.eval("load_system(model)", nargout=0)

        self.setControlAction(0)
        self.eng.set_param(self.modelName, 'FastRestart', 'on', nargout=0)
        # Start Simulation and then Instantly pause
        self.eng.set_param(self.modelName, 'SimulationCommand',
                           'start', 'SimulationCommand', 'pause', nargout=0)
        obs = self.getObservations()
        if rendering:
            # initialize plot
            self.initialize_plot()
        return obs

    def setControlAction(self, u1):
        # set value of control action
        self.eng.set_param(
            '{}/Optimal Controller/u1'.format(self.modelName), 'value', str(u1), nargout=0)

    def getObservations(self, ):
        # get system Output and Time History
        t = self.eng.eval('tHist')
        v_mph = self.eng.eval('v_mph')
        engine_spd = self.eng.eval('engine_spd')
        MG1_spd = self.eng.eval('MG1_spd')
        MG2_spd = self.eng.eval('MG2_spd')
        Acc_pad = self.eng.eval('Acc_pad')
        Dec_pad = self.eng.eval('Dec_pad')
        WheelTD = self.eng.eval('WheelTD')
        Fuel_kg = self.eng.eval('Fuel_kg')
        SOC_C = self.eng.eval('SOC_C')
        target_speed = self.eng.eval('target_speed')
        self.eng_ori = self.eng.eval('eng_ori')
        self.eng_new = self.eng.eval('eng_new')
        if (type(v_mph) == float):
            self.Fuel_kg = Fuel_kg
            self.SOC_C = SOC_C
            self.target_speed = target_speed
            # for plotting use
            self.tHist.append(t)
            # self.x1Hist.append(self.eng_ori)
            # self.x2Hist.append(self.eng_new)
            # self.x1Hist.append(v_mph  )
            # self.x2Hist.append(target_speed * 0.621371192237334)
            self.x1Hist.append(int(Fuel_kg) * 1000)
            self.x2Hist.append(int(SOC_C))
            self.obs = (v_mph, engine_spd, MG1_spd,
                   MG2_spd, Acc_pad, Dec_pad, WheelTD)
            return np.array(self.obs)
        else:
            self.Fuel_kg = Fuel_kg[-1][0]
            self.SOC_C = SOC_C[-1][0]
            self.target_speed = target_speed[-1][0]
            self.tHist.append(t[-1][0])
            # self.x1Hist.append(self.eng_ori[-1][0])
            # self.x2Hist.append(self.eng_new[-1][0])
            # self.x1Hist.append(v_mph[-1][0])
            # self.x2Hist.append(target_speed[-1][0] * 0.621371192237334)
            self.x1Hist.append(int(Fuel_kg[-1][0]) * 1000)
            self.x2Hist.append(int(SOC_C[-1][0]))
            self.obs = [v_mph[-1][0], engine_spd[-1][0], MG1_spd[-1][0],
                        MG2_spd[-1][0], Acc_pad[-1][0], Dec_pad[-1][0], WheelTD[-1][0]]
            return np.array(self.obs)

    def run_step(self, action):
        u1 = -8 + action * 4
        # Set the Control Action
        self.setControlAction(u1)
        self.eng.set_param(self.modelName, 'SimulationCommand',
                           'StepForward', nargout=0)
        obs = self.getObservations()
        # compute the reward
        self.reward_fn()
        return obs, self.last_reward, False, True

    def reward_fn(self, ):
        # reward = fuel_consumption + speed_tracking + SOC
        # here we use g/s instead of kg/s
        self.r_fuel = self.sample_time * self.Fuel_kg * 1000
        self.last_reward = - self.r_fuel

    def disconnect(self, ):
        print("eng is closed")
        self.eng.set_param(
            self.modelName, 'SimulationCommand', 'stop', nargout=0)
        self.eng.quit()

    def initialize_plot(self, ):
        # Initialize the graph
        self.fig = plt.figure()
        self.fig1, = plt.plot([], [],
                              color='red', linewidth=1)
        self.fig2, = plt.plot([], [], color='k', linewidth=1)
        # for speed tracking
        # plt.xlim(0, 800)
        # plt.ylim(-10, 100)
        # engine torque
        # plt.xlim(0, 800)
        # plt.ylim(-100, 200)
        # for fuel consumption
        plt.xlim(0, 800)
        plt.ylim(0, 4)
        plt.ylabel("Output")
        plt.xlabel("Time(s)")
        plt.title("Fuel Consumption")

    def update_fig(self, ):
        # Update the Graph
        self.fig1.set_data(np.array(self.tHist), np.array(self.x1Hist))
        # self.fig1.set_ydata(self.x1Hist)
        self.fig2.set_xdata(self.tHist)
        self.fig2.set_ydata(self.x2Hist)
        plt.ion()
        plt.draw()
        plt.pause(0.001)
        plt.clf()

    def terminate_fig(self, ):
        plt.close(self.fig)
