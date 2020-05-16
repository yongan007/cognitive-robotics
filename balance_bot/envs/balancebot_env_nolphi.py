import os
import math
import numpy as np
import gym
from gym import spaces
from gym.utils import seeding
import pybullet as p
import pybullet_data

class BalancebotEnv(gym.Env):
    metadata = {
    'render.modes': ['human', 'rgb_array'],
    'video.frames_per_second' : 50
    }
    
    def __init__(self):
        # action encodes the torque applied by the motor of the wheels
        # action encodes the torque applied by the motor of the wheels
        self.action_space = spaces.Box(-1., 1., shape=(1,), dtype='float32')
        # observation encodes pitch, gyro, com.sp.
        self.observation = []
        self.observation_space = spaces.Box(np.array([-math.pi, -math.pi, -5]), np.array([math.pi, math.pi, 5]), dtype='float32')
        self.connectmode = 0
        # starts without graphic by default

        self.physicsClient = p.connect(p.DIRECT) #Evolving version
        # self.physicsClient = p.connect(p.GUI) #Graphical version

        # used by loadURDF
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.seed()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        self.set_actuator(action)
        p.stepSimulation()
        self.observation = self.compute_observation()
        reward = self.compute_reward()
        done = self.compute_done()
        self.envStepCounter += 1
        status = "Step " + str(self.envStepCounter) + " Reward " + '{0:.2f}'.format(reward)
        p.addUserDebugText(status, [0,-1,3], replaceItemUniqueId=1)
        return np.array(self.observation), reward, done, {}

    def reset(self):
        self.vt = 0
        # current velocity pf the wheels
        self.maxV = 24.6 # max lelocity, 235RPM = 24,609142453 rad/sec
        self.envStepCounter = 0
        p.resetSimulation()
        p.setGravity(0, 0,-10) # m/s^2
        p.setTimeStep(0.01)
        # the duration of a step in sec
        planeId = p.loadURDF("plane.urdf")
        robotStartPos = [0,0,0.001]
        robotStartOrientation = p.getQuaternionFromEuler([self.np_random.uniform(low=-
        0.3, high=0.3),0,0])
        path = os.path.abspath(os.path.dirname(__file__))
        self.botId = p.loadURDF(os.path.join(path, "balancebot_simple.xml"),
        robotStartPos,
        robotStartOrientation)
        self.observation = self.compute_observation()
        return np.array(self.observation)

    def render(self, mode='human', close=False):
        if (self.connectmode == 0):
            p.disconnect(self.physicsClient)
            # connect the graphic renderer
            self.physicsClient = p.connect(p.GUI)
            self.connectmode = 1
        pass

    def set_actuator(self, action):
        deltav = action[0]
        vt = np.clip(self.vt + deltav, -self.maxV, self.maxV)
        self.vt = vt
        print(self.botId)
        p.setJointMotorControl2(bodyUniqueId=self.botId, jointIndex=0, controlMode=p.VELOCITY_CONTROL, targetVelocity=vt)
        p.setJointMotorControl2(bodyUniqueId=self.botId, jointIndex=1, controlMode=p.VELOCITY_CONTROL, targetVelocity=-vt)

    def compute_observation(self):
        robotPos, robotOrn = p.getBasePositionAndOrientation(self.botId)
        robotEuler = p.getEulerFromQuaternion(robotOrn)
        linear, angular = p.getBaseVelocity(self.botId)
        return (np.array([robotEuler[0],angular[0],self.vt], dtype='float32'))

    def compute_reward(self):
        robotPos, robotOrn = p.getBasePositionAndOrientation(self.botId)
        robotEuler = p.getEulerFromQuaternion(robotOrn)
        linear, angular = p.getBaseVelocity(self.botId)
        r = sum(linear) - sum(angular)
        # receive a bonus of 1 for balancing and pay a small cost proportional to speed
        return r  #1.0 - abs(self.vt) * 0.05

    def compute_done(self):
        # episode ends when the barycentre of the robot is too low or after 500 steps
        # cubePos, agle = p.getBasePositionAndOrientation(self.botId)
       # angle[] < 0.15 or 
        return self.envStepCounter >= 50