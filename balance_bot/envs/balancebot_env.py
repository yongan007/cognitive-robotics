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

    def __init__(self, render=False):

        self._observation = []
        self.action_space = spaces.Discrete(4)
        self.observation_space = spaces.Box(np.array([0, 0, 5]), 
                                            np.array([0, 0, 5])) # pitch, gyro, com.sp.
    
        # if (render):
        self.physicsClient = p.connect(p.GUI)
        # else:
        # self.physicsClient = p.connect(p.DIRECT)  # non-graphical version

        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF

        self._seed()
        
        # paramId = p.addUserDebugParameter("My Param", 0, 100, 50)

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):
        self._assign_throttle(action)
        p.stepSimulation()
        self._observation = self._compute_observation()
        reward = self._compute_reward()
        done = self._compute_done()

        self._envStepCounter += 1

        return np.array(self._observation), reward, done, {}

    def _reset(self):
        # reset is called once at initialization of simulation
        self.vt = 0
        self.vd = 0
        self.maxV = 24.6 # 235RPM = 24,609142453 rad/sec
        self._envStepCounter = 0

        p.resetSimulation()
        p.setGravity(0,0,-10) # m/s^2
        p.setTimeStep(0.01) # sec
        planeId = p.loadURDF("plane.urdf")
        cubeStartPos = [0,0,0.001]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

        path = os.path.abspath(os.path.dirname(__file__))
        self.botId = p.loadURDF(os.path.join(path, "balancebot_simple.xml"),
                           cubeStartPos,
                           cubeStartOrientation)

        # you *have* to compute and return the observation from reset()
        self._observation = self._compute_observation()
        return np.array(self._observation)

    # def _load_geometry(self):
    #     self.groundId = p.loadURDF("plane.urdf")

    def _assign_throttle(self, action):
        # print(action)
        dv = 0.1
        deltav = [0.1*dv, 2.*dv,5.*dv, 10.*dv][action]
        vt = clamp(self.vt + deltav, -self.maxV, self.maxV)
        self.vt = vt

        p.setJointMotorControl2(bodyUniqueId=self.botId, 
                                jointIndex=0, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocity=-vt,force=1000)
        p.setJointMotorControl2(bodyUniqueId=self.botId, 
                                jointIndex=1, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocity=-vt,force=1000)

    def _compute_observation(self):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.botId)
        cubeEuler = p.getEulerFromQuaternion(cubeOrn)
        linear, angular = p.getBaseVelocity(self.botId)
        
        x = cubePos[0]+self.vt
        y = cubePos[1]+self.vt

        return [x,y,self.vt]

        
    def _compute_reward(self):
        robotPos, robotOrn = p.getBasePositionAndOrientation(self.botId)
        robotEuler = p.getEulerFromQuaternion(robotOrn)
        linear, angular = p.getBaseVelocity(self.botId)
        r = sum(linear) - sum(angular)
        # receive a bonus of 1 for balancing and pay a small cost proportional to speed
        return r  #1.0 - abs(self.vt) * 0.05

    def _compute_done(self):
        cubePos, _ = p.getBasePositionAndOrientation(self.botId)

        return self._envStepCounter >= 2000

    def _render(self, mode='human', close=False):
        pass

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)