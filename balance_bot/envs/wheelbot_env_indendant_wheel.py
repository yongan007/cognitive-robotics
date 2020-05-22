import os
import math
import numpy as np

import gym
from gym import spaces
from gym.utils import seeding

import pybullet as p
import pybullet_data


"""
The main objective is move robot forward: 

- I observe robot pose X, Y and speed

- reward : r = sum(linear) - sum(angular)
"""

class BalancebotEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : 50
    }

    def __init__(self, render=False):

        self._observation = []
        self.action_space = spaces.Discrete(2)
        self.observation_space = spaces.Box(np.array([0, 10, 20]), 
                                            np.array([0, -10, -20])) # robot pose x,y and speed 
    
        # if (render):
        # self.physicsClient = p.connect(p.GUI)
        # else:
        self.physicsClient = p.connect(p.DIRECT)  # non-graphical version

        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF

        self._seed()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):
        self._assign_throttle_1(action)
        p.stepSimulation()
        self._observation = self._compute_observation()
        reward = self._compute_reward()
        done = self._compute_done()

        self._envStepCounter += 1

        # self._compute_collision()
        return np.array(self._observation), reward, done, {}

    def _reset(self):
        # reset is called once at initialization of simulation
        self.vt1 = 0
        self.vt2 = 0
        self.maxV = 10# 235RPM = 24,609142453 rad/sec
        self._envStepCounter = 0


        p.resetSimulation()
        p.setGravity(0,0,-10) # m/s^2
        p.setTimeStep(0.01) # sec
        self._load_geometry()

        self._load_bot()
        self._load_box()

        # you *have* to compute and return the observation from reset()
        self._observation = self._compute_observation()
        return np.array(self._observation)

    def _load_geometry(self):
        self.groundId = p.loadURDF("plane.urdf")
    
    def _load_box(self):
        boxStartPos_r = [0.7,-1.0,0.001]
        boxStartPos_l = [-0.7,-1.0,0.001]
        boxStartOrientation = p.getQuaternionFromEuler([0,0,0])
        path = os.path.abspath(os.path.dirname(__file__))
        self.boxId_right = p.loadURDF(os.path.join(path, "wall.xml"),
                           boxStartPos_r,
                           boxStartOrientation)
        self.boxId_left = p.loadURDF(os.path.join(path, "wall.xml"),
                           boxStartPos_l,
                           boxStartOrientation)
        


    def _load_bot(self):
        cubeStartPos = [0,0,0.001]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,np.pi])
        path = os.path.abspath(os.path.dirname(__file__))
        self.botId = p.loadURDF(os.path.join(path, "cilindric_bot.xml"),#cilindric_bot.xml
                           cubeStartPos,
                           cubeStartOrientation)


    def _assign_throttle_1(self, action):
 
     
        vt1 = [-self.maxV, self.maxV][action]#[-self.maxV,-0.5*self.maxV,0, 0.5*self.maxV, self.maxV]
        vt2 = [-self.maxV, self.maxV][action]
        self.vt1 = vt1
        self.vt2 = vt2

        p.setJointMotorControl2(bodyUniqueId=self.botId, 
                                jointIndex=0, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocity=vt1,force=1000)
        p.setJointMotorControl2(bodyUniqueId=self.botId, 
                                jointIndex=1, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocity=-
                                vt2,force=1000)



    def _compute_observation(self):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.botId)
        cubeEuler = p.getEulerFromQuaternion(cubeOrn)
        linear, angular = p.getBaseVelocity(self.botId)

        x = cubePos[0] #+self.vt1
        y = cubePos[1]#+self.vt1

        return [-x, -y , self.vt1]

    def _compute_collision(self):
        
         #robot info   
        part_name, robot_name = p.getBodyInfo(self.botId)   
        robot_name = robot_name.decode("utf8")
        part_name = part_name.decode("utf8")

        #contact point it return a list of contact points(14 parameters)  
        contract_point = p.getContactPoints(self.botId) 

        contract_points = np.array(contract_point)
        number_of_body = contract_points.shape[0]

        #detect if it is touch the wall 
        if number_of_body == 0 :#and contract_points[0,2] == 3 :
            detected = True 
        else:
            detected = False

        return detected
        # return a[0,3] 

        
    def _compute_reward(self):
        robotPos, robotOrn = p.getBasePositionAndOrientation(self.botId)
        robotEuler = p.getEulerFromQuaternion(robotOrn)

        linear, angular = p.getBaseVelocity(self.botId)
        r = sum(linear) #- sum(angular)

        return   self.vt1+self.vt2

    def _compute_done(self):
        # episode ends when the barycentre of the robot is too low or after 500 steps
        detected = self._compute_collision()


        return self._envStepCounter >= 1000 or detected == True

    def _render(self, mode='human', close=False):
        pass

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


#python3 ../../bin/es.py -f balancebot.ini -s 2