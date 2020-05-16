import pybullet as p
# Can alternatively pass in p.DIRECT 
client = p.connect(p.GUI)
p.setGravity(0, 0, -10, physicsClientId=client) 

import pybullet_data
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")

carId = p.loadURDF(“racecar/racecar.urdf”, basePosition=[0,0,0.2])
position, orientation = p.getBasePositionAndOrientation(carId)

for _ in range(100): 
    p.stepSimulation()