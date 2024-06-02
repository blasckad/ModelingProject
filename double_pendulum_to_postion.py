import pybullet as p
import pybullet_data
import numpy as np
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("double_pendulum.urdf")
p.setGravity(0, 0, -10)
jointIndices = [1, 2, 3, 4]  # Идентификаторы суставов

# Целевое декартовое положение (x, y, z)
targetPosition = [0.5, -0.5, -1]

endEffectorIndex = 4   # Индекс конечного эфффектора
jointPositions = list(p.calculateInverseKinematics(robotId, 
                                              endEffectorIndex,
                                              targetPosition, 
                                              ))

print("pos", jointPositions)

# Установка положений суставов
# p.resetJointState(robotId,1,jointPositions[0])
# p.resetJointState(robotId,2,jointPositions[1])
# p.resetJointState(robotId,3,jointPositions[2])
p.setJointMotorControl2(robotId, 1, p.POSITION_CONTROL, targetPosition=jointPositions[0], force=50)
p.setJointMotorControl2(robotId, 2, p.POSITION_CONTROL, targetPosition=jointPositions[1], force=50)
p.setJointMotorControl2(robotId, 3, p.POSITION_CONTROL, targetPosition=jointPositions[2], force=50)


# for i, jointIndex in enumerate(jointIndices):
#     print(i,jointIndex)
#     p.setJointMotorControl2(robotId, 
#                            jointIndex, 
#                            p.POSITION_CONTROL, 
#                            targetPosition=jointPositions[i], 
#                            force=50)

# Цикл симуляции
for _ in range(1000000):
    p.stepSimulation()
    time.sleep(1./1000.)

# Завершение симуляции
p.disconnect()