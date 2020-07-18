
import pybullet_data
import pybullet as p
import time
import math
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def reset():
    for i in range(numJoints):
        p.resetJointState(kukaId, i, rp[i])

def add_object():
    test_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[1,1,1],collisionFramePosition=[0,0,0])
    test_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1,1,1])
    test_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=test_collision, \
    baseVisualShapeIndex=test_visual, basePosition = [0, 0, 1])
    return test_body

#clid = p.connect(p.SHARED_MEMORY)
print(p.__file__)
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.loadURDF("plane.urdf", [0, 0, -0.3], useFixedBase=True)
kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

quater = p.getQuaternionFromEuler([0,0,0])
p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], quater)
kukaEndEffectorIndex = 6
numJoints = p.getNumJoints(kukaId)
if (numJoints != 7):
  exit()

#lower limits for null space
ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
#upper limits for null space
ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
#joint ranges for null space
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
#restposes for null space
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
#joint damping coefficents
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

for i in range(numJoints):
  p.resetJointState(kukaId, i, rp[i])

p.setGravity(0, 0, -10)
t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
useNullSpace = 0

count = 0
useOrientation = 1
useSimulation = 1

trailDuration = 15

logId1 = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT, "LOG0001.txt", [0, 1, 2])
logId2 = p.startStateLogging(p.STATE_LOGGING_CONTACT_POINTS, "LOG0002.txt", bodyUniqueIdA=2)

# for i in range(5):
#   print("Body %d's name is %s." % (i, p.getBodyInfo(i)[1]))

viewMatrix = p.computeViewMatrix(cameraEyePosition=[0,1.5,1.5],
                                 cameraTargetPosition=[0,0,0],
                                 cameraUpVector=[0,1,0])

projectionMatrix = p.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)

# plotting world and camera reference frame
viewMatrix_arr = np.array([viewMatrix[i-4:i] for i in range(4,20,4)])
u1 = np.matmul(np.linalg.inv(viewMatrix_arr).T,np.array([1,0,0,1]))
u2 = np.matmul(np.linalg.inv(viewMatrix_arr).T,np.array([0,1,0,1]))
u3 = np.matmul(np.linalg.inv(viewMatrix_arr).T,np.array([0,0,1,1]))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot([0,1],[0,0],[0,0],color='r')
ax.plot([0,0],[0,1],[0,0],color='g')
ax.plot([0,0],[0,0],[0,1],color='b')

ax.plot([0,u1[0]],[1.5,u1[1]],[1.5,u1[2]],color='r')
ax.plot([0,u2[0]],[1.5,u2[1]],[1.5,u2[2]],color='g')
ax.plot([0,u3[0]],[1.5,u3[1]],[1.5,u3[2]],color='b')
plt.show()

img = p.getCameraImage(width=256, height=256, viewMatrix=viewMatrix, projectionMatrix=projectionMatrix)


while 1:
    ls = p.getLinkState(kukaId, kukaEndEffectorIndex)
    endEffectorPos = [ls[4][i] for i in range(3)]
    endEffectorPos[1] = endEffectorPos[1] + 0.05

    orn = p.getQuaternionFromEuler([0, -math.pi, 0])

    jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, endEffectorPos, orn, ll, ul,
                                              jr, rp)

    p.stepSimulation()

        p.setJointMotorControlArray(bodyIndex=kukaId,
                                jointIndex=[i for i in range(self.numJoints)],
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=[jointPoses[i] for in range(self.numJoints)],
                                force=500,
                                positionGain=0.03,
                                velocityGain=1)


    # if (hasPrevPose):
    #     p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
    #     p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
