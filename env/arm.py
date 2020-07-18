import pybullet as p
from .world import World
import math

class Arm(World):
    def __init__(self, reward, replay_buffer, headless=False):
        World.__init__(self,headless=headless)
        self.armId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

        quater = p.getQuaternionFromEuler([0,0,0])
        p.resetBasePositionAndOrientation(self.armId, [0, 0, 0], quater)
        self.armEndEffectorIndex = 6
        self.numJoints = p.getNumJoints(self.armId)

        #lower limits for null space
        self.ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
        #upper limits for null space
        self.ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
        #joint ranges for null space
        self.jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
        #restposes for null space
        self.rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
        #joint damping coefficents
        self.jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

        self.reset()

        p.setGravity(0, 0, -9.81)

        t = 0.
        prevPose = [0, 0, 0]
        prevPose1 = [0, 0, 0]
        hasPrevPose = 0
        useNullSpace = 0

        count = 0
        useOrientation = 1
        useSimulation = 1

        trailDuration = 15

        self.projectionMatrix = p.computeProjectionMatrixFOV(
            fov=45.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=3.1)

        self.target_object = self.add_object()
        self.move_object(self.target_object, [-0.5,0,0.025],[0,0,0])

        self.prev_action = [0,0,0]

        self.reward = reward 
        self.replay_buffer = replay_buffer

        self._eval = False

    def reset(self):
        for i in range(self.numJoints):
            p.resetJointState(self.armId, i, self.rp[i])

    def render(self):
        viewMatrix = p.computeViewMatrix(cameraEyePosition=[0,1.5,1.5],
                                         cameraTargetPosition=[0,0,0],
                                         cameraUpVector=[0,1,0])

        img = p.getCameraImage(width=256, height=256, viewMatrix=viewMatrix, projectionMatrix=self.projectionMatrix)

        return img

    def ee_pos(self):
        ls = p.getLinkState(self.armId, self.armEndEffectorIndex)
        endEffectorPos = [ls[4][i] for i in range(3)]
        return endEffectorPos

    def step(self, action):
        prev_obs = self.get_observation()

        jointPoses = self.ik(offset=action)
        p.setJointMotorControlArray(bodyIndex=self.armId,
                                jointIndices=[i for i in range(self.numJoints)],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[jointPoses[i] for i in range(self.numJoints)],
                                forces=[500 for _ in range(self.numJoints)],
                                positionGains=[0.03 for _ in range(self.numJoints)],
                                velocityGains=[1 for _ in range(self.numJoints)])
        p.stepSimulation()

        obs = self.get_observation()

        self.prev_action = action

        r = self.reward(self.ee_pos(), self.get_pos(self.target_object))

        if not self._eval:
            self.replay_buffer.add(prev_obs + action + obs + [r])

        return obs, r

    def ik(self, offset=[0,0,0]):
        endEffectorPos = self.ee_pos()
        for i in range(len(offset)):
            endEffectorPos[i] = endEffectorPos[i] + offset[i]
        orn = p.getQuaternionFromEuler([0, -math.pi, 0])
        jointPoses = p.calculateInverseKinematics(self.armId, self.armEndEffectorIndex, endEffectorPos, orn, self.ll, self.ul,
                                                  self.jr, self.rp)
        return jointPoses

    def get_observation(self):
        cube_pos = self.get_pos(self.target_object)
        pw = p.getJointStates(self.armId, [i for i in range(self.numJoints)])
        print(list(cube_pos[0]) , self.ee_pos() , self.prev_action)
        return list(cube_pos[0]) + self.ee_pos() + self.prev_action
    
    def eval(self):
        self._eval = True
    
    def train(self):
        self._eval = False