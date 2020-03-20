import pybullet as p
import pybullet_data

class World(object):
    def __init__(self,headless=False):
        p.connect(p.DIRECT if headless else p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)

    def add_object(self):
        test_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05,0.05,0.05], rgbaColor=[1,0,0,1])
        test_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05,0.05,0.05])
        body = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=test_collision, \
        baseVisualShapeIndex=test_visual, basePosition = [0, 0, 1])
        return body

    def move_object(self, bodyId, pos, orn):
        # register objects locally
        p.resetBasePositionAndOrientation(bodyId, pos, p.getQuaternionFromEuler(orn))

    def get_pos(self, bodyId):
        pos, orn = p.getBasePositionAndOrientation(bodyId)
        orn = p.getEulerFromQuaternion(orn)
        return pos, orn

    def change_texture(self):
        return
