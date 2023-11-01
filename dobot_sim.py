import pybullet as p
import time
import pybullet_data
import math


physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -9.81)  


plane_id = p.loadURDF("plane.urdf")  


robotId = p.loadURDF(
    r"mg400\urdf\mg400_description.urdf",
    useFixedBase=True,
)


position_control_group = []
# joint index shown in the GUI
position_control_group.append(p.addUserDebugParameter("J1", -math.pi, math.pi, 0)) # J1=j1
position_control_group.append(p.addUserDebugParameter("J2", -math.pi, math.pi, 0)) # J2=j2_1
position_control_group.append(p.addUserDebugParameter("J3", -math.pi, math.pi, 0)) # J3=j3_1
position_control_group.append(p.addUserDebugParameter("J4", -math.pi, math.pi, 0)) # J4=j5

# real joint index
position_control_joint_name = [
    "j1",
    "j2_1",
    "j3_1",
    "j4_1",
    "j5",
    "j2_2",
    "j3_2",
    "j4_2",
]

while True:
    time.sleep(0.01)
    parameter = {}
   
    parameter[0] = p.readUserDebugParameter(position_control_group[0]) #j1
    parameter[1] = p.readUserDebugParameter(position_control_group[1]) #j2_1
    parameter[2] = p.readUserDebugParameter(position_control_group[2]) #j3_1
    parameter[3] = -parameter[1]-parameter[2] #j4_1=-(j2_1+j3_1)  ee always parallel to the ground
    parameter[4] = p.readUserDebugParameter(position_control_group[3]) #j5
    parameter[5] = parameter[1] #j2_2 = j2_1
    parameter[6] = -parameter[5] #j3_2 = -j2_2  The angle of the triangle remains unchanged
    parameter[7] = parameter[2]- parameter[6] #j4_2 =j3_1-j3_2

    num = 0

    for i in range(p.getNumJoints(robotId)):
        jointInfo = p.getJointInfo(robotId, i)
        jointName = jointInfo[1].decode("utf-8")
        if jointName in position_control_joint_name:
            parameter_sim = parameter[num]
            p.setJointMotorControlArray(
                robotId,
                jointIndices=[jointInfo[0]],
                controlMode=p.POSITION_CONTROL,  # Use p.POSITION_CONTROL here
                targetPositions=[parameter_sim],
                forces=[10],
            )
            num = num + 1
    p.stepSimulation()
