import pybullet as p
import time
import pybullet_data
import math

# 初始化PyBullet模拟环境
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -9.81)  # 设置重力

# 添加地板
plane_id = p.loadURDF("plane.urdf")  # 默认的地板模型

# 加载URDF模型
robotId = p.loadURDF(
    r"mg400\urdf\mg400_description.urdf",
    useFixedBase=True,
)

# 创建空列表
position_control_group = []

position_control_group.append(p.addUserDebugParameter("j1", -math.pi, math.pi, 0))
position_control_group.append(p.addUserDebugParameter("j2_1", -math.pi, math.pi, 0))
position_control_group.append(p.addUserDebugParameter("j3_1", -math.pi, math.pi, 0))

position_control_group.append(p.addUserDebugParameter("j5", -math.pi, math.pi, 0))


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
    # 将添加的参数“读”出来 
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
