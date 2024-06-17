from controller import Supervisor, Receiver
import numpy as np

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

joint_names = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]
joints = [robot.getDevice(name) for name in joint_names]

sensor_names = [
    'shoulder_pan_joint_sensor',
    'shoulder_lift_joint_sensor',
    'elbow_joint_sensor',
    'wrist_1_joint_sensor',
    'wrist_2_joint_sensor',
    'wrist_3_joint_sensor'
]

position_sensors = [robot.getDevice(name) for name in sensor_names]
for sensor in position_sensors:
    sensor.enable(timestep)
    
def move(angulosJuntas):
    for i, joint in enumerate(joints):
        joint.setPosition(angulosJuntas[i])
        

move([0.00, -1.04, 1.57, -2.09, -1.57, 0.00])        
receiver = robot.getDevice("receiver")
receiver.enable(timestep)

# obtém o nó do robô
robot_node = robot.getFromDef('roboUR5e')  # substitua 'ROBOT_DEF' pelo DEF do seu robô
# obtém o nó Pose que está no handSlot
pose_node = robot_node.getField('toolSlot').getMFNode(0)
position = pose_node.getPosition()
print('Position: ', position)
auto = False
i = 0
resultado = 0

conveyor = robot.getFromDef('esteiraOK')
conveyorNG = robot.getFromDef('esteiraNG')
conveyor.getField('speed').setSFFloat(0.0)
conveyorNG.getField('speed').setSFFloat(0.0)
# Função para converter eixo-ângulo para quaternion
def axis_angle_to_quaternion(axis_angle):
    axis = np.array(axis_angle[:3])
    angle = axis_angle[3]
    half_angle = angle / 2.0
    w = np.cos(half_angle)
    xyz = axis * np.sin(half_angle)
    return np.array([w, xyz[0], xyz[1], xyz[2]])

# Função para converter quaternion para eixo-ângulo
def quaternion_to_axis_angle(quaternion):
    w, x, y, z = quaternion
    angle = 2 * np.arccos(w)
    s = np.sqrt(1 - w*w)
    if s < 0.001: # Para evitar divisão por zero
        x, y, z = 1, 0, 0 # Arbitrário
    else:
        x /= s
        y /= s
        z /= s
    return np.array([x, y, z, angle])

# Função para multiplicar dois quaternions
def multiply_quaternions(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([w, x, y, z])
    
def rotation_matrix_to_axis_angle(orientation):
    # Converter o vetor de 9 elementos em uma matriz 3x3
    R = np.array([
        [orientation[0], orientation[1], orientation[2]],
        [orientation[3], orientation[4], orientation[5]],
        [orientation[6], orientation[7], orientation[8]]
    ])
    angle = np.arccos((np.trace(R) - 1) / 2.0)

    if angle == 0:
        return np.array([1, 0, 0, 0])

    x = (R[2, 1] - R[1, 2]) / (2 * np.sin(angle))
    y = (R[0, 2] - R[2, 0]) / (2 * np.sin(angle))
    z = (R[1, 0] - R[0, 1]) / (2 * np.sin(angle))

    return np.array([x, y, z, angle])
    
# Loop principal
while robot.step(timestep) != -1:
    if receiver.getQueueLength() > 0:
        message = receiver.getString()
        print("Received message:", message)
        if(message[0:-1] == "inicio"):
            placaTeste = robot.getFromDef("placaTeste")
            resultado = int(message[-1])
            auto = True
        receiver.nextPacket()
    if(auto):
        if(i == 0):
            move([0.00, -0.65, 1.57, -2.48, -1.57, 0.00])
        if(i == 32):
            move([0.00, -1.04, 1.57, -2.09, -1.57, 0.00])
        if(i > 32 and i <= 192):
            position = pose_node.getPosition()
            #print('Position: ', position)
            placaTranslation = placaTeste.getField('translation')
            placaTranslation.setSFVec3f([position[0], position[1], position[2] - 0.008])
            # Converter a matriz de rotação para eixo-ângulo
            initial_axis_angle = rotation_matrix_to_axis_angle(pose_node.getOrientation())            
            # Converter a rotação inicial para quaternion
            initial_quaternion = axis_angle_to_quaternion(initial_axis_angle)
            # Criar o quaternion que representa a rotação de 180 graus em relação ao eixo Y
            rotation_180_y = np.array([np.cos((12 * np.pi / 12) / 2), 0, np.sin((12 * np.pi / 12) / 2), 0])
            # Multiplicar o quaternion inicial pelo quaternion de rotação de 180 graus
            quaternion_after_180_y = multiply_quaternions(rotation_180_y, initial_quaternion)
            # Criar o quaternion que representa a rotação de -90 graus em relação ao eixo X
            rotation_minus_90_x = np.array([np.cos(-np.pi / 4), np.sin(-np.pi / 4), 0, 0])
            # Multiplicar o quaternion resultante pelo quaternion de rotação de -90 graus
            final_quaternion = multiply_quaternions(rotation_minus_90_x, quaternion_after_180_y)
            # Converter o quaternion final de volta para eixo-ângulo
            final_axis_angle = quaternion_to_axis_angle(final_quaternion)
            placaTeste.getField('rotation').setSFRotation([-final_axis_angle[0], -final_axis_angle[2],
                                                      -final_axis_angle[1], final_axis_angle[3]])
            #placaTeste.getField('rotation').setSFRotation(axis_angle)
            placaTeste.resetPhysics()
            
        if (i == 64):
            move([0.00, -1.04, 1.57, -2.8, -1.57, 0.00])
        if(resultado == 1):
            if (i == 96):
                move([resultado * 1.57 + (resultado - 1)*1.97, -1.04, 1.57, -2.8, -1.57, 0.00])
            if (i == 128):
                move([resultado * 1.57 + (resultado - 1)*1.97,  -1.04, 1.57, -2.09, -1.57, 0.00])
            if (i == 160):
                move([resultado * 1.57 + (resultado - 1)*1.97, -0.66, 1.57, -2.48, -1.57,  (resultado - 1)*0.40])
            if (i == 224):
                move([0.00, -1.04, 1.57, -2.09, -1.57, 0.00])
                auto = False
                i = -1
        else:
            if (i == 96):
                move([-2.20, -1.04, 1.57, -2.8, -1.57, 0.00])
            if (i == 128):
                move([-2.20, -1.04, 1.57, -2.11, -1.57, 0.00])
            if (i == 160):
                move([-2.20, -0.90, 2.25, -2.91, -1.57,  -0.63])
            if (i == 224):
                move([0.00, -1.04, 1.57, -2.09, -1.57, 0.00])
                auto = False
                i = -1       
        if(i > 200):
            conveyor.getField('speed').setSFFloat(0.4)
            conveyorNG.getField('speed').setSFFloat(-0.4)
        i += 1
         
