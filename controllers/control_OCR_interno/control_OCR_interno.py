from controller import Supervisor, DistanceSensor, Display, Camera
import numpy as np
import random
# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

emitter = robot.getDevice("emitter")

# Enviar mensagem
message = "Hello UR5e"
emitter.send(message.encode('utf-8'))

root_node = robot.getRoot()
children_field = root_node.getField('children')

# Initialize camera
camera = robot.getDevice('camera')
camera.enable(timestep)

display = robot.getDevice('display')
display_width = display.getWidth()
display_height = display.getHeight()

sensor_cam = robot.getDevice('distance sensor')
sensor_cam.enable(timestep)
sensor_NG = robot.getDevice('removeNG')
sensor_NG.enable(timestep)

sensor_OK = robot.getDevice('removeOK')
sensor_OK.enable(timestep)

conveyorOK = robot.getFromDef('esteiraOK')
conveyorNG = robot.getFromDef('esteiraNG')
def geraPlaca(children_field, robot):
    statusImg = random.choices([0, 1], weights=[0.60, 0.40], k=1)[0]
    if(statusImg == 0):
        strNode = (f'DEF placaTeste placaIfam {{translation 0.3 0 0.65 url ["{statusImg}.jpg"]}}')
    else:
        numErrorImg = random.randint(1, 3)
        strNode = (f'DEF placaTeste placaIfam {{translation 0.3 0 0.65 url ["{numErrorImg}.jpg"]}}')

    print(strNode)
    children_field.importMFNodeFromString(-1, strNode)
    placa_node = robot.getFromDef('placaTeste')
    return placa_node, 1 - statusImg


placaTeste, resultado = geraPlaca(children_field, robot)
resultado = 0
leituraAnterior = sensor_cam.getValue()
leituraAnteriorNG = sensor_NG.getValue()
leituraAnteriorOK = sensor_OK.getValue()
done_cap_image = False
mensagem_enviada = False
while robot.step(timestep) != -1:
    posicaoPlacaTeste = placaTeste.getPosition()
    leituraAtualNG = sensor_NG.getValue()
    leituraAtualOK = sensor_OK.getValue()

    #print('PositionPlacaTeste: ', posicaoPlacaTeste)
    if (np.isclose(posicaoPlacaTeste[0], -0.549, atol=1e-3) and np.isclose(posicaoPlacaTeste[1], -0.00, atol=1e-2)
    and np.isclose(posicaoPlacaTeste[2], 0.604, atol=1e-2) and not mensagem_enviada):
        message = "inicio" + str(resultado)
        #print(message)
        emitter.send(message.encode('utf-8'))
        done_cap_image = False
        mensagem_enviada = True

    #Remove NG or OK
    if((leituraAnteriorNG == 1000.0 and leituraAtualNG < 1000.0) or (leituraAnteriorOK == 1000.0 and leituraAtualOK < 1000.0)):
        conveyorOK.getField('speed').setSFFloat(0.0)
        conveyorNG.getField('speed').setSFFloat(0.0)
        placaTeste.remove()
        placaTeste, resultado = geraPlaca(children_field, robot)
        mensagem_enviada = False

    leituraAtualCam = sensor_cam.getValue()
    if (leituraAnterior == 1000.0 and leituraAtualCam < 1000.0 and not done_cap_image):
    #if (np.isclose(posicaoPlacaTeste[0], 0.00, atol=1e-2) and np.isclose(posicaoPlacaTeste[2], 0.604, atol=1e-2)):
        # Capture a imagem da câmera
        print("OCR")
        image = camera.getImage()

        # Converte a imagem para um array NumPy
        np_image = np.frombuffer(image, dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))

        # Redimensione a imagem se necessário para caber no display
        if (camera.getWidth() != display_width) or (camera.getHeight() != display_height):
            gray_image = cv2.resize(gray_image, (display_width, display_height))

        # Crie a imagem a partir dos bytes
        print(f"Display dimensions: {display_width}, {display_height}")
        display_image = display.imageNew(image, Display.BGRA, camera.getWidth(), camera.getHeight())

        # Exiba a imagem no Display
        display.imagePaste(display_image, 0, 0, False)

        # Libere a imagem
        display.imageDelete(display_image)

        done_cap_image = True

    leituraAnterior = leituraAtualCam
    leituraAnteriorNG = leituraAtualNG
    leituraAnteriorOK = leituraAtualOK