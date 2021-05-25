# importamos las librerías necesarias
from lib import sim  # librería para conectar con CoppeliaSim
import numpy as np
import time


# función para conectar el script con la simulación en coppelia
def connect(port):
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
    if clientID == 0:
        print("conectado a", port)
    else:
        print("no se pudo conectar")
    return clientID


# función que activa/desactiva la ventosa para succionar la pagina
'''
def setEffector(val):
    res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,
        "suctionPad", sim.sim_scripttype_childscript,"setEffector",[val],[],[],"", sim.simx_opmode_blocking)
    return res
'''


# función que aplica la cinematica inversa a partir de unas coordenadas planares
def coord2move(pz, px, ph, clientID, joint1, joint2, joint3):
    l1 = 90
    l2 = 66
    l3 = 40

    phi = np.deg2rad(ph)

    wx = px - l3 * np.cos(phi)
    wy = pz - l3 * np.sin(phi)

    delta = wx ** 2 + wy ** 2
    c2 = (delta - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    s2 = np.sqrt(1 - c2 ** 2)

    a = np.arctan2(float(s2), float(c2))
    theta_2 = a

    s1 = ((l1 + l2 * c2) * wy - l2 * s2 * wx) / delta
    c1 = ((l1 + l2 * c2) * wx + l2 * s2 * wy) / delta
    theta_1 = np.arctan2(float(s1), float(c1))
    theta_3 = phi - theta_1 - theta_2

    _ = sim.simxSetJointTargetPosition(clientID, joint1, theta_1, sim.simx_opmode_oneshot)
    _ = sim.simxSetJointTargetPosition(clientID, joint2, theta_2, sim.simx_opmode_oneshot)
    _ = sim.simxSetJointTargetPosition(clientID, joint3, theta_3, sim.simx_opmode_oneshot)


# función que aplica la cinematica inversa a partir de unas coordenadas planares con el codo de joint2 inverso
def coord2move_inv(pz, px, ph, clientID, joint1, joint2, joint3):
    # Longitud links
    l1 = 90
    l2 = 66
    l3 = 40

    phi = np.deg2rad(ph)

    wx = px - l3 * np.cos(phi)
    wy = pz - l3 * np.sin(phi)

    delta = wx ** 2 + wy ** 2
    c2 = (delta - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    s2 = np.sqrt(1 - c2 ** 2)

    theta_2 = np.arctan2(float(s2), float(c2))

    s1 = ((l1 + l2 * c2) * wy - l2 * s2 * wx) / delta
    c1 = ((l1 + l2 * c2) * wx + l2 * s2 * wy) / delta
    theta_1 = np.arctan2(float(s1), float(c1))

    theta_1 = np.deg2rad(-np.rad2deg(theta_1))
    theta_2 = np.deg2rad(-np.rad2deg(theta_2))
    theta_3 = -phi - theta_1 - theta_2

    # print('theta_1: ', rad2deg(theta_1))
    # print('theta_2: ', rad2deg(theta_2))
    # print('theta_3: ', rad2deg(theta_3))

    _ = sim.simxSetJointTargetPosition(clientID, joint1, theta_1, sim.simx_opmode_oneshot)
    _ = sim.simxSetJointTargetPosition(clientID, joint2, theta_2, sim.simx_opmode_oneshot)
    _ = sim.simxSetJointTargetPosition(clientID, joint3, theta_3, sim.simx_opmode_oneshot)


# función que calcula la coordenada Y dada una X y una trayectoria
def trajectory_Y(x, a, b, c):
    y = (a * (x ** 2)) + (b * x) + c
    return y


# función para calcular la parabola que tendrá que hacer el robot para pasar la pagina
def calc_parabola_vertex(x1, y1, x2, y2, x3, y3):
    denom = (x1 - x2) * (x1 - x3) * (x2 - x3)
    A = (x3 * (y2 - y1) + x2 * (y1 - y3) + x1 * (y3 - y2)) / denom
    B = (x3 * x3 * (y1 - y2) + x2 * x2 * (y3 - y1) + x1 * x1 * (y2 - y3)) / denom
    C = (x2 * x3 * (x2 - x3) * y1 + x3 * x1 * (x3 - x1) * y2 + x1 * x2 * (x1 - x2) * y3) / denom

    return A, B, C


# función que hace invisible la pagina, la mueve hacia la derecha y la vuelve a hacer visible, tambien cambia sus
# propiedades de colisión
def invi_alante(clientID, pagina, pag_joint, pagina1, pagina2, pagina3):
    sim.simxSetObjectIntParameter(clientID, pagina, 3019, 0, sim.simx_opmode_oneshot)
    # sim.simxSetObjectIntParameter(clientID, pagina, 10, 0, sim.simx_opmode_oneshot)
    sim.simxSetObjectIntParameter(clientID, pagina1, 10, 0, sim.simx_opmode_oneshot)
    sim.simxSetObjectIntParameter(clientID, pagina2, 10, 0, sim.simx_opmode_oneshot)
    sim.simxSetObjectIntParameter(clientID, pagina3, 10, 0, sim.simx_opmode_oneshot)

    moverPaginaIzquierda(clientID, pag_joint)

    time.sleep(6)

    # sim.simxSetObjectIntParameter(clientID, pagina, 10, 1, sim.simx_opmode_oneshot)
    sim.simxSetObjectIntParameter(clientID, pagina1, 10, 1, sim.simx_opmode_oneshot)
    sim.simxSetObjectIntParameter(clientID, pagina2, 10, 1, sim.simx_opmode_oneshot)
    sim.simxSetObjectIntParameter(clientID, pagina3, 10, 1, sim.simx_opmode_oneshot)
    sim.simxSetObjectIntParameter(clientID, pagina, 3019, 61455, sim.simx_opmode_oneshot)


# lo mismo que iniv_alantes pero mueve la pagina hacia la izquierda
def invi_atras(clientID, pagina, pag_joint, pagina1, pagina2, pagina3):
        sim.simxSetObjectIntParameter(clientID, pagina, 3019, 0, sim.simx_opmode_oneshot)
        #sim.simxSetObjectIntParameter(clientID, pagina, 10, 0, sim.simx_opmode_oneshot)
        sim.simxSetObjectIntParameter(clientID, pagina1, 10, 0, sim.simx_opmode_oneshot)
        sim.simxSetObjectIntParameter(clientID, pagina2, 10, 0, sim.simx_opmode_oneshot)
        sim.simxSetObjectIntParameter(clientID, pagina3, 10, 0, sim.simx_opmode_oneshot)

        moverPaginaDerecha(clientID, pag_joint)

        time.sleep(6)

        #sim.simxSetObjectIntParameter(clientID, pagina, 10, 1, sim.simx_opmode_oneshot)
        sim.simxSetObjectIntParameter(clientID, pagina1, 10, 1, sim.simx_opmode_oneshot)
        sim.simxSetObjectIntParameter(clientID, pagina2, 10, 1, sim.simx_opmode_oneshot)
        sim.simxSetObjectIntParameter(clientID, pagina3, 10, 1, sim.simx_opmode_oneshot)
        sim.simxSetObjectIntParameter(clientID, pagina, 3019, 61455, sim.simx_opmode_oneshot)


# función que mueve la pagina de derecha a izquierda
def moverPaginaDerecha(clientID, joint):
    _ = sim.simxSetJointTargetVelocity(clientID, joint, 0.1, sim.simx_opmode_oneshot)
    _ = sim.simxSetJointTargetPosition(clientID, joint, np.deg2rad(155), sim.simx_opmode_oneshot)


# función que mueve la pagina de izquierda a derecha
def moverPaginaIzquierda(clientID, joint):
    _ = sim.simxSetJointTargetVelocity(clientID, joint, 0.1, sim.simx_opmode_oneshot)
    _ = sim.simxSetJointTargetPosition(clientID, joint, np.deg2rad(0), sim.simx_opmode_oneshot)


# función principal que llama a otras funcuiones y se encarga de hacer el movimiento entero del brazo robotico moviendo
# la pagina de derecha a izquierda y devolviendolo a su estado en reposo
def pagina_siguiente(x_ini, y_ini, phi_ini, clientID, sensor, pag_joint, joint1, joint2, joint3, pagina, pagina1, pagina2, pagina3):
    a = sim.simxGetObjectPosition(clientID, pagina, -1, sim.simx_opmode_blocking)

    # print(a)

    if a[1][0] < 0:
        invi_alante(clientID, pagina, pag_joint, pagina1, pagina2, pagina3)

    x_act, y_act, phi_act = home_2_derecha(x_ini, y_ini, phi_ini, clientID, joint1, joint2, joint3)

    x_act, y_act, phi_act = derecha_2_izquierda(clientID, sensor, int(x_act), int(y_act), int(phi_act), pag_joint,
                                                joint1, joint2, joint3)

    izquierda_2_home(int(x_act), int(y_act), int(phi_act), x_ini, y_ini, phi_ini, clientID, joint1, joint2, joint3)


# función principal que llama a otras funcuiones y se encarga de hacer el movimiento entero del brazo robotico moviendo
# la pagina de izquierda a derecha y devolviendolo a su estado en reposo
def pagina_anterior(x_ini, y_ini, phi_ini, clientID, sensor, joint1, joint2, joint3, pagina, pag_joint, pagina1, pagina2, pagina3):
    a = sim.simxGetObjectPosition(clientID, pagina, -1, sim.simx_opmode_blocking)

    # print(a)

    if a[1][0] > 0:
        invi_atras(clientID, pagina, pag_joint, pagina1, pagina2, pagina3)

    x_ini = -x_ini
    phi_ini = -phi_ini

    x_act, y_act, phi_act = home_2_izquierda(x_ini, y_ini, phi_ini, clientID, joint1, joint2, joint3)

    x_act, y_act, phi_act = izquierda_2_derecha(clientID, sensor, int(x_act), int(y_act), int(phi_act), joint1, joint2,
                                                joint3, pag_joint)

    derecha_2_home(int(x_act), int(y_act), int(phi_act), x_ini, y_ini, phi_ini, clientID, joint1, joint2, joint3)


# función que utiliza el sensor de ultrasonidos para detectar la distancia entre la ventosa y la pagina y así poderse
# acercar al milimetro para absorberla
def acercarse_pag(clientID, sensor, x, y, phi, joint1, joint2, joint3):
    stop = False
    while not stop:
        errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
            clientID, sensor, sim.simx_opmode_blocking)

        if detectedPoint[2] > 0.12:
            y += 1
        elif detectionState:
            stop = True

        coord2move(x, y, phi, clientID, joint1, joint2, joint3)
        time.sleep(0.01)

    return int(x), int(y), int(phi)


# función igual a acercarse_pag pero utilizando coord2move_inv en vez de coord2move, necesario para pasar paginas hacia
# atras
def acercarse_pag_inv(clientID, sensor, x, y, phi, joint1, joint2, joint3):
    stop = False
    while not stop:
        errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
            clientID, sensor, sim.simx_opmode_blocking)
        # print(errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector)

        if detectedPoint[2] > 0.12:
            y += 1
        elif detectionState:
            stop = True

        coord2move_inv(x, y, phi, clientID, joint1, joint2, joint3)
        time.sleep(0.01)
    # print(phi)
    return int(x), int(y), int(phi)


# función que mueve el brazo desde la derecha a la izquierda (incluyendo el movimiento de la pagina)
# tambien se calcula la trayectoria del brazo
def derecha_2_izquierda(clientID, sensor, X, y, phi, pag_joint, joint1, joint2, joint3):
    X, y, phi = acercarse_pag(clientID, sensor, X, y, phi, joint1, joint2, joint3)

    # print(X)

    i = phi
    dist_v = 200 - X
    dist_sensor = y

    a, b, c = calc_parabola_vertex(-X, dist_sensor, 0, dist_v, X, dist_sensor)

    moverPaginaDerecha(clientID, pag_joint)
    x = 0

    for x in reversed(range(-X, X + 1)):
        y = trajectory_Y(x, a, b, c)
        coord2move(x, y, i, clientID, joint1, joint2, joint3)
        if i >= 110:
            break
        else:
            i += 0.77
        time.sleep(0.02)

    for phi in reversed(range(-13, int(i))):
        coord2move(x, y, phi, clientID, joint1, joint2, joint3)
        x = x - 1
        time.sleep(0.01)

    # setEffector(0)

    time.sleep(1)

    return x, y, phi


# función igual a derecha_2_izquierda pero mueve el brazo y la pagina en el sentido contrario
def izquierda_2_derecha(clientID, sensor, X, y, phi, joint1, joint2, joint3, pag_joint):
    # setEffector(0)

    X, y, phi = acercarse_pag_inv(clientID, sensor, X, y, phi, joint1, joint2, joint3)

    # setEffector(1)

    i = phi
    dist_v = 200 - X
    dist_sensor = y

    a, b, c = calc_parabola_vertex(X, dist_sensor, 0, dist_v, -X, dist_sensor)

    moverPaginaIzquierda(clientID, pag_joint)

    x = 0

    # print(X)

    for x in range(-X, X - 1):
        # print('x: ', x)
        y = trajectory_Y(-x, a, b, c)
        # print('y: ', y)
        # print('phi: ', i)
        coord2move_inv(-x, y, i, clientID, joint1, joint2, joint3)
        if i >= 100:
            break
        else:
            i += 0.77
        time.sleep(0.02)

    for phi in reversed(range(-13, int(i))):
        coord2move_inv(-x, y, phi, clientID, joint1, joint2, joint3)
        x = x + 1
        time.sleep(0.01)

    # setEffector(0)

    #time.sleep(1)
    x = -x
    return x, y, phi


# función que lleva el brazo desde su posición de reposo hasta su posición de lectura de la derecha
def home_2_derecha(x_ini, y_ini, phi_ini, clientID, joint1, joint2, joint3):
    coord2move(x_ini, y_ini, phi_ini, clientID, joint1, joint2, joint3)

    x_dest = 110
    y_dest = 100
    phi_dest = 13

    time.sleep(1)

    for x in reversed(range(x_dest, x_ini)):
        coord2move(x, y_ini, phi_ini, clientID, joint1, joint2, joint3)
        time.sleep(0.01)

    for y in range(y_ini, y_dest):
        coord2move(x, y, phi_ini, clientID, joint1, joint2, joint3)
        time.sleep(0.01)

    for phi in reversed(range(phi_dest, phi_ini)):
        coord2move(x, y, phi, clientID, joint1, joint2, joint3)
        time.sleep(0.01)

    # print(phi)
    return x, y, phi


# función que lleva el brazo desde su posición de reposo hasta su posición de lectura de la izquierda
def home_2_izquierda(x_ini, y_ini, phi_ini, clientID, joint1, joint2, joint3):
    coord2move_inv(x_ini, y_ini, phi_ini, clientID, joint1, joint2, joint3)

    y_aux = 120
    x_aux = -50
    x_dest = 110
    y_dest = 110
    phi_dest = 13

    time.sleep(2)

    for y in range(y_ini, y_aux):
        coord2move_inv(x_ini, y, phi_ini, clientID, joint1, joint2, joint3)
        time.sleep(0.01)

    for x in range(x_ini, x_aux):
        coord2move_inv(x, y, phi_ini, clientID, joint1, joint2, joint3)
        time.sleep(0.01)

    for phi in range(phi_ini, phi_dest):
        coord2move_inv(x, y, phi, clientID, joint1, joint2, joint3)
        time.sleep(0.01)

    for x in range(x_aux, x_dest):
        coord2move_inv(x, y, phi, clientID, joint1, joint2, joint3)
        time.sleep(0.01)

    for y in reversed(range(y_dest, y_aux)):
        coord2move_inv(x, y, phi, clientID, joint1, joint2, joint3)
        time.sleep(0.01)

    time.sleep(5)

    return x, y, phi


# función que lleva el brazo desde su posición de lectura de la izquierda hasta su posición de reposo
def izquierda_2_home(x_ini, y_ini, phi_ini, x_home, y_home, phi_home, clientID, joint1, joint2, joint3):
    x_aux = 0
    y_aux = y_ini + 30

    for y in range(y_ini, y_aux):
        coord2move(x_ini, y, phi_ini, clientID, joint1, joint2, joint3)
        time.sleep(0.01)

    for x in range(x_ini, x_aux):
        coord2move(x, y, phi_ini, clientID, joint1, joint2, joint3)
        time.sleep(0.01)

    for phi in range(phi_ini, phi_home):
        coord2move(x, y, phi, clientID, joint1, joint2, joint3)
        time.sleep(0.01)

    for x in range(x_aux, x_home + 1):
        coord2move(x, y, phi, clientID, joint1, joint2, joint3)
        time.sleep(0.01)

    for y in reversed(range(y_home, y_aux)):
        coord2move(x, y, phi, clientID, joint1, joint2, joint3)
        time.sleep(0.01)


# función que lleva el brazo desde su posición de lectura de la derecha hasta su posición de reposo
def derecha_2_home(x_ini, y_ini, phi_ini, x_home, y_home, phi_home, clientID, joint1, joint2, joint3):
    # print(x_ini, y_ini, phi_ini, x_home, y_home, phi_home)

    for x in reversed(range(x_home, x_ini)):
        coord2move_inv(x, y_ini, phi_ini, clientID, joint1, joint2, joint3)
        time.sleep(0.01)

    for phi in reversed(range(phi_home, phi_ini)):
        coord2move_inv(x, y_ini, phi, clientID, joint1, joint2, joint3)
        time.sleep(0.01)

    for y in reversed(range(y_home, y_ini)):
        coord2move_inv(x, y, phi, clientID, joint1, joint2, joint3)
        time.sleep(0.01)


# función que realiza una maniobra de transición (entre pasar paginas hacia atras y hacia delante)
# para que el giro de los servomotores no sea tan brusco en la simulación
def trasicion_atras_alante(clientID, joint1, joint2):
    j = -88

    for i in reversed(range(45, 114)):
        j += 2
        _ = sim.simxSetJointTargetPosition(clientID, joint2, np.deg2rad(j), sim.simx_opmode_oneshot)

        _ = sim.simxSetJointTargetPosition(clientID, joint1, np.deg2rad(i), sim.simx_opmode_oneshot)
        time.sleep(0.02)


# función que inicializa el robot conectando con el coppelia y identificando las IDs de las partes necesarias
def initRobot():
    clientID = connect(19999)

    retCode, brazo1 = sim.simxGetObjectHandle(clientID, 'Brazo_link0', sim.simx_opmode_blocking)
    retCode, suction = sim.simxGetObjectHandle(clientID, 'suctionPad', sim.simx_opmode_blocking)
    retCode, sensor = sim.simxGetObjectHandle(clientID, 'Proximity_sensor', sim.simx_opmode_blocking)
    retCode, joint1 = sim.simxGetObjectHandle(clientID, 'Brazo_joint0', sim.simx_opmode_blocking)
    retCode, joint2 = sim.simxGetObjectHandle(clientID, 'Brazo_joint1', sim.simx_opmode_blocking)
    retCode, joint3 = sim.simxGetObjectHandle(clientID, 'Brazo_joint2', sim.simx_opmode_blocking)
    retCode, pagina = sim.simxGetObjectHandle(clientID, 'Pagina_movil', sim.simx_opmode_blocking)
    retCode, pagina1 = sim.simxGetObjectHandle(clientID, 'Pagina_movil1', sim.simx_opmode_blocking)
    retCode, pagina2 = sim.simxGetObjectHandle(clientID, 'Pagina_movil2', sim.simx_opmode_blocking)
    retCode, pagina3 = sim.simxGetObjectHandle(clientID, 'Pagina_movil3', sim.simx_opmode_blocking)
    retCode, pag_joint = sim.simxGetObjectHandle(clientID, 'Pagina_joint', sim.simx_opmode_blocking)

    return brazo1, suction, sensor, joint1, joint2, joint3, pagina, pag_joint, clientID, pagina1, pagina2, pagina3


# función de alto nivel que se encarga de pasar pagina
def pasar_pagina(pagina_actual, clientID, sensor, pag_joint, joint1, joint2, joint3, pagina, brazo1, pagina1, pagina2, pagina3):
    print("Pasando una página")

    a = sim.simxGetObjectPosition(clientID, brazo1, -1, sim.simx_opmode_blocking)

    if a[1][2] > 1.8:
        trasicion_atras_alante(clientID, joint1, joint2)

    x_ini = 140
    y_ini = 50
    phi_ini = 45

    pagina_siguiente(x_ini, y_ini, phi_ini, clientID, sensor, pag_joint, joint1, joint2, joint3, pagina, pagina1,
                     pagina2, pagina3)

    res = pagina_actual + 1

    return res


# función de alto nivel que se encarga de retroceder una pagina
def retroceder_pagina(pagina_actual, clientID, sensor, joint1, joint2, joint3, pagina, pag_joint, pagina1, pagina2, pagina3):
    print("Retrocediendo una página")

    x_ini = 140
    y_ini = 50
    phi_ini = 45

    pagina_anterior(x_ini, y_ini, phi_ini, clientID, sensor, joint1, joint2, joint3, pagina, pag_joint, pagina1,
                    pagina2, pagina3)

    res = pagina_actual - 1
    #print("pagina_actual: ", res)

    return res


# función de alto nivel que se encarga de pasar n paginas
def pasar_n_paginas(n, pagina_actual, clientID, sensor, pag_joint, joint1, joint2, joint3, pagina, brazo1, pagina1, pagina2, pagina3):
    for i in range(0, n):
        pagina_actual = pasar_pagina(pagina_actual, clientID, sensor, pag_joint, joint1, joint2, joint3, pagina, brazo1, pagina1, pagina2, pagina3)

    return pagina_actual


# función de alto nivel que se encarga de retroceder n pagina
def retroceder_n_paginas(n, pagina_actual, clientID, sensor, joint1, joint2, joint3, pagina, pag_joint, pagina1, pagina2, pagina3):
    for i in range(0, n):
        pagina_actual = retroceder_pagina(pagina_actual, clientID, sensor, joint1, joint2, joint3, pagina, pag_joint, pagina1, pagina2, pagina3)

    return pagina_actual