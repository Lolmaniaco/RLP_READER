# Librerias necesarias
import speech_recognition as sr
import random
import os

# Librerias propias
from lib import recognize_order, recognize_voice, TTS, OCR, robot_orders, GLOBAL_VAR as gv

'''
    Muestra por pantalla y reproduce por audio un mensaje aleatorio de error.
'''
def error():
    ERROR = ["No te he entendido", "No he podido escuchar nada",
             "John, no siento las palabras", "Error 404, texto no detectado. Espera, no era ese error",
             "Ya sé por qué no hablas. La primera regla del club de la lucha es que no se habla del club de la lucha, pero yo estoy dentro" ]

    texto_leer = random.choice(ERROR)
    print(texto_leer)
    TTS.reproducir_texto_tts(texto_leer)


'''
    Lee una pagina en voz alta. Para ello llama a la funcion ocr que convierte una foto a texto. Luego reproduce el texto por los altavoces.
    Inputs: 
        foto_pagina: path de la imagen
        path: path de la carpeta que contiene la imagen

    Output: 
        - audio de la pagina
'''
def leer_pagina(rutaPagina, ruta, imagen = None):
    # Lee el texto dentro de la imagen
    texto_pagina = OCR.ocr(rutaPagina, ruta, imagen)

    # Reproduce el texto obtenido por el OCR
    pararLeer = TTS.reproducir_texto_tts(texto_pagina, True)

    return pararLeer

'''
    Lee las diferentes paginas que tiene el libro hasta que se le ordena parar. 
    Primero llama la funcion leer_pagina(foto_pagina, path) y lee la pagina actual. Luego pasa la pagina llamando a 
    pasar_pagina(pagina_actual, clientID, sensor, pag_joint, joint1, joint2, joint3) y se actualiza la pagina actual. 
    Ejecuta un bucle hasta que se terminan las paginas.
    
    Inputs: 
        lista_paginas: lista con las diferentes paginas del libro
        pagina_actual: numero de la pagina en la que el robot se encuentra
        clientID, sensor, pag_joint, joint, joint1, joint3: variables del robot que usan funciones de bajo nivel

    Output: 
        - audio del libro
'''
def leer_libro(lista_paginas, pagina_actual, clientID, sensor, pag_joint,  joint1, joint2, joint3, pagina, brazo1, pagina1, pagina2, pagina3):
    print ("Pagina actual: ", pagina_actual)

    # Se declara el path donde estaran las hipoteticas fotos hechas por la camara de la maquina
    path = 'fotos/el_perfume/'

    # Esto seria el codigo si usasemos el sensor del Coppelia. De cara a la simulacion, hemos usado directamente
    # imagenes de un libro
    '''
    imagenIzq, imagenDer = robot_orders.hacerCaptura(clientID, sensor)
    listaImagenes = [imagenIzq, imagenDer]
    
    # Se le aplican preprocesamiento en VC
    
    for fotoPagina in listaImagenes:
        rutaPagina = path +'imagenLeer.jpg'
        misc.imsave(rutaPagina, fotoPagina)
        leer_pagina(rutaPagina, path)
    '''

    # Se inicializa el contador para saber si tenemos que pasar pagina (solo pasamos cada 2 lecturas)
    # pararLeer es una variable que sirve para indicar si se ha interrumpido al robot.
    pararLeer = False
    contador = 0
    for i in range(pagina_actual, len(lista_paginas)):
        contador += 1
        # Los diferentes paths absolutos de la foto a analizar
        rutaPagina = path + "img" + str(i) + ".jpeg"

        #print("pagina: ", rutaPagina)
        #print("path: ", path)

        # Comienza a leer la pagina. Si el comando es interrumpido, sale automáticamente del for y pide otra accion
        pararLeer = leer_pagina(rutaPagina, path)
        if pararLeer:
            break

        # Mueve la pagina en la simulacion cada 2 imagenes leídas
        if contador % 2 == 0:
            pagina_actual = robot_orders.pasar_pagina(pagina_actual, clientID, sensor, pag_joint, joint1, joint2,
                                                      joint3, pagina, brazo1, pagina1, pagina2, pagina3)

        # Una vez la pagina es leida, se destruye el archivo.
        # En la simulacion no lo hacemos porque no queremos borrar las imagenes que usamos de prueba
        #os.remove(rutaPagina)

    # Si no se ha parado de leer, el libro ha finalizado el libro/la lectura entera
    if not pararLeer:
        texto_leer = "Y aqui acaba el libro"
        print(texto_leer)
        TTS.reproducir_texto_tts(texto_leer)

    # Si el robot se ha interrumpido, cambia la frase dictada y pide nuevas ordenes
    else:
        texto_leer = "Okay, paro de leer. ¿Qué hago ahora?"
        print(texto_leer)
        TTS.reproducir_texto_tts(texto_leer)

    return pagina_actual

def main():
    # Inicializa la simulacion del robot
    brazo1, suction, sensor, joint1, joint2, joint3, pagina, pag_joint, clientID, pagina1, pagina2, pagina3 = robot_orders.initRobot()

    # Inicializa el reconocedor, el translator y el microfono
    reconocedor = sr.Recognizer()
    microfono = sr.Microphone()
    traductor = recognize_order.Translator()

    # Inicia el contador de pagina actual y guarda en lista_paginas todas las fotos obtenidas
    pagina_actual = 0
    lista_paginas = os.listdir('fotos/el_perfume') #['img0.jpeg', 'img1.jpeg', 'img2.jpeg','img3.jpeg','img4.jpeg']

    # Frases que detectara READER para empezar o finalizar el programa
    SALUDO = ("hola", "ey", "hey", "dias", "okey", "ok", "okay", "tardes", "reader", "rider")
    TERMINAR = ("apagate", "apagar", "adios", "cierra", "luego", "noches")
    OTRAS = ("google", "guguel", "siri", "alexa")

    # Si tras 3 intentos no entiende ninguna orden (iniciar/terminar programa) se cierra automaticamente
    intentos = 0

    # READER se presenta
    texto_leer = "Hola, soy READER."
    print(texto_leer)
    TTS.reproducir_texto_tts(texto_leer)

    # Bucle principal del código. Aquí el programa puede acceder a sus funciones basicas
    while True:
        # Si READER detecta mas de 3 intentos fallidos, se apaga
        if intentos >= 3:
            break
        intentos += 1

        # Se escucha lo que dice el usuario y se usa una funcion para tokenizar, lemmatizar y taggear la frase
        resultado = recognize_voice.recognize_speech_from_mic(reconocedor, microfono)
        if resultado['transcription'] is not None:
            resultado, _ = recognize_order.escucharOrdenes(resultado['transcription'], traductor, 0)

        print(resultado)
        # Flag para saber si iniciar o finalizar el programa
        iniciarPrograma = gv.NOT_RECOGNIZED

        # Si se dice una palabra en la lista de SALUDO/TERMINAR/OTRAS se inicia una orden
        for palabra in reversed(resultado):
            aux = palabra[gv.FIRST].lower()
            if aux in SALUDO:
                iniciarPrograma = gv.START_PROGRAM
                break
            elif aux in TERMINAR:
                iniciarPrograma = gv.GO_SLEEP
                break
            elif aux in OTRAS:
                iniciarPrograma = gv.HORNS
                break

        if iniciarPrograma == 1:
            # READER ofrece sus servicios
            texto_leer = "¿En qué te puedo ayudar?"
            print(texto_leer)
            TTS.reproducir_texto_tts(texto_leer)
            intentos = 0

            # El programa se inicia porque se ha detectado un saludo
            while iniciarPrograma == 1:
                if intentos >= 3:
                    break

                # El usuario envia su orden y es reconocido por el voice reconocedor (se traduce a ingles)
                resultado = recognize_voice.recognize_speech_from_mic(reconocedor, microfono)
                print(resultado)
                print("Result[succes] = ", resultado['success'])

                # Si da error por no poder alcanzar la API o por error de captacion, no se hace nada
                if resultado['success'] == False:
                    intentos += 1
                    error()
                elif resultado['success'] == True and resultado['transcription'] is None:
                    intentos += 1
                    error()
                else:
                    # Si el reconocimiento es positivo, se envia al algoritmo de decision y este devuelve una orden y
                    # un numero de paginas que desplazarse
                    orden, num_paginas, texto_leer = recognize_order.recuperarOrden(resultado['transcription'], traductor)

                    # Si el robot ha entendido la orden, reinicia los intentos y responde
                    if orden != gv.UNKNOWN:
                        intentos = 0
                        print(texto_leer)
                        TTS.reproducir_texto_tts(texto_leer)

                    # Si no ha entendido la orden, responde, pero no reinicia los intentos
                    elif intentos < 2:
                        print(texto_leer)
                        TTS.reproducir_texto_tts(texto_leer)

                    # El robot no ha entendido la orden
                    if orden == gv.UNKNOWN:
                        intentos += 1

                    # La función para interrumpir al READER mientras habla se ha implementado dentro de la funcion leer_libro
                    # por lo que no hace falta hacer un caso especial en el main
                        '''
                        elif orden == gv.STOP:
                            intentos = 0
                        '''

                    # La funcion ir una pagina atras ha sido absorbida por la función BACK_N_PAGES, la cual ahora puede
                    # retroceder desde 1 a las que tenga el libro
                        '''
                        # Ir una pagina atras
                        elif orden == gv.BACK_PAGE:
                            if pagina_actual > 0:
                                pagina_actual = robot_orders.retroceder_pagina(pagina_actual, clientID, sensor, joint1, joint2,
                                                                               joint3, pagina, pag_joint)
                            else:
                                texto_leer = "...Hay un problema, estamos en la primera pagina. No puedo retroceder"
                                print(texto_leer)
                                TTS.reproducir_texto_tts(texto_leer)
                            pass
                        '''
                    # Leer el libro
                    elif orden == gv.READ_BOOK:
                        pagina_actual = leer_libro(lista_paginas, pagina_actual, clientID, sensor, pag_joint, joint1, joint2,
                                                   joint3, pagina, brazo1, pagina1, pagina2, pagina3)

                    # Ir n paginas hacia adelante
                    elif orden == gv.ADVANCE_N_PAGES:
                        if pagina_actual + num_paginas < len(lista_paginas):
                            texto_leer = "Pasando " + str(num_paginas) + " páginas"
                            print("Pasando ", str(num_paginas), " paginas")
                            TTS.reproducir_texto_tts(texto_leer)
                            pagina_actual = robot_orders.pasar_n_paginas(num_paginas, pagina_actual, clientID, sensor,
                                                                         pag_joint, joint1, joint2, joint3, pagina, brazo1,
                                                                         pagina1, pagina2, pagina3)
                            texto_leer = "Ya he terminado. ¿Qué hago ahora?"
                            print(texto_leer)
                            TTS.reproducir_texto_tts(texto_leer)

                    # Ir n paginas hacia atras
                    elif orden == gv.BACK_N_PAGES:
                        if pagina_actual >= num_paginas:
                            texto_leer = "Retrocediendo " + str(num_paginas) + " páginas"
                            print("Retrocediendo ", str(num_paginas), " paginas")
                            TTS.reproducir_texto_tts(texto_leer)
                            robot_orders.retroceder_n_paginas(num_paginas, pagina_actual, clientID, sensor, joint1, joint2,
                                                              joint3, pagina, pag_joint, pagina1, pagina2, pagina3)

                            texto_leer = "Ya he terminado. ¿Qué hago ahora?"
                            print(texto_leer)
                            TTS.reproducir_texto_tts(texto_leer)
                        else:
                            texto_leer = "...Hay un problema. No puedo retroceder tantas páginas atrás."
                            print(texto_leer)
                            TTS.reproducir_texto_tts(texto_leer)

                    # Apagar READER
                    else:
                        iniciarPrograma = gv.GO_SLEEP
                        break

        # Apagar el programa
        if iniciarPrograma == gv.GO_SLEEP:
            break

        # No ha entendido/escuchado nada
        elif iniciarPrograma == gv.NOT_RECOGNIZED:
            error()

        # Alternativa joke si se habla de un Asistente de Voz
        elif iniciarPrograma == gv.HORNS:
            texto_leer = "¿Quién es esa otra IA que has llamado? Creía que teníamos algo especial."
            intentos = 3
            print(texto_leer)
            TTS.reproducir_texto_tts(texto_leer)

    # Si se ha intentado mas de 3 veces, el robot se apaga automaticamente
    if intentos >= 3:
        texto_leer = "Bueno, creo que no nos estamos entendiendo. Mejor me voy a dormir."
        print(texto_leer)
        TTS.reproducir_texto_tts(texto_leer)

    # READER se despide
    texto_leer = "Buenas noches"
    print(texto_leer)
    TTS.reproducir_texto_tts(texto_leer)

if __name__ == '__main__':
    main()