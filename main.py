# Librerias necesarias
import speech_recognition as sr
import os

# Librerias propias
from lib import recognize_order, recognize_voice, TTS, OCR, robot_orders

'''
    Muestra por pantalla y reproduce por audio un mensaje de error.
'''
def error():
    texto_leer = "Ha habido un problema"
    print(texto_leer)
    TTS.reproducir_texto_tts(texto_leer)


'''
    Lee una página en voz alta. Para ello llama a la función ocr que convierte una foto a texto. Luego reproduce el texto por los altavoces.
    Inputs: 
        foto_pagina: path de la imagen
        path: path de la carpeta que contiene la imagen

    Output: 
        - audio de la página
'''
def leer_pagina(foto_pagina, path):
    # Lee el texto dentro de la imagen
    texto_pagina = OCR.ocr(foto_pagina, path)

    # Reproduce el texto obtenido por el OCR
    TTS.reproducir_texto_tts(texto_pagina)


'''
    Lee las diferentes páginas que tiene el libro hasta que se le ordena parar. 
    Primero llama la funcion leer_pagina(foto_pagina, path) y lee la página actual. Luego pasa la página llamando a 
    pasar_pagina(pagina_actual, clientID, sensor, pag_joint, joint1, joint2, joint3) y se actualiza la página actual. 
    Ejecuta un bucle hasta que se terminan las páginas.
    
    Inputs: 
        lista_paginas: lista con las diferentes paginas del libro
        pagina_actual: número de la página en la que el robot se encuentra
        clientID, sensor, pag_joint, joint, joint1, joint3: variables del robot que usan funciones de bajo nivel

    Output: 
        - audio del libro
'''
def leer_libro(lista_paginas, pagina_actual, clientID, sensor, pag_joint,  joint1, joint2, joint3):
    print ("Pagina actual: ", pagina_actual)

    for i in range(pagina_actual, len(lista_paginas)):
        # Se declara el path donde estaran las fotos hechas por la camara de la maquina y los nombres de las
        # diferentes fotos, las cuales estarán en orden
        path = 'fotos/el_perfume/'
        pagina = path + "img" + str(i) + ".jpeg"

        print("pagina: ", pagina)
        print("path: ", path)
        leer_pagina(pagina, path)

        # Mueve la pagina en la simulacion
        pagina_actual = robot_orders.pasar_pagina(pagina_actual, clientID, sensor, pag_joint, joint1, joint2, joint3)

    texto_leer = "Y aquí acaba el libro"
    print(texto_leer)
    TTS.reproducir_texto_tts(texto_leer)


def main():
    # Inicializa la simulación del robot
    tip, suction, sensor, joint1, joint2, joint3, pagina, pag_joint, clientID = robot_orders.initRobot()

    # Inicializa el reconocedor, el translator y el microfono
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()
    translator = recognize_order.Translator()

    # Inicia el contador de pagina actual y guarda en lista_paginas todas las fotos obtenidas
    pagina_actual = 0
    lista_paginas = os.listdir('fotos/el_perfume') #['img0.jpeg', 'img1.jpeg', 'img2.jpeg','img3.jpeg','img4.jpeg']

    # Frases que detectará READER para empezar o finalizar el programa
    SALUDO = ("hola", "ey", "hey", "días", "tardes", "reader", "rider")
    TERMINAR = ("apagate", "apagar", "adios", "cierra", "luego", "noches")

    # Si tras 3 intentos no entiende ninguna orden (iniciar/terminar programa) se cierra automáticamente
    intentos = 0
    while True:
        if intentos >= 3:
            break
        intentos += 1

        # Se escucha lo que dice el usuario y se usa una función para tokenizar, lemmatizar y taggear la frase
        result = recognize_voice.recognize_speech_from_mic(recognizer, microphone)
        if result['transcription'] is not None:
            result, _ = recognize_order.escucharOrdenes(result['transcription'], translator, 0)

        # Flag para saber si iniciar o finalizar el programa
        iniciarPrograma = 0

        for word in result:
            if word[0].lower() in SALUDO:
                iniciarPrograma = 1
            elif word[0].lower() in TERMINAR:
                iniciarPrograma = -1

        if iniciarPrograma == 1:
            # READER se presenta
            texto_leer = "Hola, soy READER. ¿En qué puedo ayudarte?"
            print(texto_leer)
            TTS.reproducir_texto_tts(texto_leer)
            intentos = 0

        while iniciarPrograma == 1:
            if intentos >= 3:
                break
            intentos += 1
            # El usuario envía su orden y es reconocido por el voice recognizer (se traduce a inglés)
            result = recognize_voice.recognize_speech_from_mic(recognizer, microphone)
            print(result)
            print("Result[succes] = ", result['success'])

            # Si da error por no poder alcanzar la API o error de captación, no se hace nada
            if result['success'] == False:
                error()
            elif result['success'] == True and result['transcription'] is None:
                error()
            else:
                # Si el reconocimiento es positivo, se envía al algoritmo de decision y este devuelve una orden y
                # un número de paginas que desplazarse
                orden, num_paginas, texto_leer = recognize_order.recuperarOrden(result['transcription'], translator)
                TTS.reproducir_texto_tts(texto_leer)

                # Parar de leer
                if orden == 1:
                    # todo: no sabemos como interrumpir la lectura del bot
                    pass

                # Ir una página atrás
                elif orden == 2:
                    if pagina_actual > 0:
                        pagina_actual = robot_orders.retroceder_pagina(pagina_actual, clientID, sensor, joint1, joint2,
                                                                   joint3)
                    else:
                        texto_leer = "...Hay un problema, estamos en la primera página. No puedo retroceder"
                        print(texto_leer)
                        TTS.reproducir_texto_tts(texto_leer)
                    pass

                # Leer el libro
                elif orden == 3:
                    leer_libro(lista_paginas, pagina_actual, clientID, sensor, pag_joint, joint1, joint2, joint3)

                # Volver a empezar a leer la página
                elif orden == 4:
                    # todo: requiere una funcion que sea capaz de interrumpir la lectura del bot
                    path = 'fotos/el_perfume/'
                    leer_pagina(pagina_actual, path)

                # Ir n página hacia adelante
                elif orden == 5:
                    #todo: el brazo hace n movimientos bien, pero la hoja no acompaña al robot después de la primera hoja
                    # lo podemos arreglar con facilidad
                    if pagina_actual + num_paginas < len(lista_paginas):
                        texto_leer = "Pasando " + str(num_paginas) + " páginas"
                        TTS.reproducir_texto_tts(texto_leer)
                        pagina_actual = robot_orders.pasar_n_paginas(num_paginas, pagina_actual, clientID, sensor, pag_joint, joint1, joint2, joint3)

                # Ir n páginas hacia atrás
                elif orden == 6:
                    #todo: el brazo hace n movimientos bien, pero la hoja no acompaña al robot
                    # lo podemos arreglar con facilidad
                    if pagina_actual > num_paginas:
                        texto_leer = "Retrocediendo " + str(num_paginas) + " páginas"
                        pagina_actual = TTS.reproducir_texto_tts(texto_leer)

                        robot_orders.retroceder_n_paginas(num_paginas, pagina_actual, clientID, sensor, joint1, joint2,
                                                          joint3)

                # Apagar READER
                elif orden == 7:
                    texto_leer = "Buenas noches"
                    TTS.reproducir_texto_tts(texto_leer)
                    break

        # Apagar el programa
        if iniciarPrograma == -1:
            texto_leer = "Buenas noches"
            TTS.reproducir_texto_tts(texto_leer)
            break

        # No ha entendido/escuchado nada
        else:
            texto_leer = "No te he entendido. ¿Puedes repetir?"
            print(texto_leer)
            TTS.reproducir_texto_tts(texto_leer)


if __name__ == '__main__':
    main()