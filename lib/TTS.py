# Librerias necesarias
from gtts import gTTS
from playsound import playsound
import multiprocessing
import speech_recognition as sr
import os

# Librerias propias
from lib import recognize_order, recognize_voice
'''
Librerias a instalar:
    - pip install gtts  -> (gtts-cli --all para comprobar si permite es-es. es-us o es)
    - pip install playsound
'''


'''
    Recibe un string, crea un archivo .mp3 con la pronunciacion del string y seguidamente reproduce el archivo de voz.
    Una vez terminado, para evitar problemas de edicion de archivos, se borra el .mp3
    En el caso de que se haya que leer una pagina, se inicializa un segundo thread para que el algoritmo siempre
    pueda estar escuchando para poder interrumpir al READER.
    
    Input:
        - texto_leer: string con la frase a decir
        - esPagina: booleano que solo se usa en la funcion leer pagina para indicar que se hay que usar un segundo thread
    Output:
        - Frase en voz.
'''
def reproducir_texto_tts(texto_leer, esPagina = False):
    # Se crea el sonido que se haria al pronunciar el string
    tts = gTTS(texto_leer, lang='es-us')

    # Se declara el nombre del archivo y se guarda el sonido tts
    NOMBRE_ARCHIVO = 'frase.mp3'
    tts.save(NOMBRE_ARCHIVO)

    # Si se va a usar el TTS para leer el texto de una pagina, hay funciones extra
    if esPagina:
        # Usamos la clase recognize_order y recognize_voice, por lo que declaramos las variables necesarias
        mic = sr.Microphone()
        recog = sr.Recognizer()
        traductor = recognize_order.Translator()

        # Declaramos una serie de palabras clave a detectar
        PARAR = ("para", "parar", "un momento", "espera", "robot", "rider")

        # Declaramos e inicializamos un thread extra
        p = multiprocessing.Process(target=playsound, args=(NOMBRE_ARCHIVO,))
        p.start()

        # Flag para finalizar el trabajo del thread si se le da la orden
        ordenParar = False

        #print("El exit code: ", p.exitcode)

        # El programa finaliza si el texto termina de dictar o si se le ordena que pare
        while p.exitcode == None and ordenParar == False:
            print("El exit code: ", p.exitcode)
            # Se recoge las palabras que dice el usuario y se pasa a traves de NLP
            frase = recognize_voice.recognize_speech_from_mic(recog, mic)
            if frase['transcription'] is not None:
                resultado, _ = recognize_order.escucharOrdenes(frase['transcription'], traductor, 0)
                #print(resultado)

                # Se chequea si alguna palabra de la frase coincide con la array de parar
                for palabra in resultado:
                    # Si se da el caso, se pone el booleano a True para que finale y se finaliza el thread y el bucle for
                    if palabra[0].lower() in PARAR:
                        ordenParar = True
                        break

            elif p.exitcode == 0:
                break

        # Finaliza el thread y se borra el archivo
        p.terminate()
        os.remove(NOMBRE_ARCHIVO)

        return ordenParar

    # Si el TTS no se usa para leer una pagina, simplemente reproduce el sonido
    else:
        playsound(NOMBRE_ARCHIVO)
        os.remove(NOMBRE_ARCHIVO)
