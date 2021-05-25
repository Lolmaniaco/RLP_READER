# Librerias necesarias
import speech_recognition as sr
'''Librerías necesarias:
    - SpeechRecognition: 
        · pip install SpeechRecognition
    - PyAudio:
        · Descargar versión acorde con la version de Python 
        en https://www.lfd.uci.edu/~gohlke/pythonlibs/#pyaudio
        · Ir a la carpeta donde está contenido
        · pip install PyAudio-0.2.11-cp(version de Python)-cp...
'''

'''
Transcribe texto recogido del micrófono con preferencia al español.

    Input:
        reconocedor: objeto tipo reconocedor de la libreria speech_recognition
        microfono: objeto tipo microphone de la libreria speech_recognition
    Output:
        Devuelve 1 diccionario con 3 claves:
        "success": Indica si el algoritmo ha transcrito con éxito
        "error":   'None' si no ha encontrado error, sino continene un string con el mensaje de error. 
                    Los errores pueden ser que no ha podido alcanzar la APi o que no ha reconocido el texto.
        "transcription":   'None' si no ha detectado nada, sino contiene un string con el texto transcrito. 
'''
def recognize_speech_from_mic(reconocedor, microfono):
    #Prepara el diccionario para devolver
    respuesta = {
        "success": False,
        "error": None,
        "transcription": None
    }

    # Comprueba que el recognizer y el micrófono hayan sido declarados
    if not isinstance(reconocedor, sr.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")
    if not isinstance(microfono, sr.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")

    # Flag que nos sirve para saber si se ha detectado un audio o solo silencio
    audioReconocido = True

    # Se usa el microfono como fuente
    with microfono as fuente:
        print("Habla ahora:")
        # Ajusta la sensibilidad del micrófono al sonido ambiente
        reconocedor.adjust_for_ambient_noise(fuente, duration=0.5)

        # Intenta grabar el sonido del micrófono
        try:
            audio = reconocedor.listen(fuente, timeout=5)
        except OSError:
            audioReconocido = False
            pass
        except sr.WaitTimeoutError:
            audioReconocido = False
            pass

    # Si el audio ha sido extraído...
    if audioReconocido:
        # ...Intenta un try/except para evitar errores al no entender la voz o no poder conectarse con la API.
        try:
            #Transcribe el texto si puede
            respuesta["transcription"] = reconocedor.recognize_google(audio, language='es-ES')
            respuesta["success"] = True
        except sr.RequestError:
            #Si no ha podido conectarse a la API, obtenemos un error
            respuesta["error"] = "API unavailable"
        except sr.UnknownValueError:
            #Si no se ha podido reconocer la voz,
            respuesta["error"] = "Unable to recognize speech"

    return respuesta

def main():
    #Inicializa el objeto tipo reconocedor (transcriptor) y micrófono
    reconocedor = sr.Recognizer()
    microfono = sr.Microphone()
    
    resultado = recognize_speech_from_mic(reconocedor, microfono)

    if resultado['success'] == True:
        print(resultado['transcription'])
    else:
        print(resultado['error'])
        
if __name__ == '__main__':
    main()