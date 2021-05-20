#Fuente: https://realpython.com/python-speech-recognition/
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
        recognizer: objeto tipo reconocedor de la libreria speech_recognition
        microphone: objeto tipo microphone de la libreria speech_recognition
    Output:
        Devuelve 1 diccionario con 3 claves:
        "success": Indica si el algoritmo ha transcrito con éxito
        "error":   'None' si no ha encontrado error, sino continene un string con el mensaje de error. 
                    Los errores pueden ser que no ha podido alcanzar la APi o que no ha reconocido el texto.
        "transcription":   'None' si no ha detectado nada, sino contiene un string con el texto transcrito. 
'''
def recognize_speech_from_mic(recognizer, microphone):
    #Prepara el diccionario para devolver
    response = {
        "success": True,
        "error": None,
        "transcription": None
    }
    
    # Comprueba que el recognizer y el micrófono hayan sido declarados
    if not isinstance(recognizer, sr.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")
    if not isinstance(microphone, sr.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")
    
    with microphone as source:
        print("Habla ahora:")
        #Ajusta la sensibilidad del micrófono al sonido ambiente
        recognizer.adjust_for_ambient_noise(source)
        #Graba el sonido del micrófono
        audio = recognizer.listen(source)
    
    #Try/except para evitar errores al no entender la voz o no poder
    #conectarse con la API.
    try:
        #Transcribe el texto si puede
        response["transcription"] = recognizer.recognize_google(audio, language='es-ES')
    except sr.RequestError:
        #Si no ha podido conectarse a la API, obtenemos un error
        response["success"] = False
        response["error"] = "API unavailable"
    except sr.UnknownValueError:
        #Si no se ha podido reconocer la voz,
        response["error"] = "Unable to recognize speech"

    return response

def main():
    #Inicializa el objeto tipo reconocedor (transcriptor) y micrófono
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()
    
    result = recognize_speech_from_mic(recognizer, microphone)

    if result['success'] == True:
        print(result['transcription'])
    else:
        print(result['error']) 
        
if __name__ == '__main__':
    main()