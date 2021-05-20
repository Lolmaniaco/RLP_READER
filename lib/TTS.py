#pip install gtts
#gtts-cli --all y comprobar si permite es-es, es-us o es
from gtts import gTTS
from playsound import playsound
import os

'''
    Recibe un string, crea un archivo .mp3 con la pronunciación del string y seguidamente reproduce el archivo de voz.
    Una vez terminado, para evitar problemas de edición de archivos, se borra el .mp3
    
    Input:
        - texto_leer: string con la frase a decir
        
    Output:
        - Frase en voz.
'''
def reproducir_texto_tts(texto_leer):
    # Se crea el sonido que se haría al pronunciar el string
    tts = gTTS(texto_leer, lang='es-us')

    # Se declara el nombre del archivo y se guarda el sonido tts
    NOMBRE_ARCHIVO = "frase.mp3"
    tts.save(NOMBRE_ARCHIVO)

    # Reproducimos el sonido
    playsound(NOMBRE_ARCHIVO)

    # Destruimos el archivo
    os.remove(NOMBRE_ARCHIVO)
