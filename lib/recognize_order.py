# Librerias necesarias
import re
import random
import nltk
from nltk.stem import WordNetLemmatizer
from nltk.corpus import stopwords
from googletrans import Translator
from text2digits import text2digits as t2d

# Librerias propias
from lib import GLOBAL_VAR as gv
'''
Librerias a instalar:
    - pip install nltk
    - pip install googletrans==3.1.0a0
    - pip install text2digits
    
    Importante instalar googletrans 3.1.0a0 porque versiones mas actualizadas
    dan errores al traducir/detectar texto.
'''


'''
Recibe una frase grabada y un objeto de tipo translator y traduce la frase.
Aplica algunos cambios en el string para eliminar mayusculas, signos de puntuacion etc.

    Input:
        fraseGrabada: string
        traductor: objeto tipo translator de la libreria googletrans
    Output:
        fraseTraducida: string
'''
def traducirFrase(fraseGrabada, traductor):
    # Se eliminan las mayusculas y se eliminan algunos signos de puntuacion
    fraseBase = fraseGrabada.lower()
    fraseBase = re.sub('\[.*?¿\]\%', ' ', fraseBase)

    # Se detecta el idioma y si es español...
    print("Frase: ", fraseBase)
    idioma = str(traductor.detect(fraseBase).lang)

    print(idioma)

    # ...se traduce la frase a ingles
    if (idioma == 'en'):
        fraseTraducida = fraseBase
    else:
        fraseBase = traductor.translate(fraseBase, src='es')
        fraseTraducida = fraseBase.text

    #print("frase traducida :", fraseTraducida)
    return fraseTraducida


'''
Recibe una frase grabada y un objeto de tipo translator. Traduce la frase al ingles y preprocesa la frase
Para facilitar su entendimiento. Se tokeniza la frase, se le aplican lemmatizers y se le eliminan las stop words

    Input:
        fraseGrabada: string
        translator: objeto tipo translator de la libreria googletrans
        traducir: flag que indicará si hay que traducir o no el texto
    Output:
        fraseTraducida: string
'''
def escucharOrdenes(fraseGrabada, traductor, traducir = 1):
    # Se traduce la frase para preprocesar las frases en ingles (mejor para el stemming y lemmatizing)
    if traducir:
        fraseTraducida = traducirFrase(fraseGrabada, traductor)
    else:
        fraseTraducida = fraseGrabada

    # Se declaran las stopwords y lemmatizers
    stop_words = set(stopwords.words('english'))
    lemmatizador = WordNetLemmatizer()

    #No hace falta sent_tokenize porque la grabacion pilla 1 sola frase -> No hay que separar frases.
    # Se tokeniza la frase recibida por palabras (parecido a string.split(), pero con mas criterios)
    listaPalabras = nltk.word_tokenize(fraseTraducida)
    listaLemmatizador = []
    #print("wordList: ", wordList)

    for palabra in listaPalabras:
        # Se le aplica lemmatizing: reduccion a la raiz generica
        listaLemmatizador.append(lemmatizador.lemmatize(palabra))

    #print("lem list :", lemmList)
    # Se eliminan las stopwords: palabras con poco valor semantico
    listaLemmatizador = [palabra for palabra in listaLemmatizador if palabra not in stop_words]

    #Se le aplica tagging para relacionar cada token con un tipo de sintagma (verbo, sujeto...)
    fraseFinal = nltk.pos_tag(listaLemmatizador)

    return fraseFinal, fraseTraducida

'''
Recibe una frase grabada y un objeto tipo translator y devuelve el numero de orden y el numero de paginas que
hacen falta moverse

    Input:
        fraseGrabada: string
        traductor: objeto tipo translator de la libreria googletrans
    Output:
        order: int de valor 0 - 6
        num_pages: numero de paginas a desplazar
'''
def recuperarOrden(fraseGrabada, traductor):
    # Se traduce y tokeniza la frase para obtener solo las palabras con valor
    fraseFinal, fraseTraducida = escucharOrdenes(fraseGrabada, traductor)
    # print(fraseFinal)

    # Palabras que detecta + frases de respuesta para esas palabras clave
    # Dichas palabras no coinciden y sirven para identificar distintas ordenes
    LEER_LIBRO = ("read", "study", "research", "studying", "scan")
    LEER_LIBRO_OK = ["Cusha oreha", "Vamos a leer!", "Empecemos.", "Prepárate, que empezamos.", "Escucha bien.",
                     "Ponte el cinturón, que llegan curvas"]

    NO_ENTENDIDO = ["Lo siento, no te he entendido", "No te he entendido bien", "Vuelve a repetir, por fa",
                      "¿Puedes repetirlo otra vez?", "Repítemelo otra vez, por favor", ]

    PAGINA_ATRAS = ("back", "come", "return", "behind")
    PAGINA_ATRAS_OK = ["Okey jefe, voy  para atrás", "Entendido", "Volvemos atrás", "Okey", "Sí", "Estupendo"]

    N_PAGINA_ADELANTE = ("advance", "get", "move", "proceed", "forward", "ahead")
    N_PAGINA_ADELANTE_OK = ["Okey, vamos adelante", "Avanzando", "Vamos", "Entendido"]

    N_PAGINAS_ATRAS = ("back", "behind", "return", "behind")
    N_PAGINAS_ATRAS_OK = ["Entendido", "Vamos hacia detrás", "Okey, retrocedemos", ]

    APAGAR = ("close", "down", "night", "evening", "goodbye", "bye")
    APAGAR_OK = ["Nos vemos la próxima vez", "Hasta la vista, beiby", "Adiós", "bai bai"]

    print("")
    print("Frase traducida :", fraseTraducida)
    print("Frase final :", fraseFinal)

    # Se inicializa el detector de numeros
    detectorNumeros = t2d.Text2Digits()

    # Se usa el detector de numeros para transformar los numeros de letras en digitos (two -> 2)
    fraseNumerada = detectorNumeros.convert(fraseTraducida)

    # Se inicializan variables relevantes para detectar cierto tipos de ordenes
    numPaginas = 0

    #print("Frase Digitalizada :", fraseNumerada)
    # Comprobamos que la frase tenga o no numeros para elegir un set de ordenes u otras
    tieneDigitos = any(map(str.isdigit, fraseNumerada))

    orden = gv.UNKNOWN
    '''
    Si algunas de las palabras de la frase contiene una palabra clave devuelve una orden.
    El orden de los if's es importante porque hay palabras mas relevantes que otras
    
    Ordenes listadas:
        - 0: No se entendio (default)
        - 1*: Parar (acción implementada dentro de otra función)
        - 2*: Ir una pagina atras (acción absorbida por Ir X paginas hacia detras)
        - 3: Leer el libro
        - 4: Ir X paginas hacia delante
        - 5: Ir X paginas hacia detras
        - 6: Apagar
    '''
    fraseRespuesta = 'Lo siento, no lo he entendido. ¿Me lo puedes repetir?'
    if not tieneDigitos:
        for palabra in fraseFinal:
            #print("palabra leyendo :", word[0].lower())
            if palabra[gv.FIRST].lower() in PAGINA_ATRAS:
                fraseRespuesta = random.choice(PAGINA_ATRAS_OK)
                orden = gv.BACK_PAGE
                numPaginas = 1
                break

            elif palabra[gv.FIRST].lower() in APAGAR:
                fraseRespuesta = random.choice(APAGAR_OK)
                orden = gv.TURN_DOWN
                break

            elif palabra[gv.FIRST].lower() in LEER_LIBRO:
                fraseRespuesta = random.choice(LEER_LIBRO_OK)
                orden = gv.READ_BOOK
                break

            else:
                orden = gv.UNKNOWN

    else:
        # Si la frase contiene un digito, se estima automaticamente que es una orden de avanzar
        # o retroceder X numero de paginas
        for palabra in fraseFinal:
            if palabra[gv.FIRST].lower() in N_PAGINA_ADELANTE:
                fraseRespuesta = random.choice(N_PAGINA_ADELANTE_OK)
                orden = gv.ADVANCE_N_PAGES

                break
            elif palabra[gv.FIRST].lower() in N_PAGINAS_ATRAS:
                fraseRespuesta = random.choice(N_PAGINAS_ATRAS_OK)
                orden = gv.BACK_N_PAGES
                break

            #default
            else:
                orden = gv.UNKNOWN

    # Si no ha encontrado ninguna coincidencia, devuelve error
    if (orden == gv.UNKNOWN):
        fraseRespuesta = random.choice(NO_ENTENDIDO)
    # Si la orden es >= 5 es una orden de tipo "mover X numero de paginas"
    elif orden >= gv.ADVANCE_N_PAGES:
        for palabra in fraseNumerada:
            if str.isdigit(palabra):
                numPaginas = int(palabra);

    #print("valor de read: ", readOrder)
    return orden, numPaginas, fraseRespuesta

def main():
    # Se inicializa el translator (fuera del bucle original)
    traductor = Translator()

    #Prueba del codigo
    print("Respuesta: ", recuperarOrden("Vuelve atras 5 paginas", traductor))

if __name__ == '__main__':
    main()