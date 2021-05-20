import re
import random
import nltk
from nltk.stem import WordNetLemmatizer
from nltk.corpus import stopwords
from googletrans import Translator
'''
Librerías a instalar:
    - pip install nltk
    - pip install googletrans==3.1.0a0
    
    Importante instalar googletrans 3.1.0a0 porque versiones más actualizadas
    dan errores al traducir/detectar texto.
'''


'''
Recibe una frase grabada y un objeto de tipo translator y traduce la frase.
Aplica algunos cambios en el string para eliminar mayúsculas, signos de puntuación etc.

    Input:
        fraseGrabada: string
        translator: objeto tipo translator de la libreria googletrans
    Output:
        fraseTraducida: string
'''
def traducirFrase(fraseGrabada, translator):
    # Se eliminan las mayúsculas y se eliminan algunos signos de puntuación
    sentence = fraseGrabada.lower()
    sentence = re.sub('\[.*?¿\]\%', ' ', sentence)

    # Se detecta el idioma y si es español...
    print("Frase: ", sentence)
    idioma = str(translator.detect(sentence).lang)

    # ...se traduce la frase a inglés
    if ('es' in idioma):
        sentence = translator.translate(sentence, src='es')
        fraseTraducida = sentence.text
    elif (idioma == 'en'):
        fraseTraducida = sentence

    #print("frase traducida :", fraseTraducida)
    return fraseTraducida

'''
Recibe una frase grabada y un objeto de tipo translator. Traduce la frase al inglés y preprocesa la frase
Para facilitar su entendimiento. Se tokeniza la frase, se le aplican lemmatizers y se le eliminan las stop words

    Input:
        fraseGrabada: string
        translator: objeto tipo translator de la libreria googletrans
    Output:
        fraseTraducida: string
'''
def escucharOrdenes(fraseGrabada, translator, traducir = 1):
    # Se traduce la frase para preprocesar las frases en inglés (mejor para el stemming y lemmatizing)
    if traducir:
        fraseTraducida = traducirFrase(fraseGrabada, translator)
    else:
        fraseTraducida = fraseGrabada

    # Se declaran las stopwords y lemmatizers
    stop_words = set(stopwords.words('english'))
    lem = WordNetLemmatizer()

    #No hace falta sent_tokenize porque la grabacion pilla 1 sola frase -> No hay que separar frases.
    # Se tokeniza la frase recibida por palabras (parecido a string.split(), pero con más criterios)
    wordList = nltk.word_tokenize(fraseTraducida)
    lemmList = []
    #print("wordList: ", wordList)

    for k in wordList:
        # Se le aplica lemmatizing: reducción a la raíz genérica
        lemmList.append(lem.lemmatize(k))

    #print("lem list :", lemmList)
    # Se eliminan las stopwords: palabras con poco valor semántico
    lemmList = [w for w in lemmList if w not in stop_words]

    #Se le aplica tagging para relacionar cada token con un tipo de sintagma (verbo, sujeto...)
    fraseFinal = nltk.pos_tag(lemmList)

    return fraseFinal, fraseTraducida

'''
Recibe una frase grabada y un objeto tipo translator y devuelve el número de orden y el número de páginas que
hacen falta moverse

    Input:
        fraseGrabada: string
        translator: objeto tipo translator de la libreria googletrans
    Output:
        order: int de valor 0 - 6
        num_pages: número de páginas a desplazar
'''
def recuperarOrden(fraseGrabada, translator):
    # Se traduce y tokeniza la frase para obtener solo las palabras con valor
    fraseFinal, fraseTraducida = escucharOrdenes(fraseGrabada, translator)
    # print(fraseFinal)

    # Palabras que detecta + frases de respuesta para esas palabras clave
    # Dichas palabras no coinciden y sirven para distintas órdenes
    READ_BOOK_OK = ["Voy a leer el libro.", "Vamos a leer!", "Empecemos.", "Prepárate, que empezamos.", "Escucha bien."]

    STOP_READING = ("stop", "moment", "wait", "hold on", "hold up", "hold-up", "break")
    STOP_READING_OK = ["Vale.", "Está bien", "Continúo cuando quieras", "Okey", "Sí", "Okey, mackey"]

    NOT_RECOGNIZED = ["Lo siento, no te he entendido", "No te he entendido bien", "Vuelve a repetir, por fa"
                      "Puedes repetir otra vez?", "Repítemelo otra vez, por favor", ]

    GO_BACK = ("back", "come", "return", "turn", "behind")
    GO_BACK_OK = ["Okey jefe, voy  para atrás", "Entendido", "Volvemos atrás", "Okey", "Sí", "Estupendo"]

    ADVANCE_PAGE = ("advance", "get", "move", "proceed", "forward", "ahead")
    ADVANCE_PAGE_OK = ["Okey, vamos adelante", "Avanzando", "Vamos", "Entendido"]

    GO_BACK_N_PAGES = ("back", "behind", "return", "turn", "behind")
    GO_BACK_N_PAGES_OK = ["Entendido", "Vamos hacia detrás", "Okey, retrocedemos", ]

    READ_AGAIN = ("again", "afresh", "anew", "beginning")
    READ_AGAIN_OK = ["Empiezo a leer de nuevo", "Vale, empiezo desde el principio", "Comienzo otra vez"]

    TURN_DOWN = ("close", "down", "night", "evening", "goodbye", "bye")
    TURN_DOWN_OK = ["Buenas noches", "Nos vemos la próxima vez", "Hasta la vista, baby", "Adiós", "Bye bye"]

    print("")
    print("Frase traducida :", fraseTraducida)
    print("Frase final :", fraseFinal)

    # Se inicializan variables relevantes para detectar cierto tipos de órdenes
    read_order = 0
    num_pages = 0
    contains_digit = any(map(str.isdigit, fraseTraducida))

    order = 0
    '''
    Si algunas de las palabras de la frase contiene una palabra clave devuelve una orden.
    El orden de los if's es importante porque hay palabras más relevantes que otras
    
    Ordenes listadas:
        - 0: No se entendio (default)
        - 1: Parar de leer
        - 2: Ir una página atrás
        - 3: Leer el libro
        - 4: Volver a empezar a leer la página
        - 5: Ir X páginas hacia delante
        - 6: Ir X páginas hacia detrás
        - 7: Apagar
    '''
    fraseRespuesta = 'Lo siento, no lo he entendido. ¿Me lo puedes repetir?'
    if not contains_digit:
        for word in fraseFinal:
            #print("palabra leyendo :", word[0].lower())
            if read_order == 0:
                if word[0].lower() in STOP_READING:
                    fraseRespuesta = random.choice(STOP_READING_OK)
                    print(fraseRespuesta)
                    order = 1
                    break

                elif word[0].lower() in GO_BACK:
                    fraseRespuesta = random.choice(GO_BACK_OK)
                    print(fraseRespuesta)
                    order = 2
                    num_pages = 1
                    break

                elif word[0].lower() in TURN_DOWN:
                    fraseRespuesta = random.choice(TURN_DOWN_OK)
                    print(fraseRespuesta)
                    order = 7
                    break

                # La palabra read está reservada para las órdenes de leer libro y leer de nuevo la página
                # por lo que tiene un if especial.
                elif word[0].lower() == 'read':
                    read_order = 1

                # default
                else:
                    order = 0
            else:
                if word[0].lower() in READ_AGAIN:
                    fraseRespuesta = random.choice(READ_AGAIN_OK)
                    print(fraseRespuesta)
                    order = 4
                    break
                else:
                    fraseRespuesta = random.choice(READ_BOOK_OK)
                    print(fraseRespuesta)
                    order = 3
                    break
    else:
        # Si la frase contiene un digito, se estima automáticamente que es una orden de avanzar
        # o retroceder X número de páginas
        for word in fraseFinal:
            if word[0].lower() in ADVANCE_PAGE:
                fraseRespuesta = random.choice(ADVANCE_PAGE_OK)
                print(fraseRespuesta)
                order = 5

                break
            elif word[0].lower() in GO_BACK_N_PAGES:
                fraseRespuesta = random.choice(GO_BACK_N_PAGES_OK)
                print(fraseRespuesta)
                order = 6
                break

            #default
            else:
                order = 0

    # Si no ha encontrado ninguna coincidencia, devuelve error
    if (order == 0):
        fraseRespuesta = random.choice(NOT_RECOGNIZED)
        print(fraseRespuesta)
    # Si la orden es >= 5 es una orden de tipo "mover X número de páginas"
    elif order >= 5:
        for word in fraseTraducida:
            if str.isdigit(word):
                num_pages = int(word);

    #print("valor de read: ", readOrder)
    return order, num_pages, fraseRespuesta

def main():
    # Se inicializa el translator (fuera del bucle original)
    translator = Translator()

    #Prueba del código
    print("Respuesta: ", recuperarOrden("Vuelve atrás 5 páginas", translator))

if __name__ == '__main__':
    main()