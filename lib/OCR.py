import pytesseract
from lib import OCR_page_dewarp
import cv2

'''
    Se requiere de las librerías:
        -PyTesseract: pip install pytesseract
        -Tesseract: programa que se instala en el PC.
            · Se puede descargar a través de 
            https://github.com/UB-Mannheim/tesseract/wiki
            · Durante la instalación se ha de especificar que se quiere 
            descargar el idioma Español.
'''

'''
Recibe un string que representa el path de la imagen.
En el programa real no se recibiría un path, sino una imagen real captada por la cámara, pero debido a limitaciones
con Coppelia Sim usaremos un placeholder.

    Input:
        file: string (path de la imagen de ejemplo)
    Output:
        texto: string con todo el texto leído por pytesseract al aplicar la función image_to_string
'''
def ocr(file, path):
    # PyTesseract lee la imagen y detecta el texto.
    # Se le establece el idioma (lang) a español (spa)
    pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract'

    image = OCR_page_dewarp.main(file, path)
    print("Image: ", image)
    image = cv2.imread(image)
    res = pytesseract.image_to_string(image, lang='spa')

    print(res)
    return res
    ##return pytesseract.image_to_string(file) ##lee en inglés y español


def main():
    # Se le indica al programa dónde está instalado tesseract para que
    # use sus herramientas
    pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract'
    
    # Se establece el idioma del texto y el path del archivo a leer
    # En el programa de la máquina real la foto se pasaría por parámetros
    filename = 'fotos/test3.jpeg'
    
    #start = time()
      
    # Se preprocesa la imagen para hacerla binaria y más legible
    # al reducir ondulaciones, malas orientaciones etc.
    #image = page_dewarp.main(filename)

    # Se lanza la función OCR
    #texto = ocr(image)

    #print("Time elapsed: ", time() - start, "s")

if __name__ == '__main__':
    main()
