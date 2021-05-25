
vList item
README.md

  

# READER
**![](https://lh5.googleusercontent.com/R9qK2_UnaY12V3FETjoRkqA0WhEKd8-2b1PxlzBfNvYMGpBjblPVx_ArtbOyyTzuq39kn-O3Ht-SmNKqYYIhsCuUEadGEblYoEWZ-vil8ieFEbYWXv7dbC1HJHpRHdKr)**

## Tabla de contenidos
- [Introducción](#introducción)
- [Descripción](#descripción)
- [Ordenes del Robot](#ordenes-del-robot)
- [Amazing Contributions](#amazing-contributions)
- [Pre-Requisitos](#pre-requisitos)
- [Componentes Hardware](#componentes-hardware)
- [Módulos Software ](#módulos-software)
  * [Visión](#visión)
  * [Habla y escucha](#habla-y-escucha)
  * [Movimiento](#movimiento)
- [Vídeo](#vídeo)
- [Autores](#autores)
- [Contribución](#contribución)

  

## Introducción

  

Esto es un proyecto para la asignatura de *Robòtica, Llenguatge i Planificació* y tiene como motivación resolver algún tipo de problema que pueda ser beneficioso para la sociedad. El proyecto cuenta con una parte de robótica, pero también tiene una parte de Visión por Computador.

  

Somos estudiantes de tercer y cuarto año del Grado de Informática en la *Universitat Autònoma de Barcelona*. Este proyecto se ha realizado durante el curso 20/21.

  

El proyecto se ha hecho totalmente online a través de Discord y Slack, debido a la pandemia del *SARS-CoV-2*.

  
  

## Descripción

  

READER es un robot cuyo objetivo es ayudar a la gente con dificultades para leer. El proyecto consiste, principalmente, en un brazo de 3 ejes planar capaz de pasar las páginas de un libro para que una cámara en posición cenital les haga fotos. Posteriormente, el contenido de estas fotos en formato *.jpeg* se pasará a texto y se reproducirá en voz alta.

Todas estas acciones serán realizadas a través de lenguaje natural, por lo que no hay ninguna necesidad de leer respuesta o pulsar botones.

  
  

## Ordenes del robot

  Las órdenes principales que puede realizar READER son las siguientes:

- Leer el libro indefinidamente.

- Parar de leer el libro.

- Apagar el robot.

- Avanzar n páginas.

- Retroceder n páginas.

Además, le hemos añadido algunos condicionales para que sea más natural y lógico su funcionamiento. Por ejemplo, el propio robot finaliza la ejecución si no escucha nada tras varios intentos, responde con diferentes frases para las mismas órdenes o se comunica con lenguaje relativamente natural.
  

![](https://lh4.googleusercontent.com/DOuOVjDbAcE6mZCYwORyI8CrHT-RlnihwCm6CBf5VGKZTeh_zyEdc7RGtCdBDz9PeMkodB58_yKmqszOytyzdzKa-yXsvdNSCu4llAaRU5oESoIKjg0LIP-Rsnx4_o_QzcihMdkj)

*READER retrocediendo una página*
  
  

## Amazing Contributions

  

-   El robot permite leer en alto libros físicos, ya sea porque no tengan una versión eBook o porque el dueño no quiera comprar una.
    
-   READER permite aprender el contenido de un libro a personas invidentes, con dificultades motrices o analfabetas, ya que todo se realiza mediante lenguaje natural.
    
-   El robot ofrece diferentes respuestas (algunas de ellas son bromas) para que sea más amigable y menos monótono.
    

  
  

## Pre-requisitos

*Python 3.8*

 

Para poder ejecutar el programa se enlistan a continuación las diferentes librerías que hay que instalar. Se dividen en librerías problemáticas, librerías para descargar y librerías del sistema. Las problemáticas tienen que instalarse con precaución para evitar problemas con las versiones.
  

**Librerías más problemáticas de descargar**

- *googletrans:* pip install googletrans==3.1.0a0

- *tesseract:* descargar a través de [https://github.com/UB-Mannheim/tesseract/wiki](https://github.com/UB-Mannheim/tesseract/wiki) y especificar la descarga del idioma español.

- *PyAudio:* descargar en [https://www.lfd.uci.edu/~gohlke/pythonlibs/#pyaudio] según la versión de Python, ir a la carpeta contenedora y lanzar ‘pip install PyAudio-...’

  
**Librerías para descargar**

- *pytesseract:* pip install pytesseract

- *Gtts:* pip install gtts

- *playsound:* pip install playsound

- *speech_recognition:* pip install SpeechRecognition

- *OpenCV:* pip install OpenCV-Python

- *nltk:* pip install nltk

- *text2digits:* pip install text2digits

- *numpy:* pip install numpy

- *scipy.optimize:* pip install scipy

  

**Librerías del sistema:**

- *time*

- *os*

- *sys*

- *datetime*

- *pil*

- *random*

- *regex*

  

## Componentes hardware

![](https://lh5.googleusercontent.com/CSzw9Pv3SuqjDE06F1WF5sPoYJY7ISh0jKmC8mypNc8Q8NcdOAq08YUJGlwhy_493-1p_3bD7PV-Khf684J5Xc4w3052-JdHQmbdSSWqkyyPc5sNM4j2BzRwOpUlMQ8QOyk55-vV)

  

1. Placa *Raspberry Pi 3B (R3B)*, núcleo del hardware encargada de procesar información y conectar todos los componentes.

2. Servomotores, conectados a la *R3B* son los ejes de movimiento del brazo robótico que pasa las páginas del libro.

3. Mini bomba de succión de aire, en la punta del brazo robótico hay una ventosa que succiona la página para poder pasarla sin dañarla.

4. Micrófono, encargado de capturar los comandos de voz para que el brazo se mueva acordemente.

5. Altavoz, componente por el cual se reproducirá la lectura del libro.

6. Cámara, componente que nos permite calcular las dimensiones del libro y leer todo su contenido.

7. LED de alta intensidad, encargado de iluminar las páginas para que no haya ningún error de lectura por falta de luz.

8. Sensor de ultrasonido, lo utilizamos para calcular la distancia entre la ventosa de succión y la página del libro, lo que nos permite acercarnos al milímetro.

    

## Módulos software

Aquí tenemos un esquema general del software del READER:
![](https://lh3.googleusercontent.com/cC0uazoPA55KijEFdGsdad19Jt2NvPKPpxv1SEFCsDhsHUIhuh3PeAEcH_sM9GLY49WvE4frrw5OeGMJodBV5RYsBK-fRDH3G08JBIgPXNM70iO-78Txft0JPsYsFYIaaymJBwIC)
A continuación hablaremos más en detalle de cada uno de ellos.
  

### Visión

![](https://lh6.googleusercontent.com/qYkZ0acNz0B82b6BtBwfqvy9V39PEtQbJeGz9feLmTz70mWhwnfoUWOQF-tjYUW2hZw4JS1-fILKLJ0vmhpUI6HpBHYFlCMSz9UaXon04DQLlF3rqRssA6Df9mK6L2-zCA9YZMXz)

Se encarga de tratar las fotografías de las páginas y obtener el texto que hay en ellas. Está formado por el *OCR* y el *page_dewarp*, el cual forma parte de la primera:

- *Page dewarp:* preprocesa la imagen para que el *OCR* la pueda leer mejor. Se encarga de aplanar la imagen y la binariza, lo que la hace bastante más legible.

- *OCR:* Utiliza PyTesseract y transcribe el texto de la imagen en un string que se envía finalmente al controlador.


En un prototipo físico real, la imagen que pasa a través del módulo sería captada por una cámara/sensor y seguidamente pasaría por una función que dividiría la imagen en dos para poder leer cada página individualmente.
     

### Habla y escucha

  

![](https://lh3.googleusercontent.com/kDVASMuM5ymUKOleSOW_PQ7BfI1QeTBdF3pZpRckdwkQ4LoKKeCC_a1vEB2fqu9KGEA-dRybR-O4FgXZ4mIAhpYkx-sG5BTvDp5oYZ8U6DP7TZvpreiGXlIErY8ahX1ERpmmC4_f)



Principalmente consta de dos partes, escuchar y hablar:

- Escuchar se encarga de, mediante el micrófono, escuchar las órdenes que se dan usando *Speech Recognition*. Después, el string detectado es pasado a través de *Natural Language Processing (NLP)* para detectar la intencionalidad de la frase. Según lo que haya dicho el usuario, el programa devolverá una orden u otra y una respuesta aleatoria.

- Hablar es capaz de reproducir texto a través de los altavoces mediante la librería *GTTS*. Cabe destacar que cuando se está leyendo una página el robot reproduce las frases en segundo plano para poder interrumpirlo en cualquier momento.

  

### Movimiento

![](https://lh5.googleusercontent.com/uD3h0blStxAzPGAdoUrYjmG5UEtqS5BCgBu4pe8mNb_ZQkqypjppQGwaPgR5T7DiYgYXsPZ4rN9z0dNmseVaYDEPngoGFY3gIsBTdXDRqr2Dvf6f95tduBzuS2RwQGWcF8VO-Mtk)

  

En el módulo de movimiento encontramos las dos grandes operaciones de alto nivel, avanzar y retroceder página, a su vez tienen una versión repetible ‘n’ veces.

  

![](https://lh6.googleusercontent.com/buU3fD1pgVSZ1B1DGFtIkFFEYjoIff3qEShlsCFeNBhmuZDT9n1kUW7tQwz9fSBxnx7wvpVJZWS9fS8SY8oK_9LZoOXIpUBerDP9iXswka3Pblvcf8hQrFVB0reiPJOPkt6reYb5)

Aquí podemos ver en detalle el ciclo de procesos que realiza la instrucción “Avanzar página”. Si la última acción del brazo ha sido retroceder una página, es necesario empezar con la maniobra de transición, donde le aplicamos cinemática directa para suavizar el cambio de posición entre un modo u otro.

Normalmente, el brazo se mueve de su posición de reposo a su posición de lectura en la parte derecha del libro, con el sensor de ultrasonidos se va acercando a la página hasta acercarse a la altura deseada, procede a activar la ventosa de succión y a calcular la trayectoria que tendrá que tomar para pasar la página sin dañar el libro. Seguidamente empieza a seguir la trayectoria anteriormente calculada actualizando el ángulo de la ventosa para no desengancharse de la página, hasta que llega a ⅗ partes del camino aproximadamente y desactivamos la succión para dejar caer la página en el otro lado del libro. El brazo vuelve de la parte izquierda a su pose de reposo esperando su próxima acción.

![](https://lh4.googleusercontent.com/eiyqhZ3XKKg9Os3K9H7IaIxL8k6clhmdzqx7A4vUhjde9eo7_RHovA6Z4utlBp9rPJFllDuMe5W3-RBJtObQgr5GtbTIj0at8-r1xrpQuZPPDwj7XAjR5UUe2N-_ZME2urQY0b5Q)
*READER avanzando páginas*

## Vídeo
  

Video demostrativo donde se ve al robot interaccionando y realizando diversas acciones.


[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/miCgWJtmiB0/0.jpg)](https://www.youtube.com/watch?v=miCgWJtmiB0)

[](https://www.youtube.com/watch?v=miCgWJtmiB0)

  
  

## Autores

  

[Gerard Trullo Salas](https://github.com/gtstrullo99) — 1494246

[Raül Salinas Natal](https://github.com/es2021uab-1497706) — 1497706

[Omar Bajja Sánchez](https://github.com/obajja) — 1498039

[Xiang Lin](https://github.com/Lolmaniaco) — 1465728


  

## Contribución

  

Gracias a Fernando Vilariño y Carlos García por ayudarnos en el proyecto y por tener fe en nosotros.

  

También queremos agradecer a Victor Batista por ofrecernos ayuda cuando teníamos dudas sobre ciertas partes de la robótica del proyecto.


