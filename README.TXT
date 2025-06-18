                                                                                                                     
     _/_/_/                        _/                                              _/_/_/                            
  _/          _/_/    _/_/_/    _/_/_/_/    _/_/    _/  _/_/  _/_/_/      _/_/      _/    _/_/_/  _/_/      _/_/_/   
 _/        _/    _/  _/    _/    _/      _/    _/  _/_/      _/    _/  _/    _/    _/    _/    _/    _/  _/    _/    
_/        _/    _/  _/    _/    _/      _/    _/  _/        _/    _/  _/    _/    _/    _/    _/    _/  _/    _/     
 _/_/_/    _/_/    _/    _/      _/_/    _/_/    _/        _/    _/    _/_/    _/_/_/  _/    _/    _/    _/_/_/      
                                                                                                            _/       
                                                                                                       _/_/

ContornoImagenes (nombre provisional) es un proyecto de C y ensamblador que consiste en un programa capaz de determinar los contornos, bordes y fondos de determinadas figuras dada una imagen cualquiera.
El proyecto utiliza diversas ecuaciones matemáticas que, junto al ajuste manual de algunas variables específicas, permite la detección del contorno de las figuras de una imagen dada.
La implementación de estas ecuaciones matemáticas se desarrollan principalmente en ensamblador, mientras que el resto de funciones (y principalmente la interfaz) se suministran como base del proyecto en formato C.
El objetivo del proyecto es el de poder entender, desarrollar e incluir código ensamblador de forma eficiente y responsable, ya sea como un programa independiente o como ensamblador en linea dentro de C.
Este proyecto ha sido creado para la asignatura de Administración y Organización de Computadores del Grado de Ingeniería en Ingeniería del Software de la UEx año 2023/2024.  

Creado por José Manuel de Torres Domínguez.  

En este repositorio se encuentra los archivos de código fuente C y los archivos de configuración de Visual Studio.
Este programa esta diseñado para ser compilado en Linux.

Información sobre compilación e importación:
El fichero que contiene la descripción del proyecto (pracaoc.pro) se encuentra disponible en la 
carpeta principal de la aplicación. Dicho fichero puede ser utilizado para importar el proyecto 
desde diferentes entornos de desarrollo. No obstante, se recomienda trabajar con Qt Creator 
(paquete qtcreator). Además del paquete qtcreator, es necesario instalar los paquetes que 
contienen las librerías y herramientas de Qt: qttools5-dev-tools, qtbase5-dev, qtchooser, qt5-
qmake y qtbase5-dev-tools.
