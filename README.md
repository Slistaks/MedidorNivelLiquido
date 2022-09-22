# Medicion de nivel de agua

## Calibracion de cada sensor:
### Caracterizacion DAC-Lazo de corriente:
-Encontrar el valor del DAC para el cual la salida del lazo de corriente es 4mA. Actualizar este valor en el macro "DAC_MIN". (De modo orientativo, este valor esta cercano a 43).  
 Para esto, modificar el valor del DAC dentro del bloque "calibracion(1) y debug" y modificando el valor del DAC hasta que se mida una corriente de salida de 4mA.  
-Encontrar el valor del DAC para el cual la salida del lazo de corriente es 20mA. Actualizar este valor en el macro "DAC_MAX". (De modo orientativo, este valor esta cercano a 241).  
 Para esto, modificar el valor del DAC dentro del bloque "calibracion(1) y debug" y modificando el valor del DAC hasta que se mida una corriente de salida de 20mA.  
-Encontrar el valor del DAC para el cual la salida del lazo de corriente es 12mA (valor medio entre 4mA y 20mA). Actualizar este valor en el macro "DAC_12mA". (De modo orientativo, este valor esta cercano a 141).  
 Para esto, modificar el valor del DAC dentro del bloque "calibracion(1) y debug" y modificando el valor del DAC hasta que se mida una corriente de salida de 12mA.  
-!! Una vez terminado lo anterior, volver a comentar el bloque "calibracion(1) y debug".
  
### Caracterizacion sensor:  
-Sumergir el sensor en agua hasta que deje de medir saturacion inferior y anotar dicha altura.  
-Continuar sumergiendo el sensor hasta llegar a la saturacion superior, y anotar la altura a la que sucede.  
-Haciendo la resta de los valores obtenidos anteriormente, actualizar el valor del macro RANGO_MM.  

### Transferencia sensor:  
-Descomentar el bloque "calibracion(2) y debug" para poder observar en el monitor serie las impresiones de capacidad absoluta.  
-Tomar varias medidas de capacidad para varias alturas, y con el vector de datos resultante encontrar la curva de orden 2 o 3 que mejor se ajuste, siendo la variable independiente la capacidad en pF y la imagen la profundidad de inmersion en mm.  
-Introducir los coeficientes en el bloque "COEFICIENTES SN1". Si se usan los de segundo orden, cambiar la macro "ORDEN_APROXIMACION".  
 Coeficientes:  
	H[mm]= Ax^3 + Bx^2 + Cx + D  	si es de orden 3, x[pF]  
	H[mm]= Ax^2 + Bx + C 		si es de orden 2, x[pF]  


### Script de octave para encontrar coeficientes:
  
  
  
  
clear all;  
clc  
hold off;  
  
% Ingresar los vectores de capacidad[pF] y altura[mm] (reemplazar los ingresados como ejemplo):  
c= [28.365,31.32,33,35.56,37.15,38.02,39.495,40.1,40.535,41,41.375];  
h= [35,285,385,485,585,685,785,885,985,1085,1185];  
  
orden= 3;  
p= polyfit(c, h, orden);  
h_aproximacion= p(1)*c.^3 + p(2)*c.^2 + p(3)*c + p(4);  
error= h_aproximacion - h;  
  
plot(c, h, 'g;Medido;')  
hold on  
plot(c, h_aproximacion, '--r')  
disp("Coeficiente A:")  
disp(num2str(p(1)))  
disp("Coeficiente B:")  
disp(num2str(p(2)))  
disp("Coeficiente C:")  
disp(num2str(p(3)))  
disp("Coeficiente D:")  
disp(num2str(p(4)))  
    
  
  
  
## Overview




### Hardware Required

To run this proyect it's needed:
  -("[borrar]poner nombre del sensor como FDX-Sensor / FDX-waterLevelSensor o etc").
  -[borrar]seguir poniendo lo que necesita, como el monitor, el sistema de control, el panel/tablero, y todo eso
  -[borrar]etc

#### Pin Assignment(esp32):

**Note:**

|                           | SDA    | SCL    |
| ------------------------- | ------ | ------ |
| ESP32/ESP32-S2 I2C Master | GPIO21 | GPIO 22|		// [borrar]corroborar los pines
| ------------------------- | ------ | ------ |

| ESP32 Sync input          |     GPIO5       |
| ESP32 packets sync pulse  |     GPIO4       |		// [borrar]documentar que es un pin para debug, para corroborar sincronismo.  
// [borrar]al inicio de cada paquete invierte el estado del gpio. (corroborar si no recuerdo mal).  
- Connection:	[borrar] no creo que haga falta, tiene un solo conector el sensor..

### Configure the project


### Build and Flash


### Example Output


### Troubleshooting

### Notas:
El numero de sensor es para tener un solo programa, y al programar el sensor en particular, se descomenta la seccion donde estan sus constantes, sin tener que anotarlas y reescribirlas cada vez que se cambia de sensor. Al no transmitir, todos pueden llamarse "sensor 1", no hay conflicto.  
Se podria hacer que las constantes, macros, etc se introduzcan en un menu de sdkconfig en lugar de tener que abrir el codigo y buscar e introducir a mano.  
Para pausar la consola de VSC: [ctrl+t,ctrl+y]


### Aca poner referencia a los scripts de octave, por ej; "script1.m grafica altura versus capacidad, introducir los datos en tal array, y ejecutar.".
