# Medicion de nivel de agua

## Calibracion de cada sensor:
### Caracterizacion DAC-Lazo de corriente:
-Encontrar el valor del DAC para el cual la salida del lazo de corriente es 4mA. Actualizar este valor en el macro "DAC_MIN". (De modo orientativo, este valor esta cercano a 43).  
-Encontrar el valor del DAC para el cual la salida del lazo de corriente es 20mA. Actualizar este valor en el macro "DAC_MAX". (De modo orientativo, este valor esta cercano a 241).  
-Encontrar el valor del DAC para el cual la salida del lazo de corriente es 12mA (valor medio entre 4mA y 20mA). Actualizar este valor en el macro "DAC_12mA". (De modo orientativo, este valor esta cercano a 141).  
  
### Caracterizacion sensor
-Sumergir el sensor en agua hasta que deje de medir saturacion inferior y anotar dicha altura.  
-Continuar sumergiendo el sensor hasta llegar a la saturacion superior, y anotar la altura a la que sucede.  
-Haciendo la resta de los valores obtenidos anteriormente, actualizar el valor del macro RANGO_MM.  
-Actualizar el valor en la inicializacion de la variable "mm_offset_cal" a RANGO_MM/2. CHEQUEAR ESTA ULTIMA PARTE.


### Aca poner algo sobre caracterizar el sensor, ir midiendo capacidad y encontrar y modificar los coeficientes.
### Acomodar script de octave para que haga todo.






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