#include <Servo.h>
const int MaxSensors = 2;                     // Número de Motores Servo
const int ServoPins[MaxSensors] = {7, 8};     // Pines donde estan los Motores Servo
const int RangingPins[MaxSensors] = {3, 2};   // Pines donde esta sensores
const int ReadingsPerSensor = 5;              // Número de lecturas para determinar posición 
const int TimePerDegree = 9;                  // Rotación del servo para no interrumpir sensores
const int MinimumTurnDistance = 3;            // Minimo de grados de rotacion del servo

// Variables
Servo ServoList[MaxSensors];                         // Array para manipular el servo.
int sensorReadings[MaxSensors][ReadingsPerSensor];   // Mantiene lecturas del sensor en una variable. 
int calculatedSenorReadings[MaxSensors];             // Calcula la distancia medida en el sensor de proximidad.
int latestReading = 0;                               // Posición actual de la matriz.
int servoLocations[MaxSensors];                      // Posición actual del sensor.
int SenorClose = 500;                                // Valor mas sercano que detecta el ping(tiempo de viaje de onda).
int SensorFar = 14000;                               // Valor mas lejano que detecta el ping (tiempo de viaje de onda).
int ServoClose[MaxSensors] = {0, 160};               // Angulo del servo cuando algo esta serca.
int ServoFar[MaxSensors] = {70,110};                 // Angulo del servo cuando algo esta lejos.

void setup() {

  //Serial.begin(115200);   				
  //Serial.println("Begin...");

  // Iniciar la ubicación del servo y mover a el para saber que esta trabajando.
  for (int i = 0; i < MaxSensors; i++){
     ServoList[i].attach(ServoPins[i]);
     delay(10);
     ServoList[i].write(ServoClose[i]);
     delay(500);
     ServoList[i].write(ServoFar[i]);
     delay(500);
     ServoList[i].detach();
   }
   delay(100);


}

void loop(){
  int i, j, oldLocation;
  unsigned long delayTime;

  // Loop de sensores de alcanze.
  for (i = 0; i < MaxSensors; i++){
    // Obtener rango de sensores
    sensorReadings[i][latestReading] = getDistance(i);
    // Calcule un promedio de lecturas / suavizadas basado en esto y últimos datos.
    calculatedSenorReadings[i] = calculateNewDistace(i);

    // Ajuste del servo en el angulo correcto.
    oldLocation = servoLocations[i];
    servoLocations[i] = map(calculatedSenorReadings[i], 0, 100, ServoClose[i], ServoFar[i]);

    if (latestReading >= ReadingsPerSensor-1){                          // No haga nada hasta que tengamos suficientes datos a la tendencia.
      if (abs(servoLocations[i]-oldLocation) >= MinimumTurnDistance){   // Sólo tratar de convertirlo si tenemos donde ir.
		  ServoList[i].attach(ServoPins[i]);
		  delay(10);
		  ServoList[i].write(servoLocations[i]);
		  delayTime = (TimePerDegree * (abs(servoLocations[i]-oldLocation))+20);      // Establecer un retardo para la próxima lectura, el ruido del motor no interfiere con las lecturas del sensor.
		  if (abs(delayTime)>500){ // Si no puede hacerlo en este período de tiempo   // Se basa en la distancia que tiene que recurrir a mantener el retraso de un tiempo mínimo, la respuesta en un máximo.
			delayTime=500;         // vamos a llegar la próxima vez. Mantenerlo sensible.
		  }
		  delay(delayTime);
		  ServoList[i].detach();
	  } else {                                          // De lo contrario, si la lectura no ha cambiado lo suficiente como escribió el antiguo valor de
	      ServoList[i].attach(ServoPins[i]);            // el servo para que se mantenga en su lugar si se trata de la aplicación de presión.
		  delay(10);
		  ServoList[i].write(oldLocation);
		  delay(50);         
		  ServoList[i].detach();   
	      servoLocations[i]=oldLocation;
	  }
    }
    delay(20); //Para arreglar el mal comportamiento del sensor izquierdo.
  }

  latestReading++; // Incrementar los conteos de lectura para saber dónde estamos.
  if (latestReading >= ReadingsPerSensor){  // Asegúrese de que no vamos a grabar más lecturas que tenemos espacio para contener.
    latestReading = ReadingsPerSensor-1;
    // Pop de la lectura más antigua de la lista.
    for (i = 0; i < MaxSensors; i++){
      for (j=0; j < ReadingsPerSensor-1; j++){
        sensorReadings[i][j] = sensorReadings[i][j+1];
      }
    }
  }
}

// function: calculateNewDistace(sensorNumber: Qué datos sensor para procesar): Calcular la distancia en el rango de 0-100.
// Aplique un poco de promedio y el suavizado en las lecturas de distancia registrados
// para cuidar de datos ruidosos.
int calculateNewDistace(int sensorNumber){
  int output = SensorFar;                      // El valor por defecto es la distancia más larga.

  float weightingFactor = 0.5;                 // La rapidez con la importancia de la lectura va disminuyendo con el tiempo. (1= no taper, 0 = divide by zero error.)
  float flickerFactor = 30;                    // Cuando el cambio es mayor que este, no haga caso a menos que sea dos en fila. (Es probable que el ruido).

  if (latestReading >= ReadingsPerSensor-1) {  // Sólo haga esto si tenemos un conjunto completo de lecturas de probar.
    int total = 0;                             // Promediarlos con una ponderación.
    float currentWeight = 1;                   // Nuevas lecturas cuentan más que las lecturas mayores.
    float percentagePossible = 0;
    boolean flickered = false;
    for (int i=ReadingsPerSensor-1; i >=0 ;i--){   // Compruebe si el parpadeo (Esto reduce el jitter con algo a la derecha en el umbral.)
      flickered = false;
      if (i==ReadingsPerSensor-1){
        if ((abs(sensorReadings[sensorNumber][i])-abs(sensorReadings[sensorNumber][i-1]) > flickerFactor) &&
           (abs(sensorReadings[sensorNumber][i-1])-abs(sensorReadings[sensorNumber][i-2]) > flickerFactor)){
          flickered = true;
        }
      }
      if (flickered==false){
        total += (sensorReadings[sensorNumber][i] * currentWeight);
        percentagePossible += currentWeight;
        currentWeight *= weightingFactor;
      }
    }
    output = total / percentagePossible;
  }
  return output;
}
// function: getDistance
// Tomar un número de sensor (no pin número) y devuelve un entero en el rango de 0-100
// 0 = más cercano, 100 = más lejos. (Es un porcentaje de la distancia que el software

int getDistance(int sensorNumber){
  long duration;   // El tiempo que tarda un pulso sónico para reflexionar.
  int out;         // El valor enviamos de vuelta de la función

  // Inicializar el sensor y dilo a enviar un ping.
  pinMode(RangingPins[sensorNumber], OUTPUT);
  digitalWrite(RangingPins[sensorNumber], LOW);
  delayMicroseconds(2);
  digitalWrite(RangingPins[sensorNumber], HIGH);
  delayMicroseconds(5);
  digitalWrite(RangingPins[sensorNumber], LOW);

  // Leer la hora en milisegundos hasta que el valor se vuelve.
  pinMode(RangingPins[sensorNumber], INPUT);
  duration = pulseIn(RangingPins[sensorNumber], HIGH);

  // Recorte los datos en mínimos y máximos y asignarla a la gama 0-100 salida.
  duration = constrain(duration, SenorClose, SensorFar);
  out = map(duration,  SenorClose, SensorFar, 0, 100);
  return out;
}
