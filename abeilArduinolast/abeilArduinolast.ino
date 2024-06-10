#include<Servo.h>
#include <Adafruit_INA219.h>
#include <Wire.h>

// crea el objeto servo
Servo myservoestacion;
Servo servDist;

Adafruit_INA219 ina219;

int pos = 0;    // posicion del servo


// Conexiones del driver L293D para un motor DC
int in1 = 6;
int in2 = 7;
int in3 = 8;
int in4 = 9;

int ldr1=A0;
int ldr2=A1;
int ldr3=A2;

const int pinServDist = 5;
const int Trigger = 4;
const int Echo = 3;   

int valldr1;
int valldr2;
int valldr3;

char data;

unsigned long prevTimeServo = millis();
unsigned long prevTimeData = millis();
unsigned long prevTimeMov = millis();
unsigned long prevTimeStop = millis();

long timeServo = 400;
long timeSendData = 1200;
int timeMov = 200;
int timeStop = 200;

bool manual = true;
bool carga = false;
bool libre = false;

bool moviendo = false;

short subiendo = 0;
short girando = 0;

float totalPower = 0;
float meanPower = 0;
float iterPower = 0;

int ciclos = 0;
int dataCicle = 0;

double recorrido[50][2]; // Distancia - Dirección
int pasoRecorrido = 0;


double mayorDistancia[2]; // Distancia - Dirección
double puntoDeCarga[2] = {0, 0};

void setup() {
  //puerto serie
  Serial.begin(9600);
  //ina219.begin(); // CUIDADO
  // Colocando los pines en modo salida
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT); 
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
     
  pinMode(ldr1, INPUT);
  pinMode(ldr2, INPUT);
  pinMode(ldr3, INPUT);  
  
  pinMode(Trigger, OUTPUT);
  pinMode(Echo, INPUT);
  
  myservoestacion.attach(2);
  servDist.attach(pinServDist);

  uint32_t currentFrequency;
  if (! ina219.begin()) {
      Serial.println("Failed to find INA219 chip");
      while (1) { delay(10); }
    }


}
 
void loop() {
  
  valldr1=analogRead(ldr1);
  valldr2=analogRead(ldr2);
  valldr3=analogRead(ldr3);

  unsigned long nowTimeServo = millis();
  unsigned long nowTimeData = millis();

  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW  = 0;

  bool parar = false;
  /*
  Serial.print("P="); Serial.print(power_mW) ; Serial.println("mW | ");
  Serial.print("Pm="); Serial.print(current_mA); Serial.println("mW | ");
  Serial.print("Gen="); Serial.print(shuntvoltage); Serial.print("mW");*/


  


  
  if (nowTimeData - prevTimeData > timeSendData)
  {
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();

    if (current_mA < 0)
    {
      current_mA = 0;
    }

    loadvoltage = busvoltage + (shuntvoltage / 1000);

    if (loadvoltage < 0)
    {
      loadvoltage = 0;
    }

    power_mW  = loadvoltage * current_mA; //ina219.getPower_mW( );
    iterPower += power_mW; 
    totalPower += power_mW; 
    dataCicle += 1;

    if (dataCicle == 10)
    {
      meanPower = iterPower / 10;
      iterPower = 0;
      dataCicle = 0;
    }
    String sendD = "P=" + String(power_mW)  + "mW | Pm=" + String(meanPower) + "mW | Gen=" + totalPower + "mW";
    Serial.println(sendD);


    prevTimeData = nowTimeData;
  }

  while (Serial.available())
  {
    data = Serial.read();

    
    if (data == '6') 
    {
      manual = true;
      carga = false;
      libre = false;

      girando = 0;
      subiendo = 0;
      dcparo();
      

    } else if (data == '7')
    {
      libre = true;
      manual = false;
      carga = false;

      dcparo();
    } else if (data == 'a')
    {
      carga = true;
      libre = false;
      manual = false;

      ciclos = 0;
      dcparo();
    }


  }

  if (manual)
  {
    if (data == '8')
    {
      dcparo();
      girando = 0;
      parar = false;
    }

    else if (data == '0')
    {
      alante();
      
    } 
    else if (data == '1')
    {
      atras();
    } 
    else if (data == '2' || girando == 2 && data != '3')
    {
      derecha();
      girando = 2;
    } 
    else if (data == '3' || girando == 3)
    {
      izquierda();
      girando = 3;
    }

    if (data == '9')
    {
      subiendo = 0;
    } 
    else if (data == '4' || subiendo == 4 && data != '5')
    {

      if (pos + 20 <= 180 && nowTimeServo - prevTimeServo > timeServo)
      {
        pos += 20;
        myservoestacion.write(pos);
        subiendo = 4;
        prevTimeServo = nowTimeServo;
      }

    }
    else if (data == '5' || subiendo == 5)
    {

      if (pos - 20 >= 0 && nowTimeServo - prevTimeServo > timeServo)
      {
        pos -= 20;
        myservoestacion.write(pos);
        subiendo = 5;
        prevTimeServo = nowTimeServo;

      }

    }
  } 

  else if (carga)
  {
    for (int i = -25; i != 50; i++)
    {

      if (valldr1+i == valldr2)
      {
          parar = true;
          break;
      }
      
      
    }

    if ((valldr1-valldr2)>30 && !parar){
      izquierda();
      delay(200);
      dcparo();
      delay (200);

      ciclos=ciclos+1;
    } 

    else if ((valldr1-valldr2) < 50 && !parar)
    {
      derecha();
      delay(200);
      dcparo();
      delay (200);
      ciclos=ciclos+1;
    }

    if (ciclos == 10)
    {
      servito();
      ciclos = 0;
    }
  }
  else if (libre)
  {
    obtenerPuntoDeCarga(power_mW, pasoRecorrido);
    if (!moviendo)
    {
      compararDistancias(servDist);
      movimiento(mayorDistancia[1]);

      recorrido[pasoRecorrido][1] = mayorDistancia[1];
      recorrido[pasoRecorrido][0] = mayorDistancia[0];
      
      moviendo = true;
      pasoRecorrido += 1;
    }

    else
    {
      if (obtenerDistancia() > 10)
      {
        dcparo();
        moviendo = false;
      }
    }

  }
}
  

void servito()
{
  int maxPos = 0;
  int maxVal = 0;

  for (int i = 0; i < 180; i += 20)
  {
    int valor = analogRead(ldr3);
    if (valor >= maxVal)
    {
      maxVal = valor;
      maxPos = i;
    }
    myservoestacion.write(i);
    
    delay(400);
  }
  myservoestacion.write(maxPos);

}



int obtenerDistancia()
{
  long tiempo; //timepo que demora en llegar el eco
  long distancia; //distancia en centimetros

  digitalWrite(Trigger, LOW);
  delayMicroseconds(4);
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(Trigger, LOW);

  tiempo = pulseIn(Echo, HIGH); //obtenemos el ancho del pulso
  distancia = tiempo / 59;           //0.034 * time / 2>

  return distancia;
}


void compararDistancias(Servo servo)
{

  mayorDistancia[0] = 0;
  for (int pos = 0; pos <= 180; pos += 90)
  {

    servo.write(pos);
    delay(500);
    long distancia = obtenerDistancia();
    if (mayorDistancia[0] < distancia)
    {
        mayorDistancia[0] = obtenerDistancia();
        mayorDistancia[1] = pos/90; //-----------Posible ERROR de tipado (float)
      }
    }

}


void obtenerPuntoDeCarga(float potencia, int pos)
{

  if (potencia > puntoDeCarga[0])
  {
    puntoDeCarga[0] = potencia;
    puntoDeCarga[1] = pos;
  }
  
}


void goBack(int paso)
{
  for (int i = paso-1; paso != 0; i--)
  {
    //Ya se pensará

  }
}

//Revisar
void movimiento(int direc)
{
  if (direc == 1)
  {
    alante();
  } 
  else if (direc == 2)
  {
    derecha();
    delay(600);
    dcparo();
      
    alante();
  } 
  else if (direc == 0)
  {
    izquierda();
    delay(600);
    dcparo();

    alante();
  }
}





void alante()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW); 
}
void atras()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH); 
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); 
}

void izquierda()
{
   digitalWrite(in1, LOW);
   digitalWrite(in2, HIGH); 
   digitalWrite(in3, HIGH);
   digitalWrite(in4, LOW); 
}

void derecha()
{
   digitalWrite(in1, HIGH);
   digitalWrite(in2, LOW); 
   digitalWrite(in3, LOW);
   digitalWrite(in4, HIGH); 
}


void dcparo(){

   digitalWrite(in1, HIGH);
   digitalWrite(in2, HIGH); 
   digitalWrite(in3, HIGH);
   digitalWrite(in4, HIGH); 
}

