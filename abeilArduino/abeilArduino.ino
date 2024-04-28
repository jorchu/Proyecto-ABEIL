#include<Servo.h>
#include <Adafruit_INA219.h>
#include <Wire.h>

// crea el objeto servo
Servo myservoestacion;
Servo servDist;

Adafruit_INA219 ina219;

int pos = 0;    // posicion del servo


// Conexiones del driver L293D para un motor DC
int enA = 10;
int in1 = 6;
int in2 = 7;
int enB = 11;
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
unsigned long prevTimePM = millis();
unsigned long prevTimeMov = millis();
unsigned long prevTimeStop = millis();

long timeServo = 400;
long timeSendData = 1200;
int timeMov = 200;
int timeStop = 200;

bool manual = false;
bool carga = true;
bool libre = false;

short subiendo = 0;
short girando = 0;

float totalPower = 0;
float meanPower = 0;
float iterPower = 0;

int ciclos=0;

double mayorDistancia[2];
double puntoDeCarga[2] = {0, 0};

void setup() {
  //puerto serie
  Serial.begin(9600);
  //ina219.begin(); // CUIDADO
  // Colocando los pines en modo salida
  pinMode(enA, OUTPUT); 
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT); 
  pinMode(enB, OUTPUT); 
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
  unsigned long nowTimePM = millis();

  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  bool parar = false;
  /*
  Serial.print("P="); Serial.print(power_mW); Serial.println("mW | ");
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
  power_mW = loadvoltage * current_mA; //ina219.getPower_mW();
    iterPower += power_mW;
    totalPower += power_mW;
    prevTimePM = nowTimePM;
    ciclos += 1;
    if (ciclos == 10)
    {
      meanPower = iterPower / 10;
      iterPower = 0;
      ciclos = 0;
    }
    String sendD = "P=" + String(power_mW) + "mW | Pm=" + String(meanPower) + "mW | Gen=" + totalPower + "mW";
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
      ciclos = 0;
      dcparo();
      

    } else if (data == '7')
    {
      libre = true;
      manual = false;
      carga = false;

      ciclos = 0;
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
      delay(200);
      dcparo();
      delay(200);
      girando = 2;
    } 
    else if (data == '3' || girando == 3)
    {
      izquierda();
      delay(200);
      dcparo();
      delay(200);
      girando = 3;
    }

    if (data == '9')
    {
      subiendo = 0;
    } 
    else if (data == '4' || subiendo == 4)
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
  } else if (libre)
  {
    automatic();
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

  // Encendemos el motor Izda
   digitalWrite(in1, HIGH);
   digitalWrite(in2, HIGH); 
   digitalWrite(in3, HIGH);
   digitalWrite(in4, HIGH); 
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
    delay(500);
  }

void movimiento(int direc)
{
    if (direc == 1)
    {
      alante();
    } 
    else if (direc == 2)
    {
      for (int i = 0; i > 3; i++)
      {
        derecha();
        delay(200);
        dcparo();
        delay(200);
      }
      alante();
    } 
    else if (direc == 0)
    {
      for (int i = 0; i > 3; i++)
      {
        izquierda();
        delay(200);
        dcparo();
        delay(200);
      }
      alante();
    }
    delay(3000);
    dcparo();

}
void automatic()
{
  compararDistancias(servDist);
  movimiento(mayorDistancia[1]);
}

void obtenerPuntoDeCarga(float potencia, int time)
{
  int pos = time * 10; // NOOO
  if (potencia > puntoDeCarga[0])
  {
    puntoDeCarga[0] = potencia;
    puntoDeCarga[1] = pos;
  }
  
}

