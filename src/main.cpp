#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <mySD.h>
#define cartao_inserido 25
#define finaliza 15
#define TEMPO_DEBOUNCE 550 //ms
#define RED 2




int contador_acionamentos = 0;
unsigned long timestamp_ultimo_acionamento = 0;
unsigned long timestamp_ultimo_acionamento1 = 0;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 50; //

double ax = -1000000, ay = -1000000 , az = -1000000; //acc - dumb values, easy to spot problem
double ox = -1000000, oy = -1000000 , oz = -1000000; //orient
double mx = -1000000, my = -1000000 , mz = -1000000; //magnectic
double gx = -1000000, gy = -1000000 , gz = -1000000; //gyro
double rx = -1000000, ry = -1000000 , rz = -1000000; //rotation
double lx = -1000000, ly = -1000000 , lz = -1000000; //linear
double grx = -1000000, gry = -1000000 , grz = -1000000; //gravity

String dataMessage;
bool habilita_gravar = 0;
const int ledaz =  2;
bool continuar;
unsigned long tseconds; 

 int var1,var2,var3,var4;
 int16_t tcomb;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29); //
File myFile;
const int chipSelect = 5;

void initSDCard(){
  myFile = SD.open("data.txt", FILE_WRITE);
  if (myFile) {
  myFile.println("Inicio da coleta: ");
  myFile.println("time;acelerometro(m/s^2);orientacao(degrees);campo_magnetico(uT);giroscopio(rad/s);vetor_de_rotacao(deg);aceleracao_linear(m/s^2);gravidade(m/s^2)");
  myFile.println("millis;ax;ay;az;ox;oy;oz;mx;my;mz;gx;gy;gz;rx;ry;rz;lx;ly;lz;grx;gry;grz;");
  myFile.flush();
  // close the file:
  myFile.close();
 
  } else 
  {
  }
  
}

void IRAM_ATTR funcao_ISR()
{
/* Conta acionamentos do botão considerando debounce */
if ( (millis() - timestamp_ultimo_acionamento) >= TEMPO_DEBOUNCE )
{
habilita_gravar = !habilita_gravar;
timestamp_ultimo_acionamento = millis();
}
}

void IRAM_ATTR funcao_cartao()
{
/* Conta acionamentos do botão considerando debounce */
if ( (millis() - timestamp_ultimo_acionamento1) >= TEMPO_DEBOUNCE )
  {
  myFile = SD.open("data.txt", FILE_WRITE); //abre o arquivo uma vez e o mantem aberto
  timestamp_ultimo_acionamento1 = millis();
  }
}

void calibra()
{
uint8_t system, gyro, accel, mag = 0;
bool saiif=0;
while(saiif == 0)
{

  bno.getCalibration(&system, &gyro, &accel, &mag);

  var1 = 83 * gyro; 
  var2 = 83 * system; 
  var3 = 83 * accel; 
  var4 = 83 * mag; 
  tcomb = (var1+var2+var3+var4);

delay(tcomb);
digitalWrite(RED, HIGH);
delay(tcomb);
digitalWrite(RED, LOW);

if (system == 3 && gyro == 3 && accel == 3 && mag == 3 ) {saiif = 1;}

  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);
}

if (saiif == 1) {digitalWrite(RED,LOW);}
Serial.println("CALIBRADO!");

}


void setup(void)
{
Serial.begin(115200);
pinMode(SS, OUTPUT);
pinMode(cartao_inserido, INPUT);
pinMode(finaliza, INPUT);
pinMode(ledaz, OUTPUT);
pinMode(RED, OUTPUT);

attachInterrupt(finaliza, funcao_ISR, FALLING);
attachInterrupt(cartao_inserido, funcao_cartao, FALLING);

  if (!SD.begin(26, 14, 13, 27)) 
  {
  return;
  }

  /* Initialise the sensor */
  if (!bno.begin())
    {
    }
setCpuFrequencyMhz(80); //crystal mounted = 40MHz, so freq_min is 10 (40/4) max 80

calibra();
digitalWrite(RED,LOW);

initSDCard();
myFile = SD.open("data.txt", FILE_WRITE); //abre o arquivo uma vez e o mantem aberto
}

void printEvent(sensors_event_t* event) {
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    ax = event->acceleration.x;
    ay = event->acceleration.y;
    az = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    ox = event->orientation.x;
    oy = event->orientation.y;
    oz = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    mx = event->magnetic.x;
    my = event->magnetic.y;
    mz = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    gx = event->gyro.x;
    gy = event->gyro.y;
    gz = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    rx = event->gyro.x;
    ry = event->gyro.y;
    rz = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    lx = event->acceleration.x;
    ly = event->acceleration.y;
    lz = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    grx = event->acceleration.x;
    gry = event->acceleration.y;
    grz = event->acceleration.z;
  }
  else {
 
  }

tseconds = millis();
dataMessage = String(tseconds) + ";" + String(ax) + ";" + String(ay) + ";" + String(az) + ";" + String(ox) + ";" + String(oy) + ";" + String(oz) + ";" + String(mx) + ";" + String(my) + ";" + String(mz) + ";" + String(gx) + ";" + String(gy) + ";" + String(gz) + ";" +  String(rx) + ";" + String(ry) + ";" + String(rz) + ";" + String(lx) + ";" + String(ly) + ";" + String(lz) + ";" + String(grx) + ";" + String(gry) + ";" + String(grz) + ";" + "\r\n";

}


void loop(void)
{   //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

//continuar = digitalRead(cartao_inserido);
if (habilita_gravar == 0) 
    {
      digitalWrite(ledaz, HIGH);

        if (myFile) 
            {
      
            myFile.print(dataMessage.c_str());
            }    
          else {
     
              }
    }
  
else if (habilita_gravar == 1)
  {
  myFile.println("Fim da coleta: ");
  myFile.flush();
  myFile.close();
  digitalWrite(ledaz, LOW);
  }

 // delay(BNO055_SAMPLERATE_DELAY_MS);
}

