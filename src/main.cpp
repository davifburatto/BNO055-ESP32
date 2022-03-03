#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <mySD.h>

//descomentar o define abaixo libera o modo debug, com mais saídas na serial, reduzindo a velocidade de execução do programa
//#define DEBUG

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 200;

double ax = -1000000, ay = -1000000 , az = -1000000; //acc - dumb values, easy to spot problem
double ox = -1000000, oy = -1000000 , oz = -1000000; //orient
double mx = -1000000, my = -1000000 , mz = -1000000; //magnectic
double gx = -1000000, gy = -1000000 , gz = -1000000; //gyro
double rx = -1000000, ry = -1000000 , rz = -1000000; //rotation
double lx = -1000000, ly = -1000000 , lz = -1000000; //linear
double grx = -1000000, gry = -1000000 , grz = -1000000; //gravity

String dataMessage;

const int cartao_inserido = 25;
const int ledaz =  2;
bool continuar;
unsigned long tseconds; 

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

//sdcard
File myFile;
const int chipSelect = 5;

//void initSDCard(void);

// Initialize SD card
void initSDCard(){
  #ifdef DEBUG
    Serial.println("Initializing Micro SD card...");
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  #endif             

  myFile = SD.open("data.txt", FILE_WRITE);
    // if the file opened okay, write to it:
  if (myFile) {
    #ifdef DEBUG
    Serial.println("Writing to data.txt...");
    #endif
    myFile.println("Inicio da coleta: ");
    
    //myFile.println(timelog.c_str());
    myFile.println("time;orientationData;angVelocityData;linearAccelData;magnetometerData;accelerometerData;gravityData");
    myFile.println("millis;ax;ay;az;ox;oy;oz;mx;my;mz;gx;gy;gz;rx;ry;rz;lx;ly;lz;grx;gry;grz;");
	myFile.flush();
    // close the file:
    myFile.close();
    #ifdef DEBUG
    Serial.println("done.");
    #endif
  } else {
    #ifdef DEBUG
    // if the file didn't open, print an error:
    Serial.println("error opening data.txt");
    #endif
  }
  
}

void setup(void)
{
  Serial.begin(115200);
      pinMode(SS, OUTPUT);
      pinMode(cartao_inserido, INPUT);
      pinMode(ledaz, OUTPUT);

  if (!SD.begin(26, 14, 13, 27)) {
  #ifdef DEBUG
    Serial.println("initialization failed!");
    while(1);
  #endif
    return;
  }
#ifdef DEBUG
  Serial.println("SD initialization done.");
  Serial.println("Orientation Sensor Test"); Serial.println("");
#endif
  /* Initialise the sensor */
  if (!bno.begin())
  {
    #ifdef DEBUG
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
    #endif
  }
setCpuFrequencyMhz(80); //crystal mounted = 40MHz, so freq_min is 10 (40/4) max 80
initSDCard();
//delay(1000);
}
void printEvent(sensors_event_t* event) {


  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    #ifdef DEBUG
    Serial.print("Accl:");
    #endif
    ax = event->acceleration.x;
    ay = event->acceleration.y;
    az = event->acceleration.z;
#ifdef DEBUG
  Serial.print("\tx= ");
  Serial.print(ax);
  Serial.print(" |\ty= ");
  Serial.print(ay);
  Serial.print(" |\tz= ");
  Serial.println(az);
#endif
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    #ifdef DEBUG
    Serial.print("Orient:");
    #endif
    ox = event->orientation.x;
    oy = event->orientation.y;
    oz = event->orientation.z;
#ifdef DEBUG
  Serial.print("\tx= ");
  Serial.print(ox);
  Serial.print(" |\ty= ");
  Serial.print(oy);
  Serial.print(" |\tz= ");
  Serial.println(oz);
#endif
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    #ifdef DEBUG
    Serial.print("Mag:");
    #endif
    mx = event->magnetic.x;
    my = event->magnetic.y;
    mz = event->magnetic.z;
#ifdef DEBUG
  Serial.print("\tx= ");
  Serial.print(mx);
  Serial.print(" |\ty= ");
  Serial.print(my);
  Serial.print(" |\tz= ");
  Serial.println(mz);
#endif
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    #ifdef DEBUG
    Serial.print("Gyro:");
    #endif
    gx = event->gyro.x;
    gy = event->gyro.y;
    gz = event->gyro.z;
#ifdef DEBUG
  Serial.print("\tx= ");
  Serial.print(gx);
  Serial.print(" |\ty= ");
  Serial.print(gy);
  Serial.print(" |\tz= ");
  Serial.println(gz);
#endif
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    #ifdef DEBUG
    Serial.print("Rot:");
    #endif
    rx = event->gyro.x;
    ry = event->gyro.y;
    rz = event->gyro.z;
#ifdef DEBUG
  Serial.print("\tx= ");
  Serial.print(rx);
  Serial.print(" |\ty= ");
  Serial.print(ry);
  Serial.print(" |\tz= ");
  Serial.println(rz);
#endif
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    #ifdef DEBUG
    Serial.print("Linear:");
    #endif
    lx = event->acceleration.x;
    ly = event->acceleration.y;
    lz = event->acceleration.z;
#ifdef DEBUG
  Serial.print("\tx= ");
  Serial.print(lx);
  Serial.print(" |\ty= ");
  Serial.print(ly);
  Serial.print(" |\tz= ");
  Serial.println(lz);
#endif
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    #ifdef DEBUG
    Serial.print("Gravity:");
    #endif
    grx = event->acceleration.x;
    gry = event->acceleration.y;
    grz = event->acceleration.z;
#ifdef DEBUG
  Serial.print("\tx= ");
  Serial.print(grx);
  Serial.print(" |\ty= ");
  Serial.print(gry);
  Serial.print(" |\tz= ");
  Serial.println(grz);
#endif
  }
  else {
    #ifdef DEBUG
    Serial.print("Unk:");
    #endif
  }
//cria a string com dados
tseconds = millis();
dataMessage = String(tseconds) + ";" + String(ax) + ";" + String(ay) + ";" + String(az) + ";" + String(ox) + ";" + String(oy) + ";" + String(oz) + ";" + String(mx) + ";" + String(my) + ";" + String(mz) + ";" + String(gx) + ";" + String(gy) + ";" + String(gz) + ";" +  String(rx) + ";" + String(ry) + ";" + String(rz) + ";" + String(lx) + ";" + String(ly) + ";" + String(lz) + ";" + String(grx) + ";" + String(gry) + ";" + String(grz) + ";" + "\r\n";

}

void loop(void)
{ 
  continuar = digitalRead(cartao_inserido);
  if (continuar == 0) {
  digitalWrite(ledaz, HIGH);
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
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
#ifdef DEBUG
  int8_t boardTemp = bno.getTemp();
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);

  Serial.println("--");
#endif
    //Append the data to file
    myFile = SD.open("data.txt", FILE_WRITE);
    // if the file opened okay, write to it:
    if (myFile) 
        {
        #ifdef DEBUG  
        Serial.println("Saving data: ");
        Serial.println(dataMessage);
        #endif
        myFile.print(dataMessage.c_str());
        // close the file:
        myFile.close();
        
        }    
    else {
      #ifdef DEBUG
        // if the file didn't open, print an error:
        Serial.println("error opening data.txt");
        #endif
        }


  delay(BNO055_SAMPLERATE_DELAY_MS);
  }
  else if (continuar == 1)  digitalWrite(ledaz, LOW);
}

