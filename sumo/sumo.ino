#include <QTRSensors.h>
#include <Wire.h>
#include <VL53L0X.h>

/*
 * PIN USE: 
   A0: Left Line Sensor (Analog)
   A1: Right Line Sensor (Analog)
   D2: Left TOF sensor XSHUT
   D3: Right TOF sensor XSHUT
   D4: Direction control of Motor A 
   D5: PWM speed control of Motor A 
   D6: PWM speed control of Motor B 
   D7: Direction control of Motor B 
   A0: Current sense of Motor A 
   A1: Current sense of Motor B 
   A4: SDA (BLUE)
   A5: SCL (GREEN)
 */
#define PWMA 5
#define PWMB 6
#define leftQtr 2
#define rightQtr 3
#define leftx 2
#define rightx 3

#define leftAddr  22
#define rightAddr  23

#define NUM_SENSORS             1 // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  2  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2

#define NUM_READINGS 10
int leftReadings[NUM_READINGS];
int rightReadings[NUM_READINGS];
int leftHistory[NUM_READINGS];
int rightHistory[NUM_READINGS];

// sensors 0 through 1 are connected to analog inputs 0 through 1, respectively
QTRSensorsAnalog qtra((unsigned char[]) {leftQtr}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];



VL53L0X left_tSensor;
VL53L0X right_tSensor;


void setup()
{
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(leftx, OUTPUT); //Pin of right sensor XSHUT
  pinMode(rightx, OUTPUT); //Pin of left sensor XSHUT
  digitalWrite(leftx, LOW); //Drive them low to turn them off..
  digitalWrite(rightx, LOW); //
  delay(500); //Chill for a sec
  Wire.begin(); // Start i2c listening...
  
  Serial.begin (250000);

  pinMode(leftx, INPUT); //Turn left sensor on
  delay(150);
  Serial.println("00");
  left_tSensor.init(true);

  Serial.println("01");
  delay(100);
  left_tSensor.setAddress((uint8_t)leftAddr);
  Serial.println("02");

  pinMode(rightx, INPUT);
  delay(150);
  right_tSensor.init(true);
  Serial.println("03");
  delay(100);
  right_tSensor.setAddress((uint8_t)rightAddr);
  Serial.println("04");
  Serial.println("addresses set");


  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;


  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
  delay(50);
  left_tSensor.setMeasurementTimingBudget(20000);
  left_tSensor.startContinuous();
  right_tSensor.setMeasurementTimingBudget(20000);
  right_tSensor.startContinuous();
  delay(500);
}
#define SPEED 70
void loop()
{
 //qtra.read(sensorValues);
 //Serial.print("Analog Values: ");
 //Serial.println(sensorValues[0]);
 //delay(100);
 //Serial.print("Time of flight distance (Left): ");
 int leftRead = left_tSensor.readRangeContinuousMillimeters(); 
 if(leftRead < 400)
 {
   analogWrite(PWMA, SPEED);
   Serial.print(leftRead);
 }
 else
 {
   analogWrite(PWMA, 0);
   Serial.print(550);
 }
 Serial.print(" ");
 //if (left_tSensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
 //Serial.println();
 //Serial.print("Time of flight distance (Right): ");
 int rightRead = right_tSensor.readRangeContinuousMillimeters(); 
 if(rightRead < 400)
 {
   analogWrite(PWMB,SPEED);
   Serial.print(rightRead);
 }
 else
 {
   analogWrite(PWMB,0);
   Serial.print(550);
 }
 //if (right_tSensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
 Serial.println();
 //delay(10);
}

void readValues(VL53L0X* sensor, uint32_t* dest, size_t numReadings)
{
  for(int i = 0; i < numReadings; i++)
  {
    *(dest + i ) = sensor->readRangeSingleMillimeters();
  }
}

