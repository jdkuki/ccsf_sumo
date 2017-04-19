#include <QTRSensors.h>
#include <Wire.h>
#include <VL53L0X.h>


/*================================= TO DO =======================================
  1. implement edge sensor code
  1.1 If one edge is detected, reverse for a moment, spin in place ~60* away from edge detected
  1.2 If two edges detected, reverse for a moment, spin in place ~90*. 
  2. install start switch + implement countdown timer
  3. implement different modes/behaviours?
*/


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

   Sensors use 5v pin for power, GND for ground.
 */
//================= temp /debugging variables   ========================
long time_since_last_read;


//================= robot control variables   ========================
#define SPEED 175                            //max motor speed. range is 0-255. 255 = 100% duty PWM, which is battery input voltage (14.8v for 4cell lipo, 11.1v for 3cell lipo)
#define TURN_SPEED_FAST_WHEEL 160
#define TURN_SPEED_SLOW_WHEEL 60
#define TOF_MAX_RANGE 400                   // throw away ranges longer than this, to avoid noise and possibly sensing people too close to the ring. 
boolean last_tof_sighted_left;                    //holds last sensor that sighted something at non-infinity
#define INFINITY_VALUE 550
boolean sensed_this_loop = false;                   //true if at least one sensor detected non-infinity this loop execution


//================= hardware variables        ========================
#define pwm_left_motor          5           //power to left motor pin
#define pwm_right_motor         6           //power to right motor pin
#define dir_left_motor          4           //direction to left motor pin, HIGH = forward
#define dir_right_motor         7           //direction to right motor pin, HIGH = forward

#define left_qtr                2           //left edge sensor pin
#define right_qtr               3           //right edge sensor pin
#define left_x                  2           //left TOF sensor "xshut" shutdown logic pin
#define right_x                 3           //right TOF sensor "xshut" shutdown logic pin

#define left_addr               22          //left TOF sensor I2C address
#define right_addr              23          //right TOF sensor I2C address

#define NUM_QTR_SENSORS               2     // number of sensors used
#define NUM_SAMPLES_PER_QTR_SENSOR    4     // average 4 analog samples per sensor reading
#define EMITTER_PIN                   QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2

#define NUM_READINGS                  10

#define TIMING_BUDGET_US              20000  //minimum internally limited to 20000, pushing this below 20000 returns it to the default of 33ms :-(


//what are these for?
int leftReadings[NUM_READINGS];
int rightReadings[NUM_READINGS];
int leftHistory[NUM_READINGS];
int rightHistory[NUM_READINGS];

// sensors 0 through 1 are connected to analog inputs 0 through 1, respectively
QTRSensorsAnalog qtra((unsigned char[]) {left_qtr, right_qtr}, NUM_QTR_SENSORS, NUM_SAMPLES_PER_QTR_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_QTR_SENSORS]; // 2 qtr sensors, 2 sent to array constructor, but array has indices [0] and [1] only

VL53L0X left_tSensor;                       //left TOF sensor object
VL53L0X right_tSensor;                      //right TOF sensor object

bool leftOn = false; //Use to know if we should change the pwm speed of each motor
bool rightOn = false;

void setup()
{ 
  Serial.begin (2000000);  //fastest possible to avoid delays due to serial write lag
  
  //set initial motor direction
  digitalWrite(dir_left_motor, HIGH);
  digitalWrite(dir_right_motor, HIGH);
  
  //"Pins configured as OUTPUT with pinMode() are said to be in a low-impedance state. 
  //This means that they can provide a substantial amount of current (40ma) to other circuits. "
  pinMode(pwm_left_motor, OUTPUT);          
  pinMode(pwm_right_motor, OUTPUT);
  pinMode(left_x, OUTPUT); //Pin of right sensor XSHUT
  pinMode(right_x, OUTPUT); //Pin of left sensor XSHUT

  //start TOF sensors
  restart_tof_sensors(); 
}

void loop()
{
  //================= enemy detection   ========================
  measure_time_since_last_call();

  //edge sensor reading and serial output
  qtra.read(sensorValues);
  Serial.print("QTR Analog Values: ");
  Serial.print(sensorValues[0]);
  Serial.print(", ");
  Serial.print(sensorValues[1]);
  Serial.print(", ");

  //===== left sensor
  Serial.print("Time of flight distance (Left): ");
  int leftRead = left_tSensor.readRangeContinuousMillimeters();
 
  if(leftRead < TOF_MAX_RANGE)
  {
    if(!leftOn) //Debug code: See if spamming analogWrites() is causing our sensors to fail;
    {
      digitalWrite(dir_left_motor, HIGH);
      analogWrite(pwm_left_motor, SPEED);
    }
    Serial.print(leftRead);
    last_tof_sighted_left= true;
    sensed_this_loop = true;
    leftOn = true;
  }
  else if (leftRead <=0)
  {
    restart_tof_sensors();
  }
  else
  {
    analogWrite(pwm_left_motor, 0);
    Serial.print(INFINITY_VALUE);
    leftOn = false;
  }
  Serial.print(", ");

  //===== right sensor
  Serial.print("Time of flight distance (Right): ");
  int rightRead = right_tSensor.readRangeContinuousMillimeters();
  
  if(rightRead < TOF_MAX_RANGE)
  {
    if(!rightOn) //Debug code: See if spamming analogWrites() is causing our sensors to fail;
    {
      digitalWrite(dir_right_motor, HIGH);
      analogWrite(pwm_right_motor,SPEED);
    }
    Serial.print(rightRead);
    last_tof_sighted_left= false;
    sensed_this_loop = true;
    rightOn = true;
  }
  else if (rightRead <=0)
  {
    restart_tof_sensors();
  }
  else
  {
    analogWrite(pwm_right_motor,0);
    Serial.print(INFINITY_VALUE);
    rightOn = false;
  }
  Serial.print(", ");
  Serial.println();

  //================= seeking unseen enemy routine   ========================
  //if both sensors returned infinity this round
  if (!sensed_this_loop)
  {
    Serial.println("Not sensed this loop!");
    //if last sensor to see the enemy was LEFT side
    if (last_tof_sighted_left== true)
    {
      digitalWrite(dir_left_motor, LOW);
      analogWrite(pwm_left_motor,TURN_SPEED_SLOW_WHEEL);
      digitalWrite(dir_right_motor, HIGH);
      analogWrite(pwm_right_motor,TURN_SPEED_FAST_WHEEL);
      Serial.println("Turning LEFT; enemy out of sight, last seen on left side");
    }
    else //must be to the RIGHT
    {
      digitalWrite(dir_right_motor, LOW);
      analogWrite(pwm_right_motor,TURN_SPEED_SLOW_WHEEL);
      digitalWrite(dir_left_motor, HIGH);
      analogWrite(pwm_left_motor,TURN_SPEED_FAST_WHEEL);
      Serial.println("Turning RIGHT; enemy out of sight, last seen on right side");      
    } 
  }

  //reset sensed_this_loop to allow a fresh look next loop
  sensed_this_loop =false;

  //delay to allow seeing the console messages better - remove for competition
  //delay(100);

}


void restart_tof_sensors()
{

  Serial.println("================= restarting TOF sensors ========================");
  //turn off motors
  digitalWrite(dir_right_motor, HIGH);
  analogWrite(pwm_right_motor,0);
  digitalWrite(dir_left_motor, HIGH);
  analogWrite(pwm_left_motor,0);
  
  //shut down the two TOF sensors so we can choose their addresses one at a time
  digitalWrite(left_x, LOW); //Drive them low to turn them off..
  digitalWrite(right_x, LOW); //
  delay(1); //Chill for a sec
  Wire.begin(); // Start i2c listening...
  
  //Turn left sensor on, address it on I2C
  pinMode(left_x, INPUT);
  //one would imagine you'd want to "digitalWrite(left_x, HIGH);" here, but it's not needed, as the sensor's xshut is impedence driven
  delay(1);
  Serial.println("00");
  left_tSensor.init(true);
  Serial.println("01");
  delay(1);
  left_tSensor.setAddress((uint8_t)left_addr);
  Serial.println("02");

  //Turn right sensor on, address it on I2C
  pinMode(right_x, INPUT);
  //one would imagine you'd want to "digitalWrite(right_x, HIGH);"  here, but it's not needed, as the sensor's xshut is impedence driven
  delay(1);
  right_tSensor.init(true);
  Serial.println("03");
  delay(1);
  right_tSensor.setAddress((uint8_t)right_addr);
  Serial.println("04");
  Serial.println("addresses set");

  //print addresses of found sensors
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
    } // end of good response
  } // end of for loop
  
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");

  // set timing mode for sensors, allowing a short budget in microseconds. 20,000us = 20ms = 50Hz read rate. 20ms is apparently the minimum that works. 
  //wondering if we should try something smaller to increase the refresh rate, since we don't care at all about range, just presence of an object.
  left_tSensor.setMeasurementTimingBudget(TIMING_BUDGET_US);
  left_tSensor.startContinuous();
  right_tSensor.setMeasurementTimingBudget(TIMING_BUDGET_US);
  right_tSensor.startContinuous();
  
}

void measure_time_since_last_call()
{
  //timing test
  time_since_last_read = millis()-time_since_last_read;
  Serial.print("ms since last read : ");
  Serial.print(time_since_last_read);
  Serial.print(", ");
  time_since_last_read = millis();
}

