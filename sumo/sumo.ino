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
   PIN USE:
   D2: Left TOF sensor (white) XSHUT
   D3: Right TOF sensor (PURPLE) XSHUT //sticky-ick-icky
   D4: Direction control of Motor A
   D5: PWM speed control of Motor AM
   D6: PWM speed control of Motor B
   D7: Direction control of Motor B
   D8: Startup Switch
   A0: Current sense of Motor A
   A1: Current sense of Motor B
   A2: Left Line (ORANGE) Sensor (Analog)
   A3: Right Line (YELLOW) Sensor (Analog)
   A4: SDA (BLUE)
   A5: SCL (GREEN)

   Sensors use 5v pin for power, GND for ground.
*/
//================= temp /debugging variables   ========================
long time_since_last_read;
bool leftOn = false; //Use to know if we should change the pwm speed of each motor
bool rightOn = false;


//================= robot control variables   ========================
#define SPEED                 255                   //max motor speed. range is 0-255. 255 = 100% duty PWM, which is battery input voltage (14.8v for 4cell lipo, 11.1v for 3cell lipo)
#define TURN_SPEED_FAST_WHEEL 150
#define TURN_SPEED_SLOW_WHEEL 75
#define TOF_MAX_RANGE         350                   // throw away ranges longer than this, to avoid noise and possibly sensing people too close to the ring. 
#define INFINITY_VALUE        350
#define QTR_LIMIT             100                   //black is considered anything greater than this value. 
#define DELAY_U_TURN          200
#define DELAY_U_TURN_REVERSE  200

boolean last_tof_sighted_left;                     //holds last sensor that sighted something at non-infinity
boolean sensed_this_loop = false;                  //true if at least one sensor detected non-infinity this loop execution
char motor_command;
int leftRead;
int rightRead;

//================= hardware variables        ========================
#define dir_left_motor          4           //direction to left motor pin, HIGH = forward
#define pwm_left_motor          5           //power to left motor pin
#define pwm_right_motor         6           //power to right motor pin
#define dir_right_motor         7           //direction to right motor pin, HIGH = forward
#define startup_pin             8

#define left_qtr                A2           //left edge sensor pin
#define right_qtr               A3           //right edge sensor pin
#define left_x                  2           //left TOF sensor "xshut" shutdown logic pin
#define right_x                 3           //right TOF sensor "xshut" shutdown logic pin
#define left_addr               22          //left TOF sensor I2C address
#define right_addr              23          //right TOF sensor I2C address

#define NUM_QTR_SENSORS               2     // number of sensors used
#define NUM_SAMPLES_PER_QTR_SENSOR    4     // average 4 analog samples per sensor reading
#define EMITTER_PIN                   QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2

#define TIMING_BUDGET_US              20000  //minimum internally limited to 20000, pushing this below 20000 returns it to the default of 33ms :-(
#define MIN_PRESS_TIME         100
// sensors 0 through 1 are connected to analog inputs 0 through 1, respectively
QTRSensorsAnalog qtra((unsigned char[]) {
  left_qtr, right_qtr
}, NUM_QTR_SENSORS, NUM_SAMPLES_PER_QTR_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_QTR_SENSORS]; // 2 qtr sensors, 2 sent to array constructor, but array has indices [0] and [1] only

VL53L0X* left_tSensor = 0;                       //left TOF sensor object
VL53L0X* right_tSensor = 0;                      //right TOF sensor object

void setup()
{
  Serial.begin (2000000);  //fastest possible to avoid delays due to serial write lag
  #ifdef STARTUP_BUTTON
  #error THIS DOESNT WORK RIGHT NOW
  Serial.println("Waiting for sensor start...");
  pinMode(startup_pin, INPUT);
  while(true)
  {
    while(digitalRead(startup_pin) == LOW); //Wait
    unsigned long pressTime = millis();
    while(digitalRead(startup_pin) == HIGH)
      Serial.println("Pressed...");// Wait
    if(millis() - pressTime > MIN_PRESS_TIME)
    {
      break;
    }
  }
  Serial.println("Robot Start!");
  //set initial motor direction
  #endif
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

  read_TOF_sensors();
  read_edge_sensors();

  //================ motor control decision logic ======================
  sensed_this_loop = true;
  if(leftRead < TOF_MAX_RANGE)
  {
    motor_command = '<';
  }

  if(rightRead < TOF_MAX_RANGE)
  {
    motor_command = '>';
  }

  if( (rightRead < TOF_MAX_RANGE) && (leftRead < TOF_MAX_RANGE) )
  {
    Serial.println("here");
    motor_command = 'F';
  }

  if(rightRead == TOF_MAX_RANGE && leftRead == TOF_MAX_RANGE)
  {
    sensed_this_loop = false;
    seek_unseen_enemy();
    
  }
  /*
   
  //==if error in readings
  if (leftRead < 0 || rightRead < 0)
  {
    //restart_tof_sensors();
    motor_command = 'S';
    Serial.println("=================== ERROR: RESETTING TOF SENSORS =================");
  }

  //==if seen left, but not right
  else if (leftRead < TOF_MAX_RANGE && rightRead >= INFINITY_VALUE)
  {
    motor_command = '<';
    last_tof_sighted_left = true;
    sensed_this_loop = true;
  }

  //==if seen right, but not left
  else if (rightRead < TOF_MAX_RANGE && leftRead >= INFINITY_VALUE)
  {
    motor_command = '>';
    last_tof_sighted_left = false;
    sensed_this_loop = true;
  }

  //==if seen left AND right
  else if (leftRead < TOF_MAX_RANGE && rightRead < TOF_MAX_RANGE)
  {
    motor_command = 'F';
    sensed_this_loop = true;
  }
*/


    //==check edge sensors for white, left is [0], right is [1],
    if (sensorValues[0] < QTR_LIMIT)
    {
      motor_command = ')';
    }

    else if (sensorValues[1] < QTR_LIMIT)
    {
      motor_command = '(';
    }

  //  //==if not seen at all
  //  else
  //  {
  //     seek_unseen_enemy();
  //  }
  if(sensed_this_loop)
  {
    drive_motors(motor_command);
  }
  //reset sensed_this_loop to allow a fresh look next loop
  sensed_this_loop = false;
  //have the motors actually do something with the instruction sent this loop
  //reset motor command. should always apply somethign each round, but if the bot sits still, we know we missed something.
  motor_command = ' ';

  Serial.println();

}

void read_edge_sensors()
{
  //edge sensor reading and serial output
  qtra.read(sensorValues);
  Serial.print("QTR Analog Values- Left: ");
  Serial.print(sensorValues[0]);
  Serial.print(", Right: ");
  Serial.print(sensorValues[1]);
  Serial.println();
}

void read_TOF_sensors()
{
  //===== read TOF sensors
  Serial.print("Time of flight distance (Left): ");
  leftRead = left_tSensor->readRangeContinuousMillimeters();
  if (leftRead > TOF_MAX_RANGE)
  {
    leftRead = INFINITY_VALUE;
  }
  Serial.print(leftRead);

  Serial.print("Time of flight distance (Right): ");
  rightRead = right_tSensor->readRangeContinuousMillimeters();
  if (rightRead > TOF_MAX_RANGE)
  {
    rightRead = INFINITY_VALUE;
  }
  Serial.println(rightRead);
}


void drive_motors(char instruction)
{
  switch (instruction)
  {
    case 'F':
      Serial.print("[Forward]");
      digitalWrite(dir_right_motor, HIGH);
      analogWrite(pwm_right_motor, SPEED);
      digitalWrite(dir_left_motor, HIGH);
      analogWrite(pwm_left_motor, SPEED);
      break;
    case 'R':
      Serial.print("[Reverse]");
      digitalWrite(dir_right_motor, LOW);
      analogWrite(pwm_right_motor, SPEED / 2);
      digitalWrite(dir_left_motor, LOW);
      analogWrite(pwm_left_motor, SPEED / 2);
      break;
    case '<':
      last_tof_sighted_left = true;
      Serial.print("[Left]");
      digitalWrite(dir_right_motor, HIGH);
      analogWrite(pwm_right_motor, TURN_SPEED_FAST_WHEEL / 2);
      digitalWrite(dir_left_motor, LOW);
      analogWrite(pwm_left_motor, TURN_SPEED_SLOW_WHEEL);
      break;
    case '>':
      last_tof_sighted_left = false;
      Serial.print("[Right]");
      digitalWrite(dir_left_motor, HIGH);
      analogWrite(pwm_left_motor, TURN_SPEED_FAST_WHEEL / 2);
      digitalWrite(dir_right_motor, LOW);
      analogWrite(pwm_right_motor, TURN_SPEED_SLOW_WHEEL);
      break;
    case '(':
      Serial.print("[Reverse]");
      digitalWrite(dir_right_motor, LOW);
      analogWrite(pwm_right_motor, SPEED / 2);
      digitalWrite(dir_left_motor, LOW);
      analogWrite(pwm_left_motor, SPEED / 2);
      delay(DELAY_U_TURN_REVERSE);

      Serial.print("[180 left-200ms delay]");
      digitalWrite(dir_right_motor, HIGH);
      analogWrite(pwm_right_motor, TURN_SPEED_SLOW_WHEEL);
      digitalWrite(dir_left_motor, LOW);
      analogWrite(pwm_left_motor, TURN_SPEED_SLOW_WHEEL);
      delay(DELAY_U_TURN);
      break;
    case ')':
      Serial.print("[Reverse]");
      digitalWrite(dir_right_motor, LOW);
      analogWrite(pwm_right_motor, SPEED / 2);
      digitalWrite(dir_left_motor, LOW);
      analogWrite(pwm_left_motor, SPEED / 2);
      delay(DELAY_U_TURN_REVERSE);

      Serial.print("[180 right-200ms delay]");
      digitalWrite(dir_right_motor, LOW);
      analogWrite(pwm_right_motor, TURN_SPEED_SLOW_WHEEL);
      digitalWrite(dir_left_motor, HIGH);
      analogWrite(pwm_left_motor, TURN_SPEED_SLOW_WHEEL / 2);
      delay(DELAY_U_TURN);
      break;
    case 'S':
      Serial.print("[stop]");
      digitalWrite(dir_right_motor, HIGH);
      analogWrite(pwm_right_motor, 0);
      digitalWrite(dir_left_motor, HIGH);
      analogWrite(pwm_left_motor, 0);
      break;
    default:
      Serial.print("[INVALID MOTOR INSTRUCTION! \']");
      Serial.print(instruction);
      Serial.println("\'");
      break;

  }
}


void seek_unseen_enemy()
{
  //================= seeking unseen enemy routine   ========================
  //if both sensors returned infinity this round
  if (!sensed_this_loop)
  {
    Serial.println("Not sensed this loop!");
    //if last sensor to see the enemy was LEFT side
    if (last_tof_sighted_left == true)
    {
      Serial.println("Turning LEFT; enemy out of sight, last seen on left side");
      motor_command = '<';

    }
    else //must be to the RIGHT
    {
      Serial.println("Turning RIGHT; enemy out of sight, last seen on right side");
      motor_command = '>';
    }
  }

}
#define VLDELAY 100
void restart_tof_sensors()
{
  if (left_tSensor != 0)
    delete left_tSensor;
  if (right_tSensor != 0)
    delete right_tSensor;
  left_tSensor = new VL53L0X();
  right_tSensor = new VL53L0X();
  Serial.println("================= restarting TOF sensors ========================");
  //turn off motors
  //motor_command = 'S';
  //drive_motors(motor_command);

  //shut down the two TOF sensors so we can choose their addresses one at a time

  //Drive them low to turn them off..
  digitalWrite(left_x, LOW);
  digitalWrite(right_x, LOW); //

  //switch them to HIGH impedence interanal resistor mode to shut them down.
  //pinMode(left_x, OUTPUT);
  //pinMode(right_x, OUTPUT);

  delay(VLDELAY); //Chill for a sec
  Wire.begin(); // Start i2c listening...

  //Turn left sensor on, address it on I2C
  //pinMode (x, INUT) switches the internal arduino pullups resistor off, so the pin is "low impedence" mode
  pinMode(left_x, INPUT);
  //one would imagine you'd want to "digitalWrite(left_x, HIGH);" here, but it's not needed, AND WILL DAMAGE THE SENSOR!
  delay(VLDELAY);
  Serial.println("00");
  left_tSensor->init(true);
  Serial.println("01");
  delay(VLDELAY);
  left_tSensor->setAddress((uint8_t)left_addr);
  Serial.println("02");

  //Turn right sensor on, address it on I2C
  //pinMode (x, INUT) switches the internal arduino pullups resistor off, so the pin is "low impedence" mode
  pinMode(right_x, INPUT);
  //one would imagine you'd want to "digitalWrite(right_x, HIGH);"  here, but it's not needed, AND WILL DAMAGE THE SENSOR!
  delay(VLDELAY);
  right_tSensor->init(true);
  Serial.println("03");
  delay(VLDELAY);
  right_tSensor->setAddress((uint8_t)right_addr);
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
  left_tSensor->setMeasurementTimingBudget(TIMING_BUDGET_US);
  left_tSensor->startContinuous();
  right_tSensor->setMeasurementTimingBudget(TIMING_BUDGET_US);
  right_tSensor->startContinuous();

}

void measure_time_since_last_call()
{
  //timing test
  time_since_last_read = millis() - time_since_last_read;
  Serial.print("ms since last read : ");
  Serial.print(time_since_last_read);
  Serial.print(", ");
  time_since_last_read = millis();
}

