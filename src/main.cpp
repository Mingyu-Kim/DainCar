// Bluetooth Motor Controller
// Programmed to run on ESP32
// written by Mingyu Kim
// mingyu(AT)mingyu.co.kr
// version v0.9

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <SerialCommand.h>
#include <Encoder.h>

#define DEBUG                         //define it to see debug via Serial1
//#define SERIALCOMMAND_DEBUG           //define it to see serial command debug via some serial
#define motorAttachPin ledcAttachPin    //rename ledcattachpin to motorattachpin to add more clarity to code
#define motorSetup ledcSetup            //renamed to add more clarity to code
#define motorWrite ledcWrite            //renamed to add more clarity to code

#define leftPWM 1                   //left forward PWM uses LEDC channal 1
#define rightPWM 2                  //right reverse PWM uses LEDC channal 2

#define standby 5

#define leftEnc1 33             // To Do. Add Encoder support
#define leftEnc2 32
#define rightEnc1 12
#define rightEnc2 13

#define leftPWMPin 25
#define rightPWMPin 26
#define leftForward 4               //left forward driver pin is connected to esp32thing's pin 13
#define leftReverse 15               //left reverse driver pin is connected to esp32thing's pin 12
#define rightForward 14              //right forward driver pin is connected to esp32thing's pin 14
#define rightReverse 27              //right reverse driver pin is connected to esp32thing's pin 127

#define PWMFrequency 5000                //PWM frequency is set to 5000Hz
#define PWMResolution 8                 //PWM duty resolution is set to 8 bits (0-255)

#ifdef DEBUG                            //rename DEBUG_PRINT to Serial.print on definition of DEBUG
 #define DEBUG_PRINT(x)  Serial.print (x)
#else
 #define DEBUG_PRINT(x)
#endif
#ifdef DEBUG
 #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
 #define DEBUG_PRINTLN(x)
#endif

BluetoothSerial BT;                     //Create Bluetooth Serial port on ESP32 and name it as BT
SerialCommand parser(BT);               //Create Serial command parser that listens on BT

Encoder leftEnc(leftEnc1, leftEnc2);
Encoder rightEnc(rightEnc1, rightEnc2);


unsigned long lastCommand = 0;          //this will store millis time when last command was parsed correctly

//this function receives control speed (0-255) and control vector -180-180 as parameter and changes motor duty accordingly
void motorController(int controlSpeed, int controlVector){
  int leftSpeed;
  int rightSpeed;
  if(controlVector > 135){
      // 136 - 180
      rightSpeed = (controlVector-135)*255/45;
      leftSpeed = -255;
      
  }else if(controlVector > 90){
      // 91 - 135
      rightSpeed = -255+(controlVector-90)*255/45;
      leftSpeed = -255;

  }else if(controlVector > 45){
      // 46 - 90
      rightSpeed = -255;
      leftSpeed = -(controlVector-45)*255/45;

  }else if(controlVector > 0){
      // 1 - 45
      rightSpeed = -255;
      leftSpeed = 255-(controlVector+0)*255/45;

  }else if(controlVector > -45){
      // 0 - -44
      rightSpeed = -(controlVector+45)*255/45;
      leftSpeed = 255;

  }else if(controlVector > -90){
      // -45 - -89
      rightSpeed = 255-(controlVector+90)*255/45;
      leftSpeed = 255;

  }else if(controlVector > -135){
      // -90 - -134
      rightSpeed = 255;
      leftSpeed = (controlVector+135)*255/45;

  }else if(controlVector > -181){
      // -135 - -180
      rightSpeed = 255;
      leftSpeed = -255 + (controlVector+180)*255/45;

  }else{
      rightSpeed = 0;
      leftSpeed = 0;
  };
  leftSpeed = leftSpeed * controlSpeed / 255;
  rightSpeed = rightSpeed * controlSpeed / 255;
  if(leftSpeed > 0){
    //Left Go Forward
    motorWrite(leftPWM, leftSpeed);
    digitalWrite(leftForward, HIGH);
    digitalWrite(leftReverse, LOW);
  }else{
    //Left Go Reverse
    motorWrite(leftPWM, -leftSpeed);
    digitalWrite(leftForward, LOW);
    digitalWrite(leftReverse, HIGH);
  }
  if(rightSpeed > 0){
    //Right Go Forward
    motorWrite(rightPWM, rightSpeed);
    digitalWrite(rightForward, HIGH);
    digitalWrite(rightReverse, LOW);
  }else{
    //Right Go Reverse
    motorWrite(rightPWM, -rightSpeed);
    digitalWrite(rightForward, LOW);
    digitalWrite(rightReverse, HIGH);
  }

  DEBUG_PRINT("INPUT SPEED: ");
  DEBUG_PRINT(controlSpeed);
  DEBUG_PRINT(" INPUT VECTOR: ");
  DEBUG_PRINT(controlVector);
  DEBUG_PRINT(" LEFT SPEED: ");
  DEBUG_PRINT(leftSpeed);
  DEBUG_PRINT(" RIGHT SPEED: ");
  DEBUG_PRINTLN(rightSpeed);
  DEBUG_PRINT("ENC LEFT SPEED: ");
  DEBUG_PRINT(leftEnc.read());
  DEBUG_PRINT(" ENC RIGHT SPEED: ");
  DEBUG_PRINTLN(rightEnc.read());
  
}
//This function returns MCU board's version
void returnVersion() {
  BT.println("Dain Jumbo V0.9");
  DEBUG_PRINTLN("Dain Jumbo V0.9");
}
//This function parses command received from BT and sends it to motorcontroller
void processMotor() {
  char *arg;
  int controlVector = 0;
  int controlSpeed = 0;
  int checkSum = 0;
  arg = parser.next();
  if (arg != NULL) {
    controlSpeed = atoi(arg);
  }
  arg = parser.next();
  if (arg != NULL) {
    controlVector = atol(arg);
  }
  arg = parser.next();
  if (arg != NULL) {
    checkSum = atol(arg);
  }
  if(checkSum == controlVector+controlSpeed-100){
    motorController(controlSpeed, controlVector);
    lastCommand = millis();
  }else{
    DEBUG_PRINTLN("INVALID");
  }
}
// general function to catch all unrecognized commands
void unrecognized(const char *command) {
  DEBUG_PRINTLN("UNRECOGNIZED COMMAND");
}

void setup() {
  pinMode(leftForward, OUTPUT);
  pinMode(leftReverse, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightReverse, OUTPUT);
  digitalWrite(leftForward, LOW);
  digitalWrite(leftReverse, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightReverse, LOW);
  #ifdef DEBUG
  Serial.begin(115200);
  #endif
  BT.begin("Dain's Jumbo"); //Bluetooth device name
  motorAttachPin(leftPWMPin, leftPWM);
  motorAttachPin(rightPWMPin, rightPWM);
  motorSetup(leftPWM, PWMFrequency, PWMResolution);
  motorSetup(rightPWM, PWMFrequency, PWMResolution);
  pinMode(standby, OUTPUT);
  digitalWrite(standby, HIGH);
  parser.addCommand("/V", returnVersion);        // Echos version information back
  parser.addCommand("/G", processMotor);         // Sends commands received from phone to processMotor function
  parser.setDefaultHandler(unrecognized);        // Handler for command that isn't matched
}

void loop() {
  parser.readSerial();
  if(millis()-lastCommand > 4000){
      motorController(0,0);         // Failsafe for motor controller. Will set speed to 0,0 when no command is received for 4s
  };
}