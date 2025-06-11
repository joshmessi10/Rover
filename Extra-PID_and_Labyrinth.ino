#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;

double Kp = 1;  //double Kp = 60; 
double Kd = 2;  //double Kd = 2.2;  
double Ki = 250;  //double Ki = 270;  
// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID
double PuntoEquilibrio = 179.5;                                     
double originalSetpoint = PuntoEquilibrio;   //double originalSetpoint = 172.50;

double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

 
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
 mpuInterrupt = true;
}
#include <Servo.h>

#include <AFMotor.h>


AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

//Bit positions in the Arduino UNO micro controller output
Servo myservo;  // create servo object to control a servo
const int servoPin = 9;
int pos = 0;  // variable to store the servo position

#define trigPin 10
#define echoPin 13
//define sound speed in cm/uS
#define SOUND_SPEED 0.034
char cmd = 'N';
long duration;
float distanceCm;
int walls[3] = {0,0,0}; //Distances from Right, Front and Left Walls.

long readDistance(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distanceCm = duration * SOUND_SPEED/2;
  return distanceCm;
}
void servoLeft(){
  myservo.write(180);
  delay(50);
}
void servoFront(){
  myservo.write(90);
  delay(50);
}
void servoRight(){
  myservo.write(0);
  delay(50);
}
void decision(){
  servoRight();
  walls[0] = readDistance();
  servoFront();
  walls[1] = readDistance();
  servoLeft();
  walls[2] = readDistance();
  //AlgoritmoDeDecision
  if(walls[0] < 30 && walls[1] > 45){ //Front Way 
    forward();
    delay(1000);
    mstop();
  }
  else if(walls[0] > 30){ //Right Way
    right();
    delay(1000);
    backward();
    delay(1000);
    mstop();
  } 
  else if(walls[0] < 30 && walls[1] < 30 && walls[2] > 30){
    left();
    delay(1000);
    backward();
    delay(1000);
    mstop();
  }
  else if(walls[0] < 30 && walls[1] < 30 && walls[2] < 30){
    right();
    delay(1000);
    backward();
    delay(1000);
    right();
    delay(1000);
    backward();
    delay(1000);
    mstop();
  }
  else if (walls[1] < 5){
    backward();
    delay(1000);
    mstop();
  }
}

void setup() {
  // put your setup code here, to run once:
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif

  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
  // turn on the DMP, now that it's ready
  mpu.setDMPEnabled(true);

  // enable Arduino interrupt detection
  attachInterrupt(0, dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();

  // set our DMP Ready flag so the main loop() function knows it's okay to use it
  dmpReady = true;

  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  
  //setup PID
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255); 
  }
  else
  {
  // ERROR!
  // 1 = initial memory load failed
  // 2 = DMP configuration updates failed
  // (if it's going to break, usually the code will be 1)
    return;
  }
  mstop();
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  myservo.attach(servoPin);
}

void loop() {
  forward();

}
void forward(){
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
  //no mpu data - performing PID calculations and output to motors 
  pid.Compute();
  
  
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
  // reset so we can continue cleanly
  mpu.resetFIFO();
  //Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  
  // track FIFO count here in case there is > 1 packet available
  // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  input = ypr[1] * 180/M_PI + 180;
  //pid.Compute();
  if(input < 178){
    output = output - input;
  }
  if(input > 200){
    output = output + input;
  }
  double speed = 100 + output;
  if(speed > 255){
    speed = 255;
  }
  motor1.setSpeed(speed);
  motor1.run(BACKWARD);
  motor2.setSpeed(speed);
  motor2.run(FORWARD);
  motor3.setSpeed(speed);
  motor3.run(FORWARD);
  motor4.setSpeed(speed);
  motor4.run(FORWARD);
  }
}
void autopilot(){
  while(Serial.read() != 'N'){
    decision();
  }
    
}

void backward(){
  motor1.setSpeed(140);
  motor1.run(FORWARD);
  motor2.setSpeed(140);
  motor2.run(BACKWARD);
  motor3.setSpeed(140);
  motor3.run(BACKWARD);
  motor4.setSpeed(140);
  motor4.run(BACKWARD);
  }
void left(){
  motor1.setSpeed(255);
  motor1.run(BACKWARD);
  //motor2.setSpeed(255);
  motor2.run(RELEASE);
  motor3.setSpeed(255);
  motor3.run(FORWARD);
  motor4.setSpeed(255);
  motor4.run(FORWARD);
  }
void right(){
  //motor1.setSpeed(255);
  motor1.run(RELEASE);
  motor2.setSpeed(255);
  motor2.run(FORWARD);
  motor3.setSpeed(255);
  motor3.run(FORWARD);
  motor4.setSpeed(255);
  motor4.run(FORWARD);
  }
void mstop(){
   motor1.run(RELEASE);
   motor2.run(RELEASE);
   motor3.run(RELEASE);
   motor4.run(RELEASE);
  } 

