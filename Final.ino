#include <TimerOne.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <Servo.h>
#include <AS5600.h>  // I2C address is 0x36. Not changeable


#define PWMA 8               // blue
#define AIN1 7               // green
#define AIN2 6               // yellow
#define STBY 38              // green
#define PWMB 13              // blue
#define BIN1 12              // green
#define BIN2 11              // yellow
#define startButtonPin 26    // yellow, left button
#define nextButtonPin 24     // white, right button
#define limitSwitchDnPin 50  // green
#define limitSwitchFdPin 52  // yellow
#define servoPin 4           // orange

Servo myservo;
AS5600 encoderV, encoderH;

void TCA9548A(uint8_t bus) {  // Intiailising multiplexer to allow encoders of same I2C address
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

// Buttons
bool startButton, nextButton, limitSwitchDn, limitSwitchFd;


const int offsetA = -1;  //Used for switching motor direction convention
const int offsetB = 1;
const int zeroingSpeed = 200;

Motor motorV = Motor(AIN1, AIN2, PWMA, offsetA, STBY);  //  +PWM gives upwards movement and CW rotation from above
Motor motorH = Motor(BIN1, BIN2, PWMB, offsetB, STBY);  // +PWM gives backward movement and ACW rotation from in front

const int servoUp = 70;  // Argument is angle in degrees
const int servoDn = 130;



//Angle readings
float rawAngleV, rawAngleH;
float offsetAngleV, offsetAngleH;
float relAngleV, relAngleH;
const float sensor2Deg = 0.0879;  // Number of degrees per sensor state(4096)

//Converting angle to linear
const int angleConvert = 8;  // 8 mm per full rotation

// Displacement
int countRV, countRH;
int subCountRV, subCountRH;
float displacementV, displacementH;

//Error and PID
float prevAngleV, prevAngleH;
float prevDispV, prevDispH;

float errorV, errorH, prevErrorV, prevErrorH;

float setpointV, setpointH;

float integralV = 0, integralH = 0;

float derivativeV, derivativeH;

float KpV, KiV, KdV;
float KpH, KiH, KdH;

int outPIDV, outPIDH;

float setpointsV[3]{ 95, 425, 95 };  // Start, Up, Down
float gainPV[3]{ 150, 50, 50 };
float gainIV[3]{ 0.0000001, 0.0000001, 0.0000001 };
float gainDV[3]{ 0, 0, 0 };


float setpointsH[2]{ 180, 25 };  // Back, forward
float gainPH[2]{ 50, 50 };
float gainIH[2]{ 0.00001, 0.00001 };
float gainDH[2]{ 0, 0 };


const float deadband = 1;


//Time
unsigned long prevTime;
unsigned long time;
unsigned long dt;

// Staging

const int maxPress = 7;

int buttonPress;


void zeroing() {  // Zeroing routine

  limitSwitchDn = digitalRead(limitSwitchDnPin);
  limitSwitchFd = digitalRead(limitSwitchFdPin);

  while (digitalRead(limitSwitchFdPin)) {  //Zeroing horizontal with front switch
    motorH.drive(-150);
    limitSwitchFd = digitalRead(limitSwitchFdPin);
  }
  motorH.drive(0);


  countRH = 0;  // Setting displacement to 0

  TCA9548A(1);
  offsetAngleH = encoderH.rawAngle();
  calcRelAngleH();

  myservo.write(servoUp);

  while (limitSwitchDn == HIGH) {  //Zeroing vertical with bottom switch
    motorV.drive(-255);
    limitSwitchDn = digitalRead(limitSwitchDnPin);
  }
  motorV.drive(0);

  countRV = 0;  // Setting displacement to 0


  TCA9548A(2);
  offsetAngleV = encoderV.rawAngle();  // Reading angle to use as offset then calculating
  calcRelAngleV();
}



// **********START**********
// **********SETUP**********





void setup() {

  Serial.begin(9600);  // Initialising serial library
  Wire.begin();        // Initilising I2C library and multiplexed AS5600 encoders
  Wire.setClock(40000);

  // This function must be called with matching argument prior to each encoder function call. Argument refers to SDA SCL pair on multiplexing board
  TCA9548A(2);  // Vert
  encoderV.begin();
  TCA9548A(1);  // Hor
  encoderH.begin();


  myservo.attach(servoPin);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);


  digitalWrite(STBY, HIGH);
  /*Driving the motor down slowly until it meets a limit switch, then driving it upwards the necessary distance 
to meet the platform frame. This position will conincide with the bottom setpoint for PID
*/




  zeroing();

  myservo.write(servoUp);

  startButton = digitalRead(startButtonPin);
  while (!startButton) {
    delay(100);
    startButton = digitalRead(startButtonPin);
  }

  /* The actuator is now at its lowest point, it needs to move to a position to be ready
to carry the platform then waiting for an input.
*/
  buttonPress = 0;
}




// **********END************
// **********SETUP**********






int buttonCount() {
  nextButton = digitalRead(nextButtonPin);
  if (nextButton) {
    delay(500);
    buttonPress++;
  }
  if (buttonPress > maxPress) {
    buttonPress = 1;
  }
}






// **********START**********
// **********LOOP***********






void loop() {

  buttonCount();

  if (buttonPress == 0) {  // Starting position. MotorV rises platform frame
    KpV = gainPV[0];
    KiV = gainIV[0];
    KdV = gainDV[0];
    setpointV = setpointsV[0];


    mainFuncV();
    motorV.drive(outPIDV);
    delay(10);
    setPrev();
    motorH.drive(0);


    Serial.print("0displacementV:    ");
    Serial.print(displacementV);
    Serial.print("      0setpoint:     ");
    Serial.print(setpointV);
    time = millis();
    Serial.print("       0time:    ");
    Serial.println(time);
  }

  else if (buttonPress == 1) {  // Starting position. Servo rotates to lift flap and motorH retracts. Ready to obtain object.
    KpH = gainPH[0];
    KiH = gainIH[0];
    KdH = gainDH[0];
    setpointH = setpointsH[0];

    mainFuncH();
    motorH.drive(outPIDH);
    delay(10);

    setPrev();
    motorV.drive(0);


    Serial.print("1displacementH:    ");
    Serial.print(displacementH);
    Serial.print("      1setpoint:     ");
    Serial.print(setpointH);
    time = millis();
    Serial.print("       1time:    ");
    Serial.println(time);
  }


  else if (buttonPress == 2) {  // MotorH extends
    KpH = gainPH[1];
    KiH = gainIH[1];
    KdH = gainDH[1];
    setpointH = setpointsH[1];

    mainFuncH();
    motorH.drive(outPIDH);
    delay(10);
    setPrev();
    motorV.drive(0);


    Serial.print("2displacementH:    ");
    Serial.print(displacementH);
    Serial.print("      2setpoint:     ");
    Serial.print(setpointV);
    time = millis();
    Serial.print("       2time:    ");
    Serial.println(time);
  }

  else if (buttonPress == 3) {  // Servo rotates to meet floor
    myservo.write(servoDn);
    motorH.drive(0);
    motorV.drive(0);
    delay(10);
  }

  else if (buttonPress == 4) {  //MotorH retracts
    KpH = gainPH[0];
    KiH = gainIH[0];
    KdH = gainDH[0];
    setpointH = setpointsH[0];

    mainFuncH();
    motorH.drive(outPIDH);
    delay(10);
    setPrev();
    motorV.drive(0);


    Serial.print("4displacementH:    ");
    Serial.print(displacementH);
    Serial.print("      4setpoint:     ");
    Serial.print(setpointH);
    time = millis();
    Serial.print("       4time:    ");
    Serial.println(time);
  }

  else if (buttonPress == 5) {  // MotorH extends and servo rotates to lift flap

    myservo.write(servoUp);

    KpH = gainPH[1];
    KiH = gainIH[1];
    KdH = gainDH[1];
    setpointH = setpointsH[1];

    mainFuncH();
    motorH.drive(outPIDH);
    delay(10);
    setPrev();
    motorV.drive(0);

    Serial.print("7displacementV:    ");
    Serial.print(displacementV);
    Serial.print("      7setpoint:     ");
    Serial.print(setpointV);
    time = millis();
    Serial.print("       7time:    ");
    Serial.println(time);

  }

  else if (buttonPress == 6) {  // MotorV rises to top position
    KpV = gainPV[1];
    KiV = gainIV[1];
    KdV = gainDV[1];
    setpointV = setpointsV[1];

    mainFuncV();
    motorV.drive(outPIDV);
    delay(10);
    setPrev();
    motorH.drive(0);

    Serial.print("6displacementV:    ");
    Serial.print(displacementV);
    Serial.print("      6setpoint:     ");
    Serial.print(setpointV);
    time = millis();
    Serial.print("       6time:    ");
    Serial.println(time);

  }

  else if (buttonPress == maxPress) {  // MotorV lowers to bottom position
    KpV = gainPV[2];
    KiV = gainIV[2];
    KdV = gainDV[2];
    setpointV = setpointsV[2];

    mainFuncV();
    motorV.drive(outPIDV);
    delay(10);
    setPrev();
    motorH.drive(0);


    Serial.print("7displacementV:    ");
    Serial.print(displacementV);
    Serial.print("      7setpoint:     ");
    Serial.print(setpointV);
    time = millis();
    Serial.print("       7time:    ");
    Serial.println(time);
  }
}




// **********END***********
// **********LOOP**********












// **********FUNCTIONS**********
// *****************************













float calcRelAngleH() {
  TCA9548A(1);
  rawAngleH = encoderH.rawAngle();
  relAngleH = (rawAngleH - offsetAngleH) * sensor2Deg;  // Relative angle converted to degrees
  if (relAngleH < 0) {
    relAngleH += 360;
  }
  return relAngleH;
}

float calcRelAngleV() {
  TCA9548A(2);
  rawAngleV = encoderV.rawAngle();
  relAngleV = (rawAngleV - offsetAngleV) * sensor2Deg;
  if (relAngleV < 0) {
    relAngleV += 360;
  }
  return relAngleV;
}




void countRFunc() {
  if (prevAngleV > 270 && relAngleV < 90) {  // Zero crossing forwards
    countRV++;
  }
  if (prevAngleV < 90 && relAngleV > 270) {  // Backwards
    countRV--;
  }
  if (prevAngleH > 270 && relAngleH < 90) {  // Forwards
    countRH++;
  }
  if (prevAngleH < 90 && relAngleH > 270) {  // Backwards
    countRH--;
  }
}


void subCountRFunc() {
  subCountRV = map(relAngleV, 0, 360, 0, 8);  // Mapping a less <360 degrees rotation to 0 - 8 (mm). Giving the distance travelled when a full rotation does not occur
  subCountRH = map(relAngleH, 0, 360, 0, 8);
}


float calcDisplacement() {
  displacementV = countRV * angleConvert + subCountRV;
  displacementH = countRH * angleConvert + subCountRH;
}
void setPrevAngle() {
  prevAngleV = relAngleV;
  prevAngleH = relAngleH;
}

void setPrevDisp() {
  prevDispV = displacementV;
  prevDispH = displacementH;
}

void error() {
  errorV = setpointV - displacementV;
  errorH = setpointH - displacementH;
}

void integral() {
  integralV += errorV * dt;
  integralH += errorH * dt;
}

void derivative() {
  derivativeV = (errorV - prevErrorV) / dt;
  derivativeH = (errorH - prevErrorH) / dt;
}







void calcPID() {

  outPIDV = KpV * errorV + KiV * integralV + KdV * derivativeV;
  outPIDH = KpH * errorH + KiH * integralV + KdH * derivativeV;

  if (abs(errorV) < deadband) {
    outPIDV = 0;
  }
  if (abs(errorH) < deadband) {
    outPIDH = 0;
  }

  outPIDV = constrain(outPIDV, -255, 255);
  outPIDH = constrain(outPIDH, -255, 255);
}


void mainFuncV() {
  calcRelAngleV();
  time = millis();
  dt = time - prevTime;  // To sample at 4 times per revolution at max speed, dt must be below 83 ms
  countRFunc();
  subCountRFunc();
  calcDisplacement();
  error();
  integral();
  derivative();
  calcPID();
}


void mainFuncH() {
  calcRelAngleH();
  time = millis();
  dt = time - prevTime;  // To sample at 4 times per revolution at max speed, dt must be below 83 ms
  countRFunc();
  subCountRFunc();
  calcDisplacement();
  error();
  integral();
  derivative();
  calcPID();
}


void setPrev() {
  setPrevAngle();
  setPrevDisp();
  prevTime = time;
}