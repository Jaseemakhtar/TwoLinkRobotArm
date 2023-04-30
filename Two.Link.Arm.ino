/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!

// #define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096) // 70
// #define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096) // 520

#define SERVOMIN  70 // This is the 'minimum' pulse length count (out of 4096) // 70
#define SERVOMAX  520 // This is the 'maximum' pulse length count (out of 4096) // 520
#define SERVOMAX_180_OFFSET 75 // 445 as 180

#define USMIN  350 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2200 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define ARM_LENGTH 85
#define TOP_ARM_START_ANGLE 42  // servo 2 restrict below angle 42 deg
#define MIN_Z_LENGTH 50 // 70mm
#define MAX_Z_LENGTH 150 // 150mm

int sensorPin = A0;   // select the input pin for the potentiometer
int sensorValue = 1;  // variable to store the value coming from the sensor

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

int zLength = 74;
int servonum = 1;

void loop() {
  // Drive each servo one at a time using writeMicroseconds(), it's not precise due to calculation rounding!
  // The writeMicroseconds() function is used to mimic the Arduino Servo library writeMicroseconds() behavior. 
  // pwm.writeMicroseconds(0, 1200);

  // for (uint16_t microsec = USMIN; microsec < USMAX; microsec++) {
  //   pwm.writeMicroseconds(servonum, microsec);
  // }

  // delay(500);
  // for (uint16_t microsec = USMAX; microsec > USMIN; microsec--) {
  //   pwm.writeMicroseconds(servonum, microsec);
  // }

  
  for(int i = MIN_Z_LENGTH; i <= MAX_Z_LENGTH; i++) {
    updateZ(i);
  }

  delay(700);

  for(int i = MAX_Z_LENGTH; i >= MIN_Z_LENGTH; i--) {
    updateZ(i);
  }

  delay(700);
}

void testServoRotate() {
  rotateServo(1, 0);
  delay(1500);
  rotateServo(1, 90);
  delay(1500);
  rotateServo(1, 180);
}

// void rotateServo(int servo, int angle) {
//   // int angleOffsetMap = map(angle, 0, 180, 0, 150); // the servos semi cirlce rotation seems from 0 to 150 :(
//   int mapped = map(angle, 0, 180, SERVOMIN, SERVOMAX - SERVOMAX_180_OFFSET);
//   pwm.setPWM(servo, 0, mapped);
// }

void rotateServo(int servo, int angle) {
  // int angleOffsetMap = map(angle, 0, 180, 0, 150); // the servos semi cirlce rotation seems from 0 to 150 :(
  int mapped = map(angle, 0, 180, USMIN, USMAX);
  pwm.writeMicroseconds(servo, mapped);
}

void updateZ(int distance) {
  int a1 = ARM_LENGTH;
  int b1 = ARM_LENGTH;
  int c1 = distance;
  int bAngle = getCosineAngle(a1, b1, c1) - TOP_ARM_START_ANGLE;

  int a0 = ARM_LENGTH;
  int b0 = distance;
  int c0 = ARM_LENGTH;
  int aAngle = getCosineAngle(a0, b0, c0) + 90;

  Serial.println(distance);
  Serial.print("B => "); Serial.print(bAngle); Serial.print(" A => "); Serial.println(aAngle); 
  rotateServo(0, aAngle);
  rotateServo(1, bAngle);
}

int getCosineAngle(int a, int b, int c) {
  double armsRatio = ((c * c) - ((a * a) + (b * b))) / (double) (-2 * a * b);
  double topArmAngleRad = acos(armsRatio);
  int topArmAngleDegree = (topArmAngleRad * 4068) / 71;
  
  return topArmAngleDegree;
}
