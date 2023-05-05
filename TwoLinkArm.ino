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

#define SERVOMIN  70 // This is the 'minimum' pulse length count (out of 4096) // 70
#define SERVOMAX  520 // This is the 'maximum' pulse length count (out of 4096) // 520

#define USMIN  350 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2200 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define ARM_LENGTH 85
#define TOP_ARM_START_OFFSET 42  // servo 2 restrict below angle 42 deg

#define LOWER_ARM 0 // Bottom servo
#define UPPER_ARM 1 // Top servo

int sensorPinX = A0;   // can be used with 2nd potentiometer to move on Y axis
int sensorXValue = 0;  

int sensorPinY = A1;   // can be used with 2nd potentiometer to move on Y axis
int sensorYValue = 0;  

double angle = 0.0;
double angleIncrementer = 0.08;
double COMPLETE_ROTATION = 2.0 * PI;
int radius = 36;
int xOffset = 80;
int yOffset = 70;

int previousServoPositions[2] = { 0, 0 };

struct ArmPosition {
  int bottom;
  int top;
};

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

void loop() {
  int x = (cos(angle) * radius) + xOffset;
  int y = (sin(angle) * radius) + yOffset;

  ArmPosition newPositions = getArmAngles(x, y);

  rotateServo(LOWER_ARM, newPositions.bottom);
  rotateServo(UPPER_ARM, newPositions.top);
  
  angle = angle + angleIncrementer;

  if (angle >= COMPLETE_ROTATION) {
    angle = 0.0;
  }

  delay(10);
}

ArmPosition getArmAngles(int x, int y) {
  ArmPosition servoPositions;

  double alpha = toDegrees(atan2(y, x));
  double h = sqrt((x * x) + (y * y));

  double beta = toDegrees(acos( (h / 2) / ARM_LENGTH));

  double gamma = 180 - (beta + 90);

  int theta1 = alpha + beta;
  servoPositions.bottom = theta1;

  int theta2 = gamma * 2;
  servoPositions.top = theta2 - TOP_ARM_START_OFFSET;

  return servoPositions;
}

double toDegrees(double rad) {
  return rad * (180 / PI);
}

void rotateServo(int servo, int angle) {
  int mapped = map(angle, 0, 180, USMIN, USMAX);
  pwm.writeMicroseconds(servo, mapped);
}
