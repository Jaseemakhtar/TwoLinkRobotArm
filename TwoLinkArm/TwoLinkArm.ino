#include <Servo.h>
#include <math.h>

#define ARM_LENGTH 85
#define TOP_ARM_START_OFFSET 42  // the top arm is assembled in such a way that it cannot go below 42 degrees and that 42 degree is servos 0

Servo lowerArm;
Servo upperArm;

int sensorPinX = A0;   // can be used with 1st potentiometer to move on X axis
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

  lowerArm.attach(3);
  upperArm.attach(5);
}

void loop() {
  int x = (cos(angle) * radius) + xOffset;
  int y = (sin(angle) * radius) + yOffset;

  ArmPosition newPositions = getArmAngles(x, y);

  rotateServo(lowerArm, newPositions.bottom);
  rotateServo(upperArm, newPositions.top);
  
  angle = angle + angleIncrementer;

  if (angle >= COMPLETE_ROTATION) {
    angle = 0.0;
  }

  delay(8);
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

void rotateServo(Servo servo, int angle) {
  servo.write(angle);
}
