#include <Sparki.h>

#define LOOP_TIME         100
#define AXLE_DIST         0.08586
#define RAD               0.025
#define VEL               0.0278551532
#define LOOP_TIME_O       100
#define VELOCITY          0.0278551532

void setup() {
  Serial1.begin(9600);
  sparki.servo(SERVO_CENTER);
}

void loop() {
  sparki.RGB(0, 0, 100);
  if (Serial1.available()) {

    char inChar = (char) Serial1.read();

    if (inChar == 'L') {

      sparki.servo(SERVO_LEFT);
      Serial1.println("SERVO_LEFT");
      return;

    } else if (inChar == 'R') {

      sparki.servo(SERVO_RIGHT);
      Serial1.println("SERVO_RIGHT");
      return;

    } else if (inChar == 'C') {

      sparki.servo(SERVO_CENTER);
      Serial1.println("SERVO_CENTER");
      return;

    } else if (inChar == 'W') {

      sparki.moveForward(5);
      Serial1.println("MOVE_FORWARD");
      return;

    } else if (inChar == 'S') {

      sparki.moveBackward(5);
      Serial1.println("MOVE_BACKWARD");
      return;

    } else if (inChar == 'X') {

      Serial1.println("STOP");
      sparki.moveStop();
      sparki.gripperStop();
      return;

    } else if (inChar == 'I') {

      Serial1.println("INVERSE_KINEMATICS");
      inverseKinematics(0.1, 0.1);
      return;

    } else if (inChar == 'O') {

      Serial1.println("ODOMETRY");
      odometry();
      return;

    } else if (inChar == 'B') {
      Serial1.println("REACTIVE_BEHAIVIORS");
      reactiveBehaviors();
      return;

    } else if (inChar == 'A') {

      sparki.moveLeft(15);
      Serial1.println("MOVE_LEFT");
      return;

     } else if (inChar == 'D') {

      sparki.moveRight(15);
      Serial1.println("MOVE_RIGHT");
      return;

    } else if (inChar == ']') {

      sparki.gripperOpen();
      Serial1.println("gripperOpen");
      return;

    } else if (inChar == '[') {

      sparki.gripperClose();
      Serial1.println("gripperClose");
      return;

    } else {
      return;
    }
  }
}

int inverseKinematics(float xG, float yG) {
  float xI                = 0.0;
  float yI                = 0.0;
  float thetaR            = 0.0;

  // float xG                = 0.15; // 15cm
  // float yG                = 0.15;
  float thetaG            = 0.0;

  float n                 = 0.0;
  float rho               = 0.0;
  float alpha             = 0.0;

  float xR                = 0.0;
  float thetaRP           = 0.0;

  float rVel;
  float lVel;
  float avgVel            = VEL;

  unsigned long int startTime;
 // sparki.clearLCD();

  startTime = millis();

  // Translate into real-world coordinate frame
  int xPoint = (xI / 10) + 50;
  int yPoint = (yI / 10) + 50;

  // Map out the path of Sparki
  sparki.drawPixel(xPoint, yPoint);
  //   sparki.updateLCD();

  // *******************************************************
  // * Feedback Controller

  rho = sqrt(pow((xI - xG), 2) + pow((yI - yG), 2));
  alpha = thetaR - atan2((yI - yG), (xI - xG)) - PI/2.0;
  n = thetaG - thetaR;

  Serial1.print("rho: "); Serial1.print(rho);
  Serial1.print(" alpha: "); Serial1.print(alpha / PI * 180.0);
  Serial1.print(" n: "); Serial1.println(n); Serial1.println("- - - -");

  // Forward speed
  xR = 0.1 * rho;

  // Rotational speed
  thetaRP = 0.1 * (alpha); // + 0.01 * n;

  // *******************************************************
  // * Inverse Kinematics [Formula 3.64]

  lVel = (2.0 * xR / RAD - thetaRP * AXLE_DIST / RAD) / 2.0;
  rVel = (2.0 * xR / RAD + thetaRP * AXLE_DIST / RAD) / 2.0;

  // Serial1.print("L: " + lVel + " "); Serial1.print("R: " + rVel);

  sparki.motorRotate(MOTOR_LEFT, DIR_CCW, lVel / (VEL / RAD) * 100.0);
  sparki.motorRotate(MOTOR_RIGHT, DIR_CW, rVel / (VEL / RAD) * 100.0);

  // *******************************************************
  // * Odometry [Formula 3.40]

  // Average rotation speeds of the L & R wheels (phi * radius)
  avgVel = (rVel * RAD + lVel * RAD) * 0.5;

  xI += cos(thetaR) * avgVel * LOOP_TIME / 1000.0;
  yI += sin(thetaR) * avgVel * LOOP_TIME / 1000.0;
  thetaR += (rVel * RAD - lVel * RAD) / AXLE_DIST * LOOP_TIME / 1000.0;

  Serial1.print("xI: "); Serial1.print(xI);
  Serial1.print(" yI: "); Serial1.print(yI);
  Serial1.print(" thetaR: "); Serial1.println(thetaR/PI * 180.0);

  // Ensure every loop is exactly 100 ms
  // while (millis() < startTime + LOOP_TIME) {}

  return 1;
}

static inline float* odometry() {
  float x                 = 0.0;
  float y                 = 0.0;
  float theta             = 0.0;

  float rVel              = 0.0;
  float lVel              = 0.0;
  float avgVel            = 0.0;
  float coords[2];

  int startTime, endTime;

  startTime = millis();
  // Average rotation speeds of the L & R wheels
  avgVel = (rVel + lVel) * 0.5;

  // Formula 3.40
  x += cos(theta) * avgVel * LOOP_TIME_O;
  y += sin(theta) * avgVel * LOOP_TIME_O;
  theta += (rVel - lVel) / AXLE_DIST * LOOP_TIME_O;

  // Translate into real-world coordinate frame
  int xPoint = (x / 10) + 50;
  int yPoint = (y / 10) + 50;

  // Map out the path of Sparki
  // sparki.drawPixel(xPoint, yPoint);
  // sparki.updateLCD();

  endTime = millis();

  Serial1.print("X: ");
  Serial1.println(x / 10);
  Serial1.print("Y: ");
  Serial1.println(y / 10);

  Serial1.print("Theta: ");
  Serial1.println(theta);

  Serial1.print("Time: ");
  Serial1.println(LOOP_TIME - (endTime - startTime));
  Serial1.println("- - - - - - - - - - - - - - -");

  // Ensure every loop is 100ms  --see if we need to delete this
  if (LOOP_TIME - (endTime - startTime) > 0) {
    delay(LOOP_TIME - (endTime - startTime));
  }

  coords[0] = x;
  coords[1] = y;
  Serial1.print(x, y);
  return coords;
}


static inline int reactiveBehaviors(){
//   Move forward
  int cm = sparki.ping();
  sparki.moveForward();
  sparki.print(cm);
  if(cm == 0.3){
//     Turn around
    delay(1000);
    sparki.moveRight(180);
    sparki.moveForward();
  }

}
