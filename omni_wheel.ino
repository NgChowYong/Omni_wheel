#include "parameter.h"
#include <PS3BT.h>
#include <usbhub.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

#include <SPI.h>

/*
*****************
**Initial setup**
*****************
*/
USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside

BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
PS3BT PS3(&Btd); // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

bool printTemperature, printAngle;
double v1, v2, v3;
double x, y, r, ang;//ang 目前角度
double ratio1;
int cases;
double ang1 = 30;//angle of tire motor 1 is 30 deg
double ang2 = 270;
double ang3 = 150;
int dir_1, dir_2, dir_3;
int* dir_1_p;
int* dir_2_p;
int* dir_3_p;

void setup() {

  /*
  ***********************
  **Initialize pin mode**
  ***********************
  */
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorIn3, OUTPUT);
  pinMode(motorIn4, OUTPUT);

  //for motor1
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);

  //for motor2
  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);

  //for motor3
  pinMode(MOTOR_A3_PIN, OUTPUT);
  pinMode(MOTOR_B3_PIN, OUTPUT);

  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);
  pinMode(PWM_MOTOR_3, OUTPUT);

  pinMode(EN_PIN_1, OUTPUT);
  digitalWrite(EN_PIN_1, HIGH);
  Serial.begin(115200);

#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }


#ifdef DEBUG
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
#endif
}


void loop() {

  /*
  ************************************
  **looping to take controller input**
  ************************************
  */
  Usb.Task();
  // get data from controller
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    double a, b, c;
    a = PS3.getAnalogHat(LeftHatX) ;
    b = PS3.getAnalogHat(LeftHatY) ;
    c = PS3.getAnalogHat(RightHatX);

    // threshold for controller to undesired avoid motion
    if (a > 137 || a < 117 || b > 137 || b < 117 || c > 137 || c < 117) {
      a = a - 127;
      b = b - 127;
      c = c - 127;
      //print
      omni_motor(a , b, c);
      motorGo( MOTOR_1, dir_1, v1) ;
      motorGo( MOTOR_2, dir_2, v2) ;
      motorGo( MOTOR_3, dir_3, v3) ;

    }
    else if (PS3.getButtonClick(TRIANGLE)) {

#ifdef DEBUG
      Serial.print(F("\r\nUp"));
#endif

      upward();
      delay(1000);
      anglestop();
    }
    else {
      // stop the motor
      motorGo( MOTOR_1, 2, 255) ;
      motorGo( MOTOR_2, 2, 255) ;
      motorGo( MOTOR_3, 2, 255) ;
    }
  }
}

// calc diff value of desire and actual value
double set_vel_dir(double angle_motor, double angle_desire, double vel_desire, double vel_result, double rotate, int* dir) {
  double temp = angle_motor - angle_desire;//angular difference (difference between the wheel and the desire position)
  vel_result = vel_desire * sin(temp / 180 * 3.14159265359);//motor velocity = component of desire velocity
  vel_result = vel_result + 0.6 * rotate;

  if (vel_result >= 0) {
    *dir = 1;
  } else {
    *dir = 0;
  }

  double temp1 = abs(vel_result);
  return temp1;
}

//Function that controls the variables: motor(0 ou 1), direction (cw ou ccw) e pwm (entra 0 e 255);
void motorGo(uint8_t motor, uint8_t dir, double ang_vel)
{
  if (motor == MOTOR_1)
  {
    if (dir == 1) //it basically controls the direction of the motor (view word doc "learning") ->CW
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if (dir == 0) //->CCW
    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);
    }
    else if (dir == 2) //->STOP
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);
    }
    analogWrite(PWM_MOTOR_1, ang_vel); //   (range from 0~255 pwm value) control speed
  }
  if (motor == MOTOR_2)
  {
    if (dir == 1)
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, HIGH);
    }
    else if (dir == 0) //->CCW
    {
      digitalWrite(MOTOR_A2_PIN, HIGH);
      digitalWrite(MOTOR_B2_PIN, LOW);
    }
    else if (dir == 2) //->STOP
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, LOW);
    }
    analogWrite(PWM_MOTOR_2, ang_vel); //   (range from 0~255 pwm value) control speed
  }
  if (motor == MOTOR_3)
  {
    if (dir == 1)
    {
      digitalWrite(MOTOR_A3_PIN, LOW);
      digitalWrite(MOTOR_B3_PIN, HIGH);
    }
    else if (dir == 0) //->CCW
    {
      digitalWrite(MOTOR_A3_PIN, HIGH);
      digitalWrite(MOTOR_B3_PIN, LOW);
    }
    else if (dir == 2) //->STOP
    {
      digitalWrite(MOTOR_A3_PIN, LOW);
      digitalWrite(MOTOR_B3_PIN, LOW);
    }
    analogWrite(PWM_MOTOR_3, ang_vel); //   (range from 0~255 pwm value) control speed
  }
}

void omni_motor(double x, double y, double rotate) {// rotate need posi neg
  //calculation from cartesian to vector
  x = -x;
  y = -y;
  rotate = -rotate;
  r = sqrt (x * x + y * y);
  ang = atan (y / x) * 180 / 3.14159265359; // arc tangent of y/x

  //settle coordination angle problem
  if (x != 0) {
    if (x > 0 && y < 0) { //add new
      ang = 360 + ang;
    } else {//x<0
      ang = 180 + ang;
    }
  } else {
    if (y >= 0) {
      ang = 90;
    } else if (y < 0) {
      ang = 270;
    }
  }

  //setup vel and direction
  dir_1_p = &dir_1;
  dir_2_p = &dir_2;
  dir_3_p = &dir_3;

  v1 = set_vel_dir(ang1, ang, r, v1, rotate, dir_1_p);
  v2 = set_vel_dir(ang2, ang, r, v2, rotate, dir_2_p);
  v3 = set_vel_dir(ang3, ang, r, v3, rotate, dir_3_p);
  
  //8 kind of cases for correction since v1,2,3 could over 255
  if (v1 == 0) {
    v1 = 1;
  }
  if (v2 == 0) {
    v2 = 1;
  }
  if (v3 == 0) {
    v3 = 1;
  }

  int no = 127;
  if (v1 < no && v2 < no && v3 < no ) {
    // case = 1;// no change
  }  else if (v1 < no && v2 < no && v3 > no ) {
    // case = 2;
    ratio1 = no / v3;
    v1 = v1 * ratio1;
    v2 = v2 * ratio1;
    v3 = no;
  }  else if (v1 < no && v2 > no && v3 < no ) {
    // case = 3;
    ratio1 = no / v2;
    v1 = v1 * ratio1;
    v2 = no;
    v3 = v3 * ratio1;
  }  else if (v1 < no && v2 > no && v3 > no ) {
    // case = 4;
    if (v3 > v2) {
      ratio1 = no / v3;
      v1 = v1 * ratio1;
      v2 = v2 * ratio1;
      v3 = no;

    } else if (v2 > v3) {
      ratio1 = no / v2;
      v1 = v1 * ratio1;
      v2 = no;
      v3 = v3 * ratio1;

    }
  }  else if (v1 > no && v2 < no && v3 < no ) {
    // case = 5;
    ratio1 = no / v1;
    v1 = no;
    v2 = v2 * ratio1;
    v3 = v3 * ratio1;

  }  else if (v1 > no && v2 < no && v3 > no ) {
    // case = 6;
    if (v3 > v1) {
      ratio1 = no / v3;
      v1 = v1 * ratio1;
      v2 = v2 * ratio1;
      v3 = no;
    } else if (v1 > v3) {
      ratio1 = no / v1;
      v1 = no;
      v2 = v2 * ratio1;
      v3 = v3 * ratio1;
    }
  }  else if (v1 > no && v2 > no && v3 < no ) {
    // case = 7;
    if (v1 > v2) {
      ratio1 = no / v1;
      v1 = no;
      v2 = v2 * ratio1;
      v3 = v3 * no;
    } else if (v2 > v1) {
      ratio1 = no / v2;
      v1 = v1 * ratio1;
      v2 = no;
      v3 = v3 * ratio1;
    }
  }  else if (v1 > no && v2 > no && v3 > no ) {
    // case = 8;
    if (v3 > v2 && v3 > v1) {
      ratio1 = no / v3;
      v1 = v1 * ratio1;
      v2 = v2 * ratio1;
      v3 = no;

    } else if (v2 > v3 && v2 > v1) {
      ratio1 = no / v2;
      v1 = v1 * ratio1;
      v2 = no;
      v3 = v3 * ratio1;
    } else {
      ratio1 = no / v1;
      v1 = no;
      v2 = v2 * ratio1;
      v3 = v3 * ratio1;
    }
  }

  v1 *= 2;
  v2 *= 2;
  v3 *= 2;

  int no2 = 10;
  if (v1 > 127 - no2 && v1 < 128) {
    v1 = 128;
  }
  if (v2 > 127 - no2 && v2 < 128) {
    v2 = 128;
  }
  if (v3 > 127 - no2 && v3 < 128) {
    v3 = 128;
  }

#ifdef DEBUG
  Serial.print("vel dir 1:");
  Serial.print(v1);  Serial.print("  ");  Serial.println(dir_1);
  Serial.print("vel dir 2:");
  Serial.print(v2);  Serial.print("  ");  Serial.println(dir_2);
  Serial.print("vel dir 3:");
  Serial.print(v3);  Serial.print("  ");  Serial.println(dir_3);
#endif

  // input to motor
  motorGo( MOTOR_1, dir_1, v1) ;
  motorGo( MOTOR_2, dir_2, v2) ;
  motorGo( MOTOR_3, dir_3, v3) ;

}
void anglestop()
{
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motorIn3, LOW);
  digitalWrite(motorIn4, LOW);
}
void upward()
{
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
}
