#include <L298N.h>
//left motor
#define leftEN 11
#define leftin1 7  
#define leftin2 6
//right motor
#define rightEN 9
#define rightin1 5
#define rightin2 4

double A;
double B;
double C;
double D;
//
//L298N leftMotor (leftEN, leftin1, leftin2);
//L298N rightMotor (rightEN, rightin1, rightin2);


//analogRead(A39)

double equation(double A, double B, double C, double D, double x, double y)
{
   return x*(A+(B-A)*y)+(1-x)*(D+(C-D)*y);
}

void setup() {
 Serial.begin(9600);
}

void loop() {
  double left = analogRead(A5)/4.011; //left = 255 right = 0
  double right = analogRead(A4)/4.011; //up = 255 down = 0
  Serial.print("left = ");
  Serial.print(left);
  Serial.print(" right = ");
  Serial.print(right);
  Serial.println("");
if(abs(left - 0.5) < 0.2 && (abs(right-0.5) < 0.2))  //      dead zone
{
  Serial.println(" dead ");
  return;
}
double toLeft;
double toRight;
//  if(x >= 0.5 && y>= 0.5)
//  {
//    //top right corner
//     toLeft = equation(255.0, -255.0, -255.0, 20.0, x, y);
//     toRight = equation(255.0, 128.0, -255.0, 20.0, x, y); 
//     Serial.print(" quad: 1 ");
//  }
//   else if(x < 0.5 && y>= 0.5)
//  {
//    Serial.print(" quad: 4 ");
//    //bottom right corner
//     toLeft = equation(20, 255, -128, -255, x, y);
//     toRight = equation(20,-255,-255, -255, x, y);  
//  }
//  else if(x < 0.5 && y < 0.5)
//  {
//    Serial.print(" quad: 3 ");
//    //bottom left corner
//     toLeft = equation(-255, 20, -255, -255, x, y);
//     toRight = equation(255, 20, -255, -128, x, y);   
//  }
//  else
//  {
//    Serial.print(" quad: 2 ");
//     toLeft = equation(128, 255, 20, -255, x, y);
//     toRight = equation(255,255, 20, 255, x, y);  
//  }
 
//  Serial.print("toRight = ");
//  Serial.print(toRight);
//  Serial.print(" toLeft ");
//  Serial.println(toLeft);
  
  //leftMotor.forward();
  //rightMotor.forward();
}
