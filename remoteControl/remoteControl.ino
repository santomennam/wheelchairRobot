const int estop1 = 6; // OUTPUT LOW
const int estop2 = 7; // pulled up to High.  These pins are connected together with a NC switch    HIGH means STOP



void setup() {
 Serial.begin(115200);
 delay(100);
 pinMode(estop1, OUTPUT);
 digitalWrite(estop1, LOW);
 pinMode(estop2, INPUT_PULLUP);
}

void loop() {
      while(digitalRead(estop2) == HIGH)
      {
        Serial.println("estop");
      }
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
