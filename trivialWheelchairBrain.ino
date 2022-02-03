#include <SoftwareSerial.h>

const int COMRX = 3;
const int COMTX = 2;

SoftwareSerial foreBrainComm(COMRX, COMTX);

void wakeWheelchair()
{
  foreBrainComm.write('w');
  foreBrainComm.write('a');
  foreBrainComm.write('k');
  foreBrainComm.write('e');
  //Serial.println("Woken");
}

void setMotorSpeeds(int left, int right)
{
  char m1 = (char)(left);
  char m2 = (char)(right);
  
  foreBrainComm.write('m');
  foreBrainComm.write(m1);
  foreBrainComm.write(m2);
  foreBrainComm.write('x');  
}

void setup()
{
  Serial.begin(9600);

  pinMode(COMRX, INPUT);
  pinMode(COMTX, OUTPUT);

  foreBrainComm.begin(9600);
  
  delay(1000); //Allow Serial communication to begin
  wakeWheelchair();
}

void go(int ms, int leftMotor, int rightMotor)
{
    while (ms > 0) {
        setMotorSpeeds(-30,30);
        delay(100);
        ms -= 100;
    }
}

void loop()
{
    go(2000, 20, 20);
    
    while (true) {
      setMotorSpeeds(0, 0);  
      delay(200);
    }
}
