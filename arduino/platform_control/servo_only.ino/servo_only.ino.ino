#include <Servo.h>
Servo servo1,servo2,servo3,servo4,servo5,servo6;
String inputString;
void setup() {
  // put your setup code here, to run once:
  servo1.attach(12);
  servo2.attach(11);
  servo3.attach(10);
  servo4.attach(9);
  servo5.attach(8);
  servo6.attach(7);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available())
  {
  
    char inChar = (char)Serial.read();
    inputString += inChar;
    if(inChar == '\n')
    {
      handle_incoming(inputString);
      //Serial.println(inputString);
      inputString = "";
    }
  }
}


void handle_incoming(String inputString)
{
  char receivedChars[inputString.length()];
  for(int i=0;i<inputString.length();i++){
    receivedChars[i] = inputString[i];
  }
  
  char * strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(receivedChars,",");      // get the first part - the string
  float sservo1= atoi(strtokIndx); // copy it to messageFromPC
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  float sservo2 = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  float sservo3 = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  float sservo4 = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  float sservo5 = atoi(strtokIndx);
  strtokIndx = strtok(NULL, "\n"); // this continues where the previous call left off
  float sservo6 = atoi(strtokIndx); 
  //Serial.println(sservo1);
  //Serial.println(sservo2);
  //Serial.println(sservo3);
  //Serial.println(sservo4);
  //Serial.println(sservo5);
  //Serial.println(sservo6);

  //phi is the "tool angle" 
  goto_angle(sservo1,sservo2,sservo3,sservo4,sservo5,sservo6);
}

void goto_angle(float sservo1,float sservo2,float sservo3,float sservo4,float sservo5,float sservo6)
{
  //etSpeedForAllServos(300);
    servo1.write(180-sservo1);
    delay(2);
    servo2.write(sservo2);
    delay(2);
    servo3.write(180-sservo3);
     delay(2);
    servo4.write(sservo4);
     delay(2);
    servo5.write(180-sservo5);
     //delay(2);
    servo6.write(sservo6);
     delay(2);
    //synchronizeAllServosStartAndWaitForAllServosToStop();
}
