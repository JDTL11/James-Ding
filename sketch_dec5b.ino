#include<SoftwareSerial.h>


// Motor Declaration
int in1 = 5;
int in2 = 6;
int in3 = 9;
int in4 = 10;


int leftLed = 2;
int rightLed = 7;

SoftwareSerial mySerial(0,1);
char data;

void setup() {

  mySerial.begin(9600);
  Serial.begin(9600);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(leftLed, OUTPUT);
  pinMode(rightLed, OUTPUT);

}

void loop() {

  

  switch (data){

    case 'F':
      forward();
      break;

    case 'G':
      backward();
      break;

    case 'L':
      turnLeft();
      break;

    case 'R':
      turnRight();
      break;

    case 'S':
      Stop();
      break;

    case 'X':
      leftSignal();
      break;
  }


}




void forward(){  //forword
  analogWrite(in1, 200); //Right Motor forword Pin 
  analogWrite(in2, 0);  //Right Motor backword Pin 
  analogWrite(in3, 200);  //Left Motor backword Pin 
  analogWrite(in4, 0); //Left Motor forword Pin 
}

void backward(){

  analogWrite(in1, 0); //Right Motor forword Pin 
  analogWrite(in2, 160);  //Right Motor backword Pin 
  analogWrite(in3, 0);  //Left Motor backword Pin 
  analogWrite(in4, 160); //Left Motor forword Pin 

}

void turnRight(){ //turnRight

  rightSignal();
  analogWrite(in1, 220);  
  analogWrite(in2, 0); 
  analogWrite(in3, 0);  
  analogWrite(in4, 90); 
}

void turnLeft(){ //turnLeft

  leftSignal();
  analogWrite(in1, 0); 
  analogWrite(in2, 90);   
  analogWrite(in3, 220); 
  analogWrite(in4, ); 

}

void Stop(){ //stop

  analogWrite(in1, 0); 
  analogWrite(in2, 0); 
  analogWrite(in3, 0); 
  analogWrite(in4, 0);  
}

void leftSignal(){
  digitalWrite(leftLed,HIGH);
  delay(200);
  digitalWrite(leftLed,LOW);
  delay(200);
  digitalWrite(leftLed,HIGH);
  delay(200);
  digitalWrite(leftLed,LOW);
  delay(200);
  digitalWrite(leftLed,HIGH);
  delay(200);
  digitalWrite(leftLed,LOW);
  delay(200);
  
}

void rightSignal(){
  digitalWrite(rightLed,HIGH);
  delay(200);
  digitalWrite(rightLed,LOW);
  delay(200);
  digitalWrite(rightLed,HIGH);
  delay(200);
  digitalWrite(rightLed,LOW);
  delay(200);
  digitalWrite(rightLed,HIGH);
  delay(200);
  digitalWrite(rightLed,LOW);
  delay(200);
  
}
