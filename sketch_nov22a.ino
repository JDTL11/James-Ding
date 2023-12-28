#include <LiquidCrystal.h>
#define ENCODER1_DIGITAL A2
#define ENCODER2_DIGITAL A3



int in1 = 12;
int in2 = 10;
int in3 = 11;
int in4 = 13;
int leftsensor = A1;
int rightsensor = A0;
int Lled;
int Rled;

volatile long encoder1Pos = 0;
volatile long encoder2Pos = 0;

float circumference_of_wheel = 21.36;
unsigned int pulseperturn = 20;
float distancePerpulse = circumference_of_wheel/pulseperturn;
float totalDistance;

const int pin_RS = 8;
const int pin_EN = 9;
const int pin_d4 = 4;
const int pin_d5 = 5;
const int pin_d6 = 6;
const int pin_d7 = 7;

LiquidCrystal lcd(pin_RS, pin_EN, pin_d4, pin_d5, pin_d6, pin_d7);

int lastState;

void updateEncoder(volatile long &encoderPos, int encoderPin){
  static int lastState = LOW;
  int currentState = digitalRead(encoderPin);
  if (currentState == HIGH && lastState == LOW){
    encoderPos++;
  }
  lastState = currentState;
}

void setup() {
  // Set all the motor control pins to outputs
  pinMode(leftsensor, INPUT);
  pinMode(rightsensor, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ENCODER1_DIGITAL, INPUT);
  pinMode(ENCODER2_DIGITAL, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_DIGITAL),[]{ updateEncoder(encoder1Pos, ENCODER1_DIGITAL);},CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_DIGITAL),[]{ updateEncoder(encoder2Pos, ENCODER2_DIGITAL);},CHANGE);
  
analogWrite(in1, 255);
digitalWrite(in2, LOW);
analogWrite(in3, 255);
digitalWrite(in4, LOW);
delay(500);

  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("Ferrrrrarri");
  lcd.setCursor(0,1);
  lcd.print("Distance: 0");
  
  

void loop() {
  	
  Lled = digitalRead(leftsensor);
  Rled = digitalRead(rightsensor);

  if (Lled == HIGH && Rled == HIGH){

    analogWrite(in1, 70);
    digitalWrite(in2, LOW);
    analogWrite(in3, 70);
    digitalWrite(in4, LOW);

  }
  
  if (Lled == 0 && Rled == 1){

    analogWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(in3, 255);
    digitalWrite(in4, LOW);

  }
  
  if (Lled == 1 && Rled == 0){

    analogWrite(in1, 255);
    digitalWrite(in2, LOW);
    analogWrite(in3, 0);
    digitalWrite(in4, HIGH);

  }

  if (Lled == LOW && Rled == LOW){
    analogWrite(in1, 0);
    digitalWrite(in2, LOW);
    analogWrite(in3, 0);
    digitalWrite(in4, LOW);

  }
}

void displayDistance(int col,int row,float distance){
  lcd.setCursor(col,row);
  lcd.print(distance, 2);
}
