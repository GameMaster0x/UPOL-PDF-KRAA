#include <NewPing.h>
#include <Servo.h>

Servo myServo;  // Deklarace instance serva

int mLp = 6;   // směr levý
int mLs = 8;   // výkon levý
int mPp = 5;   // směr pravý
int mPs = 7;   // výkon pravý

int pTrig = 12;
int pEcho = 13;
int MAX_DISTANCE = 200;
long odezva, vzdalenost;

int vst = A2;
float err;
float kor;
int v = 180;
int mL, mP;
int akt;
float P = 90;
float D = 900;
float last_err = 0;
float der = 0;


void setup() {
  pinMode(mLp, OUTPUT);
  pinMode(mLs, OUTPUT);
  pinMode(mPp, OUTPUT);
  pinMode(mPs, OUTPUT);
  
  pinMode(pTrig, OUTPUT);
  pinMode(pEcho, INPUT);
  
  analogWrite(mLp, 0);
  digitalWrite(mLs, LOW);
  analogWrite(mPp, 0);
  digitalWrite(mPs, LOW);
  delay(5000);
  Serial.begin(9600);

  myServo.attach(9);  // Inicializace piny pro servo, v tomto případě pin 9
  myServo.write(90);  // Nastavení počáteční pozice serva (v úhlu)
}



void loop() {
 spocti_PD();
  digitalWrite(pTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pTrig, LOW);
  odezva = pulseIn(pEcho, HIGH);  // maximální délku pulzu v mikrosekundách (us)
  vzdalenost = int(odezva / 58.31);    // přepočet na cm
  
  Serial.print(vzdalenost);
  Serial.println(" cm");
  
  if (vzdalenost <= 20) {
    motorStop();
  } else {
    while (analogRead(A0) < 200) {
      motorRight();
    }
    while (analogRead(A1) < 200) {
      motorLeft();
    }
 }
  digitalWrite(mLp, 240);
  analogWrite(mLs, LOW);
  digitalWrite(mPp, 120);
  analogWrite(mPs, LOW);*/
}
void spocti_PD()
{
  int sA0 = analogRead(A0);
  int sA1 = analogRead(A1);
  int sA2 = analogRead(A2);
  int sA3 = analogRead(A3);
  
  
  int err_x = (sA3*(-2)) + (sA2*(-1)) + (sA1)+ (sA0*2);
  int err_sum  = sA0 + sA1 + sA2 + sA3;
  err = (1.0 * err_x) / (1.0 * err_sum);
  
  float P_fix = err * P;
  der = err - last_err;
  last_err = err;

  float korek = P_fix + (D*der);

  mL = v + korek;
  if (mL <0) mL = 0; 
  if (mL >v) mL = v; 
 
  mP = v - korek;

  if (mP < 0) mP = 0; 
  if (mP > v) mP = v; 

  digitalWrite(mLp,LOW);
  analogWrite(mLs, mL);
  digitalWrite(mPp, LOW);
  analogWrite(mPs, mP);
  /*
  Serial.println(sA0);
  Serial.println(sA1);
  Serial.println(sA2);
  Serial.println(sA3);
  delay(250);*/
  }

void motorLeft() {
  digitalWrite(mLp, LOW);
  digitalWrite(mLs, 0);
  digitalWrite(mPp, 200);
  digitalWrite(mPs, LOW);
}

void motorRight() {
  digitalWrite(mLp, 200);
  digitalWrite(mLs, LOW);
  digitalWrite(mPp, LOW);
  digitalWrite(mPs, 0);
}

void motorStop() {
  digitalWrite(mLp, LOW);
  digitalWrite(mLs, LOW);
  digitalWrite(mPp, LOW);
  digitalWrite(mPs, LOW);
}
