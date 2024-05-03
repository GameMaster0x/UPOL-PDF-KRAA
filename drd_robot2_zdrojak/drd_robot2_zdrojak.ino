//To ze je to nakokot jeste neznamena ze tenhle kod neudela zapocet...
#include <NewPing.h>





#include <Servo.h>

Servo myServo;

int cMin[4];
int cMax[4];
int cNorm[4];
int vahy[4] = { -4, -1, 1, 4 };
int cPin[4] = { A0, A1, A2, A3 };
int mLs = 7;   // výkon levý
int mLv = 5;   // směr levý
int mPs = 8;   // výkon pravý
int mPv = 6;   // směr pravý
int pTrig = 12;
int pEcho = 13;

float P = 0.1;
float Pp = 2; //objizdeni prekazky
float I = 0; //ladit v tisicinach az desetitisicinach
float D = 0; //v jednotkach
float Err;
float ErrOld = 0; //pro vypocet derivacni slozky
float iSum = 0; //pro scitani odchylek - pro integracni slozku
int kor, mL, mP, pom; //pro ulozeni vykonu pro levy a pravy motor, korekce je dulezita
int v0 = 180; //maximalni rychlost k jezdeni
int vMax = 200;


void setup() {
  Serial.begin(9600);

  pinMode(mLs, OUTPUT); //inicializace pinů
  pinMode(mLv, OUTPUT);
  pinMode(mPs, OUTPUT);
  pinMode(mPv, OUTPUT);

  pinMode(9, OUTPUT);
  myServo.attach(9);  // Inicializace piny pro servo, v tomto případě pin 9
  myServo.write(90);  // Nastavení počáteční pozice serva (v úhlu pred sebe)... knihovna servo se hada s PVM o casovac - dochazi k chybam - pokud ho porebuji, pootocim ho, deaktivuji a pockam - viz detach
  delay(20);
  myServo.detach();
  
  //nalezeni minima/maxima (ADC = (0...1023))
  for (int i = 0; i < 4; i++) {
    cMin[i] = 1023;
    cMax[i] = 0; 
  }
  delay(5000); //po startu čeká

  //roztočení tobota a načítání max/min hodnot po dobu 5s

  digitalWrite(mLs, LOW); //směrový pin LOW => dopředu, HIGH => dozadu
  digitalWrite(mPs, HIGH);
  analogWrite(mLv, 155);
  analogWrite(mPv, 255 - 155);
  
  long t0 = millis();
  while ((millis() - t0) < 5000) { //čas v milisekundách
    for (int i = O; i < 4; i++){
      int pom = analogRead(cPin[i]);
      if (pom < cMin[i]) cMin[i] = pom;
      if (pom > cMax[i]) cMax[i] = pom; 
    }
  }

  while (analogRead(cPin[3]) < 500)
    ; // WHAT THE FUCK

  digitalWrite(mLs, LOW);
  digitalWrite(mPs, LOW);
  analogWrite(mLv, 0);
  analogWrite(mPv, 0);

  delay(5000); //5s čeká před startem 
}

void nactiCidla() { //nacteni hodnot z cidel a jejich normalizace
  for (int i = 0; i < 4; i++) {
    int pom = analogRead(cPin[i]);
    cNorm[i] = int (((pom - cMin[i]) * 100.0) / (cMax[i] - cMin[i] + 1));
    if (cNorm[i] > 100) cNorm[i] = 100;
    if (cNorm[i] < 0) cNorm[i] = 0;
  }
}

void vypoctiErr() {
  long sumErrVaha = O;
  long sumErr = 1;
  for (int i = 0; i < 4; i++) {
    sumErrVaha = sumErrVaha + (cNorm[i] * vahy[i]);
    sumErr = sumErr + cNorm[i];
  }
  Err = (float(sumErrVaha) / float(sumErr))
}

void motory(int korL) {
  mL = v0 + korL;
  mP = v0 - korL;

  if (mL > vMax) mL = vMax;
  if (mL < -vMax) mL = -vMax;
  if (mP > vMax) mP = vMax;
  if (mP < -vMax) mP = -vMax;

  if (mL < 0) {
    digitalWrite(mLs, HIGH);
    analogWrite(mLv, (255 + mL)); //mL < 0 => 255+mL
  } else {
    digitalWrite(mLs, LOW);
    analogWrite(mLv, mL));
  }
  if (mP < 0) {
    digitalWrite(mPs, HIGH);
    analogWrite(mPv, (255 + mP)); //mL < 0 => 255+mL
  } else {
    digitalWrite(mPs, LOW);
    analogWrite(mPv, mP));
  }
}

void vypoctiPID() { //PID regulace
  iSum = iSum + Err;
  if ((Err * ErrOld) < 0) iSum = 0;
  kor = P * Err + I * iSum + D * (Err - ErrOld);
  ErrOld = Err;
}

void lomCara () { //krizovatky do prava
  //osetrena prava lomena cara 
 
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
  digitalWrite(mLv, 240);
  analogWrite(mLs, LOW);
  digitalWrite(mPv, 120);
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

  digitalWrite(mLv,LOW);
  analogWrite(mLs, mL);
  digitalWrite(mPv, LOW);
  analogWrite(mPs, mP);
  /*
  Serial.println(sA0);
  Serial.println(sA1);
  Serial.println(sA2);
  Serial.println(sA3);
  delay(250);*/
  }

void motorLeft() {
  digitalWrite(mLv, LOW);
  digitalWrite(mLs, 0);
  digitalWrite(mPv, 200);
  digitalWrite(mPs, LOW);
}

void motorRight() {
  digitalWrite(mLv, 200);
  digitalWrite(mLs, LOW);
  digitalWrite(mPv, LOW);
  digitalWrite(mPs, 0);
}

void motorStop() {
  digitalWrite(mLv, LOW);
  digitalWrite(mLs, LOW);
  digitalWrite(mPv, LOW);
  digitalWrite(mPs, LOW);
}

int vzdalenost() {
  //nacteni vzdalenosti
  long odezva;
  int vzd;
  digitalWrite(pTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pTrig, HIGH);
  delayMicroseconds(5);
  digitalWrite(pTrig, LOW);
  odezva = pulseIn)pEcho, HIGH, 5000); //jak dlouho cekam v mikros
  vzd = int(odezva/58.31); //prepocitam na vzdalenost v cm
  return vzd;
}


void nacticidla - if cnorm dela orezy - pokud je bila belejsi, nebo cerna cernejsi

lomena cara 