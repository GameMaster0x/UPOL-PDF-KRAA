//Nejdrive si nainstaluj knihovnu myping a servo... Menu -> Nastroje -> Manage libraries, vyhledej a dej nainstlovat.
#include <NewPing.h>
#include <Servo.h>

Servo myServo; //vytvoreni objektu my.servo

//definice promennych (integery a desetinne floaty)

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


void setup() { //tento kod se spusti pri zapnuti robota, pote pokracuje az loop (dole)
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

  //roztočení robota a načítání max/min hodnot po dobu 5s

  digitalWrite(mLs, LOW); //směrový pin LOW => dopředu, HIGH => dozadu
  digitalWrite(mPs, HIGH);
  analogWrite(mLv, 155);
  analogWrite(mPv, 255 - 155);
  
  long t0 = millis();
  while ((millis() - t0) < 5000) { //čas v milisekundách
    for (int i = 0; i < 4; i++){
      int pom = analogRead(cPin[i]);
      if (pom < cMin[i]) cMin[i] = pom;
      if (pom > cMax[i]) cMax[i] = pom; 
    }
  }

  while (analogRead(cPin[3]) < 500);

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
  long sumErrVaha = 0;
  long sumErr = 1;
  for (int i = 0; i < 4; i++) {
    sumErrVaha = sumErrVaha + (cNorm[i] * vahy[i]);
    sumErr = sumErr + cNorm[i];
  }
  Err = (float(sumErrVaha) / float(sumErr));
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
    analogWrite(mLv, mL);
  }
  if (mP < 0) {
    digitalWrite(mPs, HIGH);
    analogWrite(mPv, (255 + mP)); //mL < 0 => 255+mL
  } else {
    digitalWrite(mPs, LOW);
    analogWrite(mPv, mP);
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
 if ((cNorm[3] > 80) and (cNorm[0] < 50)) {
  motory(vMax);
  //delay(250)
  while (cNorm[0] < 50) nactiCidla();
  motory(-vMax);
  while (cNorm[1] < 50) nactiCidla();
 }
 //osetrena leva lomena cara
  if ((cNorm[0] > 80) and (cNorm[3] < 50)) {
  motory(-vMax);
  while (cNorm[3] < 50) nactiCidla();
  motory(vMax);
  while (cNorm[2] < 50) nactiCidla();
 }
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
  odezva = pulseIn(pEcho, HIGH, 7000); //jak dlouho cekam v mikros
  vzd = int(odezva / 58.31); //prepocitam na vzdalenost v cm
  return vzd;
}

void objed() {
  myServo.attach(9);
  myServo.write(150);
  delay(20);
  myServo.detach();
  //vpravo 90 stupnu
  motory(v0);
  delay(500); //lepsi mirne pretocit 90 stupnu
  //regulace podle vzdalenosti
  while (cNorm[2] < 50) {
    nactiCidla();
    pom = vzdalenost();
    // 1 č. P-reg.
    if ((pom < 1)or(pom>30)) pom = 30;
    kor = int(Pp * (pom - 20));
    motory(kor);
    // 1 č. 2-st. reg
    //if ((pom <1)or(pom>30)) {
    //  motory(-v0);
    //}
    //else {
    //  motory(v0);
    //}
  }
  myServo.attach(9);
  myServo.write(90);
  delay(20);
  myServo.detach();

  //vpravo 90
  motory(v0);
  delay(500);
}

//tento kod se opakuje do nekonecna, kazdou iteraci se volaji jednotlive funkce
void loop() {
  pom = vzdalenost(); //pokud je vzdalenost vetsi nez 15, nebo mensi nez 1, provede se if, v opacnem pripade se vola funkce objed prekazku
  if ((pom > 15) or (pom < 1)) {
    nactiCidla();
    lomCara();
    vypoctiErr();
    vypoctiPID();
    motory(kor); //jedina funkce, kde se posila promenna kor, na zaklade ktere robot uzpusobuje rychlost motoru
  } else {
    objed();
  }
}