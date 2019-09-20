int proxInput = A0;
int wristInput = A1;

int proxPWM = 10;
int wristPWM = 11; // avoid 5 and 6

double proxIn = 0;
double wristIn = 0;

void setup() {
  pinMode(proxPWM, OUTPUT);
  pinMode(wristPWM, OUTPUT);
}

void loop() {
  proxIn = analogRead(proxInput) / 1023.0; // 0 to 1
  wristIn = analogRead(wristInput) / 1023.0;

  analogWrite(proxPWM, proxIn);
  analogWrite(wristPWM, wristIn);
}
