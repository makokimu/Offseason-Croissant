int proxInput = A4;
int wristInput = A2;

int proxPWM = 10;
int wristPWM = 11; // avoid 5 and 6

double proxIn = 0;
double wristIn = 0;

double _min = 2000.0;
double _max = -1.0;

void setup() {
  pinMode(proxPWM, OUTPUT);
  pinMode(wristPWM, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  proxIn = analogRead(proxInput) / 1023.0; // 0 to 1
  wristIn = analogRead(wristInput) / 1023.0; // 0 to 1

  if(proxIn > _max) {
    _max = proxIn;
  }
  if(proxIn < _min) {
    _min = proxIn;
  }
  

  Serial.print("prox ");
  Serial.print(proxIn * 360);

//  Serial.print(" min ");
//  Serial.print(_min);
//  Serial.print(" max ");
//  Serial.print(_max);
  
  Serial.print(" wrist ");
  Serial.print(wristIn * 360);


  
  Serial.println(" ");

  

  analogWrite(proxPWM, proxIn);
//  analogWrite(wristPWM, wristIn);
}
