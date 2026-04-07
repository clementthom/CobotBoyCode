bool actionBlack = false;
bool actionGreen = false;

void blackButtonPressed() {
  Serial.println("noir");
  actionBlack = true;
  return;
}

void greenButtonPressed() {
  Serial.println("green");
  actionGreen = true;
  return;
}



void setup() {
  // put your setup code here, to run once:
  pinMode(7, OUTPUT); //INB
  pinMode(6, OUTPUT); //INA
  pinMode(11, OUTPUT);

  pinMode(20, INPUT); //bouton noir
  pinMode(21, INPUT); //bouton vert

  attachInterrupt(digitalPinToInterrupt(20), blackButtonPressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(21), greenButtonPressed, FALLING);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(actionBlack == true) {
    digitalWrite(7, HIGH); //INB à 1
    digitalWrite(6, LOW); //INA à 0 
    analogWrite(3, 1023); //broche 3 à 0 --> PWM à 0
    delay(200); 
    analogWrite(3, 0); //broche 3 à 0 --> PWM à 0
    actionBlack = false;
  }

  if(actionGreen == true) {
    digitalWrite(7, LOW); //INB à 0
    digitalWrite(6, HIGH); //INA à 1 
    analogWrite(3, 1023); //broche 3 à 1 --> PWM à 100
    delay(200);
    analogWrite(3, 0); //broche 3 à 0 --> PWM à 0
    actionGreen = false;
  }
  
}
