#define interruptPin 2
#define motor 6
#define enable 7

int uMax = 66;
int pwmMax = (uMax * 255 / 100);

float resolucao = 600.0;
volatile int pulsos = 0;

bool rampa = false;

float t = 0;

unsigned long tempo = 0;
unsigned long intervalo = 10;


void setup() {

  pinMode(motor, OUTPUT);
  pinMode(enable, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), contaPulso, RISING);

  digitalWrite(enable, HIGH);

  Serial.begin(115200);
}

void loop() {

  while(!Serial.available()){

  }

  /*if (!rampa) {
    for (int dutty = 0; dutty < pwmMax; dutty++) {
      analogWrite(motor, dutty);
      Serial.print(dutty);
      Serial.print(",");
      Serial.println(velocidade());

      delay(10);
    }
    rampa = true;
  }


  else {
    analogWrite(motor, pwmMax);
    Serial.print(pwmMax);
    Serial.print(",");
    Serial.println(velocidade());

    delay(10);
  }*/

  analogWrite(motor, pwmMax);

  if(millis() - tempo >= intervalo){    
    Serial.println(velocidade());
    tempo = millis();
  }


}

void contaPulso() {
  pulsos++;
}

float velocidade() {
  long aux = pulsos;  
  pulsos = 0;
  return aux;
}
