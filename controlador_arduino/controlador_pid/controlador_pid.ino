#define interruptPin 2  // Pino de interrupção (Encoder A)
#define motorPin 6      // Pino PWM do motor
#define enablePin 7     // Habilita driver do motor

// Parâmetros do modelo e controle
float Ts = 0.01;  // Período de amostragem em segundos (10 ms)
double Kp = 0.0066, Ki = 0.0399, Kd = 0;  // Ganhos do controlador PID

// Variáveis de controle
double e[3] = { 0, 0, 0 }; // Erros atuais e anteriores para a equação de diferenças
double u = 0, u_1 = 0;     // Sinal de controle atual e anterior
double y = 0;              // Variável não usada no código, pode ser removida

float setpoint = 168;      // Setpoint inicial em RPM

volatile int pulsos = 0;   // Contador de pulsos do encoder, variável volátil pois é alterada na ISR
const float resolucao = 340.0;  // Número de pulsos por revolução do encoder

unsigned long tempoAnterior = 0;            // Guarda o tempo da última amostragem
const unsigned long intervalo = Ts * 1000;  // Intervalo de amostragem em milissegundos

float t = 0;  // Tempo discreto para envio via serial (incrementado em Ts)

void setup() {
  pinMode(motorPin, OUTPUT);      // Configura pino PWM do motor como saída
  pinMode(enablePin, OUTPUT);     // Configura pino enable do driver como saída
  pinMode(interruptPin, INPUT_PULLUP);  // Configura pino de interrupção com resistor pull-up

  attachInterrupt(digitalPinToInterrupt(interruptPin), contaPulso, RISING);  // Configura ISR na borda de subida do encoder
  digitalWrite(enablePin, HIGH);  // Habilita o driver do motor

  Serial.begin(115200);            // Inicializa comunicação serial a 115200 bps
  Serial.println("Digite o setpoint em RPM:");  // Mensagem inicial
}

void loop() {

  // Verifica se há dados recebidos na serial para atualizar o setpoint
  if (Serial.available() > 0) {
    String entrada = Serial.readStringUntil('\n');  // Lê a linha completa
    entrada.trim();                                 // Remove espaços em branco
    if (entrada.length() > 0) {
      setpoint = entrada.toFloat();                  // Converte para float
      Serial.print("Novo setpoint: ");
      Serial.println(setpoint);
    }
  }

  unsigned long tempoAtual = millis();  
  // Executa controle a cada intervalo (10 ms)
  if (tempoAtual - tempoAnterior >= intervalo) {
    tempoAnterior = tempoAtual;

    float rpm = calcularRPM();  // Calcula RPM atual baseado nos pulsos do encoder

    // Atualiza o vetor de erros (mais recentes nos índices menores)
    e[2] = e[1];
    e[1] = e[0];
    e[0] = setpoint - rpm;       // Erro atual: diferença entre setpoint e RPM medido

    // Se o setpoint for zero, reseta variáveis para evitar comportamento estranho
    if (setpoint == 0) {
      u = 0;
      u_1 = 0;
      e[0] = e[1] = e[2] = 0;
    }

    // Calcula o sinal de controle u usando a equação de diferenças do PID discreto
    if (e[2] == 0 && e[1] == 0) {
      u = (Kp + Ki * Ts + Kd / Ts) * e[0];
    } else if (e[2] == 0) {
      u = u_1 + Kp * (e[0] - e[1]) + Ki * Ts * e[0] + (Kd / Ts) * (e[0] - 2 * e[1]);
    } else {
      u = u_1 + Kp * (e[0] - e[1]) + Ki * Ts * e[0] + (Kd / Ts) * (e[0] - 2 * e[1] + e[2]);
    }

    // Constrói o valor PWM a partir do sinal de controle u
    int pwm = constrain(abs(u), 0, 1);  // Saturação entre 0 e 1 (esta linha não faz muito sentido)
    pwm = u * 255;                       // Ajusta escala para PWM de 0 a 255

    // Envia via serial: tempo, setpoint e rpm
    Serial.print(t);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.println(rpm);

    analogWrite(motorPin, pwm);   // Aplica PWM ao motor

    // Atualiza variáveis para próxima iteração
    u_1 = u;
    t += Ts;  // Incrementa tempo discreto
  }
}

// Rotina de interrupção para contar pulsos do encoder
void contaPulso() {
  pulsos++;
}

// Função para calcular a velocidade (RPM) a partir dos pulsos contados
float calcularRPM() {
  noInterrupts();         // Desabilita interrupções para leitura consistente
  int contagem = pulsos;  // Lê a contagem de pulsos
  pulsos = 0;             // Reseta contador
  interrupts();           // Reabilita interrupções

  // Converte contagem de pulsos para rotações por segundo e depois para RPM
  float rps = (float)contagem / resolucao / (Ts);
  return rps * 60.0;
}
