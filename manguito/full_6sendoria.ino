#define PINBUZZER  4
#define PINBOTON   12
#define AIN1       7
#define AIN2       5
#define PWMA       6
#define BIN1       9
#define BIN2       8
#define PWMB       10

int base = 0;
float Kprop = 1.2;
float Kderiv = 7.5;
float Kinte = 0.0;
int setpoint = 0;
int last_error = 0;
int pot_limite = 250;
volatile int s_p[6];
bool online;
int l_pos;

void setup() {
  Serial.begin(115200);
  Serial.println("Hola");
  pinMode(13, OUTPUT);
  Motores(0, 0);
  WaitBoton();
  Peripherals_init();
  delay(1000);
  calibracion();
  beep();
  WaitBoton();
  delay(1000);
}

void loop() {
  int line_position = GetPos();
  int Correction_power = PIDLambo(line_position, Kprop, Kderiv, Kinte);
  Motores(base + Correction_power, base - Correction_power);
  Serial.print(line_position);
  Serial.print("\t");
  Serial.println(Correction_power);
}

int PIDLambo(int pos, float Kp, float Kd, float Ki) {
  int error = pos - setpoint;
  int derivative = error - last_error;
  last_error = error;
  int pot_giro = (error * Kp + derivative * Kd);

  if (pot_giro > pot_limite)
    pot_giro = pot_limite;
  else if (pot_giro < -pot_limite)
    pot_giro = -pot_limite;
  return pot_giro;
}

void calibracion() {
  int v_s_min[6] = {1023, 1023, 1023, 1023, 1023, 1023};
  int v_s_max[6] = {0, 0, 0, 0, 0, 0};

  for (int j = 0; j < 100; j++) {
    delay(10);
    int v_s[6];
    v_s[0] = analogRead(A6);
    v_s[1] = analogRead(A5);
    v_s[2] = analogRead(A4);
    v_s[3] = analogRead(A3);
    v_s[4] = analogRead(A2);
    v_s[5] = analogRead(A1);

    for (int i = 0; i < 6; i++) {
      Serial.print(v_s[i]);
      Serial.print("\t");
    }
    Serial.println();

    for (int i = 0; i < 6; i++) {
      if (v_s[i] < v_s_min[i]) {
        v_s_min[i] = v_s[i];
      }

      if (v_s[i] > v_s_max[i]) {
        v_s_max[i] = v_s[i];
      }
    }
  }

  beep();
  beep();

  Serial.println();
  Serial.print("Mínimos ");
  Serial.print("\t");

  for (int i = 0; i < 6; i++) {
    Serial.print(v_s_min[i]);
    Serial.print("\t");
  }
  Serial.println();
  Serial.print("Máximos ");
  Serial.print("\t");

  for (int i = 0; i < 6; i++) {
    Serial.print(v_s_max[i]);
    Serial.print("\t");
  }
  Serial.println();
  Serial.println();
  Serial.println();
}

void readSensors() {
  int s[6];

  s[0] = analogRead(A6);
  s[1] = analogRead(A5);
  s[2] = analogRead(A4);
  s[3] = analogRead(A3);
  s[4] = analogRead(A2);
  s[5] = analogRead(A1);

  for (int i = 0; i < 6; i++) {
    if (s[i] < v_s_min[i]) {
      s[i] = v_s_min[i];
    }

    if (s[i] > v_s_max[i]) {
      s[i] = v_s_max[i];
    }
    s_p[i] = map(s[i], v_s_min[i], v_s_max[i], 100, 0);
  }

  int sum = s_p[0] + s_p[1] + s_p[2] + s_p[3] + s_p[4] + s_p[5];
  online = (sum > 100);
  
  if (online) {
    for (int i = 0; i < 6; i++) {
      Serial.print(s_p[i]);
      Serial.print("\t");
    }
  }
}

int GetPos() {
  readSensors();
  int prom = -3.5 * s_p[0] - 2.5 * s_p[1] - 1.5 * s_p[2] - 0.5 * s_p[3] + 0.5 * s_p[4] + 1.5 * s_p[5] + 2.5 * s_p[6] + 3.5 * s_p[7];
  int sum = s_p[0] + s_p[1] + s_p[2] + s_p[3] + s_p[4] + s_p[5] + s_p[6] + s_p[7];

  if (online) {
    return int(100.0 * prom / sum);
  } else {
    return (l_pos < 0) ? -255 : 255;
  }
}

void Peripherals_init() {
  pinMode(PINBOTON, INPUT);
  pinMode(PINBUZZER, OUTPUT);
  pinMode(13, OUTPUT);
}

void WaitBoton() {
  while (!digitalRead(PINBOTON)) {
    // Espera pasiva
  }
  tone(PINBUZZER, 2000, 100);
}

void beep() {
  digitalWrite(PINBUZZER, HIGH);
  delay(70);
  digitalWrite(PINBUZZER, LOW);
  delay(70);
}

void Motores(int left, int right) {
  MotorIz(left);
  MotorDe(right);
}

void MotorIz(int value) {
  if (value >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    value *= -1;
  }
  analogWrite(PWMA, value);
}

void MotorDe(int value) {
  if (value >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    value *= -1;
  }
  analogWrite(PWMB, value);
}
