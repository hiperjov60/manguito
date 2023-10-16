#define LED 4
#define PINBOTON 12
#define AIN1 7  // pin 1 de dirección del Motor Izquierdo
#define AIN2 10 // pin 2 de dirección del Motor Izquierdo
#define PWMA 5  // pin PWM del Motor Izquierdo
#define BIN1 8  // pin 1 de dirección del Motor Derecho
#define BIN2 9  // pin 2 de dirección del Motor Derecho
#define PWMB 6  // pin PWM del Motor Derecho

///////////////////////////PID/////////////////////////////

int base = 70;     // Velocidad base de los motores
float Kp = 0.1;  // Ajustar este coeficiente afecta la respuesta del sistema en tiempo real.
                  // Un valor alto puede hacer que el sistema responda más rápidamente, pero también puede causar oscilaciones si es demasiado grande.
float Kd =0;     // Este coeficiente es efectivo para prevenir oscilaciones y hacer que el sistema se estabilice más rápidamente después de una perturbación. }
                  // Un valor apropiado de Kd puede mejorar la estabilidad del sistema y reducir el tiempo de respuesta, pero también puede aumentar la sensibilidad al ruido en los datos.
float Ki = 0.002; //  Este coeficiente es útil para eliminar el error a largo plazo. Si el sistema tiene un sesgo o un error constante,
                  //  el término integral puede eliminarlo. Sin embargo, un valor demasiado alto puede causar oscilaciones y afectar la estabilidad del sistema.

///////////////////////////FRENOS/////////////////////////////

int porcentaje= 0.1;
int correccion= porcentaje*base;
int veladelante = base+correccion; // VELOCIDAD DEL FRENO DIRECCIÓN ADELANTE
int velatras = base-correccion;    // VELOCIDAD DEL FRENO DIRECCIÓN ATRÁS

///////////////////////////PID/////////////////////////////

int error1 = 0;
int error2 = 0;
int error3 = 0;
int error4 = 0;
int error5 = 0;
int error6 = 0;

///////////variable PID///////////////

int proporcional = 0;
int integral = 0;
int derivativo = 0;
int diferencial = 0;
int last_prop = 0;
int setpoint = 350;
int s[8];
int lectura_fondo[8];
int lectura_linea[8];
int umbral[8];
int v_s_max[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
int v_s_min[8] = {0, 0, 0, 0, 0, 0, 0, 0};
long int sumap, suma, pos, poslast, position;
int linea = 0; // linea negra o blanca 0 / 1
int pot_limite = 250;
volatile int s_p[8];
int l_pos;
boolean online;

void setup(){
    //TCCR0B = TCCR0B & B11111000 | B00000010;
    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
    TCCR0B = TCCR0B & B11111000 | B00000011;
    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz (The DEFAULT)

    Serial.begin(9600);
    pinMode(13, OUTPUT);
    WaitBoton();
    delay(1000);
    digitalWrite(13, HIGH);
    for (int i = 0; i < 100; i++){
        fondos();
        digitalWrite(LED, HIGH);
        delay(20);
        digitalWrite(LED, LOW);
        delay(30);
        }
    WaitBoton();
    for (int i = 0; i < 100; i++){
        lineas();
        digitalWrite(LED, HIGH);
        delay(20);
        digitalWrite(LED, LOW);
        delay(30);
       }
    digitalWrite(13,LOW);
    promedio();
    WaitBoton();
    digitalWrite(LED, LOW);
    delay(40);
    }

void loop(){
    frenos();
    lectura();
    PID();
    }

void WaitBoton(){ 
     while (!digitalRead(PINBOTON));
    }

void fondos(){
    lectura_fondo[0] = analogRead(A7);
    lectura_fondo[1] = analogRead(A6);
    lectura_fondo[2] = analogRead(A5);
    lectura_fondo[3] = analogRead(A4);
    lectura_fondo[4] = analogRead(A3);
    lectura_fondo[5] = analogRead(A2);
    lectura_fondo[6] = analogRead(A1);
    lectura_fondo[7] = analogRead(A0);
    for (int i = 0; i < 6; i++){
                if (lectura_fondo[i] < v_s_min[i]){
                      v_s_min[i] = lectura_fondo[i];
                      }
                }
    }
   
void lineas(){
    delay(10);
    lectura_linea[0] = analogRead(A7);
    lectura_linea[1] = analogRead(A6);
    lectura_linea[2] = analogRead(A5);
    lectura_linea[3] = analogRead(A4);
    lectura_linea[4] = analogRead(A3);
    lectura_linea[5] = analogRead(A2);
    lectura_linea[6] = analogRead(A1);
    lectura_linea[7] = analogRead(A0);
    for (int i = 0; i < 6; i++){
                if (lectura_linea[i] > v_s_max[i]){
                      v_s_max[i] = lectura_linea[i];
                      }
                }
     }   

void promedio(){
      for (int i = 0; i < 8; i++){
      umbral[i] = (lectura_linea[i] + lectura_fondo[i]) / 2;
      Serial.print (umbral[i]);
      Serial.print (" ");
      }
      Serial.println(); 
    }

int lectura(void){
    digitalWrite(13, HIGH);
    volatile int s[8];
    s[0] = analogRead(A7);
    s[1] = analogRead(A6);
    s[2] = analogRead(A5);
    s[3] = analogRead(A4);
    s[4] = analogRead(A3);
    s[5] = analogRead(A2);
    s[6] = analogRead(A1);
    s[7] = analogRead(A0);
    for (int i = 0; i < 8; i++)
    {
        if (linea == 0){
            if (s[i] <= umbral[i]){
                s[i] = 0;
                }
            else{
                s[i] = 1;
                }
            }
        else if (linea == 1){
            if (s[i] <= umbral[i]){
                s[i] = 1;
                }
            else{
                s[i] = 0;
                }
                Serial.print (s[i]);
            }  
    }
  
    if (linea == 0){
        sumap = (700 * s[0] + 600 * s[1] + 500 * s[2] + 400 * s[3] + 300 * s[4] + 200 * s[5] + 100 * s[6] + 0 * s[7]);
        }
    if (linea == 1){
        sumap = (700 * s[0] + 600 * s[1] + 500 * s[2] + 400 * s[3] + 300 * s[4] + 200 * s[5] + 100 * s[6] + 0 * s[7]);
        }
    suma = (s[0] + s[1] + s[2] + s[3] + s[4] + s[5] + s[6] + s[7]);
    pos = (sumap / suma);
    if (poslast <= 100 && pos == -1){
        pos = 0;
        }
    if (poslast >= 600 && pos == -1){
        pos = 700;
        } 
    poslast = pos;
    Serial.print (pos);
    Serial.print ("  ");
    Serial.println();
    return pos;
    }
      

void PID(){
    proporcional = pos - setpoint;
    derivativo = proporcional - last_prop;
    integral = error1 + error2 + error3 + error4 + error5 + error6;
    last_prop = proporcional;
    error6 = error5;
    error5 = error4;
    error4 = error3;
    error3 = error2;
    error2 = error1;
    error1 = proporcional;
     int diferencial = (proporcional * Kp) + (derivativo * Kd) + (integral * Ki);
       if (diferencial > base){
        diferencial = base;
        }
    else if (diferencial < -base){
        diferencial = -base;
        }
   (diferencial < 0)? Motores(base, base+diferencial):Motores(base-diferencial, base); 
  }

void frenos(){
       if (pos <= 100){
          Motores(veladelante, -velatras);
      }
      if (pos >= 600){
          Motores(-velatras, veladelante);
      }
    }

void Motores(int left, int right){
      if (left >= 0){
          digitalWrite(AIN1, HIGH);
          digitalWrite(AIN2, LOW);
        }
      else{
          digitalWrite(AIN1, LOW);
          digitalWrite(AIN2, HIGH);
          left *= -1;
      }
    analogWrite(PWMA, left);
     if (right >= 0){
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        }
    else{
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        right *= -1;
        }
    analogWrite(PWMB, right);
    }