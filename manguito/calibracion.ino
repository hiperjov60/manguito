#define PINBUZZER  4
#define PINBOTON  12


int v_s_min[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
int v_s_max[8] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile int s_p[8];
boolean online;
int pos;
int l_pos;

void setup() 
    {
        Serial.begin(115200);
        Serial.println("hola");
        digitalWrite(13, HIGH);
        Peripherals_init();
        WaitBoton();
        delay(1000);
        calibracion();
        tone(PINBUZZER, 1500, 50);
        delay(70);
        tone(PINBUZZER, 1500, 50);
        delay(70);
        WaitBoton();
        delay(1000);

    }

void loop() 
    {
        int line_position = GetPos();
        Serial.println(line_position);
        delay(1);
    }


void calibracion() 

    {

        int v_s[8];

        for (int j = 0; j < 500; j++) 
        {
            delay(10);
            v_s[0] = analogRead(A7);
            v_s[1] = analogRead(A6);
            v_s[2] = analogRead(A5);
            v_s[3] = analogRead(A4);
            v_s[4] = analogRead(A3);
            v_s[5] = analogRead(A2);
            v_s[6] = analogRead(A1);
            v_s[7] = analogRead(A0);

    for (int i = 0; i < 8; i++) 
            {
            Serial.print(v_s[i]);
            Serial.print("\t");
        }
    
    Serial.println();

    for (int i = 0; i < 8; i++) 
    {
        if (v_s[i] < v_s_min[i]) 
        {
        v_s_min[i] = v_s[i];
        }
    }

    for (int i = 0; i < 8; i++) 
    {
        if (v_s[i] > v_s_max[i]) 
        {
        v_s_max[i] = v_s[i];
        }
    }
    }

    beep();
    beep();

    Serial.println();
    Serial.print("Mínimos ");
    Serial.print("\t");

        for (int i = 0; i < 8; i++) 
        {

        Serial.print(v_s_min[i]);
        Serial.print("\t");
        }
    
    Serial.println();
    Serial.print("Máximos ");
    Serial.print("\t");

        for (int i = 0; i < 8; i++) 
        {
        Serial.print(v_s_max[i]);
        Serial.print("\t");
        }
        Serial.println();
        Serial.println();
        Serial.println();

    }

void readSensors() 
{
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
        if (s[i] < v_s_min[i]) 
        {
            s[i] = v_s_min[i];
        }

        if (s[i] > v_s_max[i])
         {
            s[i] = v_s_max[i];
        }
        s_p[i] = map(s[i], v_s_min[i], v_s_max[i], 100, 0);
    }


    volatile int sum = s_p[0] + s_p[1] + s_p[2] + s_p[3] + s_p[4] + s_p[5] + s_p[6] + s_p[7];
        if (sum > 100) 
        {
        online = 1;
        } 
        else 
        {
        online = 0;
        sum = 100;
        }
        if (online) 
        {
            for (int i = 0; i < 8; i++) 
            {
                Serial.print(s_p[i]);
                Serial.print("\t");
            }
            //Serial.println();
        }
}


int GetPos() 
{
    readSensors();
    int prom = -3.5 * s_p[0] - 2.5 * s_p[1] - 1.5 * s_p[2] - 0.5 * s_p[3] + 0.5 * s_p[4] + 1.5 * s_p[5] + 2.5 * s_p[6] + 3.5 * s_p[7];
    int sum = s_p[0] + s_p[1] + s_p[2] + s_p[3] + s_p[4] + s_p[5] + s_p[6] + s_p[7];

    if (online) 
    {
    pos = int(100.0 * prom / sum);
    } 
    else 
    {
    if (l_pos < 0) 
    {
        pos = -255;
    }
    if (l_pos >= 0) 
    {
        pos = 255;
    }
    }
    l_pos = pos;
    return pos;
}


void Peripherals_init() 
    {
    pinMode(PINBOTON, INPUT);
    pinMode(PINBUZZER, OUTPUT);
    pinMode(13,OUTPUT);
    }


void WaitBoton() 
    {   // Entra en un bucle infinito de espera.
    while (!digitalRead(PINBOTON));  // Se sale del bucle cuando se aprieta el botón
    tone(PINBUZZER, 2000, 100);      // Cuando sale del bucle, suena el buzzer
    }


void beep() 
    {
    tone(PINBUZZER, 2000, 100);
    delay(200);
    }