#include <Arduino.h>
#define DBG(a) if (0) {a}

const int PWMA  = 3;
const int DIR1A = 4;
const int DIR2A = 2;
const int BTN   = 19;

const int DEADZONE = 50;
bool pressed = 0;
int8_t incr = 1;
int16_t pwm  = 0;
void reset() {
    digitalWrite(DIR1A, 0);
    digitalWrite(DIR2A, 0);
    digitalWrite(PWMA , 0);
    digitalWrite(LED_BUILTIN,0);
    pwm = 0;
}
void setup() {
    Serial.begin(9600);
    pinMode(PWMA , OUTPUT);
    pinMode(DIR1A, OUTPUT);
    pinMode(DIR2A, OUTPUT);
    pinMode(BTN  , INPUT );
    reset();
    digitalWrite(LED_BUILTIN,1);
    while (!Serial) {}
    digitalWrite(LED_BUILTIN,0);
    Serial.println("Serial Start");
}

void loop() {
    if (digitalRead(BTN)) {
        pressed = !pressed;
        if (!pressed)
            reset();
        if (pressed) {
            digitalWrite(LED_BUILTIN,1);
        }
        DBG(Serial.println("BUTTON PRESSED"););
        delay(200);
        
    }
    if (pressed) {
        pwm += incr;
        if (abs(pwm)<DEADZONE)
            pwm += 2*incr*DEADZONE;
        if (pwm>=255 || pwm<=-255)
            incr = -incr;
        digitalWrite(DIR1A, pwm>50);
        digitalWrite(DIR2A, pwm<-50);
        analogWrite(PWMA,abs(pwm));
        DBG(
            if (pwm%10==0)
                Serial.println(pwm);
        )
    }
    delay(10);
}
