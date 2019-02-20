#include <Arduino.h>
#define DBG(a) if (1) {a}

const int PWMA  = 5;
const int DIR1A = 6;
const int DIR2A = 4;
const int ENCODER1A = 3;
const int ENCODER2A = 17;
const int PWMB  = 8;
const int DIR1B = 7;
const int DIR2B = 9;
const int ENCODER1B = 2;
const int ENCODER2B = 18;
const int BTN   = 19;

const int DEADZONE = 50;
bool pressed = 0;
int8_t incr = 1;
int16_t pwmA  = 0;
int16_t pwmB  = 0;
volatile int32_t countA = 0;
volatile int32_t countB = 0;
// 610 par tour
void encoderCountA() {
    if (digitalRead(ENCODER2A))
        countA++;
    else
        countA--;
}
void encoderCountB() {
    if (digitalRead(ENCODER2B))
        countB++;
    else
        countB--;
}
void reset() {
    digitalWrite(DIR1A, 0);
    digitalWrite(DIR2A, 0);
    digitalWrite(PWMA , 0);
    digitalWrite(DIR1B, 0);
    digitalWrite(DIR2B, 0);
    digitalWrite(PWMB , 0);
    digitalWrite(LED_BUILTIN,0);
    pwmA = 0;
    pwmB = 0;
}
void setup() {
    Serial.begin(9600);
    pinMode(PWMA , OUTPUT);
    pinMode(DIR1A, OUTPUT);
    pinMode(DIR2A, OUTPUT);
    pinMode(PWMB , OUTPUT);
    pinMode(DIR1B, OUTPUT);
    pinMode(DIR2B, OUTPUT);
    pinMode(BTN  , INPUT );
    reset();
    digitalWrite(LED_BUILTIN,1);
    while (!Serial) {}
    digitalWrite(LED_BUILTIN,0);
    Serial.println("Serial Start");
    //attachInterrupt(digitalPinToInterrupt(ENCODER1A),encoderCountA,RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER1B),encoderCountB,RISING);
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
        pwmA += incr;
        pwmB += incr;
        if (abs(pwmA)<DEADZONE)
            pwmA += 2*incr*DEADZONE;
        if (abs(pwmB)<DEADZONE)
            pwmB += 2*incr*DEADZONE;
        if (pwmA>=255 || pwmA<=-255)
            incr = -incr;
        digitalWrite(DIR1A, 1);//pwmA>50);
        digitalWrite(DIR2A, 0);//pwmA<-50);
        analogWrite(PWMA,255);//abs(pwmA));
        digitalWrite(DIR1B, 1);//pwmB>50);
        digitalWrite(DIR2B, 0);//pwmB<-50);
        analogWrite(PWMB,255);//abs(pwmB));
    }
    delay(500);
    DBG( Serial.print("A:");Serial.println(countA);)
    DBG( Serial.print("B:");Serial.println(countB);)
}
