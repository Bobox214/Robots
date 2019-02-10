const int PWMA  = 12;
const int DIR1A = 11;
const int DIR2A = 10;

void setup() {
    pinMode(PWMA , OUTPUT);
    pinMode(DIR1A, OUTPUT);
    pinMode(DIR2A, OUTPUT);

    digitalWrite(DIR1A, 1);
    digitalWrite(DIR2A, 0);
    digitalWrite(PWMA , 0);
}

void loop() {
    digitalWrite(PWMA , 1);
    delay(500);
    digitalWrite(PWMA , 0);
    delay(2000);
}
