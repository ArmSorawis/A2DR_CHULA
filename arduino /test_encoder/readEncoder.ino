#define encoderA 2
#define encoderB 3

volatile int encoderCount = 0;

void setup() {
    pinMode(encoderA, INPUT);
    pinMode(encoderB, INPUT);
    digitalWrite(encoderA, HIGH);
    digitalWrite(encoderB, HIGH);

    attachInterrupt(0, doEncoder, CHANGE);

    Serial.begin(9600);
}

void loop() {

}

void doEncoder() {
    // Raising edge
    if (digitalRead(encoderA) == HIGH) {
        if (digitalRead(encoderB) == HIGH) {
            encoderCount++;     //CW
        } else {
            encoderCount--;     //CCW
        }
    }
    // Falling edge
    else {
        if (digitalRead(encoderB) == HIGH) {
            encoderCount--;     //CCW
        } else {
            encoderCount++;     //CW
        }
    }

    Serial.println (encoderCount, DEC);


}