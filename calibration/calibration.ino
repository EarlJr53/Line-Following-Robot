
void setup()
{
    //
    // start the serial port
    //
    long baudRate = 9600;   // NOTE1: The baudRate for sending & receiving programs must match
    Serial.begin(baudRate); // NOTE2: Set the baudRate to 115200 for faster communication
    Serial.setTimeout(1);

    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    pinMode(A4, INPUT);
}

void loop()
{
    Serial.print(analogRead(A4));
    Serial.print(", ");
    Serial.print(analogRead(A3));
    Serial.print(", ");
    Serial.print(analogRead(A2));
    Serial.print(", ");
    Serial.print(analogRead(A1));
    Serial.print(", ");
    Serial.println(analogRead(A0));
}
