
void setup()
{
    //
    // start the serial port
    //
    long baudRate = 9600;   // NOTE1: The baudRate for sending & receiving programs must match
    Serial.begin(baudRate); // NOTE2: Set the baudRate to 115200 for faster communication
    Serial.setTimeout(1);

    pinMode(A0, INPUT);
}

void loop()
{
    Serial.println(analogRead(A0));
}
