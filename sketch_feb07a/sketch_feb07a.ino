int pushButton = 2;
int randNumber;
int divider;
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT);
//  delay(5000);
  Serial.write(0);
}

// the loop routine runs over and over again forever:
void loop() {
   randNumber = random(1024);
    Serial.write(randNumber/4);
//int myDelay = 1023;  // this is for milliseconds
//byte high = highByte(myDelay);
//byte low = lowByte(myDelay);
////Serial.write(
//    Serial.write(0);
//    Serial.write(high);
//    Serial.write(low);
//    
//    Serial.write(0);
  // delay in between reads for stability
  delay(200);
}
