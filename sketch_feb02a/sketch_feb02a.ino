int ledPin = 13;   

String readString;

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT); 
  Serial.begin(115200);
  while (! Serial); // Wait until Serial is ready - Leonardo
  Serial.println("ready!!");
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available()) {
    char c = Serial.read();  //gets one byte from serial buffer
    readString = c; //makes the string readString
    delay(2);  //slow looping to allow buffer to fill with next character
    if(c == 'x'){
//       Serial.println("Left"); 
        digitalWrite(ledPin, HIGH);
     } else 
     if(c == 'y'){
//       Serial.println("Right"); 
        digitalWrite(ledPin, LOW);
     }
  Serial.println(); 
//  delay(2000); 
  }
//  Serial.println(readString); 
//  if (readString == "xxx"){
//    readString = "";
//  }
//    readString = "";
  
}
