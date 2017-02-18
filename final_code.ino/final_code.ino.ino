#include <TimerOne.h>
#include <PID_v1.h> 
#include <mcp_can.h>
#include <SPI.h>

#define outMin 0

unsigned long timer;

const int analogT1 = A0;
const int analogT2 = A2;

int T1, T2; //nilai baca pin analog sensor torsi
float mapVT1, mapVT2; //nilai baca tegangan output sensor torsi
float Tscale; //nilai torsi dalam Nm
int MIN1 = 4;
int MIN2 = 5;
int MPWM = 6;
int keluaran = 0; //nilai PWM tegangan motor

const int analogArus = A3; 
int mVperAmp = 66; 
int ACSoffset = 2500; 
int ArusRaw= 0;
double Voltage1 = 0;
double Amps1 = 0;


// VARIABEL KENDALI PID
double Kp = 0.29; //Konstanta 
double Ki = 0.018;
double Kd = 0; //Konstanta Derivatif
 
double ts = 0.0068; // periode sampling 6.8ms
double OutputP; //Nilai komputasi proportional
double OutputI; //Nilai komputasi integral
double OutputD; //Nilai komputasi derivatif

double error_torque = 0.0; //nilai error torsi
double errorI = 0.0; // Galat Integral Kecepatan
double errorI_prev = 0.0; 
double errorD = 0.0; //Galat Derivatif Kecepatan
double errorD_prev = 0.0;
double OutputTot; //nilai hasil kendali total

unsigned int Setpoint;
unsigned int mapT1;

// Variabel CAN
MCP_CAN CAN0(10);
int kecepatan = 0; 
unsigned char stmp[1];

void setup() {
  Serial.begin(9600);

//  delay(15000);
  //setup GUI
  Serial.write(9);
  
  //CAN Setup
  CAN0.begin(CAN_500KBPS); //init bus, baudrate 500k

  //Setup Timer Interrupt
  Timer1.initialize(6800); //waktu sampling 6.8 ms frekuensi 5x
  Timer1.attachInterrupt(baca_torsi);

  //Setup Pin Sensor Torsi
  pinMode(A0, INPUT);
  pinMode(A2, INPUT);

  //Setup Motor Pin
  pinMode(MIN1, OUTPUT);
  pinMode(MIN2, OUTPUT);
  pinMode(MPWM, OUTPUT);

}

//MAIN PROGRAM
void loop() {
//  Serial.println("sapi");
//  Serial.println(millis ());
 timer = millis (); 
 tampilanSerial();
 kirim_CAN();
// Serial.println(kecepatan);
}

void baca_torsi(){ 
  T1 = analogRead(analogT1);
  T2 = analogRead(analogT2);
  mapVT1 = (T1/1023)*5;
  mapVT2 = (T2/1023)*5;
  
  if (T1>=520 && T2<=490)
  {
  keluaran = (T1-512); //pengali memberikan power ke motor melalui PWM, 
  //4 menghasilkan overshoot. pengali ini variable, berdasar kecepatan mobil
  keluaran = constrain(keluaran,0,255);
  analogWrite(MPWM,keluaran);
  digitalWrite(MIN1, HIGH);
  digitalWrite(MIN2, LOW);
  }
  
  if (T1<=490 && T2>=520)
  {
    keluaran = (T2-512);
    keluaran = constrain(keluaran,0,255);
    analogWrite(MPWM,keluaran);
    digitalWrite(MIN1, LOW);
    digitalWrite(MIN2, HIGH);
  }
  
  if(T1<519 && T2<519)
  {
    digitalWrite(MIN1, LOW );
    digitalWrite(MIN2, LOW);
    analogWrite(MPWM,0);
  }

  kendaliPID();

 // Pembacaan Arus dari Sensor Arus
 ArusRaw = analogRead(analogArus);
 Voltage1 = (ArusRaw / 1023.0) * 5000; // Gets you mV
 Amps1 = ((Voltage1 - ACSoffset) / mVperAmp) + 0.56;

 Tscale = ((12.94)*mapVT1)-31.25;
//  Serial.println(timer);
 if(timer%50 == 0){
  Serial.write(T1/4);
 }

}


//Kendali PID
void kendaliPID(){
  Setpoint = map(T1,0,1023,0,30); //mapping bacaan torsi
  mapT1 = map(T1,0,1023,0,30);
  if (Setpoint > 4){
    error_torque = (double)Setpoint - (double)mapT1;

    //Proportional
    OutputP = Kp*error_torque;

    //Integral
    errorI += Ki*error_torque*ts;
    OutputI = errorI;

    //Derivatif
    errorD = error_torque - errorD_prev;
    OutputD = (Kd/ts)*errorD;

    //output
    OutputTot = OutputP+OutputI+OutputD;
    if(OutputTot <= outMin){OutputTot = outMin;}

    keluaran = int (OutputTot*255);
    errorD_prev = error_torque;
  }
  else {
    errorI = 0;
    keluaran = 0;
  }
  
}


void tampilanSerial(){
 //Print Serial
// Serial.print("RawAmps = ");
// Serial.print(ArusRaw);
// Serial.print(" \t");
// Serial.print("Volt = ");
// Serial.print(Voltage1);
// Serial.print(" \t");
// Serial.print("Ampere = ");
// Serial.print(Amps1);
// Serial.print(" \t");
//
// Serial.print(" T1 = ");
// Serial.print(mapVT1);
// Serial.print(" V \t");
// Serial.print(" T2 = ");
// Serial.print(mapVT2);
// Serial.print(" V ");
// Serial.print(" Tscale = ");
// Serial.print(Tscale);
// Serial.println(" Nm ");

}

int iterator = 0;

void kirim_CAN()
{
  if ((timer%160000>= 0) && (timer%160000 <= 20000)){
    kecepatan = 0;
  }
  else if ((timer%160000>= 20000) && (timer%160000 <= 40000)){
    kecepatan = 10;
  }
  else if ((timer%160000>= 40000) && (timer%160000 <= 60000)){
    kecepatan = 20;
  }
  else if ((timer%160000>= 60000) && (timer%160000 <= 80000)){
    kecepatan = 30;
  }
  else if ((timer%160000>= 80000) && (timer%160000 <= 100000)){
    kecepatan = 40;
  }
  else if ((timer%160000>= 100000) && (timer%160000 <= 120000)){
    kecepatan = 50;
  }
  else if ((timer%160000>= 120000) && (timer%160000 <= 140000)){
    kecepatan = 60;
  }
  else if ((timer%160000>= 140000) && (timer%160000 <= 160000)){
    kecepatan = 70;
  }
  
   stmp[0] = char (kecepatan);
//    Serial.print("Timer: ");
//    Serial.print(timer);
//    Serial.print("  Kecepatan : ");
//    Serial.println(kecepatan);
    CAN0.sendMsgBuf(0x00, 0, sizeof(stmp), stmp);
}

