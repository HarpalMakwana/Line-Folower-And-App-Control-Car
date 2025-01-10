/**********************************************************
 * Program: Multi-Functional Robot (Bluetooth + IR Control)
 * Author: Harpal Makwana
 * Date: 9-1-2025
 * Contact Details:
 *   - Phone: +91 9313758130
 *   - Instagram: @future_science_Hub
 *   - YouTube: Future Science Hub
 *   - WhatsApp: +91 9313758130
 **********************************************************/

#include <SoftwareSerial.h>
SoftwareSerial BT_Serial(2, 3); // RX, TX

#include <IRremote.h>
const int RECV_PIN = A5;
IRrecv irrecv(RECV_PIN);
decode_results results;

#define enA 10 // Enable1 L298 Pin enA 
#define in1 9  // Motor1  L298 Pin in1 
#define in2 8  // Motor1  L298 Pin in2 
#define in3 7  // Motor2  L298 Pin in3 
#define in4 6  // Motor2  L298 Pin in4 
#define enB 5  // Enable2 L298 Pin enB 

#define servo A4 // Servo pin

#define R_S A0  // IR sensor Right
#define L_S A1  // IR sensor Left

#define echo A2    // Echo pin for ultrasonic sensor
#define trigger A3 // Trigger pin for ultrasonic sensor

int distance_L, distance_F = 30, distance_R;
long distance;
int set = 20;

int bt_ir_data; // Variable to receive data from the serial port and IRremote
int Speed = 130;  
int mode = 0;
int IR_data;

void setup() {
  pinMode(R_S, INPUT); // Declare IR sensor as input  
  pinMode(L_S, INPUT); // Declare IR sensor as input

  pinMode(echo, INPUT);   // Declare ultrasonic sensor Echo pin as input
  pinMode(trigger, OUTPUT); // Declare ultrasonic sensor Trigger pin as output  

  pinMode(enA, OUTPUT); // Declare as output for L298 Pin enA 
  pinMode(in1, OUTPUT); // Declare as output for L298 Pin in1 
  pinMode(in2, OUTPUT); // Declare as output for L298 Pin in2 
  pinMode(in3, OUTPUT); // Declare as output for L298 Pin in3   
  pinMode(in4, OUTPUT); // Declare as output for L298 Pin in4 
  pinMode(enB, OUTPUT); // Declare as output for L298 Pin enB 

  irrecv.enableIRIn(); // Start the IR receiver
  irrecv.blink13(true);

  Serial.begin(9600);      // Start serial communication at 9600bps
  BT_Serial.begin(9600);   // Start Bluetooth communication

  pinMode(servo, OUTPUT);

  // Initialize the servo with a test movement
  for (int angle = 70; angle <= 140; angle += 5) {
    servoPulse(servo, angle);
  }
  for (int angle = 140; angle >= 0; angle -= 5) {
    servoPulse(servo, angle);
  }
  for (int angle = 0; angle <= 70; angle += 5) {
    servoPulse(servo, angle);
  }
  delay(500);
}

void loop() {  
  if (BT_Serial.available() > 0) {
    bt_ir_data = BT_Serial.read(); 
    Serial.println(bt_ir_data);     
    if (bt_ir_data > 20) {
      Speed = bt_ir_data;
    }      
  }

  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    bt_ir_data = IRremote_data();
    Serial.println(bt_ir_data); 
    irrecv.resume(); // Receive the next value
    delay(100);
  }

  if (bt_ir_data == 8) { mode = 0; Stop(); }
  else if (bt_ir_data == 9) { mode = 1; Speed = 130; }
  else if (bt_ir_data == 10) { mode = 2; Speed = 255; }

  analogWrite(enA, Speed);
  analogWrite(enB, Speed);

  if (mode == 0) {  
    if (bt_ir_data == 1) { forword(); }  
    else if (bt_ir_data == 2) { backword(); }
    else if (bt_ir_data == 3) { turnLeft(); }
    else if (bt_ir_data == 4) { turnRight(); }
    else if (bt_ir_data == 5) { Stop(); }
    else if (bt_ir_data == 6) { turnLeft(); delay(400); bt_ir_data = 5; }
    else if (bt_ir_data == 7) { turnRight(); delay(400); bt_ir_data = 5; }
  }

  if (mode == 1) {
    if ((digitalRead(R_S) == 0) && (digitalRead(L_S) == 0)) { forword(); }
    if ((digitalRead(R_S) == 1) && (digitalRead(L_S) == 0)) { turnRight(); }
    if ((digitalRead(R_S) == 0) && (digitalRead(L_S) == 1)) { turnLeft(); }
    if ((digitalRead(R_S) == 1) && (digitalRead(L_S) == 1)) { Stop(); }
  } 

  if (mode == 2) {
    distance_F = Ultrasonic_read();
    Serial.print("S=");
    Serial.println(distance_F);
    if (distance_F > set) { forword(); }
    else { Check_side(); }
  }

  delay(10);
}

// Function definitions remain the same as in your original code
// ...
