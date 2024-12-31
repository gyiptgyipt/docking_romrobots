// Include the IRremote library
#include <IRremote.h>

const int RECV_PIN = 6; // Pin connected to the IR receiver module
IRrecv irrecv(RECV_PIN);  // Create an IRrecv object
decode_results results;   // Object to store the decoded IR signal

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  irrecv.enableIRIn(); // Start the receiver
  Serial.println("IR Receiver ready...");
}

void loop() {
  if (irrecv.decode(&results)) {  // Check if we received an IR signal
    long int decCode = results.value; // Get the decoded signal value

    // Print the decoded signal value
    //Serial.print("Received code: ");
    //Serial.println(decCode, HEX);  // Print in hexadecimal format

    // You can add conditions to check for specific codes
    if (decCode == 0xF4BA2988) {
      Serial.println("Received A");
    } else if (decCode == 0xE0E040BF) {
      Serial.println("Received B");
    } else if (decCode == 0x4004) {
      Serial.println("Received C");
    }

    irrecv.resume();  // Prepare to receive the next signal
  }
}
