// Library will be imported
#include <IRremote.h>

// Define the IR Transmitter Module pins for three transmitters
int SEND_PIN1 = 3; // First IR transmitter
int SEND_PIN2 = 4; // Second IR transmitter
int SEND_PIN3 = 5; // Third IR transmitter

// Create IRsend objects for each pin
IRsend irsend1(SEND_PIN1); 
IRsend irsend2(SEND_PIN2); 
IRsend irsend3(SEND_PIN3);

void setup() {
  Serial.begin(9600); // Initialize serial interface
}

void loop() {
  // Transmit data A, B, C for each IR transmitter
  for (int i = 0; i < 10; i++) {
    // Send data 'A' using first IR transmitter (SEND_PIN1)
    irsend1.sendSony(0xA90, 12); // Code for 'A'
    Serial.println("Sent A on PIN 3");
    
    // Send data 'B' using second IR transmitter (SEND_PIN2)
    irsend2.sendSony(0xB90, 12); // Code for 'B'
    Serial.println("Sent B on PIN 4");
    
    // Send data 'C' using third IR transmitter (SEND_PIN3)
    irsend3.sendSony(0xC90, 12); // Code for 'C'
    Serial.println("Sent C on PIN 5");

    delay(50);
}
}