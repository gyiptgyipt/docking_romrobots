// Define the analog pins for the IR sensors
const int irSensor1Pin = 4;


void setup() {
  // Initialize IR sensor pins as input
  pinMode(irSensor1Pin, INPUT);


  // Start Serial communication for debugging
  Serial.begin(9600);
  Serial.println("Start");
}

void loop() {
  // Read analog values from the IR sensors
  int irSensor1Value = analogRead(irSensor1Pin);

  // Print sensor values for debugging
  Serial.print("IR Sensor : ");
  Serial.println(irSensor1Value);


  // Add a short delay to stabilize readings
  delay(100);
}