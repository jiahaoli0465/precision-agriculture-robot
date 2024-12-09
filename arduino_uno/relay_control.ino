const int relayPin = 7;  // Pin connected to the relay
bool isRelayOn = false; 

void setup() {
  pinMode(relayPin, OUTPUT);       // Set relay pin as output
  digitalWrite(relayPin, LOW);    // Ensure relay starts OFF
  Serial.begin(9600);             // Start serial communication
}

void loop() {
  if (Serial.available() > 0) {  // Check if data is available to read
    String command = Serial.readStringUntil('\n');  // Read the incoming message
    command.trim();  // Remove any extra whitespace or newline characters

    if (command == "RELAY_ON" && !isRelayOn) {
      digitalWrite(relayPin, HIGH);  // Turn the relay ON
      isRelayOn = true;
      Serial.println("Relay is ON");
    } else if (command == "RELAY_OFF" && isRelayOn) {
      digitalWrite(relayPin, LOW);   // Turn the relay OFF
      isRelayOn = false;
      Serial.println("Relay is OFF");
    } else {
      Serial.println("Unknown command received");
    }
  }
}
