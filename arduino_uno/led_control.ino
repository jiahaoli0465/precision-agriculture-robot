bool isLedOn = false; 
void setup() {
  Serial.begin(9600);             // Initialize serial communication
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {   // Check if data is available to read
    String command = Serial.readStringUntil('\n');  // Read the incoming command
    command.trim();  // Remove any extra whitespace or newline characters

    if (command == "LED_ON" && !isLedOn) {
      digitalWrite(LED_BUILTIN, HIGH);
      isLedOn = true;
    } else if (command == "LED_OFF" && isLedOn) {
      digitalWrite(LED_BUILTIN, LOW);   
      isLedOn = false;
    } else {
      Serial.println("Unknown command received");
    }
  }
}
