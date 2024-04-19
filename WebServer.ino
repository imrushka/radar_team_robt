#include <SPI.h>
#include <Ethernet.h>
#include <NewPing.h> // Include the NewPing library
#include <Servo.h>

Servo servo; // Create a servo object to control the servo motor

const int encoderDT = 3;   // DT pin of the rotary encoder connected to pin 5
const int encoderCLK = 2;  // CLK pin of the rotary encoder connected to pin 6
const int trigPin = 5;     // Ultrasonic sensor trig pin (2nd pin)
const int echoPin = 6;
int distance = 0;     // Ultrasonic sensor echo pin (3rd pin)

#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters).
#define SERVO_PIN 4 // Pin for controlling the servo

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xEF, 0xED}; // MAC address of Ethernet shield

EthernetServer server(80);
String readString;
NewPing sonar(trigPin, echoPin, MAX_DISTANCE); // Create a NewPing object

int angle = 90; // Initial angle for the servo

void setup() {
  Serial.begin(9600);
  Ethernet.begin(mac);
  server.begin();
  Serial.print("Server is at: ");
  Serial.println(Ethernet.localIP());

  servo.attach(SERVO_PIN); // Attach the servo to SERVO_PIN
  pinMode(encoderDT, INPUT);
  pinMode(encoderCLK, INPUT);
  pinMode(trigPin, OUTPUT); // Set trig pin as output
  pinMode(echoPin, INPUT);  // Set echo pin as input
  attachInterrupt(digitalPinToInterrupt(encoderCLK), updateEncoder, CHANGE);
}

void loop() {
  EthernetClient client = server.available();
  
  servo.write(angle);
  distance = sonar.ping_cm();

  
  if (client) {
    if (client.connected()) {
      // Read ultrasonic sensor distance
      int distance = sonar.ping_cm();

      // Create JSON data
      String jsonData = "{\"angle\": " + String(angle) + ", \"distance\": " + String(distance) + "}";

      // Send JSON response to client
      client.println("HTTP/1.1 200 OK");
      client.println("Access-Control-Allow-Origin: *"); // Allow requests from any origin
      client.println("Content-Type: application/json");
      client.println("Connection: close");
      client.println();
      client.println(jsonData);

      delay(1000); // Delay to prevent flooding
      client.stop();
    }
  }
}


int currentClkState = 0;
int currentDtState = 0;
static int previousClkState = 0;
void updateEncoder() {
  currentClkState = digitalRead(encoderCLK);
  currentDtState = digitalRead(encoderDT);

  // Check for a rising edge on CLK
  if (currentClkState != previousClkState && currentClkState == HIGH) {
    // If CLK is high and DT is low, it's a clockwise rotation
    if (currentDtState == LOW) {
      angle += 5; // Increment angle for clockwise rotation
    } else {
      angle -= 5; // Decrement angle for counterclockwise rotation
    }

    // Ensure angle stays within range 0-180
    angle = constrain(angle, 0, 180);
  }
  delay(15);
  // Save the current CLK state for the next iteration
  previousClkState = currentClkState;
}
