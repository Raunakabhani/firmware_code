#include <NewPing.h>

#define ULTRASONIC_NUM 3      // Number of sensors.
#define MAX_DISTANCE 450 // Maximum distance (in cm) to ping.

NewPing sonar[ULTRASONIC_NUM] = {   // Sensor object array.
  NewPing(4, 5, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(6, 7, MAX_DISTANCE), 
  NewPing(8, 9, MAX_DISTANCE)
};

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
}

void loop() { 
  static const char* sensorNames[ULTRASONIC_NUM] = {"left ultrasonic", "middle ultrasonic", "right ultrasonic"};
  
  for (uint8_t i = 0; i < ULTRASONIC_NUM; i++) { // Loop through each sensor and display results.
    delay(100); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    Serial.print(sensorNames[i]);
    Serial.print(" - ");
    Serial.print(sonar[i].ping_cm());
    Serial.print("cm ");
  }
  Serial.println();
}
