#include <Arduino.h>
#include <pid_controller.h>
#include <serial_bridge.h>

// Variables
DATA_VECTOR recive_data;   // Input data received from serial
DATA_VECTOR control;       // Output control signals to send via serial

// Initialize level PID controller with specific gains
pid_controller pid_level(1.0, 0.0, 0.0);

// Initialize pressure PID controller with specific gains
pid_controller pid_pressure(1.0, 0.0, 0.0);

// Timing control (100 ms)
const float dt = 0.1;         // Time step in seconds
const unsigned long Ts = 100; // Sampling period in milliseconds
unsigned long last_time = 0;  // Last loop execution timestamp
unsigned long now = 0;        // Current timestamp

void setup()
{
  pinMode(2, OUTPUT);

  // Start serial communication at 115200 bps
  Serial.begin(115200);

  // Wait until serial is ready (important on some platforms)
  while (!Serial) {}

  // Turn on LED
  digitalWrite(2, HIGH);

  // Get initial time
  last_time = millis();
}

void loop() 
{
  // Read input data from serial
  recive_data = serial_read_vector();

  now = millis();

  // Execute PID control every Ts milliseconds
  if (now - last_time >= Ts) 
  {
    last_time = now;

    // Compute control signals using PID controllers
    control.numbers[0] = pid_level.pid_calculation(recive_data.numbers[0], dt);
    control.numbers[1] = pid_pressure.pid_calculation(recive_data.numbers[1], dt);
  }

  // Send control signals back via serial
  serial_write_vector(control.numbers[0], control.numbers[1]);

  // Delay to avoid flooding the serial port
  delay(100);
}