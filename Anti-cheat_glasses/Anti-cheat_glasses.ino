#include <WiFi.h>
#include <WebSocketsServer.h> // Include the WebSocket server library
#include <math.h>             // Needed for mathematical functions (like squaring)
// Replace with your network credentials
const char* ssid = "OPPO A92";
const char* password = "aya334455";

// WebSocket server runs on port 81 (standard for websockets)
WebSocketsServer webSocket = WebSocketsServer(81);

// Pin definitions
#define fsr1Pin   4               // FSR sensor 1 analog pin
#define fsr2Pin   5               // FSR sensor 2 analog pin 
#define micPin    7               // Analog input pin for MAX4466
#define reset_button_pin  18      // Push button for resetting baseline connected to digital pin 7

// Variables to store baseline FSR values
int fsr1_init = 0;
int fsr2_init = 0;
int threshold = 50;               // Threshold value to detect significant pressure change

// --- Configuration Constants ---
#define SAMPLE_RATE 16000                  // Sample rate: 16,000 samples per second
#define FRAME_SIZE 512                     // Number of samples in one frame (32ms)
#define HOP_SIZE 256                       // Number of samples to slide each time (50% overlap)
#define ENERGY_THRESHOLD 0.01f             // Energy threshold to consider frame as speech
#define ZCR_THRESHOLD 0.3f                 // Zero Crossing Rate threshold to filter out noise
bool speechDetected = false;

// --- Audio Processing Variables ---
int16_t audioBuffer[FRAME_SIZE * 2];       // buffer to hold samples (double frame for overlapping)
int sampleIndex = 0;                       // Index to track current position in audio buffer

// --- Timing Control Variables ---
unsigned long lastMicSampleTime = 0;       // Time of last audio sample (microseconds)
const unsigned long sampleIntervalMicros = 1000000 / SAMPLE_RATE;  // sample period in microseconds(Time between samples=62.5)


void setup() {
  Serial.begin(115200); // Higher baud rate for ESP32 serial communication
  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize WebSocket server event handlers
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Set reset button pin as INPUT with internal pull-up resistor
  pinMode(reset_button_pin, INPUT_PULLUP);

  // Initial baseline setup when the program starts
  setFSR();

  // MAX4466 ADC setup
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Serial.println("System initialized: FSR + VAD");  
}

void loop() {
  webSocket.loop();

  // Reset FSR baseline if button pressed
  if (digitalRead(reset_button_pin) == LOW) {
    setFSR();
    delay(1000);
  }

  // === FSR Reading ===
  int fsr1Value = analogRead(fsr1Pin);
  int fsr2Value = analogRead(fsr2Pin);
  bool fsrCheating = abs(fsr1Value - fsr1_init) > threshold || abs(fsr2Value - fsr2_init) > threshold;

  // === Audio Sampling ===
  unsigned long now = micros();
  if (now - lastMicSampleTime >= sampleIntervalMicros) {
    lastMicSampleTime = now;
    int raw = analogRead(fsr1Pin);  // Change to micPin if you have a separate mic pin
    int16_t sample = (int16_t)((raw - 2048) * 16);
    audioBuffer[sampleIndex++] = sample;

    if (sampleIndex >= FRAME_SIZE) {
      speechDetected = is_speech_frame(audioBuffer, FRAME_SIZE, ENERGY_THRESHOLD);

      // Shift buffer for next frame
      memmove(audioBuffer, audioBuffer + HOP_SIZE, (FRAME_SIZE - HOP_SIZE) * sizeof(int16_t));
      sampleIndex = FRAME_SIZE - HOP_SIZE;
    }
  }

  // === Status Decision ===
  String status = (fsrCheating || speechDetected) ? "Cheating" : "Safe";
  String message = "FSR1:" + String(fsr1Value) +
                   ", FSR2:" + String(fsr2Value) +
                   ", Speech:" + (speechDetected ? "Yes" : "No") +
                   ", Status:" + status;

  // Send message to WebSocket clients
  webSocket.broadcastTXT(message);

  // Print to Serial Monitor for debugging
  Serial.println("====== Monitoring Data ======");
  Serial.println("FSR1 value: " + String(fsr1Value));
  Serial.println("FSR2 value: " + String(fsr2Value));
  Serial.println("Speech detected: " + String(speechDetected ? "Yes" : "No"));
  Serial.println("Final status: " + status);
  Serial.println("=============================");
  Serial.println();

  delay(500);
}

// Function to calculate first values for both FSRs (baseline)
void setFSR() {
  long sum1 = 0;
  long sum2 = 0;

  // Take 10 samples to get an average
  for (int i = 0; i < 10; i++) {
    sum1 += analogRead(fsr1Pin);  // Add FSR1 reading
    sum2 += analogRead(fsr2Pin);  // Add FSR2 reading
    delay(100);
  }

  // Calculate average of the 10 samples
  fsr1_init = sum1 / 10;
  fsr2_init = sum2 / 10;

  // Print the new baselines for verification
  Serial.print("New FSR1 reference: ");
  Serial.println(fsr1_init);
  Serial.print("New FSR2 reference: ");
  Serial.println(fsr2_init);
}

// --- Function: Calculate frame energy ---
float calculate_energy(int16_t* frame, int frame_size) {
  float energy = 0.0f;
  for (int i = 0; i < frame_size; i++) {
    // Convert sample from int16 to float in range [-1.0, +1.0]
    float sample = (float)frame[i] / 32768.0f;   
    energy += sample * sample;    // Accumulate square of the sample
  }
  return energy / frame_size;     // Return average energy
}

// --- Function: Calculate zero-crossing rate (ZCR) ---
float calculate_zcr(int16_t* frame, int frame_size) {
  int zero_crossings = 0;
  for (int i = 1; i < frame_size; i++) {
    // Count a zero-crossing when the sign of the signal changes
    if ((frame[i] >= 0 && frame[i - 1] < 0) || (frame[i] < 0 && frame[i - 1] >= 0)) {
      zero_crossings++;
    }
  }
  return (float)zero_crossings / (frame_size - 1);
}

bool is_speech_frame(int16_t* frame, int frame_size, float energy_threshold) {
  float energy = calculate_energy(frame, frame_size);
  float zcr = calculate_zcr(frame, frame_size);
  return (energy > energy_threshold && zcr < ZCR_THRESHOLD);
}


// WebSocket event handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      // Send a welcome message to the newly connected client
      webSocket.sendTXT(num, "Connected to ESP32 FSR Server!");
    }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] Got Text: %s\n", num, payload);
      // You can add logic here to handle messages sent from the Flutter app
      // For example, if Flutter sends "RESET_BASELINES", call setFSR();
      if (String((char*)payload) == "RESET_BASELINES") {
        Serial.println("Received command to reset baselines from client.");
        setFSR();
        webSocket.sendTXT(num, "Baselines reset successfully!");
      }
      break;
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    // Corrected line:
    case WStype_FRAGMENT_FIN: // Changed from WStype_FRAGMENT_END
      break;
  }
}
