#include <Arduino.h>
#include <ESP_I2S.h>

// I2S pin assignments
const uint8_t I2S_MCLK = 0;   // Master clock (if used)
const uint8_t I2S_SCK = 3;    // Bit clock (BCLK)
const uint8_t I2S_WS = 2;     // Word select (LRCLK)
const uint8_t I2S_SDOUT = 1;  // Data output (to speaker)
const uint8_t I2S_SDIN = 4;   // Data input (from microphone)

I2SClass i2s;  // Create an instance of the ESP_I2S class
// I2S configuration settings (double-check these match your device needs)
i2s_data_bit_width_t dataWidth = I2S_DATA_BIT_WIDTH_32BIT;  // Currently using 32-bit words
i2s_mode_t mode = I2S_MODE_STD;                             // Standard I2S mode
i2s_slot_mode_t slot = I2S_SLOT_MODE_MONO;                  // MONO mode
i2s_chan_handle_t tx_handle;
i2s_chan_handle_t rx_handle;
i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle);

i2s_std_config_t std_cfg = {
  .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
  .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
  .gpio_cfg = {
    .mclk = I2S_GPIO_UNUSED,
    .bclk = GPIO_NUM_3,
    .ws = GPIO_NUM_2,
    .dout = GPIO_NUM_1,
    .din = GPIO_NUM_4,
    .invert_flags = {
      .mclk_inv = false,
      .bclk_inv = false,
      .ws_inv = false,
    },
  },
};

i2s_channel_init_std_mode(tx_handle, &std_cfg);
i2s_channel_init_std_mode(rx_handle, &std_cfg);

i2s_channel_enable(tx_handle);
i2s_channel_enable(rx_handle);

const uint8_t BLUE_LED = 8;  // Status LED

// Global variables for filtering (customize these as needed)
const float alpha = 0.5;  // Example filter coefficient; adjust to match your filter design.
int32_t prev_input = 0;
int32_t prev_output = 0;

#define N_TAPS 4
const float firCoeffs[N_TAPS] = { 0.25, 0.25, 0.25, 0.25 };  // Coefficients for a basic moving average filter
int32_t firBuffer[N_TAPS] = { 0 };                           // Buffer to store the last N_TAPS samples

// Function to apply the FIR filter to each new sample
int32_t applyFIRFilter(int32_t newSample) {
  // Shift previous samples to make room for the new sample
  for (int i = N_TAPS - 1; i > 0; i--) {
    firBuffer[i] = firBuffer[i - 1];
  }
  firBuffer[0] = newSample;

  // Compute the filtered output as the weighted sum of the samples
  float output = 0.0;
  for (int i = 0; i < N_TAPS; i++) {
    output += firCoeffs[i] * firBuffer[i];
  }
  return (int32_t)output;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing I2S...");

  // Set up the pin directions for any extra indicators
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, LOW);

  // Assign I2S pins. Note the order: SCK, WS, SDOUT, SDIN, MCLK.
  // Make sure these pins match your wiring and board capabilities.
  i2s.setPins(I2S_SCK, I2S_WS, I2S_SDOUT, I2S_SDIN, I2S_MCLK);

  // Initialize the I2S peripheral.
  if (!i2s.begin(I2S_MODE_STD, 44100, dataWidth, slot)) {
    Serial.println("Failed to start I2S");
    while (1) { delay(100); }
  }
  Serial.println("I2S initialized successfully.");
}

void loop() {
  const size_t BUFFER_SIZE = 512;  // Size in bytes—must be a multiple of the sample size (e.g., 4 for 32-bit samples)
  char buffer[BUFFER_SIZE];
  size_t bytesRead = i2s.readBytes(buffer, BUFFER_SIZE);

  if (bytesRead > 0) {
    // Cast to a 32-bit sample pointer and determine the number of samples.
    int32_t* samples = (int32_t*)buffer;
    size_t numSamples = bytesRead / sizeof(int32_t);

    // Process each sample similarly to your nRF52840 loop.
    for (size_t i = 0; i < numSamples; i++) {
      int32_t current_input = samples[i];

      // Basic thresholding: If the sample is small, set it to zero.
      if (current_input <= 20000 && current_input >= -20000)
        current_input = 0;

      // Example of a simple high-pass filter calculation:
      long filtered = alpha * (prev_output + current_input - prev_input);

      // Update state for filter continuity.
      prev_input = current_input;
      prev_output = filtered;

      // Optionally amplify the signal.
      samples[i] = filtered * 4;
    }

    // Write the filtered data back out through I2S.
    i2s.write((uint8_t*)buffer, bytesRead);
  }

  // Optionally, add an activity indicator or yield.
  digitalWrite(BLUE_LED, !digitalRead(BLUE_LED));
}


