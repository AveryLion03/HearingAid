#include <nrf.h>
#include "Adafruit_TinyUSB.h"

// Define 32 bit output
#ifndef I2S_CONFIG_SWIDTH_SWIDTH_32BIT
  #define I2S_CONFIG_SWIDTH_SWIDTH_32BIT (2UL)
#endif

// I2S pin definitions (adjust as needed)
#define I2S_WS_PIN    NRF_GPIO_PIN_MAP(1, 4)   // LRCK (Word Select)
#define I2S_SCK_PIN   NRF_GPIO_PIN_MAP(0, 9)   // Serial Clock (SCK)
#define I2S_SDIN_PIN  NRF_GPIO_PIN_MAP(1, 6)   // Serial Data In (from microphone)
#define I2S_SDOUT_PIN NRF_GPIO_PIN_MAP(0, 10)  // Serial Data Out (to speaker)

// Audio buffer settings
#define DMA_BUF_SIZE 256         // Number of mono samples from microphone
__ALIGN(4) static int32_t dma_buffer_a[DMA_BUF_SIZE];   // Mic input buffer (mono 24-bit in 32-bit container)
__ALIGN(4) static int32_t dma_buffer_b[DMA_BUF_SIZE];   // Speaker output buffer

// LED (for debugging/heartbeat)
#define ORANGE_LED NRF_GPIO_PIN_MAP(0, 15)

// High-pass filter parameters
// Filter parameters for cutoff frequency of 300Hz and sample rate of 62500Hz
const float f_c = 300.0;        // Cutoff frequency in Hz
const float f_s = 62500.0;      // Sample frequency in Hz
const float T = 1.0 / f_s;      // Sampling period
const float RC = 1.0 / (2.0 * 3.14159265 * f_c); // Time constant
const float alpha = RC / (RC + T);  // Filter coefficient, approx 0.9707
long previousInput = 0;
long previousOutput = 0;

// Variables to store previous state for filtering
int32_t prev_input = 0;
int32_t prev_output = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("I2S Mic → Speaker with High-Pass Filter");

  // Set up LED for heartbeat (active-low assumed)
  nrf_gpio_cfg_output(ORANGE_LED);
  nrf_gpio_pin_set(ORANGE_LED);  // Turn off LED initially

  // --- I2S Configuration ---
  NRF_I2S->CONFIG.TXEN  = I2S_CONFIG_TXEN_TXEN_Enabled << I2S_CONFIG_TXEN_TXEN_Pos;   // Enable TX (Speaker)
  NRF_I2S->CONFIG.RXEN  = I2S_CONFIG_RXEN_RXEN_Enabled << I2S_CONFIG_RXEN_RXEN_Pos;   // Enable RX (Mic)
  NRF_I2S->CONFIG.MCKEN = I2S_CONFIG_MCKEN_MCKEN_Enabled << I2S_CONFIG_MCKEN_MCKEN_Pos; // Enable Master Clock
  NRF_I2S->CONFIG.MCKFREQ = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV8 << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos; // MCK = 4 MHz
  NRF_I2S->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_64X << I2S_CONFIG_RATIO_RATIO_Pos;  // Sample Ratio = 64X
  NRF_I2S->CONFIG.MODE  = I2S_CONFIG_MODE_MODE_Master << I2S_CONFIG_MODE_MODE_Pos;   // Master Mode
  NRF_I2S->CONFIG.SWIDTH = I2S_CONFIG_SWIDTH_SWIDTH_24BIT << I2S_CONFIG_SWIDTH_SWIDTH_Pos; // For 24-bit
  NRF_I2S->CONFIG.ALIGN  = I2S_CONFIG_ALIGN_ALIGN_Left << I2S_CONFIG_ALIGN_ALIGN_Pos; // Left-aligned
  NRF_I2S->CONFIG.FORMAT = I2S_CONFIG_FORMAT_FORMAT_I2S << I2S_CONFIG_FORMAT_FORMAT_Pos; // I2S format

  // Keep Left-Only Mode for mono system.
  NRF_I2S->CONFIG.CHANNELS = I2S_CONFIG_CHANNELS_CHANNELS_Left << I2S_CONFIG_CHANNELS_CHANNELS_Pos;

  // --- I2S Pin Assignments ---
  NRF_I2S->PSEL.SCK   = (I2S_SCK_PIN << I2S_PSEL_SCK_PIN_Pos);
  NRF_I2S->PSEL.LRCK  = (I2S_WS_PIN << I2S_PSEL_LRCK_PIN_Pos);
  NRF_I2S->PSEL.SDIN  = (I2S_SDIN_PIN << I2S_PSEL_SDIN_PIN_Pos);
  NRF_I2S->PSEL.SDOUT = (I2S_SDOUT_PIN << I2S_PSEL_SDOUT_PIN_Pos);

  // --- Set Buffers ---
  NRF_I2S->RXD.PTR = (uint32_t)dma_buffer_a;  // Mic RX buffer pointer
  NRF_I2S->TXD.PTR = (uint32_t)dma_buffer_b;  // Speaker TX buffer pointer
  NRF_I2S->RXTXD.MAXCNT = DMA_BUF_SIZE;       // Buffer size in samples

  // --- Enable I2S ---
  NRF_I2S->ENABLE = 1;
  NRF_I2S->TASKS_START = 1;

  Serial.println("I2S Started");
}

void loop() {
  // Check if RX buffer is ready with new data
  if (NRF_I2S->EVENTS_RXPTRUPD) {
    NRF_I2S->EVENTS_RXPTRUPD = 0; // Clear event

    // Process each sample using a high-pass filter to reduce low-frequency wind noise.
    for (int i = 0; i < DMA_BUF_SIZE; i++) {
      int32_t current_input = dma_buffer_a[i];
      if(current_input <= 20000 && current_input >= -20000) current_input = 0;
      
      // Simple first-order high-pass filter:
      //   y[n] = x[n] - alpha*x[n-1] + beta*y[n-1]
      long filtered = alpha * (prev_output + current_input - prev_output);
      
      // Update state variables for next iteration:
      prev_input = current_input;
      prev_output = filtered;
      // if(filtered <= 10000 && filtered >= -10000) filtered = 0;
      dma_buffer_b[i] = filtered * 4; //amplify signal by 2 -> see what value works best?

      // Optionally, print every 8th filtered sample to the Serial Plotter for debugging.
      if (i % 8 == 0) {
        Serial.println(filtered);
      }
    }

    // Update the TX DMA pointer after processing.
    NRF_I2S->TXD.PTR = (uint32_t)dma_buffer_b;
  }

  // Optional: Blink the LED every 500ms as a heartbeat.
  static unsigned long lastBlinkTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastBlinkTime >= 500) {
    lastBlinkTime = currentTime;
    nrf_gpio_pin_toggle(ORANGE_LED);
  }
}
