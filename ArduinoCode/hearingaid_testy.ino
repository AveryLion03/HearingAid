#include <nrf.h>
#include "Adafruit_TinyUSB.h"
#include <SPI.h>

// I2S pins for INMP441
#define I2S_WS_PIN NRF_GPIO_PIN_MAP(1, 4)  // P1.04
#define I2S_SCK_PIN NRF_GPIO_PIN_MAP(0, 11) // P0.11
#define I2S_SDIN_PIN NRF_GPIO_PIN_MAP(1, 6)   // P1.06

// TLV5618 DAC pins
#define DAC_CS_PIN NRF_GPIO_PIN_MAP(0, 22) // P0.22 (CS)
#define DAC_CLK_PIN NRF_GPIO_PIN_MAP(0, 24) // P0.24 (SCK)
#define DAC_MOSI_PIN NRF_GPIO_PIN_MAP(1, 0) // P1.00 (MOSI)

// I2S configuration
#define SAMPLE_RATE 44100
#define SAMPLE_BITS 32
#define DMA_BUF_LEN 64
#define DMA_BUF_COUNT 8

// Audio processing
#define BUFFER_SIZE 256
int32_t audioBuffer[BUFFER_SIZE];
int bufferIndex = 0;

//LED
#define LED_BLUE PIN_LED2
#define LED_BLINK_PIN NRF_GPIO_PIN_MAP(0, 2) // P1.13 for blinking LED
#define ORANGE_LED NRF_GPIO_PIN_MAP(0, 15)  // ORANGE LED


void setup() {
  nrf_gpio_cfg_output(LED_BLINK_PIN);
  nrf_gpio_cfg_output(ORANGE_LED);
  nrf_gpio_pin_set(ORANGE_LED);  // Turn off (assuming it's active-low)
  
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Starting I2S configuration");

  // Configure I2S
  
  NRF_I2S->CONFIG.TXEN = (I2S_CONFIG_TXEN_TXEN_ENABLE << I2S_CONFIG_TXEN_TXEN_Pos); // Enable transmission
  Serial.println("TXEN configured");
  NRF_I2S->CONFIG.MCKEN = (I2S_CONFIG_MCKEN_MCKEN_ENABLE << I2S_CONFIG_MCKEN_MCKEN_Pos); // Enable MCK generator
  Serial.println("MCKEN configured");
  NRF_I2S->CONFIG.MCKFREQ = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV11  << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos; // MCKFREQ = 4 MHz
  Serial.println("MCKFREQ configured");
  NRF_I2S->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_64X << I2S_CONFIG_RATIO_RATIO_Pos; // Ratio = 64 
  Serial.println("RATIO configured");
  // Master mode, 16Bit, left aligned
  NRF_I2S->CONFIG.MODE = I2S_CONFIG_MODE_MODE_MASTER << I2S_CONFIG_MODE_MODE_Pos;
  Serial.println("MODE configured");
  NRF_I2S->CONFIG.SWIDTH = I2S_CONFIG_SWIDTH_SWIDTH_16BIT << I2S_CONFIG_SWIDTH_SWIDTH_Pos;
  Serial.println("SWIDTH configured");
  NRF_I2S->CONFIG.ALIGN = I2S_CONFIG_ALIGN_ALIGN_LEFT << I2S_CONFIG_ALIGN_ALIGN_Pos;
  Serial.println("ALIGN configured");
  NRF_I2S->CONFIG.FORMAT = I2S_CONFIG_FORMAT_FORMAT_I2S << I2S_CONFIG_FORMAT_FORMAT_Pos; // Format = I2S
  Serial.println("FORMAT configured");
  NRF_I2S->CONFIG.CHANNELS = I2S_CONFIG_CHANNELS_CHANNELS_STEREO << I2S_CONFIG_CHANNELS_CHANNELS_Pos; // Use stereo 
  Serial.println("CHANNELS configured");

  NRF_I2S->PSEL.SCK = (I2S_SCK_PIN << I2S_PSEL_SCK_PIN_Pos);
  NRF_I2S->PSEL.LRCK = (I2S_WS_PIN << I2S_PSEL_LRCK_PIN_Pos);  // Using WS instead of LRCK
  NRF_I2S->PSEL.SDIN = (I2S_SDIN_PIN << I2S_PSEL_SDOUT_PIN_Pos);  // SDIN for input
  NRF_I2S->PSEL.SDOUT = 0xFFFFFFFF; // Disable SDOUT as we're only 
  Serial.println("PINS configured");


  NRF_I2S->ENABLE = 1;
  Serial.println("I2S Enabled");

  
}

void loop() {
  // Blink the LED connected to P1.13
    nrf_gpio_pin_toggle(LED_BLINK_PIN);  // HIGH
    nrf_gpio_pin_toggle(ORANGE_LED);  // HIGH
    delay(500);
    Serial.println("Toggled");
}

