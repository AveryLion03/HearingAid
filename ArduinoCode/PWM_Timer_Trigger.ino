#define <nrf.h>

#define SCLK_PIN NRF_GPIO_PIN_MAP(0, 17)

// WS PWM Duty Cycle (Approximately 50%)
uint16_t seq1_ram[1] = {250};
void setup(){

  // Setup for SCLK Timer signal
  NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
  NRF_TIMER1->PRESCALER = 0;  // 16 MHz base clock
  NRF_TIMER1->CC[0] = 4;      // Toggle every 5 ticks (~3.2 MHz)
  NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk; // Auto-clear on CC[0]
  NRF_TIMER1->TASKS_START = 1;
  NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos) | (SCLK_PIN << GPIOTE_CONFIG_PSEL_Pos) | (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos);
  NRF_PPI->CH[0].EEP = (uint32_t)&NRF_TIMER1->EVENTS_COMPARE[0]; // TIMER event
  NRF_PPI->CH[0].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0]; // GPIOTE toggle task
  NRF_PPI->CHENSET = PPI_CHENSET_CH0_Msk; // Enable PPI channel

  // Setup for WS PWM signal -> 32 kHz
  NRF_PWM1->PSEL.OUT[0] = (20 << PWM_PSEL_OUT_PIN_Pos) | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);  // Use P0.20 for PWM1
  NRF_PWM1->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);                                              // Enable PWM 1
  NRF_PWM1->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);                                                         // Can choose between up or down
  NRF_PWM1->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_1 << PWM_PRESCALER_PRESCALER_Pos);                                 // 16 MHz
  NRF_PWM1->COUNTERTOP = (500 << PWM_COUNTERTOP_COUNTERTOP_Pos);                                                        // 320 ns
  NRF_PWM1->LOOP = (10 << PWM_LOOP_CNT_Pos);                                                                             // Loop 10 times
  NRF_PWM1->DECODER = (PWM_DECODER_LOAD_Common << PWM_DECODER_LOAD_Pos) | (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
  NRF_PWM1->SEQ[0].PTR = ((uint32_t)(seq1_ram) << PWM_SEQ_PTR_PTR_Pos);
  NRF_PWM1->SEQ[0].CNT = ((sizeof(seq1_ram) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
  NRF_PWM1->SEQ[0].REFRESH = 1;
  NRF_PWM1->SEQ[0].ENDDELAY = 1;


}
