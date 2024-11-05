/* USER CODE BEGIN Header */
/**
TIM7 - kontroliuoja ADSR
TIM7 -
**/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "MIDI.h"
#include "MidiHandlers.h"
#include "mcp4728_mod.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <adsr.h>
#include "lookup_t.h"
#include "midi2freq.h"
#include "lfo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define X 0.25 // santykis
#define NUM_ENVELOPES 4

// Define the MIDI CC numbers for the ADSR parameters
#define ATTACK_CC 74     // Brightness or Attack CC number
#define DECAY_CC 71      // Resonance or Decay CC number
#define SUSTAIN_CC 22    // Decay Time or Sustain level CC number
#define RELEASE_CC 75    // Release Time CC number
#define AMPLITUDE_CC 12   // Volume or Amplitude CC number
#define MAX_MIDI_CHANNELS 2
//#define DMA_RX_BUFFER_SIZE 256

//#define SystemCoreClock 72000000

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */


//uint8_t dma_rx_buffer[DMA_RX_BUFFER_SIZE];
//uint8_t dma_rx_data[DMA_RX_BUFFER_SIZE];
//volatile uint16_t dma_rx_index = 0;
// volatile bool update_adsr_flag = false;

/* Midi kintamieji */
/* 4 balsai */
bool first_note_active = false;
bool second_note_active = false;
bool third_note_active = false;
bool fourth_note_active = false;
uint32_t pitch1_CV;
uint32_t pitch2_CV;
uint32_t pitch3_CV;
uint32_t pitch4_CV;
uint8_t first_note;
uint8_t second_note;
uint8_t third_note;
uint8_t fourth_note;
/*  */

uint32_t ramp_counter = 0;
uint32_t count = 0;
uint16_t minFrequency = 100;   // Minimum frequency in Hz
uint16_t maxFrequency = 1000;  // Maximum frequency in Hz
uint32_t new_period = 0;
uint32_t dac_value;

extern ADSR_t adsr;
float env1;
ADSR_t envelopes[NUM_ENVELOPES]; // Array to hold the ADSR envelopes





// midi natos pagal kanalus kintamieji
bool note_active[MAX_MIDI_CHANNELS] = {false};
uint32_t pitch_CV[MAX_MIDI_CHANNELS];
uint32_t velo_CV[MAX_MIDI_CHANNELS];
uint8_t current_note[MAX_MIDI_CHANNELS];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_DAC2_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
 {
	 int DataIdx;
	 for (DataIdx = 0; DataIdx < len; DataIdx++)
	 {
		 ITM_SendChar(*ptr++);
	 }
	 return len;
 }

//typedef struct {
//    byte attack_value;
//    byte decay_value;
//    byte sustain_value;
//    byte release_value;
//    byte amplitude_value;
//} ADSR_CC_Values;

// Array to store the last known MIDI CC values for each envelope
//ADSR_CC_Values cc_values[NUM_ENVELOPES];

/* Initialize MidiInterface object */
MidiInterface Port;

void Handle_NoteOn(uint8_t status, uint8_t data1, uint8_t data2);
void Handle_NoteOff(uint8_t channel, uint8_t note, uint8_t velocity);
void Handle_CC(byte channel, byte number, byte value);
void Handle_CC16(byte channel, byte number, byte value);
void ADSR_HandleCC(byte channel, byte number, byte value);

// Add new variables for third and fourth notes


void Handle_NoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {
    uint32_t pitch_CV = (uint32_t)((note * 0.0833333333 * X) / (3.3 / 4095));  // Calculate pitch CV from MIDI note
    uint32_t velo_CV = (uint32_t)((velocity / 127.0) * 4095);
    ChannelConfig config;
    ChannelConfig_2 config2;

    if (!first_note_active) {
        pitch1_CV = pitch_CV;

        ADSR_SetGateSignal(&envelopes[0], 1);
        config.val[0] = pitch1_CV; // 12-bit DAC value for channel A
        config2.val[0] = velo_CV;  // 12-bit DAC value for channel B
        DACx60SW(&hi2c1, config, 0);
        DACx61SW(&hi2c1, config2, 0);

        HAL_GPIO_WritePin(GPIOB, gate1_Pin, GPIO_PIN_SET);  // Indicate first note is on via gate3_Pin
        first_note_active = true;  // First note is now active
        first_note = note;  // Store the note value
    } else if (!second_note_active) {
        pitch2_CV = pitch_CV;

        ADSR_SetGateSignal(&envelopes[1], 1);
        config.val[1] = pitch2_CV; // 12-bit DAC value for channel A
        config2.val[1] = velo_CV;  // 12-bit DAC value for channel B
        DACx60SW(&hi2c1, config, 1);
        DACx61SW(&hi2c1, config2, 1);

        HAL_GPIO_WritePin(GPIOB, gate2_Pin, GPIO_PIN_SET);  // Indicate second note is on via gate2_Pin
        second_note_active = true;  // Second note is now active
        second_note = note;  // Store the note value
    } else if (!third_note_active) {
        pitch3_CV = pitch_CV;

        ADSR_SetGateSignal(&envelopes[2], 1);
        config.val[2] = pitch3_CV; // 12-bit DAC value for channel A
        config2.val[2] = velo_CV;  // 12-bit DAC value for channel B
        DACx60SW(&hi2c1, config, 2);
        DACx61SW(&hi2c1, config2, 2);

        HAL_GPIO_WritePin(GPIOB, gate3_Pin, GPIO_PIN_SET);  // Indicate third note is on via gate1_Pin
        third_note_active = true;  // Third note is now active
        third_note = note;  // Store the note value
    } else if (!fourth_note_active) {
        pitch4_CV = pitch_CV;

        ADSR_SetGateSignal(&envelopes[3], 1);
        config.val[3] = pitch4_CV; // 12-bit DAC value for channel A
        config2.val[3] = velo_CV;  // 12-bit DAC value for channel B
        DACx60SW(&hi2c1, config, 3);
        DACx61SW(&hi2c1, config2, 3);

        HAL_GPIO_WritePin(GPIOE, gate4_Pin, GPIO_PIN_SET);  // Indicate fourth note is on via gate4_Pin
        fourth_note_active = true;  // Fourth note is now active
        fourth_note = note;  // Store the note value
    }
}

void Handle_NoteOff(uint8_t channel, uint8_t note, uint8_t velocity) {
    if (first_note_active && note == first_note) {
        ADSR_SetGateSignal(&envelopes[0], 0);

        HAL_GPIO_WritePin(GPIOB, gate1_Pin, GPIO_PIN_RESET);  // PB2 Turn off gate for first note
        first_note_active = false;  // First note is no longer active
    } else if (second_note_active && note == second_note) {
        ADSR_SetGateSignal(&envelopes[1], 0);
        HAL_GPIO_WritePin(GPIOB, gate2_Pin, GPIO_PIN_RESET);  // Turn off gate for second note
        second_note_active = false;  // Second note is no longer active
    } else if (third_note_active && note == third_note) {
        ADSR_SetGateSignal(&envelopes[2], 0);
        HAL_GPIO_WritePin(GPIOB, gate3_Pin, GPIO_PIN_RESET);  // Turn off gate for third note
        third_note_active = false;  // Third note is no longer active
    } else if (fourth_note_active && note == fourth_note) {
        ADSR_SetGateSignal(&envelopes[3], 0);
        HAL_GPIO_WritePin(GPIOE, gate4_Pin, GPIO_PIN_RESET);  // Turn off gate for fourth note
        fourth_note_active = false;  // Fourth note is no longer active
    }
}


//void setupPWM(uint32_t frequency) {
//    uint32_t period = (SystemCoreClock / frequency) - 1;
//
//    // Update the timer period
//    __HAL_TIM_SET_AUTORELOAD(&htim2, period);
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, period / 2); // 50% duty cycle
//    __HAL_TIM_SET_COUNTER(&htim2, 0); // Reset counter
//
//    // Start PWM if not already started
//    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
//        // PWM start error handling
//    }
//}
// Convert envelope value (0.0 - 1.0) to DAC value (0 - 4095)
uint32_t envelope_to_dac_value(float envelope_value) {
    return (uint32_t)(envelope_value * 4095.0f);
}
    // Additional configuration code
//void TIM8_UP_TIM13_IRQHandler(void)
//{
//  HAL_TIM_IRQHandler(&htim13);
//}
//void Update_TIM13_Frequency(uint8_t midi_value) {
//	uint32_t frequency = lfo_lookup[midi_value];
//	new_period = (72000000 / frequency) - 1; // Example calculation
//    __HAL_TIM_SET_AUTORELOAD(&htim13, new_period);
////    __HAL_TIM_SET_COMPARE(&htim13, new_period / 2); // 50% duty cycle
////    __HAL_TIM_SET_COUNTER(&htim13, 0); // Reset counter
//    if (HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1) != HAL_OK) {
//        // PWM start error handling
//    }
//}

//void HandleCC(byte channel, byte number, byte value) {
//    if (number == 13) {
//    			uint32_t modv_CV = (uint32_t)((value / 127.0) * 4095);
//    			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, modv_CV);
////        midi_cc40_value = value;
//        Update_TIM13_Frequency(value);
//    }
//    // Handle other CC messages
//}

//void ADSR_HandleCC(byte channel, byte number, byte value) {
////    if (channel < NUM_ENVELOPES && update_adsr_flag) {
//
//	int envelope_index = channel - 1;
////    	if (channel < NUM_ENVELOPES) {
//    	    if (envelope_index >= 0 && envelope_index < NUM_ENVELOPES) {
//    	        ADSR_t *adsr = &envelopes[envelope_index];  // Get the corresponding ADSR envelope
//
//
//        switch (number) {
//        case ATTACK_CC: // Brightness controls Attack
////        	oled("attack");
////        	cc_values[envelope_index].attack_value = value;
////            adsr->attack_rate = attack_rate_lookup[value];
//            adsr->attack_rate = attack_rate_lookup[64];
//
////            oled2("A: %d", value);
//            break;
//
//        case DECAY_CC: // Resonance controls Decay
////        	cc_values[envelope_index].decay_value = value;
////            adsr->decay_rate = attack_rate_lookup[value];   // Scale 0-127 to 0.001 - 0.01
//        	adsr->decay_rate = attack_rate_lookup[64];
//            break;
//
//        case RELEASE_CC: // Release Time controls Release
////        	cc_values[envelope_index].release_value = value;
////            adsr->release_rate = attack_rate_lookup[value]; // Scale 0-127 to 0.001 - 0.01
//        	adsr->release_rate = attack_rate_lookup[64];
//            break;
//
//        case SUSTAIN_CC: // Decay Time controls Sustain
////        	cc_values[envelope_index].sustain_value = value;
////            adsr->sustain_level = value / 127.0f;          // 2ia manau nereik float, pakaks ma=esnies reik6mies
//        	adsr->sustain_level = 64;
//            break;
//
//        case AMPLITUDE_CC:
////            cc_values[envelope_index].amplitude_value = value;  // Store amplitude value
////            adsr->amplitude = value / 127.0f;  // Scale amplitude from 0-127 to 0.0-1.0
//        	adsr->amplitude = 127;
//            break;
//
////        case 13:  // Handle MIDI CC13 for DAC output
////  //      	    			uint32_t modv_CV = (uint32_t)((value / 127.0) * 4095);
////        	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)((value / 127.0) * 4095));
//// //           Update_TIM13_Frequency(value);
////            break;
//
//
//        default:
//            // Handle other CC messages or ignore
//
//            break;
//    }
////        update_adsr_flag = false;
//    }
//
//}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

//    if (htim->Instance == TIM13) {
//        update_adsr_flag = true;  // Set the flag to update ADSR parameters
// //       oled("ADSR Flag = TRUE");
//    }

    if (htim->Instance == TIM7) {
/*  ADSR kreivių formavimas naudojant laikmatį.  */
        // Update the first envelope
        ADSR_UpdateEnvelope(&envelopes[0]);
        uint32_t dac_value1 = envelope_to_dac_value(ADSR_GetEnvelopeValue(&envelopes[0]));
        HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value1);
        // Update the second envelope
        ADSR_UpdateEnvelope(&envelopes[1]);
        uint32_t dac_value2 = envelope_to_dac_value(ADSR_GetEnvelopeValue(&envelopes[1]));
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_value2);
        //
        ADSR_UpdateEnvelope(&envelopes[2]);
        uint32_t dac_value3 = envelope_to_dac_value(ADSR_GetEnvelopeValue(&envelopes[2]));
        //
        ADSR_UpdateEnvelope(&envelopes[3]);
        uint32_t dac_value4 = envelope_to_dac_value(ADSR_GetEnvelopeValue(&envelopes[3]));
    }

//    if (htim->Instance == TIM13) {
//        ramp_counter++;
//        if (ramp_counter > 4095) {
//            ramp_counter = 0;
//        }
//        HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ramp_counter);
//    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	char tekstas[] = "LOST MY SHIT";
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_DAC1_Init();
  MX_DAC2_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM7_Init();
  MX_TIM13_Init();
  MX_TIM2_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
	Port.begin(1, &huart1, &huart1);

	Port.setHandleClock(Handle_Clock);
	Port.setHandleStart(Handle_Start);
	Port.setHandleStop(Handle_Stop);
	Port.setHandleNoteOn(Handle_NoteOn);
	Port.setHandleNoteOff(Handle_NoteOff);
//	Port.setHandleControlChange(Handle_CC);
//	Port.setHandleControlChange(Handle_CC16);
//	Port.setHandleControlChange(ADSR_HandleCC);

//	 printf("Great Succes!\n\r");

	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
    HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);
    HAL_SPI_Init(&hspi2);

    ChannelConfig config;
    ChannelConfig_2 config2;
    dac_init(&config, &config2);

    ssd1306_Init();
    ssd1306_SetCursor(5, 50);
    ssd1306_WriteString(tekstas, Font_7x10, White);
    ssd1306_UpdateScreen();

    HAL_TIM_Base_Start_IT(&htim7);
    //HAL_TIM_Base_Start_IT(&htim13);
    //HAL_TIM_Base_Start(&htim2);
    //HAL_TIM_Base_Start(&htim14);
    ADSR_Init(envelopes, NUM_ENVELOPES);

    // Initialize each ADSR envelope and set the default MIDI values
//    for (int i = 0; i < NUM_ENVELOPES; i++) {
//        ADSR_Init(&envelopes[i]);
//
//        // Initialize CC values to some default values (you can use defaults or assume MIDI will update)
//        cc_values[i].attack_value = 64;  // Default to a middle value for attack, you can choose another default
//        cc_values[i].decay_value = 64;
//        cc_values[i].sustain_value = 64;
//        cc_values[i].release_value = 64;
//        cc_values[i].amplitude_value = 127;  // Max volume by default
//
//        // Apply the stored MIDI values to initialize ADSR settings
//        ADSR_HandleCC(i, ATTACK_CC, cc_values[i].attack_value);
//        ADSR_HandleCC(i, DECAY_CC, cc_values[i].decay_value);
//        ADSR_HandleCC(i, SUSTAIN_CC, cc_values[i].sustain_value);
//        ADSR_HandleCC(i, RELEASE_CC, cc_values[i].release_value);
//        ADSR_HandleCC(i, AMPLITUDE_CC, cc_values[i].amplitude_value);
//    }
//    // Temporarily set specific parameters for the second envelope for testing
//    cc_values[1].attack_value = 80;  // Example value for attack
//    cc_values[1].decay_value = 50;   // Example value for decay
//    cc_values[1].sustain_value = 90; // Example value for sustain
//    cc_values[1].release_value = 70; // Example value for release
//    cc_values[1].amplitude_value = 127; // Max volume
//
//    // Apply the specific parameters to the second envelope
//    ADSR_HandleCC(1, ATTACK_CC, cc_values[1].attack_value);
//    ADSR_HandleCC(1, DECAY_CC, cc_values[1].decay_value);
//    ADSR_HandleCC(1, SUSTAIN_CC, cc_values[1].sustain_value);
//    ADSR_HandleCC(1, RELEASE_CC, cc_values[1].release_value);
//    ADSR_HandleCC(1, AMPLITUDE_CC, cc_values[1].amplitude_value);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Port.read();
//        ADSR_SetGateSignal(&envelopes[0], 1);  // Gate signal ON for envelope 0
//        ADSR_SetGateSignal(&envelopes[1], 0);  // Gate signal ON for envelope 1
//        HAL_Delay(500);  // Keep gate signals ON for 500 ms
//
//        // Simulate gate signal OFF for both envelopes
//        ADSR_SetGateSignal(&envelopes[0], 0);  // Gate signal OFF for envelope 0
//        ADSR_SetGateSignal(&envelopes[1], 1);  // Gate signal OFF for envelope 1
//        HAL_Delay(1000);  // Wait for 1 second before repeating

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DAC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC2_Init(void)
{

  /* USER CODE BEGIN DAC2_Init 0 */

  /* USER CODE END DAC2_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC2_Init 1 */

  /* USER CODE END DAC2_Init 1 */

  /** DAC Initialization
  */
  hdac2.Instance = DAC2;
  if (HAL_DAC_Init(&hdac2) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac2, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC2_Init 2 */

  /* USER CODE END DAC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x0010020A;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7199;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 0;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 0;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 31250;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, gate1_Pin|gate2_Pin|gate3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, gate4_Pin|All_trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : gate1_Pin gate2_Pin gate3_Pin */
  GPIO_InitStruct.Pin = gate1_Pin|gate2_Pin|gate3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : gate4_Pin All_trig_Pin */
  GPIO_InitStruct.Pin = gate4_Pin|All_trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	__NOP();
}

//void Handle_NoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {
////    if (channel >= MAX_MIDI_CHANNELS) return; // Ignore channels beyond the max limit
//
//    uint32_t new_pitch_CV = (uint32_t)((note * 0.0833333333 * X) / (3.3 / 4095));  // Calculate pitch CV from MIDI note
//    uint32_t new_velo_CV = (uint32_t)((velocity / 127.0) * 4095);
//    ChannelConfig config;
////    ChannelConfig_2 config2;
//
//    if (!note_active[channel] || note < current_note[channel]) {
//        pitch_CV[channel] = new_pitch_CV;
//        velo_CV[channel] = new_velo_CV;
//        current_note[channel] = note;
//
//        // Calculate frequency from MIDI note
////        float frequency = midi2freq[note];
////        setupPWM((uint32_t)frequency);
//
//        ADSR_SetGateSignal(&envelopes[channel-1], 1);
//        ADSR_SetGateSignal(&envelopes[1], 1);
//        config.val[0] = pitch_CV[1];
//        config.val[1] = pitch_CV[2];// 12-bit DAC value for pitch CV
//        DACx60FW(&hi2c1, config);
//
////        config2.val[channel] = velo_CV[channel];  // 12-bit DAC value for velocity CV
////        DACx61FW(&hi2c1, config2);
//
////        HAL_GPIO_WritePin(GPIOB, gate3_Pin, GPIO_PIN_SET);  // Indicate note is on via gate3_Pin
//        note_active[channel] = true;  // Note is now active
//    }
//}
//
//void Handle_NoteOff(uint8_t channel, uint8_t note, uint8_t velocity) {
////    if (channel >= MAX_MIDI_CHANNELS) return; // Ignore channels beyond the max limit
//
//    if (note_active[channel] && note == current_note[channel]) {
//        ADSR_SetGateSignal(&envelopes[channel-1], 0);
//        ADSR_SetGateSignal(&envelopes[1], 0);
//
////        HAL_GPIO_WritePin(GPIOB, gate3_Pin, GPIO_PIN_RESET);  // Turn off gate for note
//        note_active[channel] = false;  // Note is no longer active
//    }
//}
//void Handle_NoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {
//    uint32_t pitch_CV = (uint32_t)((note * 0.0833333333 * X) / (3.3 / 4095));  // Calculate pitch CV from MIDI note
//    uint32_t velo_CV = (uint32_t)((velocity / 127.0) * 4095);
//    ChannelConfig config;
//    ChannelConfig_2 config2;
//
//    if (!first_note_active) {
//        pitch1_CV = pitch_CV;
//
//
//            // Calculate frequency from MIDI note
////        float frequency = midi2freq[note];
////        setupPWM((uint32_t)frequency);
//
//        ADSR_SetGateSignal(&envelopes[0], 1);
//          config.val[0] = pitch1_CV;
//          config2.val[0] = pitch1_CV; // 12-bit DAC value for channel A
//          config.val[1] = velo_CV;  // 12-bit DAC value for channel B
//    	  DACx60FW(&hi2c1, config);
//    	  DACx61FW(&hi2c1, config2);
//
//        HAL_GPIO_WritePin(GPIOB, gate3_Pin, GPIO_PIN_SET);  // Indicate first note is on via gate3_Pin
//        first_note_active = true;  // First note is now active
//        first_note = note;  // Store the note value
//    }
////     If the first note is active and second note is not yet playing, output second note on DAC_CHANNEL_2
//    else if (!second_note_active) {
//    	ADSR_SetGateSignal(&envelopes[1], 1);
////        pitch2_CV = pitch_CV;
////        HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, pitch2_CV);
//        HAL_GPIO_WritePin(GPIOB, gate2_Pin, GPIO_PIN_SET);  // Indicate second note is on via gate2_Pin
//        second_note_active = true;  // Second note is now active
//        second_note = note;  // Store the note value
//    }
//	}
//
//void Handle_NoteOff(uint8_t channel, uint8_t note, uint8_t velocity) {
//   // If the first note is off
//
//    if (first_note_active && note == first_note)  {
//    	ADSR_SetGateSignal(&envelopes[0], 0);
//
//        HAL_GPIO_WritePin(GPIOB, gate3_Pin, GPIO_PIN_RESET);  // PB2 Turn off gate for first note
//        first_note_active = false;  // First note is no longer active
//        // Note off, stop PWM
////        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
//    }
////     If the second note is off
//    else if (second_note_active && note == second_note) {
//    	ADSR_SetGateSignal(&envelopes[1], 0);
//        HAL_GPIO_WritePin(GPIOB, gate2_Pin, GPIO_PIN_RESET);  // Turn off gate for second note
//        second_note_active = false;  // Second note is no longer active
//
//    }
//}


//void Handle_CC(byte channel, byte number, byte value) {
//	if (number == 13){
//		setupLFO(value);
//	}
//}


//
//	}
//}
//
//void setHandleControlChange(byte channel, byte number, byte value) {
//	if (number == 16){
//
////		adsr->attack_rate = (value / 127.0f) * 0.01f;
////		uint32_t modv_CV = (uint32_t)((value / 127.0) * 4095);
//		ADSR_SetSustainLevel(&envelopes[0], float(value / 127.0f));
//		env1 = value;
//
//
//	}
//}




//void Handle_CC16(byte channel, byte number, byte value) {
//	if (number == GeneralPurposeController1){					//    CC16
//		uint32_t ATTACK = (uint32_t)((value / 127.0) * 4095);
//		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ATTACK);
//	}
//}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
