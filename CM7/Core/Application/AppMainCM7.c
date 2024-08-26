#include "AppMainCM7.h"

static void Steering_PWMInit(const uint32_t internalTimerClock, const uint32_t sysclk, uint32_t* const CCRmin, uint32_t* const CCRmax);

uint8_t data[ESP_PACKET_SIZE] = {0};
uint8_t txData[4] = {0xAA, 0xBB, 0xCC, 0xDD};  // Data to send back
uint8_t receivedData[4] = {0};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);

    HAL_UART_Receive_IT(&huart2, data, 1);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_SPI_TransmitReceive_IT(&hspi1, txData, receivedData, sizeof(txData));
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); // Red
}

void AppCM7_Main()
{
    const uint32_t timerClk = ClockHandling_GetTimerClkFreq(&htim2);
    const uint32_t sysClk   = HAL_RCC_GetSysClockFreq();
    uint32_t CCRmin = 0; 
    uint32_t CCRmax = 0;
    uint8_t send = '6';
    uint32_t dataForEsp = 0x1234567F;
    uint8_t id = 0x5;


    Steering_PWMInit(timerClk, sysClk, &CCRmin, &CCRmax);

    TIM2->CCR1 = CCRmax;
    // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    // HAL_UART_Receive_IT(&huart2, data, 1);

    // if( Serializer_DataForESP(id, dataForEsp, data) != true )
    // {
    //     //error
    //     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    // }
    HAL_SPI_TransmitReceive_IT(&hspi1, txData, receivedData, sizeof(txData));
    while(1)
    {
    	//HAL_UART_Transmit(&huart2, &data, sizeof(data), HAL_MAX_DELAY);
    	// HAL_Delay(2000);
    	//HAL_SPI_TransmitReceive(&hspi1, txData, receivedData, 4, HAL_MAX_DELAY);
        //HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
        // HAL_Delay(500);
    }
}

static void Steering_PWMInit(const uint32_t internalTimerClock, const uint32_t sysclk, uint32_t* const CCRmin, uint32_t* const CCRmax)
{
    uint8_t     presc   = 0u;
    uint8_t     freq    = 50u;
    uint8_t     Dmin    = 5u;
    uint8_t     Dmax    = 10u;
    uint32_t    ARR     = 0u;
    uint32_t    timClk  = 0u;

    if( sysclk < 100000000) // 100MHz
    {
        presc = 4;
    }
    else
    {
        presc = 8;
    }

    timClk  = internalTimerClock / presc;
    ARR     = timClk / freq;

    *CCRmin = (Dmin * ARR) / 100;
    *CCRmax = (Dmax * ARR) / 100;

    htim2.Init.Prescaler    = presc - 1;
    htim2.Init.Period       = ARR;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
      Error_Handler();
    }
}
