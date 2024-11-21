/* File: main.c */

#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>

// Define the pin for the Hall effect sensor
#define HALL_SENSOR_PIN GPIO_PIN_1
#define HALL_SENSOR_PORT GPIOA
#define LED_PIN GPIO_PIN_5
#define LED_PORT GPIOA

// UART handle
UART_HandleTypeDef huart2;

// Function Prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

// Debug print function
void Debug_Print(const char *message)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
}

int main(void)
{
    // Initialize the HAL Library
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    Debug_Print("System Initialized. Waiting for sensor input...\r\n");

    // Variable to store previous state
    GPIO_PinState previousState = GPIO_PIN_SET;

    // Main loop
    while (1)
    {
        // Read the Hall effect sensor state
        GPIO_PinState currentState = HAL_GPIO_ReadPin(HALL_SENSOR_PORT, HALL_SENSOR_PIN);
        if (currentState == GPIO_PIN_RESET && previousState == GPIO_PIN_SET)
                {
                    // Magnet detected
                    Debug_Print("Magnet detected, LED ON.\r\n");
                    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);  // Turn LED ON
                }
                else if (currentState == GPIO_PIN_SET && previousState == GPIO_PIN_RESET)
                {
                    // No magnetic field
                    Debug_Print("No magnetic field detected, LED OFF.\r\n");
                    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);  // Turn LED OFF
                }

        // Update previous state
        previousState = currentState;

        HAL_Delay(200);  // Debounce delay
    }
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
    // Add your clock configuration code here
    // CubeMX-generated code typically goes here
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIOA clock
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // No pull-up or pull-down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Low speed
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    // Configure GPIO pin for the Hall effect sensor
    GPIO_InitStruct.Pin = HALL_SENSOR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;  // Pull-up to detect low signals
    HAL_GPIO_Init(HALL_SENSOR_PORT, &GPIO_InitStruct);
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief Error Handler Function
  */
void Error_Handler(void)
{
    // Stay in an infinite loop if an error occurs
    while (1)
    {
    }
}
