
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>

/* USER CODE BEGIN PV */
UART_HandleTypeDef huart1; // UART1 for HC-05 communication
UART_HandleTypeDef huart2;

char response[100] = {0}; // Buffer to store HC-05 response
char loopbackMessage[] = "Loopback Test\r\n";
char loopbackBuffer[50];

/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_USART2_UART_Init(void);

static void MX_USART1_UART_Init(void);



/* USER CODE BEGIN PFP */

void Send_AT_Command(const char *command);

void Receive_Response(char *buffer, uint16_t size);
void Debug_HC05_Receive(char *buffer, uint16_t size);


/* USER CODE END PFP */



/* USER CODE BEGIN 0 */

/* Function to redirect printf output to UART2 for debugging */

int _write(int file, char *data, int len) {

    HAL_UART_Transmit(&huart2, (uint8_t *)data, len, HAL_MAX_DELAY); // Redirect to UART2 for debugging

    return len;

}

/* USER CODE END 0 */



/**

  * @brief  The application entry point.

  * @retval int

  */

int main(void) {

    /* MCU Initialization */

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();

    MX_USART1_UART_Init(); // UART1 for HC-05

    MX_USART2_UART_Init(); // UART2 for debugging (console)



    /* Test Communication with HC-05 */

    char loopbackMessage[] = "Loopback Test\r\n";

    char loopbackBuffer[50] = {0};
    char response[100] = {0};








    // Transmit data



    Send_AT_Command("AT");
    Debug_HC05_Receive(response, sizeof(response));



    /* Print HC-05 Response */

    printf("Response from HC-05: %s\n", response);

    while (1) {

        // Main loop

    }

}

void Debug_HC05_Receive(char *buffer, uint16_t size) {
    printf("\n--- Starting HC-05 Debugging ---\n");

    // Step 1: Clear the buffer
    memset(buffer, 0, size);
    printf("Buffer cleared.\n");

    // Step 2: Test UART Transmission
    const char *test_command = "AT";
    printf("Sending test command: %s\n", test_command);
    HAL_StatusTypeDef tx_status = HAL_UART_Transmit(&huart1, (uint8_t *)test_command, strlen(test_command), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);

    if (tx_status != HAL_OK) {
        printf("Error during transmission. HAL Error: %d\n", tx_status);
        return;
    }
    printf("Command sent successfully.\n");

    // Step 3: Wait for a response
    printf("Waiting for response...\n");
    HAL_StatusTypeDef rx_status = HAL_UART_Receive(&huart1, (uint8_t *)buffer, size - 1, 5000);

    if (rx_status == HAL_OK) {
        printf("Response received successfully.\n");

        // Step 4: Print raw bytes in Hexadecimal
        printf("Raw Response (Hex): ");
        for (int i = 0; i < strlen(buffer); i++) {
            printf("%02X ", buffer[i]);
        }
        printf("\n");

        // Step 5: Print readable response
        printf("Readable Response: %s\n", buffer);
    } else if (rx_status == HAL_TIMEOUT) {
        printf("Timeout occurred. Possible causes:\n");
        printf("- HC-05 not in AT mode.\n");
        printf("- HC-05 not connected properly.\n");
        printf("- Incorrect baud rate.\n");
    } else {
        printf("Error during reception. HAL Error: %d\n", rx_status);
    }

    // Step 6: Additional Debugging Information
    printf("Debugging complete. Summary:\n");
    printf("- Transmission Status: %s\n", (tx_status == HAL_OK) ? "SUCCESS" : "FAILED");
    printf("- Reception Status: %s\n", (rx_status == HAL_OK) ? "SUCCESS" :
             (rx_status == HAL_TIMEOUT) ? "TIMEOUT" : "ERROR");
}


/* Function to send AT command to HC-05 */

void Send_AT_Command(const char *command) {

    // Transmit the command string

    HAL_UART_Transmit(&huart1, (uint8_t *)command, strlen(command), HAL_MAX_DELAY);

    // Append CR+LF for proper AT command formatting

    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);

}



/* Function to receive response from HC-05 */





void Receive_Response(char *buffer, uint16_t size) {

    memset(buffer, 0, size); // Clear the buffer

    printf("Waiting for response...\n");

    HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, (uint8_t)buffer, size - 1, 5000); // Timeout: 5000ms

    if (status == HAL_OK) {

        printf("Raw Response (Hex): ");

        for (int i = 0; i < strlen(buffer); i++) {

            printf("%02X ", buffer[i]); // Print raw bytes in hexadecimal

        }

        printf("\nReadable Response: %s\n", buffer); // Print the string

    } else if (status == HAL_TIMEOUT) {

        printf("Timeout occurred while waiting for response.\n");

    } else {

        printf("Error receiving response. HAL Error: %d\n", status);

    }

}


/**
  * @brief USART1 Initialization Function
  * @retval None
  */
void SystemClock_Config(void)

{

RCC_OscInitTypeDef RCC_OscInitStruct = {0};

RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};



/** Configure the main internal regulator output voltage

*/

__HAL_RCC_PWR_CLK_ENABLE();

__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);



/** Initializes the RCC Oscillators according to the specified parameters

* in the RCC_OscInitTypeDef structure.

*/

RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;

RCC_OscInitStruct.HSIState = RCC_HSI_ON;

RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

RCC_OscInitStruct.PLL.PLLM = 16;

RCC_OscInitStruct.PLL.PLLN = 336;

RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;

RCC_OscInitStruct.PLL.PLLQ = 7;

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

}





static void MX_USART1_UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 38400; // HC-05 default baud rate
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief USART2 Initialization Function
  * @retval None
  */
static void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 38400; // Debugging UART baud rate
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Alternate function push-pull mode
    GPIO_InitStruct.Pull = GPIO_NOPULL; // No pull-up or pull-down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief Error Handler Function
  * @retval None
  */
void Error_Handler(void) {
    __disable_irq();
    while (1) {
        // Optional: Add code to indicate error (e.g., toggle an LED)
    }
}
