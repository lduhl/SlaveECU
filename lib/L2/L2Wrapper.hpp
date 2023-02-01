/*

*/

#pragma once

#include "can.h"
//#include "usart.h"
#include "gpio.h"
//#include "stm32f1xx_hal.h"
//#include "stm32f1xx_hal_can.h"

class L2Wrapper

//struct L2Wrapper
{
    public:
        L2Wrapper()
        {

        }
        
    private:
    CAN_TxHeaderTypeDef TxHeader;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t TxData[8] = {0,};
	uint8_t RxData[8] = {0,};
	uint32_t TxMailbox = 0;
	uint8_t trans_str[30];

    //void SystemClock_Config(void);

    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
    {
        if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {  
                if(RxData[6] == 0x31){
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                }
                if(RxData[6] == 0x32){
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                }

        }
    }
    void CAN_interrupt(void){}

    void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
    {
        uint32_t er = HAL_CAN_GetError(hcan);
        /*sprintf(trans_str,"ER CAN %lu %08lX", er, er);
        HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 100);*/
    }

    void HAL_CAN_Send()
    {
            while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);

            if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
            {
                //TODO: can send error teport
            }
            HAL_Delay(500);
    }




    int main(void)
    {

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_CAN_Init();
   



    TxHeader.StdId = 0x07B0;
	TxHeader.ExtId = 0;
	TxHeader.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
	TxHeader.IDE = CAN_ID_STD;   // CAN_ID_EXT
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;
	
	RxHeader.StdId = 0x07B0;
	RxHeader.ExtId = 0;
	RxHeader.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
	RxHeader.IDE = CAN_ID_STD;   // CAN_ID_EXT
	RxHeader.DLC = 7;
	
	

	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
	
	HAL_CAN_Start(&hcan);
	
	TxHeader.StdId = 0x07B0;
	TxHeader.DLC = 7;
	TxData[0] = 0x44;
	TxData[1] = 0x53;
	TxData[2] = 0x46;
	TxData[3] = 0x30;
	TxData[4] = 0x30;
	TxData[5] = 0x30;
	TxData[6] = 0x33;
	TxData[7] = 0x00;



    return 1;
    }

    

     /**
     * @brief System Clock Configuration
     * @retval None
     */
    void SystemClock_Config()
    {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        //error handler
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
        //error handler
    }
    }

    /* USER CODE BEGIN 4 */

    /* USER CODE END 4 */

    /**
     * @brief  Period elapsed callback in non blocking mode
     * @note   This function is called  when TIM3 interrupt took place, inside
     * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
     * a global variable "uwTick" used as application time base.
     * @param  htim : TIM handle
     * @retval None
     */
    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
    {
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM3) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
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
        tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
    }
    #endif /* USE_FULL_ASSERT */

};
