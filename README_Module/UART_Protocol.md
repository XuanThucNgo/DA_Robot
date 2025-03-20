Return README.md

## Receive data from ROS using Interrupts and DMA.
Use interrupts for data reception and DMA to offload data handling, ensuring the CPU remains focused on processing the motor speed PID control.
```
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == huart3.Instance) {
    // Null-terminate the received data to ensure it is a proper C string
    Rx_data[Size] = '\0';
    sscanf((char*)Rx_data, "%f r %f l", &vel_R_getFromRaspi, &vel_L_getFromRaspi);
    // Clear the buffer
    memset(Rx_data, 0, sizeof(Rx_data));
    // Re-enable DMA reception
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Rx_data, sizeof(Rx_data) - 1);
  }
}
```
## Send back encoder pulse data to the embedded computer.
Use a timer interrupt to accurately send data every 5ms.
```
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	// TIM5: (72*1000)/72000000hz = 1ms
	if(htim->Instance == TIM5) {
		tick++;
		if (tick == 50) {
			tick = 0;
			if (flagStop == false) {
				printf("%.0fr%.0fl\n", tickR, tickL);
			}
		}
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}
```

