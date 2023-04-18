#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "radioenge_modem.h"
#include "main.h"

extern osTimerId_t PeriodicSendTimerHandle;
extern osThreadId_t AppSendTaskHandle;
extern ADC_HandleTypeDef hadc1;
extern osEventFlagsId_t ModemStatusFlagsHandle;
extern TIM_HandleTypeDef htim3;

void LoRaWAN_RxEventCallback(uint8_t *data, uint32_t length, uint32_t port, int32_t rssi, int32_t snr)
{
}

void PeriodicSendTimerCallback(void *argument)
{
}
extern osMessageQueueId_t TemperatureQueueHandle;
char msg[256];
void ReadFromADCTaskCode(void *argument)
{
    uint32_t read;
    TEMPERATURE_OBJ_t data;
    data.seq_no = 0;
    while (1)
    {
        // read LM35 Temperature
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        read = HAL_ADC_GetValue(&hadc1);
        data.seq_no = data.seq_no + 1;
        data.temp_oCx100 = (int32_t)(33000 * ((float)read / 4096));
        // Send Message
        osMessageQueuePut(TemperatureQueueHandle, &data, 0U, osWaitForever);
        osDelay(30000);
    }
}

void AppSendTaskCode(void *argument)
{
    TEMPERATURE_OBJ_t rcv_data;
    osStatus_t status;
    while (1)
    {
        status = osMessageQueueGet(TemperatureQueueHandle, &rcv_data, NULL,
                                   osWaitForever); // wait for message
        if (status == osOK)
        {
            sprintf(msg, "Temperature reading %d: %d.%d oC\r\n", rcv_data.seq_no, rcv_data.temp_oCx100 / 100, rcv_data.temp_oCx100 % 100);
            LoRaSend(169, msg);
            //SendToUART(msg, strlen(msg));
        }
    }
}
