#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "string.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <sys/time.h>

#define PRINT_TIME 1
#undef  PRINT_TIME

#define BUF 512
#define BUF_SIZE (1024)

#define UART_STATUS_BUSY "BUSY"
#define UART_STATUS_FREE "FREE"

// Set following wait values ZERO for release
// But set > 0 for debugging
#define ROUTE_QUEUE_WAIT    5
#define UART_QUEUE_WAIT     5

typedef enum {
    UART1 = 1,
    UART2 = 2,
} uart_num_t;

static const char *TAG = "uart_test";

xQueueHandle xQueueUart1Event;
xQueueHandle xQueueUart2Event;
xQueueHandle xQueueUart1Data;
xQueueHandle xQueueUart2Data;

void setup_muxed_uarts(int uart_num, int pin);

void setup_muxed_uarts(int uart_num, int pin){
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, pin, 34, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_flush(uart_num);
}

void uart1_task(void *pvParameter)
{
    const uart_num_t UARTNUM = UART1;
    char* rxmesage;
    char* repl_data = UART_STATUS_BUSY;
    while(1){
    	if( xQueueUart1Data != 0 ) {
			if( (xQueueReceive( xQueueUart1Data, &( rxmesage ), ( portTickType ) UART_QUEUE_WAIT )) == pdTRUE)
			{
                ESP_LOGI(TAG, "\tUART%d value CONSUMED on queue: %s ", UARTNUM, rxmesage);
                ESP_LOGI(TAG, "\tUART%d REPLY send: %s  ",UARTNUM, repl_data);
                uart_write_bytes(UARTNUM, (const char *)rxmesage, 10);
                uart_wait_tx_done(UARTNUM, 2);
                repl_data = UART_STATUS_FREE;
                xQueueSend(xQueueUart1Event,(void *)&repl_data,(TickType_t )0); 
			}
        }
    }
}

void uart2_task(void *pvParameter)
{
    const uart_num_t UARTNUM = UART2;
    char* rxmesage;
    char* repl_data = UART_STATUS_BUSY;
    while(1){
    	if( xQueueUart2Data != 0 ) {
			if( (xQueueReceive( xQueueUart2Data, &( rxmesage ), ( portTickType ) UART_QUEUE_WAIT )) == pdTRUE)
			{
                ESP_LOGI(TAG, "\tUART%d value CONSUMED on queue: %s ", UARTNUM, rxmesage);
                repl_data = UART_STATUS_BUSY;
                uart_write_bytes(UARTNUM, (const char *)rxmesage, 10);
                uart_wait_tx_done(UARTNUM, 2);
                repl_data = UART_STATUS_FREE;
                xQueueSend(xQueueUart2Event,(void *)&repl_data,(TickType_t )0); 
			}
        }
    }
}

void routing_task(void *pvParameter){
	char *mydata1="1ABCDEFGHIJKLMNOPQRSTUVWXYZ";
	char *mydata2="2ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    char  *repl_data1="";
    char  *repl_data2="";
    int count=0;
    char* rxmesage;
    struct timeval tv_now;
    char* strFree = UART_STATUS_FREE;
#ifdef PRINT_TIME
    int64_t start=0, diff1=0, diff2=0;
#endif
    int isUartBusy[2] = {0}; // set to 1 when UART busy
//    uart_num_t uartToSend;

    while(1){
#ifdef PRINT_TIME
        start = esp_timer_get_time();
#endif
        asprintf(&repl_data1,"%s %d",mydata1, count);
        asprintf(&repl_data2,"%s %d",mydata2, count);

        if( (xQueueReceive( xQueueUart1Event, &( rxmesage ), ( portTickType ) ROUTE_QUEUE_WAIT )) == pdTRUE) {
            ESP_LOGI(TAG, "\tROUTING - UART1 value consumed on queue: %s",rxmesage);
            if (strcmp(rxmesage, strFree) == 0) {
                isUartBusy[0] = 0;
                ESP_LOGI(TAG, "\tROUTING - UART1: STRCPM PASS");
            }
        }
        if( (xQueueReceive( xQueueUart2Event, &( rxmesage ), ( portTickType ) ROUTE_QUEUE_WAIT )) == pdTRUE) {
            ESP_LOGI(TAG, "\tROUTING - UART2 value consumed on queue: %s",rxmesage);
            if (strcmp(rxmesage, strFree) == 0) {
                isUartBusy[1] = 0;
                ESP_LOGI(TAG, "\tROUTING - UART2: STRCPM PASS");
            }
        }
        if((count % 2) == 0){
            if(isUartBusy[0] == 0){
                isUartBusy[0] = 1;
                ESP_LOGI(TAG, "value sent on xQueueUart1Data: %s ",repl_data1);
                xQueueSend(xQueueUart1Data,(void *)&repl_data1,(TickType_t )0); 
            }
        }
        if((count % 2) == 1){
            if(isUartBusy[1] == 0){
                isUartBusy[1] = 1;
                ESP_LOGI(TAG, "value sent on xQueueUart2Data: %s ",repl_data2);
                xQueueSend(xQueueUart2Data,(void *)&repl_data2,(TickType_t )0); 
            }
        }
#ifdef PRINT_TIME
        diff1 = esp_timer_get_time() - start;
#endif
        vTaskDelay(50/portTICK_PERIOD_MS); //wait for a second
        gettimeofday(&tv_now, NULL);
#ifdef PRINT_TIME
        ESP_LOGI(TAG, "time: %ld %ld count=%d",tv_now.tv_sec, tv_now.tv_usec/1000, count);       
        diff2 = esp_timer_get_time() - start;
        ESP_LOGI(TAG, " diff1=%d  diff2=%d   ",(int)diff1, (int)diff2);
#endif
        count++;
    }
}

void app_main()
{
	uint8_t* dataSerial = (uint8_t*) malloc(BUF);
	xQueueUart1Event = xQueueCreate( 10, sizeof(dataSerial));
	xQueueUart2Event = xQueueCreate( 10, sizeof(dataSerial));
	xQueueUart1Data = xQueueCreate( 10, sizeof(dataSerial));
	xQueueUart2Data = xQueueCreate( 10, sizeof(dataSerial));
    ESP_LOGI(TAG, "Queue is created");
    setup_muxed_uarts(1, 18);
    setup_muxed_uarts(2, 19);
    vTaskDelay(1000/portTICK_PERIOD_MS); //wait for a second
    xTaskCreate(&routing_task,"routing_task",1024*8,NULL,1,NULL);
    ESP_LOGI(TAG, "routing_task task  started");
    xTaskCreate(&uart1_task,"uart1_task",1024*8,NULL,1,NULL);
    xTaskCreate(&uart2_task,"uart2_task",1024*8,NULL,1,NULL);
    ESP_LOGI(TAG, "uart_task task  started");
}