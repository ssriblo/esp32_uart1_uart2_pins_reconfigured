#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "string.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define BUF 1024
#define BUF_SIZE (1024)

xQueueHandle xQueueUart1Event;
xQueueHandle xQueueUart2Event;
xQueueHandle xQueueUart1Data;
xQueueHandle xQueueUart2Data;

typedef enum {
    UART1_STATUS,              /*!< UART data event*/
    UART2_STATUS,              /*!< UART data event*/
} multy_uart_event_type_t;

typedef struct {
    multy_uart_event_type_t type; /*!< UART event type */
    bool flag;      
} multy_uart_event_t;

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
    const int UARTNUM = 1;
    char* rxmesage;
    char* repl_data = "";
    setup_muxed_uarts(UARTNUM, 18);
    while(1){
    	if( xQueueUart1Event != 0 ) {
			if( (xQueueReceive( xQueueUart1Data, &( rxmesage ), ( portTickType ) 10 )) == pdTRUE)
			{
                printf("\tUART%d value CONSUMED on queue: %s \n", UARTNUM, rxmesage);
                repl_data = "MULUART-1-STATUS";
                xQueueSend(xQueueUart1Event,(void *)&repl_data,(TickType_t )0); 
                printf("\tUART%d REPLY send: %s  \n",UARTNUM, repl_data);
                uart_write_bytes(UARTNUM, (const char *)rxmesage, 10);
                uart_wait_tx_done(UARTNUM, 2);
                vTaskDelay(500/portTICK_PERIOD_MS); //wait for 500 ms
			}
        }
    }
}

void uart2_task(void *pvParameter)
{
    const int UARTNUM = 2;
    char* rxmesage;
    char* repl_data = "";
    setup_muxed_uarts(UARTNUM, 18);
    while(1){
    	if( xQueueUart1Event != 0 ) {
			if( (xQueueReceive( xQueueUart2Data, &( rxmesage ), ( portTickType ) 10 )) == pdTRUE)
			{
                printf("\tUART%d value CONSUMED on queue: %s  \n", UARTNUM, rxmesage);
                repl_data = "MULUART-2-STATUS";
                xQueueSend(xQueueUart2Event,(void *)&repl_data,(TickType_t )0); 
                printf("\tUART%d REPLY send: %s  \n",UARTNUM, repl_data);
                uart_write_bytes(UARTNUM, (const char *)rxmesage, 10);
                uart_wait_tx_done(UARTNUM, 2);
                vTaskDelay(500/portTICK_PERIOD_MS); //wait for 500 ms
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

    while(1){
        asprintf(&repl_data1,"%s %d",mydata1, count);
        asprintf(&repl_data2,"%s %d",mydata2, count);

        if( (xQueueReceive( xQueueUart1Event, &( rxmesage ), ( portTickType ) 10 )) == pdTRUE) {
            printf("\tROUTING - value consumed on queue: %s \n",rxmesage);
        }
        if( (xQueueReceive( xQueueUart2Event, &( rxmesage ), ( portTickType ) 10 )) == pdTRUE) {
            printf("\tROUTING - value consumed on queue: %s \n",rxmesage);
        }
        if((count % 20) == 0){
            printf("value sent on queue: %s \n",repl_data1);
            xQueueSend(xQueueUart1Data,(void *)&repl_data1,(TickType_t )0); 
        }
        if((count % 20) == 1){
            printf("value sent on queue: %s \n",repl_data2);
            xQueueSend(xQueueUart2Data,(void *)&repl_data2,(TickType_t )0); 
        }
        vTaskDelay(50/portTICK_PERIOD_MS); //wait for a second
        count++;
    }
}

void app_main()
{
	xQueueUart1Event = xQueueCreate( 10, BUF);
	xQueueUart2Event = xQueueCreate( 10, BUF);
	xQueueUart1Data = xQueueCreate( 10, BUF);
	xQueueUart2Data = xQueueCreate( 10, BUF);
    printf("Queue is created\n");
    vTaskDelay(1000/portTICK_PERIOD_MS); //wait for a second
    xTaskCreate(&routing_task,"routing_task",1024*4,NULL,5,NULL);
    printf("routing_task task  started\n");
    xTaskCreate(&uart1_task,"uart1_task",1024*4,NULL,10,NULL);
    xTaskCreate(&uart2_task,"uart2_task",1024*4,NULL,10,NULL);
    printf("uart_task task  started\n");
}