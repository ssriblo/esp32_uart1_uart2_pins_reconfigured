#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "string.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define BUF 1024
xQueueHandle xQueue;
#define BUF_SIZE (1024)

typedef enum {
    UART1_STATUS,              /*!< UART data event*/
    UART2_STATUS,              /*!< UART data event*/
} multy_uart_event_type_t;

typedef struct {
    multy_uart_event_type_t type; /*!< UART event type */
    bool flag;      
} multy_uart_event_t;

void setup_muxed_uarts(int uart_num);

void setup_muxed_uarts(int uart_num){
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    if(uart_num == 1){
        uart_set_pin(1, 18, 34, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if(uart_num == 2){
        uart_set_pin(2, 19, 35, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    uart_flush(uart_num);
}

void uart1_task(void *pvParameter)
{
    char* rxmesage;
    char* repl_data = "";
    setup_muxed_uarts(1);
    int64_t start, diff;
    while(1){
    	if( xQueue != 0 ) {
			if( (xQueuePeek( xQueue, &( rxmesage ), ( portTickType ) 10 )) == pdTRUE)
			{
                if(rxmesage[0] == '1') {
                    printf("\tUART-1 value received on queue: %s len=%d \n",rxmesage, uxQueueMessagesWaiting(xQueue));
    			    xQueueReceive( xQueue, &( rxmesage ), ( portTickType ) 10 ); //  event consumed 
                    printf("\tUART-1 value CONSUMED on queue: %s len=%d \n",rxmesage, uxQueueMessagesWaiting(xQueue));
                    repl_data = "MULUART-1-STATUS";
                    printf("\tUART-1 REPLY send: %s len=%d \n",repl_data, uxQueueMessagesWaiting(xQueue));
                    xQueueSend(xQueue,(void *)&repl_data,(TickType_t )0); 
            
                    start = esp_timer_get_time();
                    uart_write_bytes(1, (const char *)rxmesage, 10);
                    uart_wait_tx_done(1, 2);
                    diff = esp_timer_get_time() - start;
                    printf(" diff=%d    \n",(int)diff);

                }
                vTaskDelay(500/portTICK_PERIOD_MS); //wait for 500 ms
			}
    }
    }
}

void uart2_task(void *pvParameter)
{
    char* rxmesage;
    char* repl_data = "";
    setup_muxed_uarts(2);
    while(1){
    	if( xQueue != 0 ) {
			if( (xQueuePeek( xQueue, &( rxmesage ), ( portTickType ) 10 )) == pdTRUE)
			{
                    printf("\tUART-2 value received on queue: %s len=%d \n",rxmesage, uxQueueMessagesWaiting(xQueue));
    			    xQueueReceive( xQueue, &( rxmesage ), ( portTickType ) 10 ); //  event consumed 
                    printf("\tUART-2 value CONSUMED on queue: %s len=%d \n",rxmesage, uxQueueMessagesWaiting(xQueue));
                    repl_data = "MULUART-2-STATUS";
                    printf("\tUART-2 REPLY send: %s len=%d \n",repl_data, uxQueueMessagesWaiting(xQueue));
                    xQueueSend(xQueue,(void *)&repl_data,(TickType_t )0); 

                    uart_write_bytes(2, (const char *)rxmesage, 10);
                    uart_wait_tx_done(2, 2);

                }
                vTaskDelay(500/portTICK_PERIOD_MS); //wait for 500 ms
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

        if( (xQueuePeek( xQueue, &( rxmesage ), ( portTickType ) 10 )) == pdTRUE)
        {
            if(rxmesage[0] == 'M') {
                xQueueReceive( xQueue, &( rxmesage ), ( portTickType ) 10 ); //  event consumed 
                printf("\tROUTING - value consumed on queue: %s len=%d\n",rxmesage, uxQueueMessagesWaiting(xQueue));
            }
        }

        if((count % 20) == 0){
            printf("value sent on queue: %s \n",repl_data1);
            xQueueSend(xQueue,(void *)&repl_data1,(TickType_t )0); 
        }
        if((count % 20) == 1){
            printf("value sent on queue: %s \n",repl_data2);
            xQueueSend(xQueue,(void *)&repl_data2,(TickType_t )0); 
        }
        vTaskDelay(50/portTICK_PERIOD_MS); //wait for a second
        count++;

    }
}

void app_main()
{
	uint8_t* dataSerial = (uint8_t*) malloc(BUF);
	xQueue = xQueueCreate( 10, sizeof(dataSerial));
    if(xQueue != NULL){
        printf("Queue is created\n");
        vTaskDelay(1000/portTICK_PERIOD_MS); //wait for a second
        xTaskCreate(&routing_task,"routing_task",1024*4,NULL,5,NULL);
        printf("routing_task task  started\n");
        xTaskCreate(&uart1_task,"uart1_task",1024*4,NULL,10,NULL);
        xTaskCreate(&uart2_task,"uart2_task",1024*4,NULL,10,NULL);
        printf("uart_task task  started\n");
    }else{
        printf("Queue creation failed");
    }
}