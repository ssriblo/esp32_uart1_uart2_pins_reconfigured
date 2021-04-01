#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "string.h"

#define BUF 1024
xQueueHandle xQueue;

typedef enum {
    UART1_STATUS,              /*!< UART data event*/
    UART2_STATUS,              /*!< UART data event*/
} multy_uart_event_type_t;

typedef struct {
    multy_uart_event_type_t type; /*!< UART event type */
    bool flag;      
} uart_event_t;

void uart1_task(void *pvParameter)
{
    char* rxmesage;
    char* repl_data = "";
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
        xTaskCreate(&uart1_task,"uart1_task",1024*4,NULL,5,NULL);
        xTaskCreate(&uart2_task,"uart2_task",1024*4,NULL,5,NULL);
        printf("uart_task task  started\n");
    }else{
        printf("Queue creation failed");
    }
}