#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "string.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <sys/time.h>
#include "esp_intr_alloc.h"
#include "esp_attr.h"
#include "driver/timer.h"

#define PRINT_TIME 1
#undef  PRINT_TIME

#define BUF 512
#define BUF_SIZE (1024)

#define RXPINUART1  34
#define RXPINUART2  35

#define UART_STATUS_BUSY "BUSY"
#define UART_STATUS_FREE "FREE"

// Set following wait values ZERO for release
// But set > 0 for debugging
#define ROUTE_QUEUE_WAIT    1
#define UART_QUEUE_WAIT     1

/////////  FOR TEST ONLY !! /////////////////////////////////////////////////////
#define GPIO_OUTPUT_IO_0    23
#define GPIO_OUTPUT_IO_1    23
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))
//////////////////////////////////////////////////////////////////////////////////

#define BAUDRATE    9600

int pinsList[] = {13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27};

typedef enum {
    UART1 = 1,
    UART2 = 2,
} uart_num_t;

static intr_handle_t s_timer_handle;

static const char *TAG = "uart_test";
static int currentPinList[2] = {42};

xQueueHandle xQueueUart1Event;
xQueueHandle xQueueUart2Event;
xQueueHandle xQueueUart1Data;
xQueueHandle xQueueUart2Data;
xQueueHandle xQueueTimer;

void setup_muxed_uarts(int uart_num, int baud_rate);
static void timer_isr(void* arg);
void init_timer(int timer_period_us);
void pinReconfigure(int pin, uart_num_t uartnum);
void setup_gpio();

void setup_gpio(){
    uint64_t pin_bit_mask = 0;          /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
    for(int i=0; i<sizeof(pinsList)/sizeof(pinsList[0]); i++){
        pin_bit_mask = pin_bit_mask | (1ULL<<pinsList[i]);
    }
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = pin_bit_mask;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void pinReconfigure(int txpin, uart_num_t uartnum){
    int rxpin;
    int previousTxPin = currentPinList[uartnum];
    if(uartnum == UART1){
        rxpin = RXPINUART1;
    }else{
        rxpin = RXPINUART2;
    }
    if(previousTxPin == txpin){
        // Previous pin for the UART the same as new one
        // Nothing to do for reconfiguration
    }else{
        // Let reset previous pin
        gpio_reset_pin(previousTxPin);
        gpio_set_level(previousTxPin, 1);
        currentPinList[uartnum] = txpin;
        ESP_ERROR_CHECK(uart_set_pin(uartnum, txpin, rxpin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    }

}

static void timer_isr(void* arg)
{
    char* repl_data = "TIMER";
    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[0].config.alarm_en = 1;
    xQueueSend(xQueueTimer,(void *)&repl_data,(TickType_t )0); 
//    static int cnt = 0;
//    cnt++;
//    gpio_set_level(23, cnt % 2);
}

void init_timer(int timer_period_us)
{
    timer_config_t config = {
            .alarm_en = true,
            .counter_en = false,
            .intr_type = TIMER_INTR_LEVEL,
            .counter_dir = TIMER_COUNT_UP,
            .auto_reload = true,
            .divider = 80   /* 1 us per tick */
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timer_period_us);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_isr, NULL, 0, &s_timer_handle);

    timer_start(TIMER_GROUP_0, TIMER_0);
}

void setup_muxed_uarts(int uart_num, int baud_rate){
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
//    ESP_ERROR_CHECK(uart_set_pin(uart_num, txpin, 34, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_flush(uart_num);
}

void uart1_task(void *pvParameter)
{
    const uart_num_t uartnum = UART1;
    char* rxmesage;
    char* repl_data = UART_STATUS_BUSY;
    while(1){
    	if( xQueueUart1Data != 0 ) {
			if( (xQueueReceive( xQueueUart1Data, &( rxmesage ), ( portTickType ) UART_QUEUE_WAIT )) == pdTRUE)
			{
                ESP_LOGI(TAG, "\tUART%d value CONSUMED on queue: %s ", uartnum, rxmesage);
                uart_tx_chars(uartnum, (const char *)rxmesage, 10);
                uart_wait_tx_idle_polling(uartnum);
//                uart_wait_tx_done(uartnum, 2);
                repl_data = UART_STATUS_FREE;
                xQueueSend(xQueueUart1Event,(void *)&repl_data,(TickType_t )0); 
			}
        }
    }
}

void uart2_task(void *pvParameter)
{
    const uart_num_t uartnum = UART2;
    char* rxmesage;
    char* repl_data = UART_STATUS_BUSY;
    while(1){
    	if( xQueueUart2Data != 0 ) {
			if( (xQueueReceive( xQueueUart2Data, &( rxmesage ), ( portTickType ) UART_QUEUE_WAIT )) == pdTRUE)
			{
                ESP_LOGI(TAG, "\tUART%d value CONSUMED on queue: %s ", uartnum, rxmesage);
                uart_tx_chars(uartnum, (const char *)rxmesage, 10);
                uart_wait_tx_idle_polling(uartnum);
//                uart_wait_tx_done(uartnum, 2);
                repl_data = UART_STATUS_FREE;
                xQueueSend(xQueueUart2Event,(void *)&repl_data,(TickType_t )0); 
			}
        }
    }
}

void routing_task(void *pvParameter){
	char *mydata="ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    ///////////////////////////////////////////
    // Format:
    // {<Number[0...15]>, <pin>}
    // pin list is:
    // 13 14 16 17 18 19 21 22 23 25 26 27
    ///////////////////////////////////////////
#define MSGNUM    16
    int pinMsgList[MSGNUM][2] = { 
        {0,13},
        {1,14},
        {2,16},
        {3,17},
        {4,18},
        {5,18},
        {6,18},
        {7,18},
        {8,18},
        {9,19},
        {10,21},
        {11,22},
        {12,23},
        {13,25},
        {14,26},
        {15,27},
        };
    int cycle16 = 0;
    int pin;
    char  *repl_data="";
    char* rxmesage;
#ifdef PRINT_TIME
    struct timeval tv_now;
#endif
    char* strFree = UART_STATUS_FREE;
    uart_num_t uartnum;
    int isUartBusy[3] = {0}; // set to 1 when UART busy ([0] - is not used!)

#ifdef PRINT_TIME
    int64_t start=0, diff1=0, diff2=0;
#endif

    while(1){
#ifdef PRINT_TIME
        start = esp_timer_get_time();
#endif
        if( (xQueueReceive( xQueueUart1Event, &( rxmesage ), ( portTickType ) ROUTE_QUEUE_WAIT )) == pdTRUE) {
            ESP_LOGI(TAG, "\tROUTING - UART1 value consumed on queue: %s",rxmesage);
            uartnum = UART1;
            if (strcmp(rxmesage, strFree) == 0) {
                isUartBusy[uartnum] = 0;
            }
        }
        if( (xQueueReceive( xQueueUart2Event, &( rxmesage ), ( portTickType ) ROUTE_QUEUE_WAIT )) == pdTRUE) {
            ESP_LOGI(TAG, "\tROUTING - UART2 value consumed on queue: %s",rxmesage);
            uartnum = UART2;
            if (strcmp(rxmesage, strFree) == 0) {
                isUartBusy[uartnum] = 0;
            }
        }

        // Let get new message and send to someone of the UARTs
        if( (xQueueReceive( xQueueTimer, &( rxmesage ), ( portTickType ) ROUTE_QUEUE_WAIT )) == pdTRUE) {
            pin = pinMsgList[cycle16][1];           
            ESP_LOGI(TAG, "\tROUTING - ***** TIMER EXPIRED *****  cycle16=%d pin=%d", cycle16, pin);
            cycle16++;
            cycle16 = cycle16 % MSGNUM;
            asprintf(&repl_data,"%d %s", pin, mydata);
//#if 0
            uartnum = UART1;
            if( (isUartBusy[uartnum] == 0)  && (currentPinList[UART2] != pin) ){
                isUartBusy[uartnum] = 1;
                pinReconfigure(pin, uartnum);
                ESP_LOGI(TAG, "ROUTING - value sent on UART-%d: %s ", uartnum, repl_data);
                xQueueSend(xQueueUart1Data,(void *)&repl_data,(TickType_t )0); 
            }else{
                uartnum = UART2;
                    if( (isUartBusy[uartnum] == 0)  && (currentPinList[UART1] != pin) ){
                    isUartBusy[uartnum] = 1;
                    pinReconfigure(pin, uartnum);
                    ESP_LOGI(TAG, "ROUTING - value sent on UART-%d: %s ", uartnum, repl_data);
                    xQueueSend(xQueueUart2Data,(void *)&repl_data,(TickType_t )0); 
                }else{
                    // Both UARTs are busy
                    // Let ignore this packet. But the better to store packet at the rignbuffer or at the xQueue
                    ESP_LOGI(TAG, "ROUTING - Both UARTs are busy MSG=%s ", repl_data);
                }
            }
//#endif
        } // if( (xQueueReceive( xQueueTimer,...
#ifdef PRINT_TIME
            diff1 = esp_timer_get_time() - start;
            gettimeofday(&tv_now, NULL);
            ESP_LOGI(TAG, "time: %ld %ld count=%d",tv_now.tv_sec, tv_now.tv_usec/1000, count);       
            diff2 = esp_timer_get_time() - start;
            ESP_LOGI(TAG, " diff1=%d  diff2=%d   ",(int)diff1, (int)diff2);
#endif
    } // while(1) ...
} // void routing_task(...

void app_main()
{
	uint8_t* dataSerial = (uint8_t*) malloc(BUF);
	xQueueUart1Event = xQueueCreate( 10, sizeof(dataSerial));
	xQueueUart2Event = xQueueCreate( 10, sizeof(dataSerial));
	xQueueUart1Data = xQueueCreate( 10, sizeof(dataSerial));
	xQueueUart2Data = xQueueCreate( 10, sizeof(dataSerial));
	xQueueTimer = xQueueCreate( 10, sizeof(dataSerial));
    ESP_LOGI(TAG, "Queue is created");
    esp_log_level_set(TAG, ESP_LOG_NONE);     // Disabe local log
    esp_log_level_set("gpio", ESP_LOG_NONE);  // Disable log at GPIO module when pin resets

//    setup_gpio();
    setup_muxed_uarts(UART1, BAUDRATE);
    setup_muxed_uarts(UART2, BAUDRATE);
    // Let setup all pins for output with high level. It help to switch UART's pin without low level break issue
    for(int i=0; i<sizeof(pinsList)/sizeof(pinsList[0]); i++){
        gpio_set_level(pinsList[i], 1);
        // ESP_ERROR_CHECK(uart_set_pin(UART1, pinsList[i], RXPINUART1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        // ESP_ERROR_CHECK(uart_set_pin(UART2, pinsList[i], RXPINUART2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    }
    vTaskDelay(1000/portTICK_PERIOD_MS); //wait for a second
    xTaskCreate(&routing_task,"routing_task",1024*8,NULL,1,NULL);
    ESP_LOGI(TAG, "routing_task task  started");
    xTaskCreate(&uart1_task,"uart1_task",1024*8,NULL,1,NULL);
    xTaskCreate(&uart2_task,"uart2_task",1024*8,NULL,1,NULL);
    ESP_LOGI(TAG, "uart_task task  started");
    init_timer(100*1000);
}