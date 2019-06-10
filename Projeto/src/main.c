#include "asf.h"
#include "main.h"
#include <string.h>

#include "OLED1/gfx_mono_ug_2832hsweg04.h"
#include "OLED1/gfx_mono_text.h"
#include "OLED1/sysfont.h"

/************************************************************************/
/* GENERIC DEFINES                                                      */
/************************************************************************/

#define ELED1_PIO           PIOA 
#define ELED1_PIO_ID        ID_PIOA                
#define ELED1_PIO_IDX		0u                    
#define ELED1_PIO_IDX_MASK  (1u << ELED1_PIO_IDX)   

#define ELED2_PIO           PIOC                  
#define ELED2_PIO_ID        ID_PIOC              
#define ELED2_PIO_IDX		30u                    
#define ELED2_PIO_IDX_MASK  (1u << ELED2_PIO_IDX)   

#define ELED3_PIO           PIOB              
#define ELED3_PIO_ID        ID_PIOB               
#define ELED3_PIO_IDX		2                    
#define ELED3_PIO_IDX_MASK  (1u << ELED3_PIO_IDX) 

#define EBUT1_PIO			PIOD
#define EBUT1_PIO_ID		ID_PIOD
#define EBUT1_PIO_IDX		28
#define EBUT1_PIO_IDX_MASK (1u << EBUT1_PIO_IDX)

#define EBUT2_PIO			PIOC
#define EBUT2_PIO_ID		ID_PIOC
#define EBUT2_PIO_IDX		31
#define EBUT2_PIO_IDX_MASK (1u << EBUT2_PIO_IDX)

#define EBUT3_PIO           PIOB
#define EBUT3_PIO_ID        ID_PIOB
#define EBUT3_PIO_IDX       2
#define EBUT3_PIO_IDX_MASK (1u << EBUT3_PIO_IDX)

#define 	_FFS(x)

/************************************************************************/
/* generic globals                                                      */
/************************************************************************/
SemaphoreHandle_t xSemaphoreLED;
QueueHandle_t xQueueUART;
QueueHandle_t xQueueOLED;
/************************************************************************/
/*  RTOS    (defines + globals)                                         */
/************************************************************************/

#define TASK_STRING_STACK_SIZE            (4*4096/sizeof(portSTACK_TYPE))
#define TASK_STRING_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_IO_STACK_SIZE                (4096/sizeof(portSTACK_TYPE))
#define TASK_IO_STACK_PRIORITY            (tskIDLE_PRIORITY)
#define TASK_OLED_STACK_SIZE              (2*4096/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY          (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}
extern void vApplicationIdleHook(void)
{
	
}
extern void vApplicationTickHook(void)
{
}
extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}


/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

int protocol_check_led(char *string);
void io_init(void);
void led_on(uint id, uint on);

/************************************************************************/
/* IRQS / callbacks                                                     */
/************************************************************************/

void USART1_Handler(void){

  BaseType_t xHigherPriorityTaskWoken = pdFALSE; // freeRTOs
  uint32_t ret = usart_get_status(CONF_UART);  // ACK IRQ
  char c;
  // Por qual motivo entrou na interrupçcao ? Pode ser varios!
  //  1. RX: Dado disponível para leitura
  if(ret & US_IER_RXRDY){
	  usart_serial_getchar(CONF_UART, &c);
	  xHigherPriorityTaskWoken = pdFALSE;
	  xQueueSendFromISR(xQueueUART,  &c, &xHigherPriorityTaskWoken);
  } 
}

void button0_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreLED, &xHigherPriorityTaskWoken);
}

void button1_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreLED, &xHigherPriorityTaskWoken);
}

void button2_callback(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreLED, &xHigherPriorityTaskWoken);
}



/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

void task_string(void *pvParameters){
	xQueueUART = xQueueCreate(52, sizeof(char));
	xQueueOLED = xQueueCreate(2, sizeof(char[52]));

	// Ativa interrupcao UART1 na recpcao de daods
	usart_enable_interrupt(CONF_UART, US_IER_RXRDY);
	NVIC_EnableIRQ(CONSOLE_UART_ID);
	NVIC_SetPriority(CONSOLE_UART_ID, 5);
	char b[52];
	uint i = 0;
  
	while(1){
		if (xQueueReceive( xQueueUART, &b[i], ( TickType_t )  10 / portTICK_PERIOD_MS)){	
				if(b[i]=='\n'){
					b[i] = NULL;
					i=0;
					printf("recebido: %s\n", b);
					xQueueSend(xQueueOLED, &b, 10);
					if (protocol_check_led(b) == 1){
						xSemaphoreGive(xSemaphoreLED);
					}
			}
			else{
				i++;
			}
		}
	}
}



void task_io(void *pvParameters){
	
   xSemaphoreLED = xSemaphoreCreateBinary();

	io_init();
	int blinkiquito = 0;
	
	
	if (xSemaphoreLED == NULL)
	printf("falha em criar o semaforo \n");
	
	while(1){
	  if (blinkiquito == 1){
		  		  led_on(1,1);
		  		  led_on(2,1);
		  		  led_on(3,1);
		  		  vTaskDelay(200);
		  		  led_on(1,0);
		  		  led_on(2,0);
		  		  led_on(3,0);
		  		  vTaskDelay(200);
	  }
	  if ((xSemaphoreTake(xSemaphoreLED, ( TickType_t ) 100) == pdTRUE)){
		 blinkiquito = !blinkiquito;
	  }
  }
}

void task_oled1(void *pvParameters){
	char b[52];
	gfx_mono_ssd1306_init();	
	
 while(1){
	if(xQueueReceive(xQueueOLED, b, 10))
		gfx_mono_draw_string(b, 50,16, &sysfont);	 vTaskDelay(100);	 }
 }  




/************************************************************************/
/* CONFIGS/ INITS                                                       */
/************************************************************************/

static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate =		CONF_UART_BAUDRATE,
		.charlength =	CONF_UART_CHAR_LENGTH,
		.paritytype =	CONF_UART_PARITY,
		.stopbits =		CONF_UART_STOP_BITS,
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);

}


 


/************************************************************************/
/* functions                                                            */
/************************************************************************/

void io_init(void){
		pmc_enable_periph_clk(EBUT1_PIO_ID);
		pmc_enable_periph_clk(EBUT2_PIO_ID);
		pmc_enable_periph_clk(EBUT3_PIO_ID);
		pmc_enable_periph_clk(ELED1_PIO_ID);
		pmc_enable_periph_clk(ELED2_PIO_ID);
		pmc_enable_periph_clk(ELED3_PIO_ID);
		
		pio_set_input(EBUT1_PIO, EBUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
		pio_set_input(EBUT2_PIO, EBUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
		pio_set_input(EBUT3_PIO, EBUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
		pio_set_output(ELED1_PIO, ELED1_PIO_IDX_MASK, 0, 0, 0);
		pio_set_output(ELED2_PIO, ELED2_PIO_IDX_MASK, 0, 0, 0);
		pio_set_output(ELED3_PIO, ELED3_PIO_IDX_MASK, 0, 0, 0);
		
		pio_handler_set(EBUT1_PIO, EBUT1_PIO_ID, EBUT1_PIO_IDX_MASK, PIO_IT_FALL_EDGE, button0_callback);
		pio_handler_set(EBUT2_PIO, EBUT2_PIO_ID, EBUT2_PIO_IDX_MASK, PIO_IT_FALL_EDGE, button1_callback);
		pio_handler_set(EBUT3_PIO, EBUT3_PIO_ID, EBUT3_PIO_IDX_MASK, PIO_IT_FALL_EDGE, button2_callback);

		NVIC_EnableIRQ(EBUT1_PIO_ID);
		NVIC_EnableIRQ(EBUT2_PIO_ID);
		NVIC_EnableIRQ(EBUT3_PIO_ID);
		
		NVIC_SetPriority(EBUT1_PIO_ID, 5);
		NVIC_SetPriority(EBUT2_PIO_ID, 5);
		NVIC_SetPriority(EBUT3_PIO_ID, 5);
		
		pio_enable_interrupt(EBUT1_PIO, EBUT1_PIO_IDX_MASK);
		pio_enable_interrupt(EBUT2_PIO, EBUT2_PIO_IDX_MASK);
		pio_enable_interrupt(EBUT3_PIO, EBUT2_PIO_IDX_MASK);
		
		pio_clear(ELED1_PIO, ELED1_PIO_IDX_MASK);
		pio_set(ELED2_PIO, ELED2_PIO_IDX_MASK);
		pio_set(ELED3_PIO, ELED3_PIO_IDX_MASK);
	
}


  
void led_on(uint id, uint on){
	if(on == 0){
		if(id == 1){
			pio_set(ELED1_PIO, ELED1_PIO_IDX_MASK);
		}
		if(id == 2){
			pio_set(ELED2_PIO, ELED2_PIO_IDX_MASK);
		}
		if(id == 3){
			pio_set(ELED3_PIO, ELED3_PIO_IDX_MASK);
		}
	}
	
	else if(on == 1){
		if(id == 1){
			pio_clear(ELED1_PIO, ELED1_PIO_IDX_MASK);
		}
		if(id == 2){
			pio_clear(ELED2_PIO, ELED2_PIO_IDX_MASK);
		}
		if(id == 3){
			pio_clear(ELED3_PIO, ELED3_PIO_IDX_MASK);
		}
	}
}

int protocol_check_led(char *string){
	char led =  "LEDS";
	if(strcmp(string, "LEDS") == 0){
		return 1;
	}
	else{
		return 0;
	}
}

/************************************************************************/
/* MAIN                                                                 */
/************************************************************************/
int main(void)
{
	/* Initialize the board. */
	sysclk_init();
	board_init();

	/* Initialize the UART console. */
	configure_console();
   
	if (xTaskCreate(task_io, "io", TASK_IO_STACK_SIZE, NULL, TASK_IO_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create io task\r\n");
	}
		
	if (xTaskCreate(task_string, "string", TASK_STRING_STACK_SIZE, NULL, TASK_STRING_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create string task\r\n");
	}
	
	if (xTaskCreate(task_oled1, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create oled task\r\n");
	}


	vTaskStartScheduler();
	
	while(1) {};
	return 0;

}
