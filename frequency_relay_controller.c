#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <system.h>
#include "sys/alt_irq.h"
#include "io.h"
#include "altera_avalon_pio_regs.h"

// Scheduler includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "FreeRTOS/timers.h"

#include "tasks.h"

//Task Priorities
#define LED_UPDATE_TASK_PRIORITY			4

// Definition of Task Stacks
#define   TASK_STACKSIZE       2048

// Definition of Message Queue
#define   MSG_QUEUE_SIZE  100
QueueHandle_t msgqueue;
static QueueHandle_t Q_freq_data;



//timer
TimerHandle_t timer;

// Definition of Semaphore
SemaphoreHandle_t led_sem;

// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);

//global variable
int modeSelect = 0; //0 - Stable, 1 - Load Management, 2 - Maintenance

void freq_relay(){
	#define SAMPLING_FREQ 16000.0
	double temp = SAMPLING_FREQ/(double)IORD(FREQUENCY_ANALYSER_BASE, 0);

	xQueueSendToBackFromISR( Q_freq_data, &temp, pdFALSE );

	return;
}

void button_interrupt_isr()
{

	xSemaphoreGiveFromISR(led_sem, 0);

	if (modeSelect == 0){
		printf("Maintenance Mode\n");
	  modeSelect = 2;
	  xTimerStopFromISR(timer,0);
	} else {
	  modeSelect = 0;
		printf("Stable Mode\n");
	  xTimerStartFromISR(timer,0);
	}

	// clears the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	return;
}


int main(int argc, char* argv[], char* envp[])
{

	Q_freq_data = xQueueCreate( 100, sizeof(double) );

	initOSDataStructs();
	initCreateTasks();

	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x4);
	alt_irq_register(PUSH_BUTTON_IRQ, 0, button_interrupt_isr);
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);

	//Timer
//	timer = xTimerCreate("Load Shed Timer", pdMS_TO_TICKS(500), pdTRUE, NULL, vTimerCallback);

	vTaskStartScheduler();

	while(1){

	}

  return 0;
}

// This function simply creates a message queue and a semaphore
int initOSDataStructs(void)
{
	led_sem = xSemaphoreCreateBinary();
	return 0;
}

// This function creates the tasks used in this example
int initCreateTasks(void)
{

	xTaskCreate(led_update_task, "led_update_task", TASK_STACKSIZE, NULL, LED_UPDATE_TASK_PRIORITY, NULL);
	return 0;
}
