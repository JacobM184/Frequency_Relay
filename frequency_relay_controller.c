#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <system.h>
#include "sys/alt_irq.h"
#include "io.h"
#include "altera_avalon_pio_regs.h"


// Task State definitions
#define MAINTAIN				2
#define LOADMANAGE				1
#define STABLE					0
// Definition of Task Stacks
#define   TASK_STACKSIZE       2048

#define ROC_THRESHOLD 20
#define FREQ_THRESHOLD 49

QueueHandle_t Q_freq_calc;
QueueHandle_t Q_freq_data;

// Definition of Semaphore
SemaphoreHandle_t mode_sem;

// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);

// Task Priorities
#define FREQ_CALC_PRIORITY		3
#define STABILITY_TASK_PRIORITY 4
#define SWITCH_POLL_PRIORITY	1

// Global Variables
uint8_t stability = 1;
uint8_t saveSwitch = 0x0;
uint8_t switchState = 0x0;

void freq_relay(){
	#define SAMPLING_FREQ 16000.0
	double temp = SAMPLING_FREQ/(double)IORD(FREQUENCY_ANALYSER_BASE, 0);
	xQueueSendFromISR(Q_freq_calc, &temp, 0);
	return;
}

void freq_calc_task(void *pvParameter){
	double freq[100], dfreq[100], freqData[2];
	int i = 99;

	while(1){
		while(uxQueueMessagesWaiting(Q_freq_calc) != 0){
			xQueueReceive(Q_freq_calc, &freq+i, 0);

			if(i==0){
				dfreq[0] = (freq[0]-freq[99]) * 2.0 * freq[0] * freq[99] / (freq[0]+freq[99]);
			}
			else{
				dfreq[i] = (freq[i]-freq[i-1]) * 2.0 * freq[i]* freq[i-1] / (freq[i]+freq[i-1]);
			}
			freqData[0] = freq[i];
			freqData[1] = dfreq[i];

			xQueueSend(Q_freq_data,freqData,0);

			i =	++i%100; //point to the next data (oldest) to be overwritten
		}
	}

}

void stability_task(void *pvParameter){

	double freqData[2];
	while(1){
		while(uxQueueMessagesWaiting(Q_freq_calc) != 0){
			xQueueReceive(Q_freq_data, freqData, 0);

			if((freqData[0] < FREQ_THRESHOLD) || (freqData[1] > ROC_THRESHOLD)){
				stability = 0;
				saveSwitch = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F;
			}else{
				stability = 1;
			}

		}

	}
}

void switch_poll_task(void *pvParameter){
	while(1){
		switchState = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F;
	}
}

void led_control_task(void *pvParameter){
	while(1){
		if(uxSempahoreGetCount(mode_sem) == 1){
			//load manange
		}else{
			//maintenance mode
		}

	}
}

int main(int argc, char* argv[], char* envp[])
{
	Q_freq_calc = xQueueCreate(100, sizeof(double));
	Q_freq_data = xQueueCreate(1, (sizeof(double)*2));

	initOSDataStructs();
	initCreateTasks();

	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);

	vTaskStartScheduler();

	while(1){

	}

  return 0;
}

// This function simply creates a message queue and a semaphore
int initOSDataStructs(void)
{
	mode_sem = xSemaphoreCreateBinary();
	return 0;
}

// This function creates the tasks used in this example
int initCreateTasks(void)
{
	xCreateTask(freq_calc_task, "freq_calc_task", TASK_STACKSIZE, NULL, FREQ_CALC_PRIORITY, NULL);
	xCreateTask(stability_task, "stability_task", TASK_STACKSIZE, NULL, STABILITY_TASK_PRIORITY, NULL);
	xCreateTask(switch_poll_task, "switch_poll_task", TASK_STACKSIZE, NULL, SWITCH_POLL_PRIORITY, NULL);

	return 0;
}
