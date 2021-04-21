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

//Task Priorities
#define LED_UPDATE_TASK_PRIORITY				4
#define SWITCH_POLL_TASK_PRIORITY				4
#define FREQ_CALC_TASK_PRIORITY					4
#define CHECK_STABILITY_TASK_PRIORITY			4
#define LOAD_MANAGE_TASK_PRIORITY				4

// Task State definitions
#define MAINTAIN				2
#define LOADMANAGE				1
#define STABLE					0
// Definition of Task Stacks
#define   TASK_STACKSIZE       2048

#define ROC_THRESHOLD 20
#define FREQ_THRESHOLD 49

//timer
TimerHandle_t timer;

// Definition of Semaphore
SemaphoreHandle_t led_sem;

// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);

//Queue
QueueHandle_t Q_freq_data;
QueueHandle_t Q_stab_data;
QueueHandle_t Q_stability;

//Global Variables
int modeSelect = MAINTAIN;
double stab_data[2];

uint8_t uiSwitchValue;
uint8_t greenLedValue = 0x0;
uint8_t stability;

unsigned int LEDs;

//function declarations
void switch_poll_task(void *pvParameters);
void led_update_task(void *pvParameters);
void freq_calc_task(void *pvParameters);
void load_manage_task(void *pvParameters);

void handler_task(void *pvParameter);

void freq_calc_task(void *pvParameter);
void check_stability_task(void *pvParameter);

void timer_task(void *pvParameter);
void shed_loads();

void reconnect_loads();


void button_interrupt_isr()
{

	//xSemaphoreGiveFromISR(led_sem, 0);

	if (modeSelect != MAINTAIN){
		printf("Maintenance Mode\n");
		modeSelect = MAINTAIN;
		xTimerStopFromISR(timer,0);
	} else {
		modeSelect = STABLE;
		printf("Stable Mode\n");
		xTimerResetFromISR(timer,0);
	}

	// clears the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	return;
}

void shed_loads(){

	uint8_t i;

	for (i = 0x1; i < 0x1F; i*=2){
		if(IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & i){
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, (IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE)& ~i));

			greenLedValue = (greenLedValue || i);
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, greenLedValue);
			printf("load shed");
			return;
		}
	}
}

void reconnect_loads(){

}

void load_manage_task(void *pvParameters){

	double rec_stability;
	while(1){
		while(uxQueueMessagesWaiting( Q_stability ) != 0){
			xQueueReceive(Q_stability, &rec_stability,0);
			printf("Rec Stab: %f\n",rec_stability);
			if (rec_stability == 0){
				shed_loads();
			} else {
				reconnect_loads();
			}
		}
	}
}

void freq_calc_task(void *pvParameter){


	double freq[100], dfreq[100];
	int i = 99;

	while(1){
		if (modeSelect != MAINTAIN){
			while(uxQueueMessagesWaiting( Q_freq_data ) != 0){
				xQueueReceive(Q_freq_data,freq+i, 0);

				if(i==0){
					dfreq[0] = (freq[0]-freq[99]) * 2.0 * freq[0] * freq[99] / (freq[0]+freq[99]);
				}
				else{
					dfreq[i] = (freq[i]-freq[i-1]) * 2.0 * freq[i]* freq[i-1] / (freq[i]+freq[i-1]);
				}

				if (dfreq[i] > 100.0){
					dfreq[i] = 100.0;
				}

				stab_data[0] = freq[i];
				stab_data[1] = dfreq[i];

				xQueueSend(Q_stab_data,&stab_data,0);

				i =	++i%100; //point to the next data (oldest) to be overwritten
			}
		}
	}
}
void check_stability_task(void *pvParameter){
	double rec_stabData[2];
	double prevStability = stability;
	while(1){
		if (modeSelect != MAINTAIN){
			while(uxQueueMessagesWaiting( Q_stab_data ) != 0){
				xQueueReceive(Q_stab_data,&rec_stabData, 0);

				double freq = rec_stabData[0];
				double RoC = rec_stabData[1];

				if (freq < FREQ_THRESHOLD || RoC > ROC_THRESHOLD){
					stability = 0;
					modeSelect = LOADMANAGE;
//					printf("Sys unstable\n");
				} else {
					stability = 1;
					//modeSelect = STABLE;
//					printf("Sys stable\n");
				}

				if (prevStability != stability){
					xTimerReset(timer,0);
					printf("RESET\n");
				}

				prevStability = stability;
			}
		}
	}
}

void switch_poll_task(void *pvParameter){
    while(1){
    	if (modeSelect != LOADMANAGE){
    		uiSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
			uiSwitchValue = uiSwitchValue & 0x1F;
    	}
    }
}

void led_update_task(void *pvParameter){
    while(1){
        if (modeSelect == MAINTAIN){
            IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F);
            IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0x0);



        }else if (modeSelect == LOADMANAGE){
        	uiSwitchValue = (~IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F) & uiSwitchValue;
        	IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE,uiSwitchValue);


        }
        vTaskDelay(100);
    }
}

void freq_relay(){
	#define SAMPLING_FREQ 16000.0
	double temp = SAMPLING_FREQ/(double)IORD(FREQUENCY_ANALYSER_BASE, 0);

	xQueueSendToBackFromISR( Q_freq_data, &temp, pdFALSE );

	return;
}

void vTimerCallback(xTimerHandle t_timer){
	double sysStability = stability;
//	xQueueSend(Q_stability,&sysStability,0);
	printf("tout: %f\n",sysStability);

}

int main(int argc, char* argv[], char* envp[])
{

	Q_freq_data = xQueueCreate( 100, sizeof(double) );
	Q_stab_data = xQueueCreate( 2, sizeof(stab_data) );
	Q_stability = xQueueCreate( 1, sizeof(double) );

	initOSDataStructs();
	initCreateTasks();

	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x4);
	alt_irq_register(PUSH_BUTTON_IRQ, 0, button_interrupt_isr);
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);

	//Timer
	timer = xTimerCreate("Load Shed Timer", pdMS_TO_TICKS(500), pdTRUE, NULL, vTimerCallback);
	if (xTimerStart(timer, 0) != pdPASS){
		printf("Cannot start timer");
	}

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
	xTaskCreate(switch_poll_task, "switch_poll_task", TASK_STACKSIZE, NULL, SWITCH_POLL_TASK_PRIORITY, NULL);
	xTaskCreate(check_stability_task, "check_stability_task", TASK_STACKSIZE, NULL, CHECK_STABILITY_TASK_PRIORITY, NULL);
	xTaskCreate(freq_calc_task, "freq_calc_task", TASK_STACKSIZE, NULL, FREQ_CALC_TASK_PRIORITY, NULL);
	xTaskCreate(load_manage_task, "load_manage_task", TASK_STACKSIZE, NULL, LOAD_MANAGE_TASK_PRIORITY, NULL);
	return 0;
}
