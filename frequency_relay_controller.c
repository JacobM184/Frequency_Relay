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
#define STABLE					2
#define MAINTAIN				1
#define LOADMANAGE				0

//lcd
FILE *lcd;
// Definition of Task Stacks
#define   TASK_STACKSIZE       2048

#define ROC_THRESHOLD 50
#define FREQ_THRESHOLD 49

QueueHandle_t Q_freq_calc;
QueueHandle_t Q_freq_data;
QueueHandle_t Q_switch_state;
QueueHandle_t Q_stability;

TimerHandle_t timer500;

// Definition of Semaphore
SemaphoreHandle_t stablephore;


// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);

// Task Priorities
#define SWITCH_POLL_PRIORITY		1
#define LCD_PRIORITY				2
#define LED_CTRL_PRIORITY			3
#define FREQ_CALC_PRIORITY			4
#define STABILITY_TASK_PRIORITY 	5
#define LOAD_MANAGE_PRIORITY		6


// Global Variables
uint8_t stability = 1;
uint8_t saveSwitch = 0x0;
uint8_t prevStability = 1;

uint8_t mode = MAINTAIN;

//function declarations
void shed_loads();
uint8_t get_i(uint8_t switch_state);
void reconnect_loads();
uint8_t check_loads();

void freq_relay(){
	#define SAMPLING_FREQ 16000.0
	double temp = SAMPLING_FREQ/(double)IORD(FREQUENCY_ANALYSER_BASE, 0);
	xQueueSendFromISR(Q_freq_calc, &temp, 0);
	return;
}

void button_isr(){
	if (mode == MAINTAIN){
		mode = LOADMANAGE;
		printf("MODE: %d\n",mode);
		xTimerStartFromISR(timer500,0);
		saveSwitch = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F;

	} else {
		mode = MAINTAIN;
		printf("MODE: %d\n",mode);
		xTimerStopFromISR(timer500,0);
	}

	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}

void freq_calc_task(void *pvParameter){
	double freq[100], dfreq[100], freqData[2];
	int i = 99;

	while(1){
		xQueueReceive(Q_freq_calc, freq+i, portMAX_DELAY);

		if(i==0){
			dfreq[0] = (freq[0]-freq[99]) * 2.0 * freq[0] * freq[99] / (freq[0]+freq[99]);
		}
		else{
			dfreq[i] = (freq[i]-freq[i-1]) * 2.0 * freq[i]* freq[i-1] / (freq[i]+freq[i-1]);
		}
		freqData[0] = freq[i];
		freqData[1] = dfreq[i];

//			printf("Freq: %f\n",freq[i]);
		xQueueSend(Q_freq_data,freqData,0);

		i =	++i%100; //point to the next data (oldest) to be overwritten

	}

}

void stability_task(void *pvParameter){

	double freqData[2];
	while(1){
		xQueueReceive(Q_freq_data, freqData, portMAX_DELAY);

		if((freqData[0] < FREQ_THRESHOLD) || (freqData[1] > ROC_THRESHOLD)){
			stability = 0;

		}else{
			stability = 1;
		}

//		printf("Stab: %d\n",stability);
		if((stability != prevStability) && (mode != MAINTAIN)){
			xTimerReset(timer500,0);
			prevStability = stability;
		}
		xQueueSend(Q_stability, &stability,0);
	}
}

void load_manage_task(void *pvParameter){
	uint8_t rec_stability;
	uint8_t load_op;
	while(1){

		xSemaphoreTake(stablephore,portMAX_DELAY);
		xQueueReceive(Q_stability,&rec_stability,0);

		printf("Rec Stab: %d\n",rec_stability);
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE,(IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE) & saveSwitch));
		if (rec_stability == 0){
//			printf("Unstable\n");
			shed_loads();

		} else {
//			printf("Stable\n");
			reconnect_loads();
//			if (check_loads()){
//				mode=STABLE;
//			}
		}


	}
}

void shed_loads(){
	uint8_t i;
	uint8_t ledValueR,ledValueG;

	for (i = 0x1; i<=0x1F ;i*=2){
		if (IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE)& i){

			ledValueR = (IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE)&(~i)) & 0x1F;
			ledValueG = (IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE) || i) & 0x1F;

			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE,ledValueR);
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE,ledValueG);

			break;
		}
	}

}

void reconnect_loads(){

	uint8_t i;
	uint8_t ledValueR,ledValueG;
	for (i = get_i(saveSwitch); i > 0; i/=2){
		ledValueG = (IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE)&(~i)) & 0x1F;
		ledValueR = (IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE) || i) & 0x1F;

		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE,ledValueR);
		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE,ledValueG);
	}
}

uint8_t get_i(uint8_t switch_state){
	uint8_t i;
	if(switch_state <= 0x1F && switch_state >= 0x10){
		i = 0x10;
	}else if(switch_state < 0x10 && switch_state >= 0x8){
		i = 0x8;
	}else if(switch_state < 0x8 && switch_state >= 0x4){
		i = 0x4;
	}else if(switch_state < 0x4 && switch_state >= 0x2){
		i = 0x2;
	}else if(switch_state < 0x2 && switch_state >= 0x1){
		i = 0x1;
	}else{
		i = 0x0;
	}
	return i;
}

uint8_t check_loads(){

	if(IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE)== saveSwitch){
		return 1;
	} else{
		return 0;
	}
}

void switch_poll_task(void *pvParameter){
	uint8_t prevSwitchState = 0x0;
	uint8_t switchState;
	while(1){
		if(mode == MAINTAIN || mode == STABLE){
			switchState = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F;
		}else{
			switchState = saveSwitch & IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
			saveSwitch = switchState;
		}

		if (prevSwitchState != switchState){
			xQueueSend(Q_switch_state, &switchState,0);
		}
		prevSwitchState = switchState;

	}
}

void led_control_task(void *pvParameter){
	uint8_t rec_switchState;
	uint8_t loadLEDs;
	uint8_t rec_load_op;
	while(1){
		if (mode == MAINTAIN || mode == STABLE){
			xQueueReceive(Q_switch_state,&rec_switchState,portMAX_DELAY);
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE,rec_switchState);
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE,0x0);
		} else {




		}
	}
}

void lcd_task(void *pvParameter){
	  lcd = fopen(CHARACTER_LCD_NAME, "w");
	while (1){
		#define ESC 27
        #define CLEAR_LCD_STRING "[2J"
        fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
        fprintf(lcd, "Mode: %d\n", mode);
        vTaskDelay(100);
	}
}
void vTimerCallback(xTimerHandle t_timer){
	xSemaphoreGive(stablephore);
}

int main(int argc, char* argv[], char* envp[])
{



	initOSDataStructs();
	initCreateTasks();
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x4);
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);
	alt_irq_register(PUSH_BUTTON_IRQ, 0, button_isr);

	timer500 = xTimerCreate("Timer500", pdMS_TO_TICKS(500), pdTRUE, NULL, vTimerCallback);
	vTaskStartScheduler();

	while(1){

	}

  return 0;
}

// This function simply creates a message queue and a semaphore
int initOSDataStructs(void)
{
	Q_freq_calc = xQueueCreate(100, sizeof(double));
	Q_freq_data = xQueueCreate(10, (sizeof(double)*2));
	Q_switch_state = xQueueCreate(10, sizeof(uint8_t));
	Q_stability = xQueueCreate(10, sizeof(uint8_t));

	stablephore = xSemaphoreCreateBinary();
	return 0;
}

// This function creates the tasks used in this example
int initCreateTasks(void)
{
	xTaskCreate(freq_calc_task, "freq_calc_task", TASK_STACKSIZE, NULL, FREQ_CALC_PRIORITY, NULL);
	xTaskCreate(stability_task, "stability_task", TASK_STACKSIZE, NULL, STABILITY_TASK_PRIORITY, NULL);
	xTaskCreate(load_manage_task, "load_manage_task", TASK_STACKSIZE, NULL, LOAD_MANAGE_PRIORITY, NULL);

	xTaskCreate(switch_poll_task, "switch_poll_task", TASK_STACKSIZE, NULL, SWITCH_POLL_PRIORITY, NULL);
	xTaskCreate(led_control_task, "led_control_task", TASK_STACKSIZE, NULL, LED_CTRL_PRIORITY, NULL);
	xTaskCreate(lcd_task, "lcd_task", TASK_STACKSIZE, NULL, LCD_PRIORITY, NULL);

	return 0;
}
