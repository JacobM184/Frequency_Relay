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
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"

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

double freq_Threshold = 49;
double RoC_Threshold = 50;

QueueHandle_t Q_freq_calc;
QueueHandle_t Q_freq_data;
QueueHandle_t Q_switch_state;
QueueHandle_t Q_stability;
QueueHandle_t Q_key;
QueueHandle_t Q_timestamp;

TimerHandle_t timer500;

// Definition of Semaphore
SemaphoreHandle_t stablephore;
SemaphoreHandle_t LEDaphore;
SemaphoreHandle_t chronophore;


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
#define KEY_PRIORITY				7


// Global Variables
uint8_t stability = 1;
uint8_t saveSwitch = 0x0;
uint8_t saveSwitch2 = 0x0;
uint8_t prevStability = 1;
uint8_t ledValueG = 0x0;
uint8_t ledValueR = 0x0;

uint8_t mode = MAINTAIN;

int check = 1;

//function declarations
void shed_loads();
uint8_t get_i(uint8_t switch_state);
void reconnect_loads();
uint8_t check_loads();
uint8_t keyToDig(unsigned char key);


void freq_relay(){
	#define SAMPLING_FREQ 16000.0
	double temp = SAMPLING_FREQ/(double)IORD(FREQUENCY_ANALYSER_BASE, 0);
	xQueueSendFromISR(Q_freq_calc, &temp, 0);
	return;
}

void button_isr(){

	// condition for changing to load management
	if (mode == MAINTAIN){
		mode = LOADMANAGE;
		printf("MODE: %d\n",mode);

		// start timer to being load management
		xTimerStartFromISR(timer500,0);

		// save the switch state at the moment of mode change
		saveSwitch = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F;
//		saveSwitch2 = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F;

		// save the Red LED state at the moment of mode change
		ledValueR = saveSwitch;
		ledValueG = 0x0;

	} else { // condition for changing to maintenance mode
		mode = MAINTAIN;
		printf("MODE: %d\n",mode);

		// stop the timer
		xTimerStopFromISR(timer500,0);
	}

	// clear edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}


void ps2_isr (void* context, alt_u32 id)
{
  char ascii;
  int status = 0;
  unsigned char key = 0;

  KB_CODE_TYPE decode_mode;
  status = decode_scancode (context, &decode_mode , &key , &ascii) ;
  if ( status == 0 ) //success
  {
    // print out the result
    switch ( decode_mode )
    {
      case KB_ASCII_MAKE_CODE :
//        printf ( "ASCII   : %x\n", key );
        break ;

      case KB_BINARY_MAKE_CODE :
//        printf ( "MAKE CODE : %x\n", key );
        break ;

      default :
//        printf ( "DEFAULT   : %x\n", key );
        break ;
    }
    IOWR(SEVEN_SEG_BASE,0 ,key);
    if(check && mode == MAINTAIN){
    	xQueueSendFromISR(Q_key,&key,0);
    	check = 0;
    } else {
    	check =1;
    }


    usleep(1000);
  }
}

void freq_calc_task(void *pvParameter){

	// intialise arrays to hold frequency and RoC of frequency data
	double freq[100], dfreq[100], freqData[2];

	// initialise i
	int i = 99;

	while(1){

		// receive frequwncy data from freq_relay()
		xQueueReceive(Q_freq_calc, freq+i, portMAX_DELAY);

		// calculate RoC
		if(i==0){
			dfreq[0] = (freq[0]-freq[99]) * 2.0 * freq[0] * freq[99] / (freq[0]+freq[99]);
		}
		else{
			dfreq[i] = (freq[i]-freq[i-1]) * 2.0 * freq[i]* freq[i-1] / (freq[i]+freq[i-1]);
		}

		// store frequency
		freqData[0] = freq[i];
		// store RoC of frequency
		freqData[1] = dfreq[i];

//			printf("Freq: %f\n",freq[i]);

		// send data
		xQueueSend(Q_freq_data,freqData,0);

		i =	++i%100; //point to the next data (oldest) to be overwritten

	}

}

void stability_task(void *pvParameter){

	//  initialise variable to store freq and RoC
	double freqData[2];
	int start_time;
	while(1){

		// recieve data from freq_calc_task()
		xQueueReceive(Q_freq_data, freqData, portMAX_DELAY);

		// check if data violates set thresholds
		if((freqData[0] < freq_Threshold) || (freqData[1] > RoC_Threshold)){


			start_time = xTaskGetTickCount();
			stability = 0;
//			printf("Unstable\n");

			if(xSemaphoreTake(chronophore,10) == pdTRUE){ // no real reason for 10 tick delay - just felt like it
				xQueueSend(Q_timestamp, &start_time,0);
			}


		}else{
//			printf("Stable\n");
			stability = 1;
			xQueueReset(Q_timestamp);
		}

//		printf("Stab: %d\n",stability);

		// reset timer on stability stae change
		if((stability != prevStability) && (mode != MAINTAIN)){
			xTimerReset(timer500,0);
			prevStability = stability;
		}

		// send stability state
		xQueueSend(Q_stability, &stability,0);
	}
}

void load_manage_task(void *pvParameter){

	// initialise variable to store recieved data
	uint8_t rec_stability;
	int end_time;
	int rec_start_time;
	int dtime;
	// uint8_t load_op;
	while(1){

		// blocking SemaphoreTake function to ensure that the task is blocked until timer times out
		xSemaphoreTake(stablephore,portMAX_DELAY);

		// recieve data from queue
		xQueueReceive(Q_stability,&rec_stability,0);
		// receive starting timestamp from stability task
//		xQueueReceive(Q_timestamp,&rec_start_time,0);
//		xQueueReset(Q_timestamp);

		// update saveSwitch for when swithces turned off
		saveSwitch = (IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & saveSwitch);
		// update red LED value
		ledValueR = saveSwitch;
		// check value of stability state
		if (rec_stability == 0){
//			printf("Unstable\n");

			// if the mode is stable, but state is unstable, change mode
			if(mode == STABLE){
				saveSwitch = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
				mode = LOADMANAGE;
			}
			// shed loads when unstable
			shed_loads();
			// give semaphore to led control
			xSemaphoreGive(LEDaphore);

			// get time after load shed
			end_time = xTaskGetTickCount();
			// calculate response time

			if(xQueueReceive(Q_timestamp,&rec_start_time,0) == pdTRUE){
				dtime = (end_time - rec_start_time);

				xQueueReset(Q_timestamp); // unsure if this is necessary
				printf("Time Taken: %d\n", dtime);
				printf("End: %d\n", (end_time));
				printf("Start: %d\n\n", (rec_start_time));
			}


			rec_start_time = 0;
		} else {

//			printf("Getting stable\n");
			 //check if all loads are connected when stable state



			if (check_loads()){
				saveSwitch = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
				ledValueR = saveSwitch;
				xQueueReset(Q_switch_state);
				mode=STABLE;
//				printf("Mode is STABLE\n");
			}else{

				// reconnect loads if all loads not reconnected
				reconnect_loads();
				// give semaphore to led control
				xSemaphoreGive(LEDaphore);

			}
		}


	}
}

void shed_loads(){
	// intitalise i
	uint8_t i;

	// for loop to check if any of the leds are 'on', individually
	for (i = 0x1; i<=0x1F; i*=2){

		// check if load is 'on'
		if ((saveSwitch & i) != 0){

			// update Red LED value
			ledValueR = ledValueR & saveSwitch;
			ledValueR = (ledValueR & (~i));
			saveSwitch = ledValueR;

			// update Green LED value
			ledValueG = (ledValueG | i) ;

			// ensure that managed loads switched off when corresponding switch is off
//			ledValueG = ledValueG & IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
			
			//break from loop
			break;
		}
	}
}

void reconnect_loads(){

	// initialise i
	uint8_t i,j;

	// update saveSwitch state
	if (saveSwitch == 0){
		saveSwitch = ledValueG;
	}

	// for loop to cycle through Green LEDs that are on, starting from highest priority
	for (i = get_i(saveSwitch); i > 0; i/=2){

		// check if GreenLED on
		if(ledValueG & i){

			j = i & IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);

			// update Green LED value
			ledValueG = (ledValueG & (~i)) & 0x1F;

			// update Red LED value if corresponding switch on
			if(j){
				ledValueR = ((ledValueR | j) & 0x1F) ;
				saveSwitch = ledValueR;
			}

			// break from loop
			break;
		}
	}
}

uint8_t get_i(uint8_t switch_state){

	// initialise i
	uint8_t i;

	// check if switch state between 11111 and 10000
	if(switch_state <= 0x1F && switch_state >= 0x10){
		i = 0x10;
	}else if(switch_state < 0x10 && switch_state >= 0x8){ // check if switch state between 10000 and 01000
		i = 0x8;
	}else if(switch_state < 0x8 && switch_state >= 0x4){ // check if switch state between 01000 and 00100
		i = 0x4;
	}else if(switch_state < 0x4 && switch_state >= 0x2){ // check if switch state between 00100 and 00010
		i = 0x2;
	}else if(switch_state < 0x2 && switch_state >= 0x1){ // check if switch state between 00010 and 00001
		i = 0x1;
	}else{
		i = 0x0;
	}
	return i;
}

uint8_t check_loads(){

	// check if the Red LEDs are == to the saved switch state
	if(ledValueG == 0){

		//update saveSwitch
		saveSwitch = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);

		return 1;
	} else{
		
		return 0;
	}
}

void switch_poll_task(void *pvParameter){

	// initialise vairiable for temporary storage of switch state
	uint8_t switchState;

	while(1){
		
		// read switches
		switchState = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F;

		// send switch state via queue
		xQueueSend(Q_switch_state, &switchState,0);

	}
}

void led_control_task(void *pvParameter){

	// initialise variable to receive switch state
	uint8_t rec_switchState;

	while(1){


		// check if mode is Maintenance or stability
		if (mode == MAINTAIN || mode == STABLE){
			// receive switch state from switch_poll_task()
			xQueueReceive(Q_switch_state,&rec_switchState,portMAX_DELAY);
//			rec_switchState = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F;
			// update Red LEDS
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE,rec_switchState);
//			printf("LED R led ctrl %d\n",rec_switchState);

			// turn off Green LEDs
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE,0x0);
		}else{
			xSemaphoreTake(LEDaphore, 100);
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE,ledValueR);
			 ledValueG = ledValueG & IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, (ledValueG));

			//vTaskDelay(50);
		}
	}


}

void lcd_task(void *pvParameter){

	// open LCD file in write mode
	lcd = fopen(CHARACTER_LCD_NAME, "w");

	while (1){

		// clear display
		#define ESC 27
        #define CLEAR_LCD_STRING "[2J"
        fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);

		// print mode
        fprintf(lcd, "Mode: %d\n", mode);

		// delay to stop flickering
        vTaskDelay(100);
	}
}

void key_task(void *pvParameter){
	unsigned char rec_key;
	unsigned char freq_threshold[6];
	unsigned char RoC_threshold[3];
	unsigned char select;
	uint8_t temp = 0;
	uint8_t count = 0;
	int i = 0;

	// receive initial key press value so that there is no issue on startup
	xQueueReceive(Q_key, &rec_key, portMAX_DELAY);

	// potentially consider a xQueueReset() instead?

	while(1){

		// check if the mode is maintainence
		if(mode == MAINTAIN){

			// block until key received
			xQueueReceive(Q_key, &rec_key, portMAX_DELAY);

			// increment or decrement frequency values by 1 
			// for correct key presses
			if (rec_key == 0x55){
				freq_Threshold ++;
			} else if (rec_key == 0x4e){
				freq_Threshold --;

			// increment or decrement RoC values by 0.1
			// for correct key presses
			} else if (rec_key == 0x79){
				RoC_Threshold +=.1;
			} else if (rec_key == 0x7b){
				RoC_Threshold -=.1;
			}


			// output new frequenct and RoC thresholds
			printf("Freq: %f\n", freq_Threshold);
			printf("RoC: %f\n", RoC_Threshold);
		}
	}
}

void vTimerCallback(xTimerHandle t_timer){

	// give semaphore to start response timer
	xSemaphoreGive(chronophore);

	// give semaphore to unblock load management 
	xSemaphoreGive(stablephore);
}

int main(int argc, char* argv[], char* envp[])
{


	// intialise tasks, queues and semaphores
	initOSDataStructs();
	initCreateTasks();

	//ps2 device setup
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);

	if(ps2_device == NULL){
		printf("can't find PS/2 device\n");
		return 1;
	}

	alt_up_ps2_enable_read_interrupt(ps2_device);
	alt_irq_register(PS2_IRQ, ps2_device, ps2_isr);

	// clear edge capture register for push buttons
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);

	// create a interrupt mask for push buttons
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x4);

	// register push button interrupt
	alt_irq_register(PUSH_BUTTON_IRQ, 0, button_isr);

	// register frequency analyser interrupt
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);

	// create timer
	timer500 = xTimerCreate("Timer500", pdMS_TO_TICKS(500), pdTRUE, NULL, vTimerCallback);

	// start the task scheduler
	vTaskStartScheduler();

	while(1){

	}

  return 0;
}

// This function simply creates message queue(s) and semaphore(s)
int initOSDataStructs(void)
{
	Q_freq_calc = xQueueCreate(100, sizeof(double));
	Q_freq_data = xQueueCreate(10, (sizeof(double)*2));
	Q_switch_state = xQueueCreate(10, sizeof(uint8_t));
	Q_stability = xQueueCreate(10, sizeof(uint8_t));
	Q_key = xQueueCreate(10, sizeof(unsigned char));
	Q_timestamp = xQueueCreate(10, sizeof(int));

	stablephore = xSemaphoreCreateBinary();
	LEDaphore = xSemaphoreCreateBinary();
	chronophore = xSemaphoreCreateBinary();
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
	xTaskCreate(key_task, "key_task", TASK_STACKSIZE, NULL, KEY_PRIORITY, NULL);
	return 0;
}
