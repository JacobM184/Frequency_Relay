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



//Global Variable

uint8_t uiSwitchValue = 0; 	//entire switch
uint8_t ledValue = 0;
uint8_t switchState = 0;	//switch state of 5 switches

extern int modeSelect;

void handler_task(void *pvParameter){

}

void check_stability_task(void *pvParameter){
}

void timer_task(void *pvParameter){
}
void shed_loads(){
}

void reconnect_loads(){

}

void switch_poll_task(void *pvParameters){
	while(1){
		uiSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE );
		uiSwitchValue = uiSwitchValue & 0x1F;
	}
}

void led_update_task(void *pvParameters){
	while (1){
		if(modeSelect == 0 || modeSelect == 2){
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & 0x1F);
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0x0);
		}

		vTaskDelay(100);
	}
}
