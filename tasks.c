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

//Global Variables
uint8_t modeSelect = 0; // 0 - Stable | 1 - Load Management | 2 - Maintenance

uint8_t uiSwitchValue; //
uint8_t ledValue;


void stable_task(void *pvParameter){
    while(1){
        
    }
}

void maintenance_mode_task(void *pvParameter){
    while(1){

    }
}

void switch_poll_task(void *pvParameter){
    while(1){
        uiSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
        uiSwitchValue = uiSwitchValue & 0x1F;
    }

}
void led_update_task(void *pvParameter){
    while(1){
        unsigned int LEDs;
        LEDs = ~ledValue & 0x1F;

        if (modeSelect == 0){
            IOWR_AVALON_ALTERA_PIO_DATA(RED_LEDS_BASE, (~IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE)) & 0X1F);
            switch_state = 
        }
    }
}