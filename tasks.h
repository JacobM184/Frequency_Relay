#ifndef TASKS_H_
#define TASKS_H_
void handler_task(void *pvParameter);

void check_stability_task(void *pvParameter);

void timer_task(void *pvParameter);
void shed_loads();

void reconnect_loads();
void switch_poll_task(void *pvParameters);

void led_update_task(void *pvParameters);


#endif /* TASKS_H_ */



