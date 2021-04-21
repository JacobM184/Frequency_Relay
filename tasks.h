void stable_task(void *pvParameter);

void load_manage_mode_task(void *pvParameter);

void maintenance_mode_task(void *pvParameter);

void check_stability_task(void *pvParameter);
void timer_task(void *pvParameter);
void shed_loads();
void reconnect_loads();
void switch_poll_task(void *pvParameter);
void led_update_task(void *pvParameter);