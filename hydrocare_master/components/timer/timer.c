#include "timer.h"


gptimer_handle_t gptimer = NULL;

/**
 * @brief Updated Callback leveraging the user_data context pointer
 */
static bool IRAM_ATTR on_timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) 
{
    bool high_task_awoken = false; 

    // Cast the untyped void pointer back to your boolean pointer
    volatile bool *stream_flag = (volatile bool *)user_data;
    
    if (stream_flag != NULL) {
        *stream_flag = true; // Safely flip the flag without a global reference
    }

    return high_task_awoken;
}

// Pass the address of your flag variable directly into init
void initTimer(volatile bool *target_flag)
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = TIMER_ALARM_US,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = on_timer_alarm_cb,
    };
    
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, (void *)target_flag));

    ESP_ERROR_CHECK(gptimer_enable(gptimer));
}

void setTimer(void)
{
    ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer, 0));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

void disableTimer(void)
{
    ESP_ERROR_CHECK(gptimer_stop(gptimer));
}