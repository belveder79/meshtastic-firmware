#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && defined(REEDCOUNTER_SENSOR_EN)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "ReedCounterSensor.h"
#include "TelemetrySensor.h"

ReedCounterSensor::ReedCounterSensor() : TelemetrySensor(meshtastic_TelemetrySensorType_SENSOR_UNSET, "ReedCounter") 
{
    m_currentCount = 0;
}

static xQueueHandle pcnt_event_queue;   // A queue to handle pulse counter events

#if defined(RAK_4631) && RAK_4631 == 1

// Interrupt Service Routine (ISR)
void pcnt_interrupt_handler() 
{
  // Static variable retains its value between ISR executions
  static uint32_t lastInterruptTime = 0; 
  uint32_t currentTime = millis();

  // If the pulse arrives too fast, skip it (it's switch bounce)
  if (currentTime - lastInterruptTime > REEDCOUNTER_PCNT_HIGH_LIMIT) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Send the valid pulse timestamp to the queue
    xQueueSendFromISR(pcnt_event_queue, &currentTime, &xHigherPriorityTaskWoken);
    
    lastInterruptTime = currentTime; // Update the lockout timer

    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}

#else // RAK4631

  #ifdef OLDPCNT
    /* A sample structure to pass events from the PCNT
    * interrupt handler to the main program.
    */
    typedef struct {
        int unit;  // the PCNT unit that originated an interrupt
        uint32_t status; // information on the event type that caused the interrupt
    } pcnt_evt_t;

    /* Decode what PCNT's unit originated an interrupt
    * and pass this information together with the event type
    * the main program using a queue.
    */
    static void IRAM_ATTR pcnt_interrupt_handler(void *arg)
    {
        pcnt_unit_t pcnt_unit = (pcnt_unit_t)((int)arg);
        pcnt_evt_t evt;
        evt.unit = pcnt_unit;
        /* Save the PCNT event type that caused an interrupt
        to pass it to the main program */
        pcnt_get_event_status(pcnt_unit, &evt.status);
        xQueueSendFromISR(pcnt_event_queue, &evt, NULL);
    }

  #else
    // Interrupt callback function triggered when the pulse count threshold is reached
    static bool pcnt_on_reach_cb(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) 
    {
        BaseType_t high_task_wakeup = pdFALSE;
        QueueHandle_t queue = (QueueHandle_t)user_ctx;
        
        // Send the current count/event to the FreeRTOS queue
        xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
        
        return (high_task_wakeup == pdTRUE);
    }
#endif

#endif // RAK4631

/* Contact detection task */
static void checkreed(void *arg)
{
    LOG_DEBUG("===================== Contact sensor monitoring started");
    ReedCounterSensor* instance = static_cast<ReedCounterSensor*>(arg);
    int count_val;
    while (1) {
        if (xQueueReceive(pcnt_event_queue, &count_val, portMAX_DELAY)) {
            LOG_DEBUG("Reed Switch Triggered! Event threshold reached.");
            instance->increaseCounterAndResetHandle();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  /* Check every 1000ms */
    }
}

void ReedCounterSensor::increaseCounterAndResetHandle()
{
    m_currentCount++;

#if defined(RAK_4631) && RAK_4631 == 1

#else // RAK4631
  #ifdef OLDPCNT
      pcnt_counter_pause(m_pcnt_unit);
      pcnt_counter_clear(m_pcnt_unit);
      pcnt_counter_resume(m_pcnt_unit);
  #else
      // Clear the counter so we wait for the next pulse event
      pcnt_unit_clear_count(m_pcnt_unit);
  #endif
#endif // RAK4631
}

bool ReedCounterSensor::initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev)
{
  LOG_DEBUG("===================== Init sensor: %s", sensorName);

#if defined(RAK_4631) && RAK_4631 == 1

  // Create the background worker task
  xTaskCreate(checkreed, "reed_check", 2048, this, 1, &m_Handle);

  // 1. Create a Queue to pass events from ISR to the main task
  pcnt_event_queue = xQueueCreate(10, sizeof(int));
  if (pcnt_event_queue == NULL) {
    LOG_ERROR("===================== Queue creation failed");
    return false;
  }

  pinMode(REEDCOUNTER_GPIO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REEDCOUNTER_GPIO), pcnt_interrupt_handler, FALLING);
    
  LOG_DEBUG("===================== Monitoring Reed Switch...");
#else // RAK4631
    // 1. Create a Queue to pass events from ISR to the main task
    pcnt_event_queue = xQueueCreate(10, sizeof(int));
    if (pcnt_event_queue == NULL) {
        LOG_ERROR("===================== Queue creation failed");
        return false;
    }

  #ifdef OLDPCNT
    m_pcnt_unit = PCNT_UNIT_0;

    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = REEDCOUNTER_GPIO,
        .ctrl_gpio_num = -1,
        
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_KEEP, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge

        // Set the maximum and minimum limit values to watch
        .counter_h_lim = REEDCOUNTER_PCNT_HIGH_LIMIT,
        .counter_l_lim = REEDCOUNTER_PCNT_LOW_LIMIT,
        .unit = m_pcnt_unit,
        .channel = PCNT_CHANNEL_0,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(m_pcnt_unit, 1000);
    pcnt_filter_enable(m_pcnt_unit);

    /* Set threshold 0 and 1 values and enable events to watch */
    pcnt_set_event_value(m_pcnt_unit, PCNT_EVT_THRES_1, 1);
    pcnt_event_enable(m_pcnt_unit, PCNT_EVT_THRES_1);
    pcnt_set_event_value(m_pcnt_unit, PCNT_EVT_THRES_0, -5);
    pcnt_event_enable(m_pcnt_unit, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    //pcnt_event_enable(m_pcnt_unit, PCNT_EVT_ZERO);
    //pcnt_event_enable(m_pcnt_unit, PCNT_EVT_H_LIM);
    //pcnt_event_enable(m_pcnt_unit, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(m_pcnt_unit);
    pcnt_counter_clear(m_pcnt_unit);

    /* Install interrupt service and add isr callback handler */
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(m_pcnt_unit, pcnt_interrupt_handler, (void *)m_pcnt_unit);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(m_pcnt_unit);

#else
    // 2. Configure the PCNT unit
    pcnt_unit_config_t unit_config = {
        .low_limit = REEDCOUNTER_PCNT_LOW_LIMIT,
        .high_limit = REEDCOUNTER_PCNT_HIGH_LIMIT,
    };
    m_pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &m_pcnt_unit));

    // 3. Configure the input channel (connects GPIO to the PCNT unit)
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = REEDCOUNTER_GPIO,
        .level_gpio_num = -1, // No level signal used
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(m_pcnt_unit, &chan_config, &pcnt_chan));

    // 4. Set counting actions for rising and falling edges
    // Counts up on the positive edge, holds on the negative edge. 
    // Adjust based on your reed switch's "Normally Open (N.O.)" or "Normally Closed (N.C.)" logic.
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

    // 5. Setup Glitch Filter (essential for debouncing a physical reed switch)
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000, // Filter out glitches less than 1 microsecond
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(m_pcnt_unit, &filter_config));

    // 6. Register the Watchpoint/Threshold and enable interrupts
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(m_pcnt_unit, 1)); // Trigger interrupt on every 1st pulse
    
    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach_cb,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(m_pcnt_unit, &cbs, pcnt_event_queue));

    // 7. Enable and Start the PCNT Unit
    ESP_ERROR_CHECK(pcnt_unit_enable(m_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(m_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(m_pcnt_unit));
#endif
    xTaskCreate(checkreed, "reed_check", 4096, this, 10, &m_Handle);
#endif // RAK4631

    return true;
}

bool ReedCounterSensor::getMetrics(meshtastic_Telemetry *measurement)
{
    measurement->variant.environment_metrics.has_rainfall_1h = false;
    measurement->variant.environment_metrics.rainfall_1h = 0.0f;
    measurement->variant.environment_metrics.has_rainfall_24h = true;
    measurement->variant.environment_metrics.rainfall_24h = m_currentCount * 0.2f;
    return true;
}

#endif