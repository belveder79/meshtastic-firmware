#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && defined(REEDCOUNTER_SENSOR_EN)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "ReedCounterSensor.h"
#include "TelemetrySensor.h"

static QueueHandle_t pcnt_event_queue;

ReedCounterSensor::ReedCounterSensor() : TelemetrySensor(meshtastic_TelemetrySensorType_SENSOR_UNSET, "ReedCounter") 
{
    m_currentCount = 0;
}

// Interrupt callback function triggered when the pulse count threshold is reached
static bool pcnt_on_reach_cb(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) 
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    
    // Send the current count/event to the FreeRTOS queue
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    
    return (high_task_wakeup == pdTRUE);
}

/* Contact detection task */
static void checkreed(void *arg)
{
    LOG_INFO("===================== Contact sensor monitoring started");
    ReedCounterSensor* instance = static_cast<ReedCounterSensor*>(arg);
    int count_val;
    while (1) {
        if (xQueueReceive(pcnt_event_queue, &count_val, portMAX_DELAY)) {
            LOG_INFO("Reed Switch Triggered! Event threshold reached.");
            instance->increaseCounterAndResetHandle();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  /* Check every 1000ms */
    }
}

void ReedCounterSensor::increaseCounterAndResetHandle()
{
    m_currentCount++;
    // Clear the counter so we wait for the next pulse event
    pcnt_unit_clear_count(m_pcnt_unit);
}

bool ReedCounterSensor::initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev)
{
    LOG_INFO("===================== Init sensor: %s", sensorName);

    // 1. Create a Queue to pass events from ISR to the main task
    pcnt_event_queue = xQueueCreate(10, sizeof(int));
    if (pcnt_event_queue == NULL) {
        LOG_ERROR("===================== Queue creation failed");
        return false;
    }

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

    LOG_INFO("===================== Monitoring Reed Switch...");
    xTaskCreate(checkreed, "reed_check", 4096, this, 10, &m_Handle);
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