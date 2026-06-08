#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "TelemetrySensor.h"

#ifdef OLDPCNT
  #include "driver/pcnt.h"
#else
  #include " driver/pulse_cnt.h"
#endif

class ReedCounterSensor : public TelemetrySensor
{
  public:
    ReedCounterSensor();
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;
    virtual bool initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev) override;
    void increaseCounterAndResetHandle();
  private:
    int32_t m_currentCount;
    TaskHandle_t m_Handle;
#ifdef OLDPCNT    
    pcnt_unit_t m_pcnt_unit;
#else
    pcnt_unit_handle_t m_pcnt_unit;
#endif
};

#endif