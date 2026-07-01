#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "TelemetrySensor.h"

#include "RAK13010_SDI12.h"

class RAK13010Sensor : public TelemetrySensor
{
  public:
    RAK13010Sensor();
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;
    virtual bool initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev) override;
  private:
    RAK_SDI12* m_SDI12;
    bool ReadData();

    String m_sensorID;
    typedef struct {
      float m_soil_moisture_F;
      float m_bulk_ec_corrected_I;
      float m_temperature_G;
      float m_temperature_H;
      float m_bulk_ec_J;
      float m_real_dielectric_permittivity_L;
      float m_imaginary_dielectric_permittivity_M;
      float m_pore_water_ec_K;
      float m_imaginary_dielectric_permittivity_N;
      float m_dielectric_loss_tangent_O;
      float m_diode_temperature_P;
      time_t m_timestamp;
    } Stevens;
    Stevens m_sensorreadings;
};

#endif