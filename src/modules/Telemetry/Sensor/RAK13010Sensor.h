#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && defined(RAK13010_SENSOR_EN)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "TelemetrySensor.h"

#include <map>

#include "RAK13010_SDI12.h"

class RAK13010Sensor : public TelemetrySensor
{
  public:
    RAK13010Sensor();
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;
    virtual bool initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev) override;
  private:

   typedef enum {
      STEVENS = 0,
      WINDSONIC = 1,
      UNKNOWN = 9
    } SDISensorType;

    class SDISensor {
      public:
        SDISensor() : m_readOK(false) , m_type(SDISensorType::UNKNOWN) {}
        const SDISensorType getType() const { return m_type; }
        bool m_readOK;
      protected:
        SDISensor(SDISensorType type) : m_type(type) {}
        virtual ~SDISensor() {};
        SDISensorType m_type;
    };
    class WindSonic : public SDISensor {
      public:
        WindSonic() : SDISensor(SDISensorType::WINDSONIC) {}
        virtual ~WindSonic() {};
        uint16_t m_direction;
        float m_magnitude;
        float m_status;
    };
    class Stevens : public SDISensor {
      public:
        Stevens() : SDISensor(SDISensorType::STEVENS) {};
        virtual ~Stevens() {};
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
    };

    RAK_SDI12* m_SDI12;
    bool ReadData();

    bool CheckActive(char i);
    bool QuerySensorType(char i);
    void ScanAddressSpace();
    std::map<char,SDISensor*> m_sensors;

};

#endif