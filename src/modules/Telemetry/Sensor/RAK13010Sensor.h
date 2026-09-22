#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && defined(RAK13010_SENSOR_EN)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "TelemetrySensor.h"
#include "concurrency/Lock.h"

#include <deque>
#include <map>

#include "RAK13010_SDI12.h"

class RAK13010Sensor : public TelemetrySensor
{
  public:
    RAK13010Sensor();
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;
    virtual bool initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev) override;

    // Gust/lull: average of the top/bottom WIND_GUSTLULL_FRACTION of samples within the
    // trailing WIND_GUSTLULL_WINDOW_MS. Returns false if no sample has landed in that
    // window yet (e.g. right after boot).
    bool getWindGust(float &out);
    bool getWindLull(float &out);

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
#if GILL_CONTINUOUS_AVG_POLAR
        uint16_t m_dirmax;
        float m_magmax;
#endif
        uint8_t m_status;
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

#if GILL_CONTINUOUS_AVG_POLAR
    bool ConfigGillXHPM(char i);
#endif
    bool CheckActive(char i);
    bool QuerySensorType(char i);
    void ScanAddressSpace();
    std::map<char,SDISensor*> m_sensors;

    // ---- Background WindSonic polling (GILL_CONTINUOUS_AVG_POLAR == 0 path: plain
    // instantaneous M!+D0! queries, polled as fast as the sensor allows, with gust/lull
    // computed here in firmware instead of relying on the sensor's own -- broken, on our
    // hardware -- R2! averaging mode). See src/modules/Telemetry/Sensor/RAK13010Sensor.cpp
    // for the full design rationale.

    struct WindSample {
        uint32_t timestampMs;
        float speed;
        uint16_t direction;
        uint8_t status;
    };

    static constexpr uint32_t WIND_BUFFER_WINDOW_MS = 300000;   // 300s FIFO retention
    static constexpr uint32_t WIND_GUSTLULL_WINDOW_MS = 60000;  // 60s gust/lull window
    static constexpr float WIND_GUSTLULL_FRACTION = 0.10f;      // top/bottom 10%, min 1 sample

    std::deque<WindSample> m_windBuffer; // time-pruned FIFO; single producer (background task)
    concurrency::Lock m_windLock;        // protects m_windBuffer
    concurrency::Lock m_sdiBusLock;      // protects m_SDI12 (shared between ReadData() and the
                                          // background wind task -- see RAK13010Sensor.cpp)
    TaskHandle_t m_windTaskHandle = nullptr;
    char m_windSonicAddress = 0; // SDI-12 address of the polled WindSonic; 0 = none found

    static void windPollTaskTrampoline(void *arg);
    bool ReadWindSonicOnce(char address, WindSample &out, uint32_t &timeoutMsUsed);
    void pushWindSample(const WindSample &s);
    bool computeGustLull(bool wantGust, float &out);
};

#endif