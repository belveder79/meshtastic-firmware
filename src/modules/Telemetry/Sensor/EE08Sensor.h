#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "TelemetrySensor.h"

#include <map>

typedef struct st_E2_Return
{
    unsigned char DataByte;
    unsigned char Status;
} st_E2_Return;

// declaration of functions
//-----------------------------------------------------------------------------
st_E2_Return knl_E2bus_readByteFromSlave(unsigned char ControlByte);
st_E2_Return knl_E2bus_setInternalAddressPointer(uint8_t deviceAddress, unsigned char addressPtr);
st_E2_Return knl_E2bus_writeDataToAddressPointer(uint8_t deviceAddress, unsigned char addressPtr, unsigned char databyte);
void knl_E2bus_start(void);
void knl_E2bus_stop(void);
void knl_E2bus_sendByte(unsigned char);
unsigned char knl_E2bus_readByte(void);
void knl_E2bus_delay(unsigned int value);
char knl_E2bus_check_ack(void);
void knl_E2bus_send_ack(void);
void knl_E2bus_send_nak(void);
void knl_E2bus_set_SDA(void);
void knl_E2bus_clear_SDA(void);
unsigned char knl_E2bus_read_SDA(void);
void knl_E2bus_set_SCL(void);
void knl_E2bus_clear_SCL(void);
void knl_init();

// constant definition
//-----------------------------------------------------------------------------
#define CB_TYPELO 0x11  // ControlByte for reading Sensortype Low-Byte
#define CB_TYPESUB 0x21 // ControlByte for reading Sensor-Subtype
#define CB_AVPHMES 0x31 // ControlByte for reading Available physical measurements
#define CB_TYPEHI 0x41  // ControlByte for reading Sensortype High-Byte
#define CB_STATUS 0x71  // ControlByte for reading Statusbyte
#define CB_MV1LO 0x81   // ControlByte for reading Measurement value 1 Low-Byte
#define CB_MV1HI 0x91   // ControlByte for reading Measurement value 1 High-Byte
#define CB_MV2LO 0xA1   // ControlByte for reading Measurement value 2 Low-Byte
#define CB_MV2HI 0xB1   // ControlByte for reading Measurement value 2 High-Byte
#define CB_MV3LO 0xC1   // ControlByte for reading Measurement value 3 Low-Byte
#define CB_MV3HI 0xD1   // ControlByte for reading Measurement value 3 High-Byte
#define CB_MV4LO 0xE1   // ControlByte for reading Measurement value 4 Low-Byte
#define CB_MV4HI 0xF1   // ControlByte for reading Measurement value 4 High-Byte
// #define E2_DEVICE_ADR 0 // Address of E2-Slave-Device
// declaration of functions
//-----------------------------------------------------------------------------
unsigned int fl_E2bus_Read_SensorType(uint8_t deviceAddress);     // read Sensortype from E2-Interface
unsigned char fl_E2bus_Read_SensorSubType(uint8_t deviceAddress); // read Sensor Subtype from E2-Interface
unsigned char fl_E2bus_Read_AvailablePhysicalMeasurements(uint8_t deviceAddress);
// read available physical Measurements from E2-Interface
float fl_E2bus_Read_RH(uint8_t deviceAddress);             // Read Measurement Value 1 (relativ Humidity [%RH])
float fl_E2bus_Read_Temp(uint8_t deviceAddress);           // Read Measurement Value 2 (Temperature [°C])
float fl_E2bus_Read_CO2_RAW(uint8_t deviceAddress);        // Read Measurement Value 3 (CO2 RAW [ppm])
float fl_E2bus_Read_CO2_MEAN(uint8_t deviceAddress);       // Read Measurement Value 4 (CO2 MEAN [ppm])
unsigned char fl_E2bus_Read_Status(uint8_t deviceAddress); // read Statusbyte from E2-Interface
unsigned char fl_E2bus_Read_CustomAddress(uint8_t deviceAddress, unsigned char addressPtr, bool setPtrBeforeRead = true); // read custom address
unsigned char fl_E2bus_Write_CustomAddress(uint8_t deviceAddress, unsigned char addressPtr, unsigned char dataByte); // write custom address


void fl_init();

class EE08Sensor : public TelemetrySensor
{
  public:
    EE08Sensor();
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;
    virtual bool initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev) override;
    virtual float getHumidity(uint8_t deviceAddress);
    virtual float getTemp(uint8_t deviceAddress);

  private:
    class EE08 {
      public:
        EE08() : m_readOK(false) {}
        bool m_readOK;
        float m_humidity;
        float m_temperature;
      protected:
        virtual ~EE08() {};
    };
    std::map<uint8_t,EE08*> m_sensors;
};

#endif