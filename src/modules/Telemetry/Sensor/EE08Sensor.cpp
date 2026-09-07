#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && defined(EE08_SENSOR_EN)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "EE08Sensor.h"
#include "TelemetrySensor.h"

#if !defined(RAK_4631)
  #define GPIO_OUTPUT_PIN_SEL  (1ULL<<EE08_SENSOR_SCL_PIN)
  #define GPIO_INPUT_PIN_SEL  (1ULL<<EE08_SENSOR_SDA_PIN)
#endif 
// Definitions
//-----------------------------------------------------------------------------
#define ACK 0 
#define NAK 1
#define RETRYS 3
#if !defined(RAK_4631)
  #define DELAY_FACTOR 300 // 600 matches approx 1kHz // 300 matches 2kHz // 150 matches 4kHz
#else
  #define DELAY_FACTOR 150
#endif
st_E2_Return knl_E2bus_readByteFromSlave(unsigned char ControlByte)
// read byte from slave with controlbyte
{
    unsigned char Checksum;
    unsigned char counter = 0;
    st_E2_Return E2_Return;
    E2_Return.Status = 1;
    while (E2_Return.Status && counter < RETRYS)
    // RETRYS...Number of read attempts
    {
        knl_E2bus_start();                // send E2 start condition
        knl_E2bus_sendByte(ControlByte);  // send 0xA1 (example for reading Temp_Low byte)
        if (knl_E2bus_check_ack() == ACK) // ACK received?
        {
            //printf("Ack received!\n");
            E2_Return.DataByte = knl_E2bus_readByte();
            // read Temp_low (example for reading Temp_Low byte)
            knl_E2bus_send_ack();            // send ACK
            Checksum = knl_E2bus_readByte(); // read checksum
            knl_E2bus_send_nak();            // send NACK
            //printf("ControlByte: %u, Checksum: %u\n",ControlByte, Checksum);
            if (((ControlByte + E2_Return.DataByte) % 0x100) == Checksum)
                // checksum OK?
                E2_Return.Status = 0;
        }
        knl_E2bus_stop(); // send E2 stop condition
        //printf("Counter: %d\n",counter);
        counter++;
    }
    return E2_Return;
}

st_E2_Return knl_E2bus_setInternalAddressPointer(uint8_t deviceAddress, unsigned char addressPtr)
// read byte from slave with controlbyte
{
    unsigned char Checksum;
    unsigned char counter = 0;
    st_E2_Return E2_Return;
    E2_Return.Status = 1;
    while (E2_Return.Status && counter < RETRYS)
    // RETRYS...Number of read attempts
    {
        knl_E2bus_start();                // send E2 start condition
        unsigned char ControlByte = 0x50 | (deviceAddress << 1);
        knl_E2bus_sendByte(ControlByte);  // send 0x50 for setting custom
        if (knl_E2bus_check_ack() == ACK) // ACK received?
        {
            unsigned char AddressByte = addressPtr;
            knl_E2bus_sendByte(AddressByte);
            if (knl_E2bus_check_ack() == ACK)
            {
                unsigned char DataByte = addressPtr;
                knl_E2bus_sendByte(DataByte);  
                if (knl_E2bus_check_ack() == ACK)
                {
                    Checksum = (ControlByte + AddressByte + DataByte) % 0x100;
                    knl_E2bus_sendByte(Checksum);  // checksum
                    if (knl_E2bus_check_ack() == ACK)
                    {
                        E2_Return.Status = 0;
                    }
                }
            }
        }
        knl_E2bus_stop(); // send E2 stop condition
        //printf("Counter: %d\n",counter);
        counter++;
    }
    return E2_Return;
}

st_E2_Return knl_E2bus_writeDataToAddressPointer(uint8_t deviceAddress, unsigned char addressPtr, unsigned char databyte)
// read byte from slave with controlbyte
{
    unsigned char Checksum;
    unsigned char counter = 0;
    st_E2_Return E2_Return;
    E2_Return.Status = 1;

    LOG_WARN("********************* Writing data to sensor! I hope you know what you are doing! *****************");

    while (E2_Return.Status && counter < RETRYS)
    // RETRYS...Number of read attempts
    {
        knl_E2bus_start();                // send E2 start condition
        unsigned char ControlByte = 0x10 | (deviceAddress << 1);
        knl_E2bus_sendByte(ControlByte);  // send 0x50 for setting custom
        if (knl_E2bus_check_ack() == ACK) // ACK received?
        {
            unsigned char AddressByte = addressPtr;
            knl_E2bus_sendByte(AddressByte);  
            if (knl_E2bus_check_ack() == ACK)
            {
                unsigned char DataByte = databyte;
                knl_E2bus_sendByte(DataByte);  
                if (knl_E2bus_check_ack() == ACK)
                {
                    Checksum = (ControlByte + AddressByte + DataByte) % 0x100;
                    knl_E2bus_sendByte(Checksum);  // checksum
                    if (knl_E2bus_check_ack() == ACK)
                    {
                        E2_Return.Status = 0;
                    }
                }
            }
        }
        knl_E2bus_stop(); // send E2 stop condition
        //printf("Counter: %d\n",counter);
        counter++;
    }
    return E2_Return;
}

void knl_E2bus_start(void) // send start condition to E2-Interface
{
    knl_E2bus_set_SDA();
    knl_E2bus_set_SCL();
    knl_E2bus_delay(30);
    knl_E2bus_clear_SDA();
    knl_E2bus_delay(30);
}
void knl_E2bus_stop(void) // send stop condition to E2-Interface
{
    knl_E2bus_clear_SCL();
    knl_E2bus_delay(20);
    knl_E2bus_clear_SDA();
    knl_E2bus_delay(20);
    knl_E2bus_set_SCL();
    knl_E2bus_delay(20);
    knl_E2bus_set_SDA();
}

void knl_E2bus_sendByte(unsigned char value) // send byte to E2-Interface
{
    unsigned char mask;
    for (mask = 0x80; mask > 0; mask >>= 1)
    {
        knl_E2bus_clear_SCL();
        knl_E2bus_delay(10);
        if ((value & mask) != 0)
        {
            knl_E2bus_set_SDA();
        }
        else
        {
            knl_E2bus_clear_SDA();
        }
        knl_E2bus_delay(20);
        knl_E2bus_set_SCL();
        knl_E2bus_delay(30);
        knl_E2bus_clear_SCL();
    }
    knl_E2bus_set_SDA();
}
unsigned char knl_E2bus_readByte(void) // read Byte from E2-Interface
{
#if defined(RAK_4631) && RAK_4631 == 1
    pinMode(EE08_SENSOR_SDA_PIN, INPUT);
#endif
    unsigned char data_in = 0x00;
    unsigned char mask = 0x80;
    for (mask = 0x80; mask > 0; mask >>= 1)
    {
        knl_E2bus_clear_SCL();
        knl_E2bus_delay(30);
        knl_E2bus_set_SCL();
        knl_E2bus_delay(15);
        if (knl_E2bus_read_SDA())
        {
            data_in |= mask;
        }
        knl_E2bus_delay(15);
        knl_E2bus_clear_SCL();
    }
#if defined(RAK_4631) && RAK_4631 == 1
    pinMode(EE08_SENSOR_SDA_PIN, OUTPUT_S0S1);
#endif    
    return data_in;
}
char knl_E2bus_check_ack(void) // check for acknowledge
{
#if defined(RAK_4631) && RAK_4631 == 1
    pinMode(EE08_SENSOR_SDA_PIN, INPUT);
#endif
    unsigned char input;
    knl_E2bus_clear_SCL();
    knl_E2bus_delay(30);
    knl_E2bus_set_SCL();
    knl_E2bus_delay(15);
    input = knl_E2bus_read_SDA();
    knl_E2bus_delay(15);
#if defined(RAK_4631) && RAK_4631 == 1
    pinMode(EE08_SENSOR_SDA_PIN, OUTPUT_S0S1);
#endif
    // SDA = LOW ==> ACK, SDA = HIGH ==> NAK
    return (input == NAK);
}
void knl_E2bus_send_ack(void) // send acknowledge
{
    knl_E2bus_clear_SCL();
    knl_E2bus_delay(15);
    knl_E2bus_clear_SDA();
    knl_E2bus_delay(15);
    knl_E2bus_set_SCL();
    knl_E2bus_delay(28);
    knl_E2bus_clear_SCL();
    knl_E2bus_delay(2);
    knl_E2bus_set_SDA();
}

void knl_E2bus_send_nak(void) // send NOT-acknowledge
{
    knl_E2bus_clear_SCL();
    knl_E2bus_delay(15);
    knl_E2bus_set_SDA();
    knl_E2bus_delay(15);
    knl_E2bus_set_SCL();
    knl_E2bus_delay(30);
    knl_E2bus_set_SCL();
}
void knl_E2bus_delay(unsigned int count) // knl_E2bus_delay function
{
    //vTaskDelay(count);
    //printf("%d\n",portTICK_PERIOD_MS);
    
    volatile unsigned int count2;
    count2 = count;
    count2 = count2 * DELAY_FACTOR;
    // adapt "DELAY_FACTOR" to match the target frequence for E2-Interface communiation
    while (--count2 != 0)
        ;
}

#if defined(RAK_4631) && RAK_4631 == 1
// adapt this code for your target processor !!! Value = 1 ==> Physical Signal is High, Value = 0 == > Physical Signal is Low 
void knl_E2bus_set_SDA(void)
{
    digitalWrite(EE08_SENSOR_SDA_PIN, 1); // set port-pin (SDA)
}
void knl_E2bus_clear_SDA(void)
{
    digitalWrite(EE08_SENSOR_SDA_PIN, 0); // clear port-pin (SDA)
}
unsigned char knl_E2bus_read_SDA(void)
{
    return digitalRead(EE08_SENSOR_SDA_PIN); // read SDA-pin status 
}
void knl_E2bus_set_SCL(void)
{
    digitalWrite(EE08_SENSOR_SCL_PIN, 1); // set port-pin (SCL)
}
void knl_E2bus_clear_SCL(void)
{
    digitalWrite(EE08_SENSOR_SCL_PIN, 0); // clear port-pin (SCL)
}

void knl_init()
{
    // IF RAK13010 IS NOT INITIALIZED WE NEED TO ENABLE THE 12V OUTPUT RAIL ON THE RAK13010 FOR 12V 
    // ON THE EE08 SENSOR!
#if !defined(RAK13010_SENSOR_EN)
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);  // Power the sensors.
#endif

    pinMode(EE08_SENSOR_SCL_PIN, OUTPUT_S0S1);
    pinMode(EE08_SENSOR_SDA_PIN, OUTPUT_S0S1);
    vTaskDelay(pdMS_TO_TICKS(2000));

    knl_E2bus_set_SDA();
    knl_E2bus_set_SCL();
}

#else // RAK4631

// adapt this code for your target processor !!! Value = 1 ==> Physical Signal is High, Value = 0 == > Physical Signal is Low 
void knl_E2bus_set_SDA(void)
{
    gpio_set_level((gpio_num_t)EE08_SENSOR_SDA_PIN, 1); // set port-pin (SDA)
}
void knl_E2bus_clear_SDA(void)
{
    gpio_set_level((gpio_num_t)EE08_SENSOR_SDA_PIN, 0); // clear port-pin (SDA)
}
unsigned char knl_E2bus_read_SDA(void)
{
    return gpio_get_level((gpio_num_t)EE08_SENSOR_SDA_PIN); // read SDA-pin status 
}
void knl_E2bus_set_SCL(void)
{
    gpio_set_level((gpio_num_t)EE08_SENSOR_SCL_PIN, 1); // set port-pin (SCL)
}
void knl_E2bus_clear_SCL(void)
{
    gpio_set_level((gpio_num_t)EE08_SENSOR_SCL_PIN, 0); // clear port-pin (SCL)
}

void knl_init()
{
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    //bit mask of the pins, use GPIO4 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    knl_E2bus_set_SDA();
    knl_E2bus_set_SCL();
}

#endif // RAK4631

void fl_init()
{
    knl_init();
}

float fl_E2bus_Read_Temp(uint8_t deviceAddress) // Read Measurement Value 2 (Temperature [°C])
{
    st_E2_Return E2_Return;
    float Temp;
    unsigned char Temp_LB, Temp_HB;
    Temp = -300;
    E2_Return = knl_E2bus_readByteFromSlave(CB_MV2LO | (deviceAddress << 1));
    Temp_LB = E2_Return.DataByte;
    if (E2_Return.Status == 0)
    {
        E2_Return = knl_E2bus_readByteFromSlave(CB_MV2HI | (deviceAddress << 1));
        Temp_HB = E2_Return.DataByte;
        if (E2_Return.Status == 0)
        {
            Temp = (Temp_LB + (float)(Temp_HB) * 256) / 100 - 273.15;
        }
    }
    return Temp;
}

float fl_E2bus_Read_RH(uint8_t deviceAddress) // Read Measurement Value 1 (relative Humidity [%RH])
{
    st_E2_Return E2_Return;
    float RH;
    unsigned char RH_LB, RH_HB;
    RH = -1;
    E2_Return = knl_E2bus_readByteFromSlave(CB_MV1LO | (deviceAddress << 1));
    RH_LB = E2_Return.DataByte;
    //printf("  %u\n",RH_LB);
    if (E2_Return.Status == 0)
    {
        E2_Return = knl_E2bus_readByteFromSlave(CB_MV1HI | (deviceAddress << 1));
        RH_HB = E2_Return.DataByte;
        //printf("  %u\n",RH_HB);
        if (E2_Return.Status == 0)
        {
            RH = (RH_LB + (float)(RH_HB) * 256) / 100;
        }
        else
            LOG_ERROR("===================== Status high byte failed!\n");
    }
    else
    {
        LOG_ERROR("===================== Status low byte failed!\n");
    }
    return RH;
}

float fl_E2bus_Read_CO2_RAW(uint8_t deviceAddress) // Read Measurement Value 3 (CO2 RAW [ppm])
{
    st_E2_Return E2_Return;
    float CO2_RAW;
    unsigned char CO2_LB, CO2_HB;
    CO2_RAW = -1;
    E2_Return = knl_E2bus_readByteFromSlave(CB_MV3LO | (deviceAddress << 1));
    CO2_LB = E2_Return.DataByte;
    if (E2_Return.Status == 0)
    {
        E2_Return = knl_E2bus_readByteFromSlave(CB_MV3HI | (deviceAddress << 1));
        CO2_HB = E2_Return.DataByte;
        if (E2_Return.Status == 0)
        {
            CO2_RAW = CO2_LB + (float)(CO2_HB) * 256;
        }
    }
    return CO2_RAW;
}

float fl_E2bus_Read_CO2_MEAN(uint8_t deviceAddress) // Read Measurement Value 4 (CO2 MEAN [ppm])
{
    st_E2_Return E2_Return;
    float CO2_MEAN;
    unsigned char CO2_LB, CO2_HB;
    CO2_MEAN = -1;
    E2_Return = knl_E2bus_readByteFromSlave(CB_MV4LO | (deviceAddress << 1));
    CO2_LB = E2_Return.DataByte;
    if (E2_Return.Status == 0)
    {
        E2_Return = knl_E2bus_readByteFromSlave(CB_MV4HI | (deviceAddress << 1));
        CO2_HB = E2_Return.DataByte;
        if (E2_Return.Status == 0)
        {
            CO2_MEAN = CO2_LB + (float)(CO2_HB) * 256;
        }
    }
    return CO2_MEAN;
}

unsigned char fl_E2bus_Read_Status(uint8_t deviceAddress) // read Statusbyte from E2-Interface
{
    st_E2_Return E2_Return;
    E2_Return = knl_E2bus_readByteFromSlave(CB_STATUS | (deviceAddress << 1));
    if (E2_Return.Status == 1)
    {
        E2_Return.DataByte = 0xFF;
    }
    return E2_Return.DataByte;
}

unsigned int fl_E2bus_Read_SensorType(uint8_t deviceAddress) // read Sensortype from E2-Interface
{
    st_E2_Return E2_Return;
    unsigned int Type;
    unsigned char Type_LB, Type_HB;
    Type = 0xFFFF;
    E2_Return = knl_E2bus_readByteFromSlave(CB_TYPELO | (deviceAddress << 1));
    Type_LB = E2_Return.DataByte;
    if (E2_Return.Status == 0)
    {
        E2_Return = knl_E2bus_readByteFromSlave(CB_TYPEHI | (deviceAddress << 1));
        Type_HB = E2_Return.DataByte;
        if (E2_Return.Status == 0)
        {
            Type = Type_LB + (unsigned int)(Type_HB) * 256;
        }
    }
    return Type;
}

unsigned char fl_E2bus_Read_CustomAddress(uint8_t deviceAddress, unsigned char addressPtr, bool setPtrBeforeRead)
{
    st_E2_Return E2_Return; 
    // if we do not set it, we assume we do autoincrement in multiple readings
    if(setPtrBeforeRead)
        E2_Return = knl_E2bus_setInternalAddressPointer(deviceAddress, addressPtr);
    else
        E2_Return.Status = 0;

    if (E2_Return.Status == 0) // went ok!
    {
        E2_Return = knl_E2bus_readByteFromSlave(0x51 | (deviceAddress << 1));
        if (E2_Return.Status == 1)
        {
            E2_Return.DataByte = 0xFF;
        }
        return E2_Return.DataByte;
    }
    E2_Return.DataByte = 0xFF;
    return E2_Return.DataByte;
}

unsigned char fl_E2bus_Write_CustomAddress(uint8_t deviceAddress, unsigned char addressPtr, unsigned char dataByte)
{
    st_E2_Return E2_Return;
    E2_Return = knl_E2bus_writeDataToAddressPointer(deviceAddress, addressPtr, dataByte);
    if (E2_Return.Status == 1)
    {
        E2_Return.DataByte = 0xFF;
    }
    return E2_Return.DataByte;
}


unsigned char fl_E2bus_Read_SensorSubType(uint8_t deviceAddress) // read Sensor Subtype from E2-Interface
{
    st_E2_Return E2_Return;
    E2_Return = knl_E2bus_readByteFromSlave(CB_TYPESUB | (deviceAddress << 1));
    if (E2_Return.Status == 1)
    {
        E2_Return.DataByte = 0xFF;
    }
    return E2_Return.DataByte;
}

unsigned char fl_E2bus_Read_AvailablePhysicalMeasurements(uint8_t deviceAddress)
// read available physical Measurements from E2-Interface
{
    st_E2_Return E2_Return;
    E2_Return = knl_E2bus_readByteFromSlave(CB_AVPHMES | (deviceAddress << 1));
    if (E2_Return.Status == 1)
    {
        E2_Return.DataByte = 0xFF;
    }
    return E2_Return.DataByte;
}

EE08Sensor::EE08Sensor() : TelemetrySensor(meshtastic_TelemetrySensorType_SENSOR_UNSET, "EE08") {}

bool EE08Sensor::initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev)
{
    LOG_DEBUG("===================== Init sensor: %s", sensorName);

    fl_init(); 

    uint8_t deviceAddress = 0;
    for(; deviceAddress < 8; deviceAddress++)
    {
        vTaskDelay(pdMS_TO_TICKS(200));
        unsigned int SensorType = fl_E2bus_Read_SensorType(deviceAddress); // read Sensortype from E2-Interface

        // seems that we found one
        if(SensorType < 65535)
        {
            String type;
            type += char(64 + ((SensorType >> 12) & 0x0F));
            type += char(64 + ((SensorType >> 8) & 0x0F));
            type += char(48 + ((SensorType >> 4) & 0x0F));
            type += char(48 + (SensorType & 0x0F));
            LOG_DEBUG("===================== Sensortype %s found on address %u... ",type.c_str(), deviceAddress);

            // read Sensor Subtype from E2-Interface
            unsigned char SensorSubType = fl_E2bus_Read_SensorSubType(deviceAddress);
            LOG_DEBUG("===================== SensorSubtype: %u",SensorSubType);

                        // needs setting internal address pointer
            unsigned char FirmwareVersion = fl_E2bus_Read_CustomAddress(deviceAddress, 0x00);
            LOG_DEBUG("===================== FirmwareVersion: %u",FirmwareVersion);

            unsigned char FirmwareSubVersion = fl_E2bus_Read_CustomAddress(deviceAddress, 0x01, false); // don't need to set counter due to auto-increment
            LOG_DEBUG("===================== FirmwareSubVersion: %u",FirmwareSubVersion);

            unsigned char E2Spec = fl_E2bus_Read_CustomAddress(deviceAddress, 0x02, false); // don't need to set counter due to auto-increment
            LOG_DEBUG("===================== E2Spec: %u",E2Spec);

            // read available physical Measurements from
            unsigned char AvPhMes = fl_E2bus_Read_AvailablePhysicalMeasurements(deviceAddress);
            LOG_DEBUG("===================== Available: %d",AvPhMes);

            unsigned char addr = fl_E2bus_Read_CustomAddress(deviceAddress, 0xC0);
            LOG_DEBUG("===================== Bus Address: %u",addr);

            unsigned char SupportedFunctions = fl_E2bus_Read_CustomAddress(deviceAddress, 0x07);
            LOG_DEBUG("===================== SupportedFunctions: %u",SupportedFunctions);
            if (SupportedFunctions < 255)
            {
                
                vTaskDelay(pdMS_TO_TICKS(50));

                // querying serial is supported...
                if (SupportedFunctions & 0x01) {
                    int j = 0;
                    String serial;
                    for (unsigned char i = 0xA0; i < 0xAF; i++) {
                        unsigned char dec = fl_E2bus_Read_CustomAddress(deviceAddress, i, j==0);
                        serial += char(dec); j++; 
                        vTaskDelay(pdMS_TO_TICKS(50));  
                    }
                    LOG_DEBUG("===================== Serial: %s",serial.c_str());
                }

                if(SupportedFunctions & 0x02) {
                    LOG_DEBUG("===================== Address Change should be actually supported - change to 0x02");
                    unsigned char AddressChanged = fl_E2bus_Write_CustomAddress(deviceAddress, 0xC0, 0x02); // change address to 2
                    LOG_DEBUG("===================== Address Change: %d", AddressChanged);
                }
            }

            unsigned char Status = fl_E2bus_Read_Status(deviceAddress);
            LOG_DEBUG("===================== Status: %d",Status);

            m_sensors[deviceAddress] = new EE08();        
        }
    }
    return m_sensors.size() > 0;
}

float EE08Sensor::getHumidity(uint8_t deviceAddress)
{
    float humidity = fl_E2bus_Read_RH(deviceAddress); // Read Measurement Value 1 (rel.v Humidity [%RH])
    LOG_DEBUG("===================== Humidity: %.3f",humidity);
    return humidity;
}

float EE08Sensor::getTemp(uint8_t deviceAddress)
{
    float temperature = fl_E2bus_Read_Temp(deviceAddress); // Read Measurement Value 2 (Temperature [°C])
    LOG_DEBUG("===================== Temperature: %.3f",temperature);
    return temperature;
}

bool EE08Sensor::getMetrics(meshtastic_Telemetry *measurement)
{   
    vTaskDelay(pdMS_TO_TICKS(2000));

    // read all sensors
    for(auto it : m_sensors)
    {
        LOG_DEBUG("===================== Reading sensor: %u",it.first);
        LOG_DEBUG("===================== Status: %u", fl_E2bus_Read_Status(it.first));

        vTaskDelay(pdMS_TO_TICKS(200));

        it.second->m_temperature = getTemp(it.first);
        it.second->m_humidity = getHumidity(it.first);
        it.second->m_readOK = it.second->m_temperature > -299 && it.second->m_humidity > -1;

        if(it.second->m_readOK)
        {
            // 01 is already occupied, so use another definition
            if(measurement->variant.environment_metrics.has_ee08_temperature_01)
            {
                // 02 is already occupied, so use another definition
                if(measurement->variant.environment_metrics.has_ee08_temperature_02)
                {
                    // 03 is already occupied, so use another definition
                    if(measurement->variant.environment_metrics.has_ee08_temperature_03)
                    {
                        measurement->variant.environment_metrics.has_ee08_temperature_04 = true;
                        measurement->variant.environment_metrics.has_ee08_relative_humidity_04 = true;
                        measurement->variant.environment_metrics.ee08_temperature_04 = it.second->m_temperature;
                        measurement->variant.environment_metrics.ee08_relative_humidity_04 = it.second->m_humidity;  
                    }
                    else
                    {
                        measurement->variant.environment_metrics.has_ee08_temperature_03 = true;
                        measurement->variant.environment_metrics.has_ee08_relative_humidity_03 = true;
                        measurement->variant.environment_metrics.ee08_temperature_03 = it.second->m_temperature;
                        measurement->variant.environment_metrics.ee08_relative_humidity_03 = it.second->m_humidity;                
                    }
                }
                else
                {
                measurement->variant.environment_metrics.has_ee08_temperature_02 = true;
                measurement->variant.environment_metrics.has_ee08_relative_humidity_02 = true;
                measurement->variant.environment_metrics.ee08_temperature_02 = it.second->m_temperature;
                measurement->variant.environment_metrics.ee08_relative_humidity_02 = it.second->m_humidity;                
                }
            }
            else
            {
                measurement->variant.environment_metrics.has_ee08_temperature_01 = true;
                measurement->variant.environment_metrics.has_ee08_relative_humidity_01 = true;
                measurement->variant.environment_metrics.ee08_temperature_01 = it.second->m_temperature;
                measurement->variant.environment_metrics.ee08_relative_humidity_01 = it.second->m_humidity;
            }
        }
    }
    return m_sensors.size() > 0;
}

#endif