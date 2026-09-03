#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && defined(RAK13010_SENSOR_EN)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "RAK13010Sensor.h"
#include "TelemetrySensor.h"

// example largely taken from
// https://github.com/RAKWireless/WisBlock/tree/master/examples/common/IO/RAK13010_SDI_12_BUS

//#define WB_IO2 (34)
//#define WB_IO5 (9)	   // SLOT_D

#define TX_PIN    WB_IO6   // The pin of the SDI-12 data bus.
#define RX_PIN    WB_IO5   // The pin of the SDI-12 data bus.
#define OE        WB_IO4   // Output enable pin, active low.

RAK13010Sensor::RAK13010Sensor() : TelemetrySensor(meshtastic_TelemetrySensorType_SENSOR_UNSET, "RAK13010") {}

bool RAK13010Sensor::CheckActive(char i) 
{
  String myCommand = "";
  myCommand        = "";
  myCommand += (char)i;  // Sends basic 'acknowledge' command [address][!].
  myCommand += "!";

  bool sdiMsgReady = false;
  for (int j = 0; j < 3; j++) // 3 tries
  {
    m_SDI12->sendCommand(myCommand);
    m_SDI12->clearBuffer();
    vTaskDelay(pdMS_TO_TICKS(30));
    if (m_SDI12->available()) 
      return true;
  }
  m_SDI12->clearBuffer();
  return false;
}

bool RAK13010Sensor::QuerySensorType(char i)
{
  uint8_t serialMsgRflag = 1;
  boolean sdiMsgReady = false;
  String sdiMsgStr = "";
  
  bool knownSensor = false; // indicate that sensor is known to us 
  int timeoutCnt = 1000;
  // loop emulation
  while(serialMsgRflag && timeoutCnt-- > 0)
  {
    int avail = m_SDI12->available();
    if (avail < 0) 
    {
      m_SDI12->clearBuffer();  // Buffer is full,clear.
    }  
    else if (avail > 0)  
    {
      for (int a = 0; a < avail; a++) 
      {
        char inByte2 = m_SDI12->read();
        if (inByte2 == '\n') 
        {
          sdiMsgReady = true;
        } 
        else 
        {
          sdiMsgStr += String(inByte2);
        }
      }
    }

    if (sdiMsgReady)
    {
      LOG_DEBUG("<<<< %s", sdiMsgStr.c_str());
      String manufacturer = sdiMsgStr.substring(4,12);
      String manufacturerOld = sdiMsgStr.substring(3,11);
      if(manufacturer.compareTo("STEVENSW") == 0 || manufacturerOld.compareTo("STEVENSW") == 0)
      {
        LOG_DEBUG("==== Type is Stevens Waters HydraProbe!");
        m_sensors[i] = new Stevens();
        knownSensor = true;
      }
      else if(manufacturer.compareTo("GillInst") == 0)
      { 
        LOG_DEBUG("==== Type is Gill Windsonic!");
        m_sensors[i] = new WindSonic();
        knownSensor = true;
      }
      else
      {
        LOG_DEBUG("==== Unknown Type found: %s", manufacturer.c_str());
        // TODO:
        // check other ones and add appropriately
      }

      sdiMsgReady = false;  // Reset String for next SDI-12 message.
      sdiMsgStr   = "";
      serialMsgRflag = 0; 
    }

    if (serialMsgRflag)
    {
      if(serialMsgRflag == 1)
      {
        serialMsgRflag = 2;
        String cmd(String(i) + "I!");
        m_SDI12->sendCommand(cmd);
        LOG_DEBUG((String(">>>> ") + cmd).c_str());
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return (timeoutCnt > 0) && knownSensor;
}

void RAK13010Sensor::ScanAddressSpace() 
{
  LOG_DEBUG("==== Scanning Address space...");
  for (char i = '0'; i <= '9'; i++) // Scan address space 0-9.
  {
    if (CheckActive(i)) 
    { 
      LOG_DEBUG("==== Sensor found on address %c",i);
      if(QuerySensorType(i))
        LOG_DEBUG("==== Sensor added on address %c",i);
    }
  }
  
  for (char i = 'a'; i <= 'z'; i++) // Scan address space a-z.
  {
    if (CheckActive(i))
    { 
      LOG_DEBUG("==== Sensor found on address %c",i);
      if(QuerySensorType(i))
        LOG_DEBUG("==== Sensor added on address %c",i);
    }
  }

  for (char i = 'A'; i <= 'Z'; i++) // Scan address space A-Z.
  {
    if (CheckActive(i)) 
    { 
      LOG_DEBUG("==== Sensor found on address %c",i);
      if(QuerySensorType(i))
        LOG_DEBUG("==== Sensor added on address %c",i);
    }
  }
  LOG_DEBUG("==== Sensor scan complete!");
}

bool RAK13010Sensor::initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev)
{
  LOG_DEBUG("Init sensor: %s", sensorName);

  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);  // Power the sensors.

  vTaskDelay(pdMS_TO_TICKS(1000));

  LOG_DEBUG("==== Opening SDI-12 bus.");
  m_SDI12 = new RAK_SDI12(RX_PIN,TX_PIN,OE);

  m_SDI12->begin();  // Initiate serial connection to SDI-12 bus.
  LOG_DEBUG("==== Starting communication...");
  vTaskDelay(pdMS_TO_TICKS(1000));

  m_SDI12->forceListen();

  // Start Sweep of SDI-bus and add sensors found
  ScanAddressSpace();

  // indicate that we are good if the # of sensors is > 0
  return m_sensors.size() > 0;
}

int getIndices(const String sdiMsgStr, const int expected, int &idx0, int &idx1, int &idx2, int &idx3)
{
        int idx0p = sdiMsgStr.indexOf("+",0); idx0p = idx0p < 0 ? sdiMsgStr.length() : idx0p;     
        int idx0m = sdiMsgStr.indexOf("-",0); idx0m = idx0m < 0 ? sdiMsgStr.length() : idx0m;
        idx0 = idx0m < idx0p ? idx0m : idx0p;
        int idx1p = sdiMsgStr.indexOf("+",idx0+1); idx1p = idx1p < 0 ? sdiMsgStr.length() : idx1p;   
        int idx1m = sdiMsgStr.indexOf("-",idx0+1); idx1m = idx1m < 0 ? sdiMsgStr.length() : idx1m;
        idx1 = idx1m < idx1p ? idx1m : idx1p;
        if(expected > 2)
        {
          int idx2p = sdiMsgStr.indexOf("+",idx1+1); idx2p = idx2p < 0 ? sdiMsgStr.length() : idx2p;   
          int idx2m = sdiMsgStr.indexOf("-",idx1+1); idx2m = idx2m < 0 ? sdiMsgStr.length() : idx2m;
          idx2 = idx2m < idx2p ? idx2m : idx2p;
          idx3 = sdiMsgStr.indexOf("#",idx2+1);
        }
        else
        {
          idx2 = sdiMsgStr.indexOf("#",idx1+1);
          idx3 = sdiMsgStr.length();
        }
        return expected;
}

bool RAK13010Sensor::ReadData() 
{
  // flag that reading any sensor was ok
  bool anyReadOk = false;

  // run through all registered sensors
  for(auto it : m_sensors)
  {
    // get bus address from map
    char sdiSensorAddress = it.first;
    // get pointer to sensor
    SDISensor* sdiSensor = it.second;
    // get type of sensor
    SDISensorType sdiSensorType = sdiSensor->getType();

    // RUN ENTIRE DATA QUERY 

    uint8_t serialMsgRflag = 3;
    boolean sdiMsgReady = false;
    String sdiMsgStr = "";

    int measurementTimeout0 = 1000; // set to 1000 ms per default
    int measurementTimeout1 = 1000; 
    
    int timeoutCnt = 10000;
    // loop emulation
    while(serialMsgRflag && timeoutCnt-- > 0)
    {
      int avail = m_SDI12->available();
      if (avail < 0) 
      {
        m_SDI12->clearBuffer();  // Buffer is full,clear.
      }  
      else if (avail > 0)  
      {
        for (int a = 0; a < avail; a++) 
        {
          char inByte2 = m_SDI12->read();
          if (inByte2 == '\n') 
          {
            sdiMsgReady = true;
          } 
          else 
          {
            sdiMsgStr += String(inByte2);
          }
        }
      }

      if (sdiMsgReady)
      {
        LOG_DEBUG("<<<< %s", sdiMsgStr.c_str());
        
        // DISASSEMBLE STRING HERE!!!      
        if(serialMsgRflag == 4)
        {
          // Stevens:
          // result of M! should give 90029, which says 9 (address) 002 (2 seconds until measurement ready) 9 (9 fields in D0, D1 and D2)
          // this means that the timeout should be 2 seconds until requesting!
          // GillInst:
          // result of M! should give a0053, which says a (address) 005 (5 seconds until measurement ready) 3 (3 fields in D0)
          // this means that the timeout should be 5 seconds until requesting!
          if(sdiMsgStr.length() >= 5)
          {
            int ts = atoi(sdiMsgStr.substring(1,4).c_str());
            LOG_DEBUG("Timeout-1 value is %d seconds (%s)",ts,sdiMsgStr.substring(1,4).c_str());
            measurementTimeout0 = 1000 * ts;
          }
          serialMsgRflag = 5;
        }
        if(serialMsgRflag == 6)
        {
          int idx0, idx1, idx2, idx3;
          int fields = getIndices(sdiMsgStr, 3, idx0, idx1, idx2, idx3);
          switch(sdiSensorType)
          {
            case SDISensorType::STEVENS:
            {
              // result of D0! should give something like 
              // 9+0.000+0.001+25.0#
              // F – Soil Moisture
              // I – Bulk EC (Temp Corrected)
              // G – Temperature (C)
              Stevens* sensorreadings = reinterpret_cast<Stevens*>(sdiSensor);
              sensorreadings->m_soil_moisture_F = atof(sdiMsgStr.substring(idx0+1,idx1).c_str());
              sensorreadings->m_bulk_ec_corrected_I = atof(sdiMsgStr.substring(idx1+1,idx2).c_str());
              sensorreadings->m_temperature_G = atof(sdiMsgStr.substring(idx2+1,idx3).c_str());
              LOG_DEBUG("F: %f - I: %f - G: %f", sensorreadings->m_soil_moisture_F, sensorreadings->m_bulk_ec_corrected_I, sensorreadings->m_temperature_G);

              serialMsgRflag = 7; // continue with other readings
              break;
            }
            case SDISensorType::WINDSONIC:
            {
              // result of D0! should give something like 
              // a+083+000.02+00#
              // direction
              // magnitude
              // status
              WindSonic* sensorreadings = reinterpret_cast<WindSonic*>(sdiSensor);
              sensorreadings->m_direction = atoi(sdiMsgStr.substring(idx0+1,idx1).c_str()); // cast to uint16
              sensorreadings->m_magnitude = atof(sdiMsgStr.substring(idx1+1,idx2).c_str());
              sensorreadings->m_status = atof(sdiMsgStr.substring(idx2+1,idx3).c_str());
              LOG_DEBUG("dir: %f - mag: %f - status: %f", sensorreadings->m_direction, sensorreadings->m_magnitude, sensorreadings->m_status);
              
              serialMsgRflag = 0; // exit flag
              break;
            }
            default:
              ;;
          }
        }
        if(serialMsgRflag == 8)
        {
          int idx0, idx1, idx2, idx3;
          int fields = getIndices(sdiMsgStr, 3, idx0, idx1, idx2, idx3);          
          switch(sdiSensorType)
          {
            case SDISensorType::STEVENS:
            {
              // result of D1
              // 9+77.1+0.001+1.701#
              // H – Temperature (F) 
              // J – Bulk EC 
              // L – Real Dielectric Permittivity
              Stevens* sensorreadings = reinterpret_cast<Stevens*>(sdiSensor);
              sensorreadings->m_temperature_H = atof(sdiMsgStr.substring(idx0+1,idx1).c_str());
              sensorreadings->m_bulk_ec_J = atof(sdiMsgStr.substring(idx1+1,idx2).c_str());
              sensorreadings->m_real_dielectric_permittivity_L = atof(sdiMsgStr.substring(idx2+1,idx3).c_str());
              LOG_DEBUG("H: %f - J: %f - L: %f", sensorreadings->m_temperature_H, sensorreadings->m_bulk_ec_J, sensorreadings->m_real_dielectric_permittivity_L);
              break;
            }
            default:
              ;;
          }
          serialMsgRflag = 9;
        }    
        if(serialMsgRflag == 10) {
          int idx0, idx1, idx2, idx3;
          int fields = getIndices(sdiMsgStr, 3, idx0, idx1, idx2, idx3);
          switch(sdiSensorType)
          {
            case SDISensorType::STEVENS:
            {
              // result of D2
              // 9+0.295-0.038+0.173#
              // M – Imaginary Dielectric Permittivity
              // K – Pore Water EC
              // O – Dielectric Loss Tangent  
              Stevens* sensorreadings = reinterpret_cast<Stevens*>(sdiSensor);        
              sensorreadings->m_imaginary_dielectric_permittivity_M = atof(sdiMsgStr.substring(idx0+1,idx1).c_str());
              sensorreadings->m_pore_water_ec_K = atof(sdiMsgStr.substring(idx1+1,idx2).c_str());
              sensorreadings->m_dielectric_loss_tangent_O = atof(sdiMsgStr.substring(idx2+1,idx3).c_str());
              LOG_DEBUG("M: %f - K: %f - O: %f", sensorreadings->m_imaginary_dielectric_permittivity_M, sensorreadings->m_pore_water_ec_K, sensorreadings->m_dielectric_loss_tangent_O);
              break;
            }
            default:
              ;;
          }
          serialMsgRflag = 11;
        }
        if(serialMsgRflag == 12)
        {
          if(sdiMsgStr.length() >= 5)
          {
            int ts = atoi(sdiMsgStr.substring(1,4).c_str());
            LOG_DEBUG("Timeout-1 value is %d seconds (%s)",ts,sdiMsgStr.substring(1,4).c_str());
            measurementTimeout1 = 1000 * ts;
          }

          serialMsgRflag = 13;
        }
        if(serialMsgRflag == 14)
        {
          int idx0, idx1, idx2, idx3;
          int fields = getIndices(sdiMsgStr, 3, idx0, idx1, idx2, idx3); 
          switch(sdiSensorType)
          {
            case SDISensorType::STEVENS:
            {       
              // result of D0
              // 9+1.702+0.293+0.293#
              // L – Real Dielectric Permittivity
              // M – Imaginary Dielectric Permittivity
              // N – Imaginary Dielectric Permittivity
              Stevens* sensorreadings = reinterpret_cast<Stevens*>(sdiSensor);  
              sensorreadings->m_imaginary_dielectric_permittivity_N = atof(sdiMsgStr.substring(idx2+1,idx3).c_str());
              LOG_DEBUG("N: %f", sensorreadings->m_imaginary_dielectric_permittivity_M);
              break;
            }
            default:
              ;;
          }
          serialMsgRflag = 15;
        }    
        if(serialMsgRflag == 16) {
          int idx0, idx1, idx2, idx3;
          int fields = getIndices(sdiMsgStr, 2, idx0, idx1, idx2, idx3);   
          switch(sdiSensorType)
          {
            case SDISensorType::STEVENS:
            {       
              // result of D1
              // 9+0.172+24.9#
              // O – Dielectric Loss Tangent 
              // P – Diode Temperature
              Stevens* sensorreadings = reinterpret_cast<Stevens*>(sdiSensor);  
              sensorreadings->m_diode_temperature_P = atof(sdiMsgStr.substring(idx1+1,idx2).c_str());
              LOG_DEBUG("P: %f", sensorreadings->m_diode_temperature_P);
              break;
            }
            default:
              ;;
          }
          // exit loop
          serialMsgRflag = 0;
        }
        sdiMsgReady = false;  // Reset String for next SDI-12 message.
        sdiMsgStr   = "";
      }

      if (serialMsgRflag)
      {
        if(serialMsgRflag == 3)
        {
          serialMsgRflag = 4;
          String cmd(String(sdiSensorAddress) + "M!");
          m_SDI12->sendCommand(cmd);
          LOG_DEBUG(">>>> %s",cmd.c_str());
        }
        if(serialMsgRflag == 5)
        {
          serialMsgRflag = 6;
          vTaskDelay(pdMS_TO_TICKS(measurementTimeout0)); // <==== THIS TIMEOUT IS REPORTED BY THE SENSOR ON M! COMMAND
          String cmd(String(sdiSensorAddress) + "D0!");
          m_SDI12->sendCommand(cmd);
          LOG_DEBUG(">>>> %s",cmd.c_str());
          m_SDI12->clearBuffer();
        }
        if(serialMsgRflag == 7)
        {
          serialMsgRflag = 8;
          vTaskDelay(pdMS_TO_TICKS(250));
          String cmd(String(sdiSensorAddress) + "D1!");
          m_SDI12->sendCommand(cmd);
          LOG_DEBUG(">>>> %s",cmd.c_str());
          m_SDI12->clearBuffer();
        }
        if(serialMsgRflag == 9)
        {
          serialMsgRflag = 10;
          vTaskDelay(pdMS_TO_TICKS(250));
          String cmd(String(sdiSensorAddress) + "D2!");
          m_SDI12->sendCommand(cmd);
          LOG_DEBUG(">>>> %s",cmd.c_str());
          m_SDI12->clearBuffer();
        } 
        // ACCORDING TO DOCUMENTATION
        if(serialMsgRflag == 11)
        {
          serialMsgRflag = 12;
          vTaskDelay(pdMS_TO_TICKS(250));
          String cmd(String(sdiSensorAddress) + "M1!");
          m_SDI12->sendCommand(cmd);
          LOG_DEBUG(">>>> %s",cmd.c_str());
          m_SDI12->clearBuffer();
        }
        if(serialMsgRflag == 13)
        {
          serialMsgRflag = 14;
          vTaskDelay(pdMS_TO_TICKS(measurementTimeout1));
          String cmd(String(sdiSensorAddress) + "D0!");
          m_SDI12->sendCommand(cmd);
          LOG_DEBUG(">>>> %s",cmd.c_str());
          m_SDI12->clearBuffer();
        }
        if(serialMsgRflag == 15)
        {
          serialMsgRflag = 16;
          vTaskDelay(pdMS_TO_TICKS(250));
          String cmd(String(sdiSensorAddress) + "D1!");
          m_SDI12->sendCommand(cmd);
          LOG_DEBUG(">>>> %s",cmd.c_str());
          m_SDI12->clearBuffer();
        }
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Reading values done, now set success flag if ok
    switch(sdiSensorType)
    {
      case SDISensorType::STEVENS:
      {       
        Stevens* sensorreadings = reinterpret_cast<Stevens*>(sdiSensor);  
        sensorreadings->m_readOK = timeoutCnt > 0;
        anyReadOk |= timeoutCnt > 0;
        break;
      }
      case SDISensorType::WINDSONIC:
      {
        WindSonic* sensorreadings = reinterpret_cast<WindSonic*>(sdiSensor);  
        sensorreadings->m_readOK = timeoutCnt > 0;
        anyReadOk |= timeoutCnt > 0;
        break;
      }
      default:
        ;;
    }
  }
  return anyReadOk;
}

bool RAK13010Sensor::getMetrics(meshtastic_Telemetry *measurement)
{
  bool dataRead = ReadData();
  if(dataRead)
  {   
    // run through all registered sensors
    for(auto it : m_sensors)
    {
      // get bus address from map
      char sdiSensorAddress = it.first;
      // get pointer to sensor
      SDISensor* sdiSensor = it.second;
      // get type of sensor
      SDISensorType sdiSensorType = sdiSensor->getType();
      switch(sdiSensorType)
      {
        case SDISensorType::STEVENS:
        {       
          // maximum is 4 sensors for stephenswaters
          Stevens* sensorreadings = reinterpret_cast<Stevens*>(sdiSensor);  
          if(sensorreadings->m_readOK)
          {
            // 01 is already occupied, so use another definition
            if(measurement->variant.environment_metrics.has_swhp_soil_temperature_01)
            {
              // 02 is already occupied, so use another definition
              if(measurement->variant.environment_metrics.has_swhp_soil_temperature_02)
              {
                // 03 is already occupied, so use another definition
                if(measurement->variant.environment_metrics.has_swhp_soil_temperature_03)
                {
                  measurement->variant.environment_metrics.has_swhp_soil_temperature_04 = true;
                  measurement->variant.environment_metrics.has_swhp_soil_moisture_04 = true;
                  measurement->variant.environment_metrics.swhp_soil_temperature_04 = sensorreadings->m_temperature_G;
                  measurement->variant.environment_metrics.swhp_soil_moisture_04 = sensorreadings->m_soil_moisture_F;  
                }
                else
                {
                  measurement->variant.environment_metrics.has_swhp_soil_temperature_03 = true;
                  measurement->variant.environment_metrics.has_swhp_soil_moisture_03 = true;
                  measurement->variant.environment_metrics.swhp_soil_temperature_03 = sensorreadings->m_temperature_G;
                  measurement->variant.environment_metrics.swhp_soil_moisture_03 = sensorreadings->m_soil_moisture_F;                
                }
              }
              else
              {
                measurement->variant.environment_metrics.has_swhp_soil_temperature_02 = true;
                measurement->variant.environment_metrics.has_swhp_soil_moisture_02 = true;
                measurement->variant.environment_metrics.swhp_soil_temperature_02 = sensorreadings->m_temperature_G;
                measurement->variant.environment_metrics.swhp_soil_moisture_02 = sensorreadings->m_soil_moisture_F;                
              }
            }
            else
            {
              measurement->variant.environment_metrics.has_swhp_soil_temperature_01 = true;
              measurement->variant.environment_metrics.has_swhp_soil_moisture_01 = true;
              measurement->variant.environment_metrics.swhp_soil_temperature_01 = sensorreadings->m_temperature_G;
              measurement->variant.environment_metrics.swhp_soil_moisture_01 = sensorreadings->m_soil_moisture_F;
            }
          }
          break;
        }
        case SDISensorType::WINDSONIC:
        {
          WindSonic* sensorreadings = reinterpret_cast<WindSonic*>(sdiSensor);  
          if(sensorreadings->m_readOK)
          {
            measurement->variant.environment_metrics.has_wind_direction = true;
            measurement->variant.environment_metrics.has_wind_speed = true;

            measurement->variant.environment_metrics.wind_direction = sensorreadings->m_direction;
            measurement->variant.environment_metrics.wind_speed = sensorreadings->m_magnitude;
          }
          break;
        }
        default:
          ;;
      }
    }
    return true;
  }
  return false;
}

#endif