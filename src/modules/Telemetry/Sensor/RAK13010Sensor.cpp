#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && defined(RAK13010_SENSOR_EN)

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "RAK13010Sensor.h"
#include "TelemetrySensor.h"
#include "concurrency/LockGuard.h"
#include "mesh/Throttle.h"

#include <algorithm>
#include <vector>

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

#if GILL_CONTINUOUS_AVG_POLAR
bool RAK13010Sensor::ConfigGillXHPM(char i)
{
  uint8_t serialMsgRflag = 1;
  boolean sdiMsgReady = false;
  String sdiMsgStr = "";
  
  int timeoutCnt = 5000;
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
          LOG_DEBUG("recv: %s", sdiMsgStr.c_str());
        }
      }
    }

    if (sdiMsgReady)
    {
      LOG_DEBUG("<<<< %s", sdiMsgStr.c_str());
      // TODO: check retval of XHPM command

      sdiMsgReady = false;  // Reset String for next SDI-12 message.
      sdiMsgStr   = "";
      serialMsgRflag = 0; 
    }

    if (serialMsgRflag)
    {
      if(serialMsgRflag == 1)
      {
        serialMsgRflag = 2;
        String cmd(String(i) + "XHPM!");
        m_SDI12->sendCommand(cmd);
        LOG_DEBUG((String(">>>> ") + cmd).c_str());
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return (timeoutCnt > 0);
}
#endif

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
#if GILL_CONTINUOUS_AVG_POLAR
        if(!ConfigGillXHPM(i))
        {
          LOG_WARN("==== Setting XHPM mode failed!");
        }
#endif
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

  // If a GILL WindSonic was found, start the background poller that keeps the rolling
  // gust/lull buffer fed (see ReadWindSonicOnce()/windPollTaskTrampoline() above).
  int windSonicCount = 0;
  for (auto it : m_sensors)
  {
    if (it.second->getType() == SDISensorType::WINDSONIC)
    {
      if (windSonicCount == 0)
        m_windSonicAddress = it.first;
      windSonicCount++;
    }
  }
  if (windSonicCount > 1)
    LOG_WARN("RAK13010: %d WindSonic sensors found, only polling address %c", windSonicCount, m_windSonicAddress);
  if (m_windSonicAddress != 0)
  {
    LOG_DEBUG("RAK13010: starting background WindSonic poller on address %c", m_windSonicAddress);
    xTaskCreate(windPollTaskTrampoline, "wind_poll", 4096, this, 1, &m_windTaskHandle);
  }

  // indicate that we are good if the # of sensors is > 0
  return m_sensors.size() > 0;
}

// Response to M!/M1! is "atttn": address, ttt = seconds until measurement ready, n = field count.
static int parseMeasurementTimeoutSeconds(const String &sdiMsgStr, bool log = 1)
{
  int ts = atoi(sdiMsgStr.substring(1, 4).c_str());
  if(log)
    LOG_DEBUG("Timeout value is %d seconds (%s)", ts, sdiMsgStr.substring(1, 4).c_str());
  return ts;
}

int getIndices(const String sdiMsgStr, const int expected, int &idx0, int &idx1, int &idx2, int &idx3, int &idx4, int& idx5)
{
        int idx0p = sdiMsgStr.indexOf("+",0); idx0p = idx0p < 0 ? sdiMsgStr.length() : idx0p;     
        int idx0m = sdiMsgStr.indexOf("-",0); idx0m = idx0m < 0 ? sdiMsgStr.length() : idx0m;
        idx0 = idx0m < idx0p ? idx0m : idx0p;
        int idx1p = sdiMsgStr.indexOf("+",idx0+1); idx1p = idx1p < 0 ? sdiMsgStr.length() : idx1p;   
        int idx1m = sdiMsgStr.indexOf("-",idx0+1); idx1m = idx1m < 0 ? sdiMsgStr.length() : idx1m;
        idx1 = idx1m < idx1p ? idx1m : idx1p;
        if(expected > 2)
        {
          // strikes for 3 or more
          int idx2p = sdiMsgStr.indexOf("+",idx1+1); idx2p = idx2p < 0 ? sdiMsgStr.length() : idx2p;   
          int idx2m = sdiMsgStr.indexOf("-",idx1+1); idx2m = idx2m < 0 ? sdiMsgStr.length() : idx2m;
          idx2 = idx2m < idx2p ? idx2m : idx2p;
          if(expected > 3) 
          {
            // strikes for 4 or more
            int idx3p = sdiMsgStr.indexOf("+",idx2+1); idx3p = idx3p < 0 ? sdiMsgStr.length() : idx3p;   
            int idx3m = sdiMsgStr.indexOf("-",idx2+1); idx3m = idx3m < 0 ? sdiMsgStr.length() : idx3m;
            idx3 = idx3m < idx3p ? idx3m : idx3p;
            if(expected > 4)
            { 
              // strikes for 5
              int idx4p = sdiMsgStr.indexOf("+",idx3+1); idx4p = idx4p < 0 ? sdiMsgStr.length() : idx4p;   
              int idx4m = sdiMsgStr.indexOf("-",idx3+1); idx4m = idx4m < 0 ? sdiMsgStr.length() : idx4m;
              idx4 = idx4m < idx4p ? idx4m : idx4p;
              idx5 = sdiMsgStr.indexOf("#",idx4+1);
            }
            else
            { // strikes for 4 
              idx4 = sdiMsgStr.indexOf("#",idx3+1);
              idx5 = sdiMsgStr.length();
            }
          }
          else
          { // strikes for 3
            idx3 = sdiMsgStr.indexOf("#",idx2+1);
            idx4 = idx5 = sdiMsgStr.length();
          }
        }
        else
        { // strikes for 2
          idx2 = sdiMsgStr.indexOf("#",idx1+1);
          idx3 = idx4 = idx5 = sdiMsgStr.length();
        }
        return expected;
}

bool RAK13010Sensor::ReadData()
{
  // Serialize against the background WindSonic poller (windPollTaskTrampoline()), which
  // drives the same m_SDI12 bus object from a different task.
  concurrency::LockGuard busLock(&m_sdiBusLock);

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

    // WindSonic is polled separately by the background task (ReadWindSonicOnce(), see
    // below) so it can run at close to its own natural cadence instead of only at
    // telemetry-broadcast time; getMetrics() reads its buffered samples directly.
    if (sdiSensorType == SDISensorType::WINDSONIC)
      continue;

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
            measurementTimeout0 = 1000 * parseMeasurementTimeoutSeconds(sdiMsgStr);
          }
          serialMsgRflag = 5;
        }
        if(serialMsgRflag == 6)
        {
          int idx0, idx1, idx2, idx3, idx4, idx5;
          switch(sdiSensorType)
          {
            case SDISensorType::STEVENS:
            {
              int fields = getIndices(sdiMsgStr, 3, idx0, idx1, idx2, idx3, idx4, idx5);
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
#if GILL_CONTINUOUS_AVG_POLAR
              int fields = getIndices(sdiMsgStr, 5, idx0, idx1, idx2, idx3, idx4, idx5);
              // result of R2! should give something like
              // <dir_from_vectorav><mag_from_vectorav><dir_at_mag_scalarmax><mag_scalarmax><status>
              // a+090+000.02+123+000.12+00#
              // direction
              // magnitude
              // status
              WindSonic* sensorreadings = reinterpret_cast<WindSonic*>(sdiSensor);
              sensorreadings->m_direction = atoi(sdiMsgStr.substring(idx0+1,idx1).c_str()); // cast to uint16
              sensorreadings->m_magnitude = atof(sdiMsgStr.substring(idx1+1,idx2).c_str());
              sensorreadings->m_dirmax = atoi(sdiMsgStr.substring(idx2+1,idx3).c_str());
              sensorreadings->m_magmax = atof(sdiMsgStr.substring(idx3+1,idx4).c_str());
              sensorreadings->m_status = atoi(sdiMsgStr.substring(idx4+1,idx5).c_str());
              LOG_DEBUG("dir: %u - mag: %f - maxdir: %u - maxmag: %f - status: %u", sensorreadings->m_direction, 
                sensorreadings->m_magnitude, sensorreadings->m_dirmax, sensorreadings->m_magmax, sensorreadings->m_status);
#else
              int fields = getIndices(sdiMsgStr, 3, idx0, idx1, idx2, idx3, idx4, idx5);
              // result of D0! should give something like 
              // a+083+000.02+00#
              // direction
              // magnitude
              // status
              WindSonic* sensorreadings = reinterpret_cast<WindSonic*>(sdiSensor);
              sensorreadings->m_direction = atoi(sdiMsgStr.substring(idx0+1,idx1).c_str()); // cast to uint16
              sensorreadings->m_magnitude = atof(sdiMsgStr.substring(idx1+1,idx2).c_str());
              sensorreadings->m_status = atoi(sdiMsgStr.substring(idx2+1,idx3).c_str());
              LOG_DEBUG("dir: %u - mag: %f - status: %u", sensorreadings->m_direction, sensorreadings->m_magnitude, sensorreadings->m_status);
#endif
              serialMsgRflag = 0; // exit flag
              break;
            }
            default:
              ;;
          }
        }
        if(serialMsgRflag == 8)
        {
          int idx0, idx1, idx2, idx3, idx4, idx5;
          int fields = getIndices(sdiMsgStr, 3, idx0, idx1, idx2, idx3, idx4, idx5);          
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
          int idx0, idx1, idx2, idx3, idx4, idx5;
          int fields = getIndices(sdiMsgStr, 3, idx0, idx1, idx2, idx3, idx4, idx5);
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
            measurementTimeout1 = 1000 * parseMeasurementTimeoutSeconds(sdiMsgStr);
          }

          serialMsgRflag = 13;
        }
        if(serialMsgRflag == 14)
        {
          int idx0, idx1, idx2, idx3, idx4, idx5;
          int fields = getIndices(sdiMsgStr, 3, idx0, idx1, idx2, idx3, idx4, idx5); 
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
          int idx0, idx1, idx2, idx3, idx4, idx5;
          int fields = getIndices(sdiMsgStr, 2, idx0, idx1, idx2, idx3, idx4, idx5);   
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
#if GILL_CONTINUOUS_AVG_POLAR          
          // switch types
          switch(sdiSensorType)
          {
            case SDISensorType::STEVENS:
            {
#endif
              String cmd(String(sdiSensorAddress) + "M!");
              m_SDI12->sendCommand(cmd);
              LOG_DEBUG(">>>> %s",cmd.c_str());
              serialMsgRflag = 4;
              break;
#if GILL_CONTINUOUS_AVG_POLAR
            }
            case SDISensorType::WINDSONIC:
            {
              String cmd(String(sdiSensorAddress) + "R2!");
              m_SDI12->sendCommand(cmd);
              LOG_DEBUG(">>>> %s",cmd.c_str());
              serialMsgRflag = 6;
              break;
            }
            default:
              ;;
          }
#endif
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

// ---- Background WindSonic polling ----------------------------------------------------
//
// GILL_CONTINUOUS_AVG_POLAR == 0 path only: the sensor's own onboard R2! averaging mode
// is known-broken against our hardware (see commit 1e45c22ac), so instead we poll plain
// instantaneous M!+D0! queries from a dedicated task and compute gust/lull ourselves from
// a rolling buffer of samples. This mirrors ReedCounterSensor's xTaskCreate precedent
// (RAK_4631 branch) for background sensor polling.
//
// m_sdiBusLock serializes this task's queries against ReadData()'s Stevens-only sweep
// (still called synchronously from getMetrics()), since both drive the same shared,
// non-thread-safe m_SDI12 bus object. m_windLock separately protects m_windBuffer, which
// this task writes to and getMetrics()/getWindGust()/getWindLull() read from.

bool RAK13010Sensor::ReadWindSonicOnce(char address, WindSample &out, uint32_t &timeoutMsUsed)
{
  uint8_t serialMsgRflag = 3;
  boolean sdiMsgReady = false;
  String sdiMsgStr = "";

  int measurementTimeoutMs = 1000; // default; overwritten by the sensor's own M! response
  int timeoutCnt = 10000;

  while (serialMsgRflag && timeoutCnt-- > 0)
  {
    int avail = m_SDI12->available();
    if (avail < 0)
    {
      m_SDI12->clearBuffer(); // Buffer is full, clear.
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
#if GILL_DEBUG_READ      
      LOG_DEBUG("<<<< %s", sdiMsgStr.c_str());
#endif
      if (serialMsgRflag == 4)
      {
        if (sdiMsgStr.length() >= 5)
          measurementTimeoutMs = 1000 * parseMeasurementTimeoutSeconds(sdiMsgStr, GILL_DEBUG_READ);
        serialMsgRflag = 5;
      }
      if (serialMsgRflag == 6)
      {
        int idx0, idx1, idx2, idx3, idx4, idx5;
        getIndices(sdiMsgStr, 3, idx0, idx1, idx2, idx3, idx4, idx5);
        // result of D0! looks like a+083+000.02+00#: direction, magnitude, status
        out.direction = atoi(sdiMsgStr.substring(idx0 + 1, idx1).c_str());
        out.speed = atof(sdiMsgStr.substring(idx1 + 1, idx2).c_str());
        out.status = atoi(sdiMsgStr.substring(idx2 + 1, idx3).c_str());
#if GILL_DEBUG_READ
        LOG_DEBUG("wind dir: %u - speed: %f - status: %u", out.direction, out.speed, out.status);
#endif
        serialMsgRflag = 0; // done
      }

      sdiMsgReady = false;
      sdiMsgStr = "";
    }

    if (serialMsgRflag == 3)
    {
      String cmd(String(address) + "M!");
      m_SDI12->sendCommand(cmd);
#if GILL_DEBUG_READ      
      LOG_DEBUG(">>>> %s", cmd.c_str());
#endif
      serialMsgRflag = 4;
    }
    if (serialMsgRflag == 5)
    {
      serialMsgRflag = 6;
      vTaskDelay(pdMS_TO_TICKS(measurementTimeoutMs)); // sensor-reported delay from the M! response
      String cmd(String(address) + "D0!");
      m_SDI12->sendCommand(cmd);
#if GILL_DEBUG_READ
      LOG_DEBUG(">>>> %s", cmd.c_str());
#endif
      m_SDI12->clearBuffer();
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }

  timeoutMsUsed = measurementTimeoutMs;
  return timeoutCnt > 0;
}

void RAK13010Sensor::pushWindSample(const WindSample &s)
{
  concurrency::LockGuard g(&m_windLock);
  m_windBuffer.push_back(s);
  while (!m_windBuffer.empty() && !Throttle::isWithinTimespanMs(m_windBuffer.front().timestampMs, WIND_BUFFER_WINDOW_MS))
    m_windBuffer.pop_front();
}

bool RAK13010Sensor::computeGustLull(bool wantGust, float &out)
{
  concurrency::LockGuard g(&m_windLock);
  std::vector<float> speeds;
  // m_windBuffer is time-ordered oldest -> newest, so walking backwards lets us stop as
  // soon as we leave the 60s window instead of scanning the whole 300s buffer.
  for (auto it = m_windBuffer.rbegin(); it != m_windBuffer.rend(); ++it)
  {
    if (!Throttle::isWithinTimespanMs(it->timestampMs, WIND_GUSTLULL_WINDOW_MS))
      break;
    speeds.push_back(it->speed);
  }
  if (speeds.empty())
    return false;

  std::sort(speeds.begin(), speeds.end());
  size_t n = std::min(speeds.size(), std::max<size_t>(1, (size_t)(speeds.size() * WIND_GUSTLULL_FRACTION)));
  float sum = 0;
  for (size_t i = 0; i < n; i++)
    sum += wantGust ? speeds[speeds.size() - 1 - i] : speeds[i];
  out = sum / n;
  return true;
}

bool RAK13010Sensor::getWindGust(float &out)
{
  return computeGustLull(true, out);
}

bool RAK13010Sensor::getWindLull(float &out)
{
  return computeGustLull(false, out);
}

void RAK13010Sensor::windPollTaskTrampoline(void *arg)
{
  RAK13010Sensor *self = static_cast<RAK13010Sensor *>(arg);
  uint32_t lastPollStartMs = 0;
  while (true)
  {
    // Cap the loop to at most one measurement per second, regardless of how quickly the
    // sensor's own M!/D0! round trip completes -- if it's slower than 1s (the common
    // case; see the M!-timeout note above ReadWindSonicOnce()), this simply has no effect
    // and each cycle runs back-to-back at whatever cadence the hardware allows.
    if (lastPollStartMs != 0 && Throttle::isWithinTimespanMs(lastPollStartMs, 1000))
    {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    lastPollStartMs = millis();

    WindSample s{};
    uint32_t timeoutMsUsed = 0;
    bool ok;
    {
      concurrency::LockGuard bus(&self->m_sdiBusLock);
      ok = self->ReadWindSonicOnce(self->m_windSonicAddress, s, timeoutMsUsed);
    }
    if (ok)
    {
      s.timestampMs = millis();
      self->pushWindSample(s);
    }
    else
    {
      LOG_WARN("RAK13010: WindSonic read failed, retrying");
      vTaskDelay(pdMS_TO_TICKS(250)); // backoff, avoid busy-looping a wedged/unplugged sensor
    }
  }
}

bool RAK13010Sensor::getMetrics(meshtastic_Telemetry *measurement)
{
  bool anyOk = false;

  // Stevens (soil probe) path: unchanged, still a synchronous SDI-12 sweep. WindSonic
  // entries are skipped inside ReadData() now (see above), so this only touches Stevens.
  bool stevensDataRead = ReadData();
  if (stevensDataRead)
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
      if (sdiSensorType != SDISensorType::STEVENS)
        continue;

      // maximum is 4 sensors for stephenswaters
      Stevens* sensorreadings = reinterpret_cast<Stevens*>(sdiSensor);
      if(sensorreadings->m_readOK)
      {
        anyOk = true;
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
    }
  }

  // WindSonic path: read the latest sample straight from the background poller's buffer
  // instead of independently querying SDI-12 (that bus is now solely owned by the
  // background task + the Stevens sweep above, serialized via m_sdiBusLock).
  {
    concurrency::LockGuard g(&m_windLock);
    if (!m_windBuffer.empty())
    {
      const WindSample &latest = m_windBuffer.back();
      measurement->variant.environment_metrics.has_wind_direction = true;
      measurement->variant.environment_metrics.has_wind_speed = true;
      measurement->variant.environment_metrics.wind_direction = latest.direction;
      measurement->variant.environment_metrics.wind_speed = latest.speed;
      anyOk = true;
    }
  }

  float gust, lull;
  if (getWindGust(gust))
  {
    measurement->variant.environment_metrics.has_wind_gust = true;
    measurement->variant.environment_metrics.wind_gust = gust;
  }
  if (getWindLull(lull))
  {
    measurement->variant.environment_metrics.has_wind_lull = true;
    measurement->variant.environment_metrics.wind_lull = lull;
  }

  return anyOk;
}

#endif