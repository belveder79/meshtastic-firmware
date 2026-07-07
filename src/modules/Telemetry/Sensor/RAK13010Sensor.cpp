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

bool RAK13010Sensor::initDevice(TwoWire *bus, ScanI2C::FoundDevice *dev)
{
  LOG_DEBUG("Init sensor: %s", sensorName);

  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);  // Power the sensors.

  vTaskDelay(pdMS_TO_TICKS(500));

  LOG_DEBUG("==== Opening SDI-12 bus.");
  m_SDI12 = new RAK_SDI12(RX_PIN,TX_PIN,OE);

  m_SDI12->begin();  // Initiate serial connection to SDI-12 bus.
  LOG_DEBUG("==== Starting communication...");
  vTaskDelay(pdMS_TO_TICKS(500));

  m_SDI12->forceListen();

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
        }
      }
    }

    if (sdiMsgReady)
    {
      LOG_DEBUG("<<<< %s", sdiMsgStr.c_str());
      // TODO: DISASSEMBLE STRING HERE!!!
      // Result of 9I! is something like 912STEVENSW0560126.302GSN00289667#
      m_sensorID = sdiMsgStr;

      sdiMsgReady = false;  // Reset String for next SDI-12 message.
      sdiMsgStr   = "";
      serialMsgRflag = 0; 
    }

    if (serialMsgRflag)
    {
      if(serialMsgRflag == 1)
      {
        serialMsgRflag = 2;
        String cmd(String(RAK13010_SENSOR_ID) + "I!");
        m_SDI12->sendCommand(cmd);
        LOG_DEBUG((String(">>>> ") + cmd).c_str());
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return timeoutCnt > 0;
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
        // result of M! should give 90029, which says 9 (address) 002 (2 seconds until measurement ready) 9 (9 fields in D0, D1 and D2)
        // this means that the timeout should be 2 seconds until requesting!
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
        // result of D0! should give something like 
        // 9+0.000+0.001+25.0#
        // F – Soil Moisture
        // I – Bulk EC (Temp Corrected)
        // G – Temperature (C)
        int idx0, idx1, idx2, idx3;
        int fields = getIndices(sdiMsgStr, 3, idx0, idx1, idx2, idx3);
        m_sensorreadings.m_soil_moisture_F = atof(sdiMsgStr.substring(idx0+1,idx1).c_str());
        m_sensorreadings.m_bulk_ec_corrected_I = atof(sdiMsgStr.substring(idx1+1,idx2).c_str());
        m_sensorreadings.m_temperature_G = atof(sdiMsgStr.substring(idx2+1,idx3).c_str());
        LOG_DEBUG("F: %f - I: %f - G: %f", m_sensorreadings.m_soil_moisture_F, m_sensorreadings.m_bulk_ec_corrected_I, m_sensorreadings.m_temperature_G);
        serialMsgRflag = 7;
      }
      if(serialMsgRflag == 8)
      {
        // result of D1
        // 9+77.1+0.001+1.701#
        // H – Temperature (F) 
        // J – Bulk EC 
        // L – Real Dielectric Permittivity
        int idx0, idx1, idx2, idx3;
        int fields = getIndices(sdiMsgStr, 3, idx0, idx1, idx2, idx3);
        m_sensorreadings.m_temperature_H = atof(sdiMsgStr.substring(idx0+1,idx1).c_str());
        m_sensorreadings.m_bulk_ec_J = atof(sdiMsgStr.substring(idx1+1,idx2).c_str());
        m_sensorreadings.m_real_dielectric_permittivity_L = atof(sdiMsgStr.substring(idx2+1,idx3).c_str());
        LOG_DEBUG("H: %f - J: %f - L: %f", m_sensorreadings.m_temperature_H, m_sensorreadings.m_bulk_ec_J, m_sensorreadings.m_real_dielectric_permittivity_L);
        serialMsgRflag = 9;
      }    
      if(serialMsgRflag == 10) {
        // result of D2
        // 9+0.295-0.038+0.173#
        // M – Imaginary Dielectric Permittivity
        // K – Pore Water EC
        // O – Dielectric Loss Tangent
        int idx0, idx1, idx2, idx3;
        int fields = getIndices(sdiMsgStr, 3, idx0, idx1, idx2, idx3);       
        m_sensorreadings.m_imaginary_dielectric_permittivity_M = atof(sdiMsgStr.substring(idx0+1,idx1).c_str());
        m_sensorreadings.m_pore_water_ec_K = atof(sdiMsgStr.substring(idx1+1,idx2).c_str());
        m_sensorreadings.m_dielectric_loss_tangent_O = atof(sdiMsgStr.substring(idx2+1,idx3).c_str());
        LOG_DEBUG("M: %f - K: %f - O: %f", m_sensorreadings.m_imaginary_dielectric_permittivity_M, m_sensorreadings.m_pore_water_ec_K, m_sensorreadings.m_dielectric_loss_tangent_O);
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
        // result of D0
        // 9+1.702+0.293+0.293#
        // L – Real Dielectric Permittivity
        // M – Imaginary Dielectric Permittivity
        // N – Imaginary Dielectric Permittivity
        int idx0, idx1, idx2, idx3;
        int fields = getIndices(sdiMsgStr, 3, idx0, idx1, idx2, idx3);     
        m_sensorreadings.m_imaginary_dielectric_permittivity_N = atof(sdiMsgStr.substring(idx2+1,idx3).c_str());
        LOG_DEBUG("N: %f", m_sensorreadings.m_imaginary_dielectric_permittivity_M);
        serialMsgRflag = 15;
      }    
      if(serialMsgRflag == 16) {
        // result of D1
        // 9+0.172+24.9#
        // O – Dielectric Loss Tangent 
        // P – Diode Temperature
        int idx0, idx1, idx2, idx3;
        int fields = getIndices(sdiMsgStr, 2, idx0, idx1, idx2, idx3);   
        m_sensorreadings.m_diode_temperature_P = atof(sdiMsgStr.substring(idx1+1,idx2).c_str());
        LOG_DEBUG("P: %f", m_sensorreadings.m_diode_temperature_P);
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
        String cmd(String(RAK13010_SENSOR_ID) + "M!");
        m_SDI12->sendCommand(cmd);
        LOG_DEBUG(">>>> %s",cmd.c_str());
      }
      if(serialMsgRflag == 5)
      {
        serialMsgRflag = 6;
        vTaskDelay(pdMS_TO_TICKS(measurementTimeout0)); // <==== THIS TIMEOUT IS REPORTED BY THE SENSOR ON M! COMMAND
        String cmd(String(RAK13010_SENSOR_ID) + "D0!");
        m_SDI12->sendCommand(cmd);
        LOG_DEBUG(">>>> %s",cmd.c_str());
        m_SDI12->clearBuffer();
      }
      if(serialMsgRflag == 7)
      {
        serialMsgRflag = 8;
        vTaskDelay(pdMS_TO_TICKS(250));
        String cmd(String(RAK13010_SENSOR_ID) + "D1!");
        m_SDI12->sendCommand(cmd);
        LOG_DEBUG(">>>> %s",cmd.c_str());
        m_SDI12->clearBuffer();
      }
      if(serialMsgRflag == 9)
      {
        serialMsgRflag = 10;
        vTaskDelay(pdMS_TO_TICKS(250));
        String cmd(String(RAK13010_SENSOR_ID) + "D2!");
        m_SDI12->sendCommand(cmd);
        LOG_DEBUG(">>>> %s",cmd.c_str());
        m_SDI12->clearBuffer();
      } 
      // ACCORDING TO DOCUMENTATION
      if(serialMsgRflag == 11)
      {
        serialMsgRflag = 12;
        vTaskDelay(pdMS_TO_TICKS(250));
        String cmd(String(RAK13010_SENSOR_ID) + "M1!");
        m_SDI12->sendCommand(cmd);
        LOG_DEBUG(">>>> %s",cmd.c_str());
        m_SDI12->clearBuffer();
      }
      if(serialMsgRflag == 13)
      {
        serialMsgRflag = 14;
        vTaskDelay(pdMS_TO_TICKS(measurementTimeout1));
        String cmd(String(RAK13010_SENSOR_ID) + "D0!");
        m_SDI12->sendCommand(cmd);
        LOG_DEBUG(">>>> %s",cmd.c_str());
        m_SDI12->clearBuffer();
      }
      if(serialMsgRflag == 15)
      {
        serialMsgRflag = 16;
        vTaskDelay(pdMS_TO_TICKS(250));
        String cmd(String(RAK13010_SENSOR_ID) + "D1!");
        m_SDI12->sendCommand(cmd);
        LOG_DEBUG(">>>> %s",cmd.c_str());
        m_SDI12->clearBuffer();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return timeoutCnt > 0;
}

bool RAK13010Sensor::getMetrics(meshtastic_Telemetry *measurement)
{
    bool dataRead = ReadData();
    if(dataRead)
    {   
      measurement->variant.environment_metrics.has_soil_temperature = true;
      measurement->variant.environment_metrics.has_soil_moisture = true;

      measurement->variant.environment_metrics.soil_temperature = m_sensorreadings.m_temperature_G;
      measurement->variant.environment_metrics.soil_moisture = m_sensorreadings.m_soil_moisture_F;
      return true;
    }
    return false;
}

#endif