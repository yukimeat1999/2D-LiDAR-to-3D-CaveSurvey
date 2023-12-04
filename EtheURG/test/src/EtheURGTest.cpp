// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  EtheURGTest.cpp
 * @brief URG component with Ethernet connection. (test code)
 *
 */
// </rtc-template>

#include "EtheURGTest.h"

// Module specification
// <rtc-template block="module_spec">
static const char* const etheurg_spec[] =
  {
    "implementation_id", "EtheURGTest",
    "type_name",         "EtheURGTest",
    "description",       "URG component with Ethernet connection.",
    "version",           "1.0.0",
    "vendor",            "Y. Fujii",
    "category",          "Sensor",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.connect_flag", "Ethe",
    "conf.default.serial_port_name", "\\\\.\\COM1",
    "conf.default.serial_baud_rate", "115200",
    "conf.default.ethe_IP_add", "192.168.0.10",
    "conf.default.ethe_port", "10940",
    "conf.default.geometry_x", "0.0",
    "conf.default.geometry_y", "0.0",
    "conf.default.geometry_z", "0.0",
    "conf.default.geometry_roll", "0.0",
    "conf.default.geometry_pitch", "0.0",
    "conf.default.geometry_yaw", "0.0",

    // Widget
    "conf.__widget__.connect_flag", "radio",
    "conf.__widget__.serial_port_name", "text",
    "conf.__widget__.serial_baud_rate", "text",
    "conf.__widget__.ethe_IP_add", "text",
    "conf.__widget__.ethe_port", "text",
    "conf.__widget__.geometry_x", "text",
    "conf.__widget__.geometry_y", "text",
    "conf.__widget__.geometry_z", "text",
    "conf.__widget__.geometry_roll", "text",
    "conf.__widget__.geometry_pitch", "text",
    "conf.__widget__.geometry_yaw", "text",
    // Constraints
    "conf.__constraints__.connect_flag", "(Serial,Ethe)",

    "conf.__type__.connect_flag", "string",
    "conf.__type__.serial_port_name", "string",
    "conf.__type__.serial_baud_rate", "int",
    "conf.__type__.ethe_IP_add", "string",
    "conf.__type__.ethe_port", "int",
    "conf.__type__.geometry_x", "double",
    "conf.__type__.geometry_y", "double",
    "conf.__type__.geometry_z", "double",
    "conf.__type__.geometry_roll", "double",
    "conf.__type__.geometry_pitch", "double",
    "conf.__type__.geometry_yaw", "double",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
EtheURGTest::EtheURGTest(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_rangeIn("range", m_range)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
EtheURGTest::~EtheURGTest()
{
}



RTC::ReturnCode_t EtheURGTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("range", m_rangeIn);
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("connect_flag", m_connect_flag, "Ethe");
  bindParameter("serial_port_name", m_serial_port_name, "\\\\.\\COM1");
  bindParameter("serial_baud_rate", m_serial_baud_rate, "115200");
  bindParameter("ethe_IP_add", m_ethe_IP_add, "192.168.0.10");
  bindParameter("ethe_port", m_ethe_port, "10940");
  bindParameter("geometry_x", m_geometry_x, "0.0");
  bindParameter("geometry_y", m_geometry_y, "0.0");
  bindParameter("geometry_z", m_geometry_z, "0.0");
  bindParameter("geometry_roll", m_geometry_roll, "0.0");
  bindParameter("geometry_pitch", m_geometry_pitch, "0.0");
  bindParameter("geometry_yaw", m_geometry_yaw, "0.0");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t EtheURGTest::onFinalize()
{
  return RTC::RTC_OK;
}
*/


//RTC::ReturnCode_t EtheURGTest::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t EtheURGTest::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t EtheURGTest::onActivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EtheURGTest::onDeactivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EtheURGTest::onExecute(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t EtheURGTest::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t EtheURGTest::onError(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t EtheURGTest::onReset(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t EtheURGTest::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t EtheURGTest::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


bool EtheURGTest::runTest()
{
    return true;
}


extern "C"
{
 
  void EtheURGTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(etheurg_spec);
    manager->registerFactory(profile,
                             RTC::Create<EtheURGTest>,
                             RTC::Delete<EtheURGTest>);
  }
  
}
