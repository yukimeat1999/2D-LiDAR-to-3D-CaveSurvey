// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  MeasurementSystemTest.cpp
 * @brief Measurement System for 3D point cloud data creation component using 2D LiDAR and Dynamixel. (test code)
 *
 */
// </rtc-template>

#include "MeasurementSystemTest.h"

// Module specification
// <rtc-template block="module_spec">
static const char* const measurementsystem_spec[] =
  {
    "implementation_id", "MeasurementSystemTest",
    "type_name",         "MeasurementSystemTest",
    "description",       "Measurement System for 3D point cloud data creation component using 2D LiDAR and Dynamixel.",
    "version",           "1.0.0",
    "vendor",            "Y. Fujii",
    "category",          "Controller",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.DEVICE_NAME", "\\\\.\\COM1",
    "conf.default.BAUDRATE", "57600",
    "conf.default.encoder_resolution_", "4096",

    // Widget
    "conf.__widget__.DEVICE_NAME", "text",
    "conf.__widget__.BAUDRATE", "text",
    "conf.__widget__.encoder_resolution_", "text",
    // Constraints

    "conf.__type__.DEVICE_NAME", "string",
    "conf.__type__.BAUDRATE", "int",
    "conf.__type__.encoder_resolution_", "int",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
MeasurementSystemTest::MeasurementSystemTest(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_rangeOut("range", m_range),
    m_new_PointCloudIn("new_PointCloud", m_new_PointCloud)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
MeasurementSystemTest::~MeasurementSystemTest()
{
}



RTC::ReturnCode_t MeasurementSystemTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("new_PointCloud", m_new_PointCloudIn);
  
  // Set OutPort buffer
  addOutPort("range", m_rangeOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("DEVICE_NAME", m_DEVICE_NAME, "\\\\.\\COM1");
  bindParameter("BAUDRATE", m_BAUDRATE, "57600");
  bindParameter("encoder_resolution_", m_encoder_resolution_, "4096");
  // </rtc-template>
  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MeasurementSystemTest::onFinalize()
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t MeasurementSystemTest::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t MeasurementSystemTest::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t MeasurementSystemTest::onActivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MeasurementSystemTest::onDeactivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MeasurementSystemTest::onExecute(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t MeasurementSystemTest::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t MeasurementSystemTest::onError(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MeasurementSystemTest::onReset(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t MeasurementSystemTest::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t MeasurementSystemTest::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


bool MeasurementSystemTest::runTest()
{
    return true;
}


extern "C"
{
 
  void MeasurementSystemTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(measurementsystem_spec);
    manager->registerFactory(profile,
                             RTC::Create<MeasurementSystemTest>,
                             RTC::Delete<MeasurementSystemTest>);
  }
  
}
