// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  PointCloud_ReaderTest.cpp
 * @brief PointCloud Reader (test code)
 *
 */
// </rtc-template>

#include "PointCloud_ReaderTest.h"

// Module specification
// <rtc-template block="module_spec">
#if RTM_MAJOR_VERSION >= 2
static const char* const pointcloud_reader_spec[] =
#else
static const char* pointcloud_reader_spec[] =
#endif
  {
    "implementation_id", "PointCloud_ReaderTest",
    "type_name",         "PointCloud_ReaderTest",
    "description",       "PointCloud Reader",
    "version",           "1.0.0",
    "vendor",            "Y. Fujii",
    "category",          "Reader",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.FILE_NAME", "Your_PointCloud_File.ply",

    // Widget
    "conf.__widget__.FILE_NAME", "text",
    // Constraints

    "conf.__type__.FILE_NAME", "string",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
PointCloud_ReaderTest::PointCloud_ReaderTest(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_File_PointCloudIn("File_PointCloud", m_File_PointCloud)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
PointCloud_ReaderTest::~PointCloud_ReaderTest()
{
}



RTC::ReturnCode_t PointCloud_ReaderTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("File_PointCloud", m_File_PointCloudIn);
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("FILE_NAME", m_FILE_NAME, "Your_PointCloud_File.ply");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PointCloud_ReaderTest::onFinalize()
{
  return RTC::RTC_OK;
}
*/


//RTC::ReturnCode_t PointCloud_ReaderTest::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t PointCloud_ReaderTest::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t PointCloud_ReaderTest::onActivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t PointCloud_ReaderTest::onDeactivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t PointCloud_ReaderTest::onExecute(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t PointCloud_ReaderTest::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t PointCloud_ReaderTest::onError(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t PointCloud_ReaderTest::onReset(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t PointCloud_ReaderTest::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t PointCloud_ReaderTest::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


bool PointCloud_ReaderTest::runTest()
{
    return true;
}


extern "C"
{
 
  void PointCloud_ReaderTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(pointcloud_reader_spec);
    manager->registerFactory(profile,
                             RTC::Create<PointCloud_ReaderTest>,
                             RTC::Delete<PointCloud_ReaderTest>);
  }
  
}
