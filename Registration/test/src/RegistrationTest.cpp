// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  RegistrationTest.cpp
 * @brief Registration comp (test code)
 *
 */
// </rtc-template>

#include "RegistrationTest.h"

// Module specification
// <rtc-template block="module_spec">
static const char* const registration_spec[] =
  {
    "implementation_id", "RegistrationTest",
    "type_name",         "RegistrationTest",
    "description",       "Registration comp",
    "version",           "1.0.0",
    "vendor",            "Y. Fujii",
    "category",          "Registration",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
RegistrationTest::RegistrationTest(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_new_PointCloudOut("new_PointCloud", m_new_PointCloud),
    m_merge_PointCloudIn("merge_PointCloud", m_merge_PointCloud),
    m_LocalizationIn("Localization", m_Localization)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
RegistrationTest::~RegistrationTest()
{
}



RTC::ReturnCode_t RegistrationTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("merge_PointCloud", m_merge_PointCloudIn);
  addInPort("Localization", m_LocalizationIn);
  
  // Set OutPort buffer
  addOutPort("new_PointCloud", m_new_PointCloudOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t RegistrationTest::onFinalize()
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t RegistrationTest::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t RegistrationTest::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t RegistrationTest::onActivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t RegistrationTest::onDeactivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t RegistrationTest::onExecute(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t RegistrationTest::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t RegistrationTest::onError(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t RegistrationTest::onReset(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t RegistrationTest::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t RegistrationTest::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


bool RegistrationTest::runTest()
{
    return true;
}


extern "C"
{
 
  void RegistrationTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(registration_spec);
    manager->registerFactory(profile,
                             RTC::Create<RegistrationTest>,
                             RTC::Delete<RegistrationTest>);
  }
  
}
