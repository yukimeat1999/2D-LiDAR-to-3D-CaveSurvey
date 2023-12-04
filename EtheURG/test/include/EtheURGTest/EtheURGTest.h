// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  EtheURGTest.h
 * @brief URG component with Ethernet connection. (test code)
 *
 */
// </rtc-template>

#ifndef ETHEURG_TEST__H
#define ETHEURG_TEST_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

/*!
 * @class EtheURGTest
 * @brief URG component with Ethernet connection.
 *
 */
class EtheURGTest
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  EtheURGTest(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~EtheURGTest() override;

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  // <rtc-template block="activity">
  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   RTC::ReturnCode_t onInitialize() override;

  /***
   *
   * The finalize action (on ALIVE->END transition)
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // RTC::ReturnCode_t onFinalize() override;

  /***
   *
   * The startup action when ExecutionContext startup
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id) override;

  /***
   *
   * The shutdown action when ExecutionContext stop
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id) override;

  /***
   *
   * The activated action (Active state entry action)
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id) override;

  /***
   *
   * The deactivated action (Active state exit action)
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id) override;

  /***
   *
   * The execution action that is invoked periodically
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id) override;

  /***
   *
   * The aborting action when main logic error occurred.
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id) override;

  /***
   *
   * The error action in ERROR state
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // RTC::ReturnCode_t onError(RTC::UniqueId ec_id) override;

  /***
   *
   * The reset action that is invoked resetting
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // RTC::ReturnCode_t onReset(RTC::UniqueId ec_id) override;
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id) override;

  /***
   *
   * The action that is invoked when execution context's rate is changed
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id) override;
  // </rtc-template>

  bool runTest();

 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * Select the connection method to be used.
   * If "Serial" is selected for "connect_flag", set
   * "serial_port_name" and "serial_baud_rate".
   * If "ethe" is selected for "connect_flag", set "ethe_IP_add"
   * and "ethe_port".
   * - Name: connect_flag connect_flag
   * - DefaultValue: Ethe
   */
  std::string m_connect_flag;
  /*!
   * If "Serial" is selected for "connect_flag", set
   * "serial_port_name" and "serial_baud_rate".
   * [Windows]: \\\\.\\COM*
   * - Name: serial_port_name serial_port_name
   * - DefaultValue: \\\\.\\COM1
   */
  std::string m_serial_port_name;
  /*!
   * If "Serial" is selected for "connect_flag", set
   * "serial_port_name" and "serial_baud_rate".
   * - Name: serial_baud_rate serial_baud_rate
   * - DefaultValue: 115200
   */
  int m_serial_baud_rate;
  /*!
   * If "ethe" is selected for "connect_flag", set "ethe_IP_add"
   * and "ethe_port".
   * - Name: ethe_IP_add ethe_IP_add
   * - DefaultValue: 192.168.0.10
   */
  std::string m_ethe_IP_add;
  /*!
   * If "ethe" is selected for "connect_flag", set "ethe_IP_add"
   * and "ethe_port".
   * - Name: ethe_port ethe_port
   * - DefaultValue: 10940
   */
  int m_ethe_port;
  /*!
   * 
   * - Name:  geometry_x
   * - DefaultValue: 0.0
   */
  double m_geometry_x;
  /*!
   * 
   * - Name:  geometry_y
   * - DefaultValue: 0.0
   */
  double m_geometry_y;
  /*!
   * 
   * - Name:  geometry_z
   * - DefaultValue: 0.0
   */
  double m_geometry_z;
  /*!
   * 
   * - Name:  geometry_roll
   * - DefaultValue: 0.0
   */
  double m_geometry_roll;
  /*!
   * 
   * - Name:  geometry_pitch
   * - DefaultValue: 0.0
   */
  double m_geometry_pitch;
  /*!
   * 
   * - Name:  geometry_yaw
   * - DefaultValue: 0.0
   */
  double m_geometry_yaw;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::RangeData m_range;
  /*!
   */
  RTC::InPort<RTC::RangeData> m_rangeIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>

};


extern "C"
{
  DLL_EXPORT void EtheURGTestInit(RTC::Manager* manager);
};

#endif // ETHEURG_TEST_H
