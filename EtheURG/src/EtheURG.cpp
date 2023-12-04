// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  EtheURG.cpp
 * @brief URG component with Ethernet connection.
 *
 */
// </rtc-template>

#include "EtheURG.h"

// Variable declaration
urg_t urg;
int ret;
int min_step;
int max_step;
double minAngle;
double maxAngle;
int encoder_resolution_;
double AngularRes;
int min_dis;
int max_dis;
double frequency_;
long* length_data;         // range array
int length_data_size;      // Number of range array
int scan_times = 1;        // Number of scans
int skip_scan = 0;         // Number of Thinning between Steps
bool conected_flg = false;

// Module specification
// <rtc-template block="module_spec">
static const char* const etheurg_spec[] =
  {
    "implementation_id", "EtheURG",
    "type_name",         "EtheURG",
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
EtheURG::EtheURG(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_rangeOut("range", m_range)
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
EtheURG::~EtheURG()
{
}



RTC::ReturnCode_t EtheURG::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  addOutPort("range", m_rangeOut);

  
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

  conected_flg = false;
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t EtheURG::onFinalize()
{
  return RTC::RTC_OK;
}
*/


//RTC::ReturnCode_t EtheURG::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t EtheURG::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t EtheURG::onActivated(RTC::UniqueId /*ec_id*/)
{    
    std::cout << "[Info] The selected connection method is \" " << m_connect_flag << " \"." << std::endl;
    ret = 0;

    // Make connections to the sensor
    if (m_connect_flag == "Serial") {
        conected_flg = true;
        ret = urg_open(&urg, URG_SERIAL, m_serial_port_name.c_str(), m_serial_baud_rate);
        std::cout << "[Info] Starting URG in (Serial Port Name: " << m_serial_port_name <<
            ", Serial Baud Rate: " << m_serial_baud_rate << ")" << std::endl;
    }
    else if (m_connect_flag == "Ethe") {
        conected_flg = true;
        ret = urg_open(&urg, URG_ETHERNET, m_ethe_IP_add.c_str(), m_ethe_port);
        std::cout << "[Info] Starting URG in (Ethernet IP Address: " << m_ethe_IP_add <<
            ", Ethernet Port Number: " << m_ethe_port << ")" << std::endl;
    }

    // Set Error
    if (ret != 0) {
        conected_flg = false;
        RTC_INFO(("[ERRO] URG is not founded!"));
        return RTC::RTC_ERROR;
    }

    std::cout << "[Info] Waiting...." << std::endl;
    std::cout << "[Info] Starting RTC..." << std::endl;

    // Allocate space for data reception
    length_data = (long*)malloc(sizeof(long) * urg_max_data_size(&urg));

    // Get minimum and maximum steps
    urg_step_min_max(&urg, &min_step, &max_step);
    std::cout << "[Info] Step         : [" << min_step << ", " << max_step << "]" << std::endl;

    // Convert step to rad
    minAngle = urg_step2rad(&urg, min_step);
    maxAngle = urg_step2rad(&urg, max_step);
    encoder_resolution_ = urg.area_resolution;
    AngularRes = 2 * M_PI / encoder_resolution_;//(360.0 / encoder_resolution_) * (M_PI / 180.0);

    // Set minimum and maximum angles
    std::cout << "[Info] Min Angle    : " <<  minAngle  << "\t\t[rad]" << std::endl;
    std::cout << "[Info] Max Angle    :  " << maxAngle  << "\t\t[rad]" << std::endl;
    std::cout << "[Info] Angular Res  : " << AngularRes << "\t[rad]" << std::endl;
    m_range.config.minAngle   = minAngle;
    m_range.config.maxAngle   = maxAngle;
    m_range.config.angularRes = AngularRes;

    // Sst minimum and maximum angles
    min_dis = urg.min_distance;
    max_dis = urg.max_distance;
    std::cout << "[Info] Min Range    : " << min_dis << "\t\t[mm]" << std::endl;
    std::cout << "[Info] Max Range    : " << max_dis << "\t\t[mm]" << std::endl;
    m_range.config.minRange = min_dis;
    m_range.config.maxRange = max_dis;

    // Scan Frequency
    frequency_ = (double)(1.0 / (urg.scan_usec / 1000000.0));
    std::cout << "[Info] Frequency    : " << frequency_ << "\t\t[Hz]" << std::endl;
    m_range.config.frequency = frequency_;

    // Set the sensor geometry
    std::cout << "[Info] Offset x     : " << m_geometry_x     << "\t\t\t[mm]"     << std::endl;
    std::cout << "[Info] Offset y     : " << m_geometry_y     << "\t\t\t[mm]"     << std::endl;
    std::cout << "[Info] Offset z     : " << m_geometry_z     << "\t\t\t[mm]"     << std::endl;
    std::cout << "[Info] Offset roll  : " << m_geometry_roll  << "\t\t\t[deg]" << std::endl;
    std::cout << "[Info] Offset pitch : " << m_geometry_pitch << "\t\t\t[deg]" << std::endl;
    std::cout << "[Info] Offset yaw   : " << m_geometry_yaw   << "\t\t\t[deg]" << std::endl;
    m_range.geometry.geometry.pose.position.x = m_geometry_x;
    m_range.geometry.geometry.pose.position.y = m_geometry_y;
    m_range.geometry.geometry.pose.position.z = m_geometry_z;
    m_range.geometry.geometry.pose.orientation.r = m_geometry_roll;
    m_range.geometry.geometry.pose.orientation.p = m_geometry_pitch;
    m_range.geometry.geometry.pose.orientation.y = m_geometry_yaw;

    m_range.config.rangeRes = NULL;

    return RTC::RTC_OK;
}


RTC::ReturnCode_t EtheURG::onDeactivated(RTC::UniqueId /*ec_id*/)
{
    // Sensor termination process
    RTC_INFO(("[Info] Deactivating RTC...."));
    if (conected_flg) {
        conected_flg = false;
        // Close the connection to the sensor
        urg_close(&urg);
    }
    RTC_INFO(("       OK!"));
    return RTC::RTC_OK;
}


RTC::ReturnCode_t EtheURG::onExecute(RTC::UniqueId /*ec_id*/)
{
    // Start measuring distance data
    ret = urg_start_measurement(&urg, URG_DISTANCE, scan_times, skip_scan, 1);

    // Get distance data from the sensor
    length_data_size = urg_get_distance(&urg, length_data, NULL);

    // Set the array size
    if (length_data_size != m_range.ranges.length()) {
        m_range.ranges.length(length_data_size);
    }

    // Assign the acquired distance data in order
    for (int i = 0; i < length_data_size; i++) {
        m_range.ranges[i] = length_data[i];
        //std::cout << i << " : " <<length_data[i] << ", " << std::endl;
    }
    //std::cout << " //////////////////////////// " << std::endl;

    // Output to OutPort
    m_rangeOut.write();
    
    return RTC::RTC_OK;
}


//RTC::ReturnCode_t EtheURG::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t EtheURG::onError(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t EtheURG::onReset(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t EtheURG::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t EtheURG::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}



extern "C"
{
 
  void EtheURGInit(RTC::Manager* manager)
  {
    coil::Properties profile(etheurg_spec);
    manager->registerFactory(profile,
                             RTC::Create<EtheURG>,
                             RTC::Delete<EtheURG>);
  }
  
}
