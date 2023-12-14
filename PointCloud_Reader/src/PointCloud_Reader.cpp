// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  PointCloud_Reader.cpp
 * @brief PointCloud Reader
 *
 */
// </rtc-template>

#include "PointCloud_Reader.h"

#define MAX_PC_COMM 170000
bool Read_flag = false;

// Point Cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

// Module specification
// <rtc-template block="module_spec">
#if RTM_MAJOR_VERSION >= 2
static const char* const pointcloud_reader_spec[] =
#else
static const char* pointcloud_reader_spec[] =
#endif
  {
    "implementation_id", "PointCloud_Reader",
    "type_name",         "PointCloud_Reader",
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
PointCloud_Reader::PointCloud_Reader(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_File_PointCloudOut("File_PointCloud", m_File_PointCloud)
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
PointCloud_Reader::~PointCloud_Reader()
{
}



RTC::ReturnCode_t PointCloud_Reader::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  addOutPort("File_PointCloud", m_File_PointCloudOut);

  
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


void RedirectVTKOutputWindow() {
    vtkOutputWindow::SetInstance(nullptr); // デフォルトのvtkOutputWindowを無効にする
}

int PointCloudReader(const std::string& FileName) {
    std::cerr << "[INFO] New PointCloud FileName received!" << std::endl;
    std::cerr << "[INFO] PointCloud File is being read...." << std::endl;
    std::cerr << "[INFO] Read " << FileName << std::endl;

    RedirectVTKOutputWindow();

    // PLYファイルから点群を読み込む
    if (pcl::io::loadPLYFile(FileName, *cloud) == -1) {
        // 読み込みエラー時の処理
        std::cerr << "[Error] File was not found: " << FileName << std::endl;
        return -1;
    }
    std::cerr << "[INFO] Reading of PointCloud File is completed!" << std::endl;
    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << FileName << std::endl;

    return 0;
}


/*
RTC::ReturnCode_t PointCloud_Reader::onFinalize()
{
  return RTC::RTC_OK;
}
*/


//RTC::ReturnCode_t PointCloud_Reader::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t PointCloud_Reader::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t PointCloud_Reader::onActivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Activating PointCloud_Reader...." << std::endl;

    Read_flag = true;
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    std::cerr << "[INFO] Activated PointCloud_Reader OK!" << std::endl;
    return RTC::RTC_OK;
}


RTC::ReturnCode_t PointCloud_Reader::onDeactivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Deactivating PointCloud_Reader...." << std::endl;

    Read_flag = false;

    std::cerr << "[INFO] Deactivated PointCloud_Reader OK!" << std::endl;
    return RTC::RTC_OK;
}


RTC::ReturnCode_t PointCloud_Reader::onExecute(RTC::UniqueId /*ec_id*/)
{
    if (Read_flag) {
        int Read_OK = PointCloudReader(m_FILE_NAME);

        if (Read_OK == 0) {
            // 点群の配列数を取得
            size_t new_PC_size = cloud->width * cloud->height;

            // 点群を分割して送信

            std::cerr << "[INFO] The PointCloud File is divided..." << std::endl;
            int OutputCloud_times = 0;

            // 点群を分割して通信
            for (size_t j = 0; j < new_PC_size; j += MAX_PC_COMM) {
                size_t endIdx = std::min(j + MAX_PC_COMM, new_PC_size);

                // 部分点群の作成
                PointCloudTypes::PointCloud subCloud;

                // 点群データの有無の確認
                subCloud.is_dense = (::CORBA::Boolean)cloud->is_dense;
                // subCloudのサイズを設定
                subCloud.width  = endIdx - j;
                subCloud.height = cloud->height;

                // subCloudのデータを格納するためにバッファを確保
                subCloud.data.length(sizeof(float) * (endIdx - j) * 3);

                // バッファへのポインタを取得
                float* dest = (float*)subCloud.data.get_buffer();

                // cloudからsubCloudにデータをコピー
                for (size_t i = j; i < endIdx; i++) {
                    dest[0] = cloud->points[i].x;
                    dest[1] = cloud->points[i].y;
                    dest[2] = cloud->points[i].z;
                    dest += 3;
                }

                // OutPortデータへ代入
                // 点群データの有無の確認
                m_File_PointCloud.is_dense = subCloud.is_dense;
                // subCloudのサイズを設定
                m_File_PointCloud.width  = subCloud.width;
                m_File_PointCloud.height = subCloud.height;
                m_File_PointCloud.data   = subCloud.data;

                // memory release for RTCPCL_Cloud
                subCloud.data.release();

                OutputCloud_times++;
                std::cerr << "[INFO] " << OutputCloud_times << " \"File_PointCloud\" is output..." << std::endl;

                // OutPortから出力する
                while (!m_File_PointCloudOut.write()) {
                    RTC::DataPortStatusList stat = m_File_PointCloudOut.getStatusList();

                    for (size_t i(0), len(stat.size()); i < len; ++i) {
                        if (stat[i] != RTC::DataPortStatus::PORT_OK) {
                            std::cout << "[ERRO] Error in connector number " << i << " with status: " << RTC::toString(stat[i]) << std::endl;
                            Sleep(0.5 * 1000);
                        }
                    }
                }
                Sleep(0.5 * 1000);
            }
            cloud.reset();
            Read_OK = 1;
            Read_flag = false;
        }
    }
    return RTC::RTC_OK;
}


//RTC::ReturnCode_t PointCloud_Reader::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t PointCloud_Reader::onError(RTC::UniqueId /*ec_id*/)
{
    Read_flag = false;
    return RTC::RTC_OK;
}


RTC::ReturnCode_t PointCloud_Reader::onReset(RTC::UniqueId /*ec_id*/)
{
    Read_flag = false;
    return RTC::RTC_OK;
}


//RTC::ReturnCode_t PointCloud_Reader::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t PointCloud_Reader::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}



extern "C"
{
 
  void PointCloud_ReaderInit(RTC::Manager* manager)
  {
    coil::Properties profile(pointcloud_reader_spec);
    manager->registerFactory(profile,
                             RTC::Create<PointCloud_Reader>,
                             RTC::Delete<PointCloud_Reader>);
  }
  
}
