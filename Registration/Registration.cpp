// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  Registration.cpp
 * @brief Registration comp
 *
 */
// </rtc-template>

#include "Registration.h"

#define MAX_PC_COMM 170000

// PointCloud
pcl::PointCloud<pcl::PointXYZ>::Ptr new_PointCloud_;
pcl::PointCloud<pcl::PointXYZ>::Ptr merge_PointCloud_;

// Time
std::chrono::system_clock::time_point startTime;
std::chrono::system_clock::duration maxTimeDifference = std::chrono::seconds(3);

bool new_PC_flag;
bool new_PC_1th_flag;
bool new_PC_read_flag;
bool new_PC_output_flag;
bool PC_Read_flag;
int  InputCloud_times;

// Module specification
// <rtc-template block="module_spec">
static const char* const registration_spec[] =
  {
    "implementation_id", "Registration",
    "type_name",         "Registration",
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
Registration::Registration(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_new_PointCloudIn("new_PointCloud", m_new_PointCloud),
    m_merge_PointCloudOut("merge_PointCloud", m_merge_PointCloud),
    m_LocalizationOut("Localization", m_Localization)
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
Registration::~Registration()
{
}

pcl::PointCloud<pcl::PointXYZ>::Ptr boxfilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr src) {

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_ds(new pcl::PointCloud<pcl::PointXYZ>);

    vg.setInputCloud(src);
    vg.filter(*pcd_ds);

    return pcd_ds;
}



RTC::ReturnCode_t Registration::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("new_PointCloud", m_new_PointCloudIn);
  
  // Set OutPort buffer
  addOutPort("merge_PointCloud", m_merge_PointCloudOut);
  addOutPort("Localization", m_LocalizationOut);

  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>

  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Registration::onFinalize()
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t Registration::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t Registration::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t Registration::onActivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Activating Registration...." << std::endl; 
    
    new_PC_flag        = true;
    new_PC_1th_flag    = true;
    new_PC_read_flag   = false;
    new_PC_output_flag = false;
    PC_Read_flag       = false;

    InputCloud_times = 0;

    m_Localization.data.position.x = 0.0;
    m_Localization.data.position.y = 0.0;
    m_Localization.data.position.z = 0.0;

    new_PointCloud_   = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    merge_PointCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    std::cerr << "[INFO] Activated Registration OK!" << std::endl;

    return RTC::RTC_OK;
}


RTC::ReturnCode_t Registration::onDeactivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Deactivating Registration...." << std::endl; 
    
    PC_Read_flag = false;
    new_PointCloud_.reset();
    merge_PointCloud_.reset();

    std::cerr << "[INFO] Deactivated Registration OK!" << std::endl;

    return RTC::RTC_OK;
}

void print4x4Matrix(const Eigen::Matrix4d& matrix)
{
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}


RTC::ReturnCode_t Registration::onExecute(RTC::UniqueId /*ec_id*/)
{
    // 点群の読み込み /////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    if (new_PC_flag) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // new_PointCloudからデータを読み込む
        if (m_new_PointCloudIn.isNew()) {
            m_new_PointCloudIn.read();
            if (new_PC_1th_flag) {
                std::cerr << "[INFO] \"new_PointCloud\" received!" << std::endl;
                new_PC_1th_flag = false;
            }
            InputCloud_times++;
            std::cerr << "[INFO] " << InputCloud_times << " \"new_PointCloud\" is being read..." << std::endl;

            // RTCPCLからPCLに型変換
            cloud->is_dense = m_new_PointCloud.is_dense;
            cloud->points.resize(m_new_PointCloud.width * m_new_PointCloud.height);
            float* src = (float*)m_new_PointCloud.data.get_buffer();
            for (size_t i = 0; i < cloud->points.size(); i++) {
                cloud->points[i].x = src[0];
                cloud->points[i].y = src[1];
                cloud->points[i].z = src[2];
                src += 3;
            }
            new_PC_read_flag = true;
        }

        // 現在時刻を取得
        auto currentTime = std::chrono::system_clock::now();

        if (new_PC_read_flag) {
            // 最新データ受信時間の更新
            startTime = currentTime;
        }
        auto timeDifference = currentTime - startTime;

        // 一定時間以内に次の点群が受信された場合、new_PointCloud_に追加
        if (timeDifference <= maxTimeDifference) {
            *new_PointCloud_ += *cloud;
            new_PC_read_flag   = false;
            new_PC_output_flag = true;
        }
        else if (new_PC_output_flag) {
            // 指定した時間を超えたらループを終了
            new_PC_flag        = false;
            new_PC_output_flag = false;
            PC_Read_flag       = true;
        }
        cloud.reset();

        if (PC_Read_flag) {
            // 処理の追加 /////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setLeafSize(0.1f, 0.1f, 0.1f);
            static pcl::PointCloud<pcl::PointXYZ>::Ptr ds_input(new pcl::PointCloud<pcl::PointXYZ>);
            vg.setInputCloud(new_PointCloud_);
            vg.filter(*ds_input);

            if (merge_PointCloud_->size() == 0) {
                merge_PointCloud_->resize(ds_input->size());
                for (int i = 0; i < ds_input->size(); i++) {
                    merge_PointCloud_->points[i].x = ds_input->points[i].x;
                    merge_PointCloud_->points[i].y = ds_input->points[i].y;
                    merge_PointCloud_->points[i].z = ds_input->points[i].z;
                }
                
                new_PC_flag        = true;
                new_PC_1th_flag    = true;
                new_PC_read_flag   = false;
                new_PC_output_flag = false;
                PC_Read_flag       = false;

                InputCloud_times = 0;
                new_PointCloud_.reset();
                new_PointCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
                return RTC::RTC_OK;
            }

            // x座標を指定した距離だけ移動する
            int translation_distance = 10;
            for (size_t i = 0; i < ds_input->points.size(); ++i) {
                ds_input->points[i].x += translation_distance;
            }

            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            static pcl::PointCloud<pcl::PointXYZ>::Ptr final_res(new pcl::PointCloud<pcl::PointXYZ>);
            int Process_OK = 0; // 関数等からの戻り値によって点群の出力を開始
            icp.setInputSource(ds_input);
            icp.setInputTarget(merge_PointCloud_);
            icp.setMaxCorrespondenceDistance(30); // 最大対応距離の設定
            icp.setMaximumIterations(50);         // 最大反復回数の設定 (基準1)
            icp.setTransformationEpsilon(1e-8);   // 変換イプシロンの設定 (基準 2)
            //icp.setEuclideanFitnessEpsilon(1);    // ユークリッド距離差イプシロンの設定(基準3)
            icp.align(*final_res);                // アライメントを実行する
            Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
            transformation_matrix = icp.getFinalTransformation().cast<double>();
            print4x4Matrix(transformation_matrix);
            m_Localization.data.position.x = transformation_matrix(0, 3) - translation_distance;
            m_Localization.data.position.y = transformation_matrix(1, 3);
            m_Localization.data.position.z = transformation_matrix(2, 3);

            // merge_PointCloud_のサイズを拡張して、final_resのデータを追加
            int current_size = merge_PointCloud_->size();
            merge_PointCloud_->resize(current_size + final_res->size());
            for (int i = current_size; i < merge_PointCloud_->size(); i++) {
                merge_PointCloud_->points[i].x = final_res->points[i- current_size].x;
                merge_PointCloud_->points[i].y = final_res->points[i- current_size].y;
                merge_PointCloud_->points[i].z = final_res->points[i- current_size].z;
            }

            // 点群の出力 /////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////

            if (Process_OK == 0) {
                // 点群の配列数を取得
                size_t merge_PC_size = merge_PointCloud_->width * merge_PointCloud_->height;

                // 点群を分割して送信

                std::cerr << "[INFO] \"merge_PointCloud\" is divided..."<< merge_PointCloud_->size() << std::endl;
                int OutputCloud_times = 0;

                // 点群を分割して通信
                for (size_t j = 0; j < merge_PC_size; j += MAX_PC_COMM) {
                    size_t endIdx = std::min(j + MAX_PC_COMM, merge_PC_size);

                    // 部分点群の作成
                    PointCloudTypes::PointCloud subCloud;

                    // 点群データの有無の確認
                    subCloud.is_dense = (::CORBA::Boolean)merge_PointCloud_->is_dense;
                    // subCloudのサイズを設定
                    subCloud.width  = endIdx - j;
                    subCloud.height = merge_PointCloud_->height;

                    // subCloudのデータを格納するためにバッファを確保
                    subCloud.data.length(sizeof(float) * (endIdx - j) * 3);

                    // バッファへのポインタを取得
                    float* dest = (float*)subCloud.data.get_buffer();

                    // merge_PointCloud_からsubCloudにデータをコピー
                    for (size_t i = j; i < endIdx; i++) {
                        dest[0] = merge_PointCloud_->points[i].x;
                        dest[1] = merge_PointCloud_->points[i].y;
                        dest[2] = merge_PointCloud_->points[i].z;
                        dest += 3;
                    }

                    // OutPortデータへ代入
                    // 点群データの有無の確認
                    m_merge_PointCloud.is_dense = subCloud.is_dense;
                    // subCloudのサイズを設定
                    m_merge_PointCloud.width  = subCloud.width;
                    m_merge_PointCloud.height = subCloud.height;
                    m_merge_PointCloud.data   = subCloud.data;

                    // memory release for RTCPCL_Cloud
                    subCloud.data.release();

                    OutputCloud_times++;
                    std::cerr << "[INFO] " << OutputCloud_times << " \"merge_PointCloud\" is output..." << std::endl;

                    // OutPortから出力する
                    while (!m_merge_PointCloudOut.write()) {
                        RTC::DataPortStatusList stat = m_merge_PointCloudOut.getStatusList();

                        for (size_t i(0), len(stat.size()); i < len; ++i) {
                            if (stat[i] != RTC::DataPortStatus::PORT_OK) {
                                std::cout << "[ERRO] Error in connector number " << i << " with status: " << RTC::toString(stat[i]) << std::endl;
                                Sleep(0.5 * 1000);
                            }
                        }
                    }
                    Sleep(0.5 * 1000);
                }

                Process_OK = 1;

                // Localizationの出力 /////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////////////////
               
                m_LocalizationOut.write();

                new_PC_flag        = true;
                new_PC_1th_flag    = true;
                new_PC_read_flag   = false;
                new_PC_output_flag = false;
                PC_Read_flag       = false;

                InputCloud_times = 0;
                new_PointCloud_.reset();
                new_PointCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            }
        }
    }
    return RTC::RTC_OK;
}


//RTC::ReturnCode_t Registration::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t Registration::onError(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Registration::onReset(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t Registration::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t Registration::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}



extern "C"
{
 
  void RegistrationInit(RTC::Manager* manager)
  {
    coil::Properties profile(registration_spec);
    manager->registerFactory(profile,
                             RTC::Create<Registration>,
                             RTC::Delete<Registration>);
  }
  
}
