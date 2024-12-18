// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  Analyses.cpp
 * @brief Analyses comp
 *
 */
// </rtc-template>

#include "Analyses.h"
#include "gng.h"

#define MAX_PC_COMM 170000

// PointCloud
pcl::PointCloud<pcl::PointXYZ>::Ptr merge_PointCloud_;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr analyses_PointCloud_;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_analyses_PointCloud_;

// Time
std::chrono::system_clock::time_point startTime;
std::chrono::system_clock::duration maxTimeDifference = std::chrono::seconds(3);

bool merge_PC_flag;
bool merge_PC_1th_flag;
bool merge_PC_read_flag;
bool merge_PC_output_flag;
bool PC_Read_flag;
int  InputCloud_times;

struct gng* gng_net;
double **pointcloud_data;

int epoch;
int MaxEpoch;
int epoch_times;
int edges_num;

// Module specification
// <rtc-template block="module_spec">
static const char* const analyses_spec[] =
  {
    "implementation_id", "Analyses",
    "type_name",         "Analyses",
    "description",       "Analyses comp",
    "version",           "1.0.0",
    "vendor",            "Y. Fujii",
    "category",          "Analyses",
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
Analyses::Analyses(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_merge_PointCloudIn("merge_PointCloud", m_merge_PointCloud),
    m_analyses_PointCloudOut("analyses_PointCloud", m_analyses_PointCloud)
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
Analyses::~Analyses()
{
}

void RedirectVTKOutputWindow() {
    // デフォルトのvtkOutputWindowを無効にする
    vtkOutputWindow::SetInstance(nullptr);
    vtkObject::GlobalWarningDisplayOff();
}

// 点群とGNGのノードとエッジを表示
void GNGView(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1,
    struct gng* net,
    const std::string& PointName) {
    RedirectVTKOutputWindow();
    // Create viewer name
    std::string ViewerName = PointName + " Viewer";
    // Create PCL viewer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(ViewerName));
    std::cerr << "[INFO] " << ViewerName << " is opened." << std::endl;

    int c0 = 0;
    // 古いノードとエッジを削除
    viewer->removeAllShapes(c0);
    viewer->removeAllPointClouds(c0);
    // Setting viewport borders
    viewer->createViewPort(0.0, 0.0, 1.0, 1.0, c0);
    // Set background color
    viewer->setBackgroundColor(1.0, 1.0, 1.0, c0);
    // Delete previous PointCloud data
    viewer->removePointCloud("cloud1", c0);
    // Add the loaded PointCloud to the viewer
    viewer->addPointCloud(cloud1, "cloud1", c0);
    // Set point size
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "cloud1");
    viewer->addCoordinateSystem(1.0, "coordinate system", 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0.0, 0.0, 100.0, // カメラ位置 (x, y, z)
                              0.0, 0.0, 0.0,   // 注視点 (x, y, z)
                              0.0, 1.0, 0.0);  // カメラの上方向ベクトル (x, y, z)

    // GNGのノードを点として描画
    if (net->node_n > 0) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr nodes(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (int i = 0; i < net->node_n; i++) {
            pcl::PointXYZRGB point;
            point.x = net->node[i][0];
            point.y = net->node[i][1];
            point.z = net->node[i][2];
            point.r = static_cast<uint8_t>(net->node[i][3] * 255);
            point.g = static_cast<uint8_t>(net->node[i][4] * 255);
            point.b = static_cast<uint8_t>(net->node[i][5] * 255);
            nodes->points.push_back(point);
        }
        viewer->addPointCloud(nodes, "nodes");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "nodes");
    }
    else {
        std::cerr << "[INFO] No nodes to display." << std::endl;
    }

    int edges = 0;
    // GNGのエッジを線として描画
    bool edges_exist = false;
    for (int i = 0; i < net->node_n; i++) {
        for (int j = 0; j < net->node_n; j++) {
            if (net->edge[i][j] == 1 && net->edge[j][i] == 1) {
                edges_exist = true;
                pcl::PointXYZ p1, p2;
                p1.x = net->node[i][0];
                p1.y = net->node[i][1];
                p1.z = net->node[i][2];
                p2.x = net->node[j][0];
                p2.y = net->node[j][1];
                p2.z = net->node[j][2];
                edges++;

                std::stringstream ss;
                ss << "line" << i << "_" << j;
                viewer->addLine(p1, p2,
                    net->node[i][3] * 255.0,
                    net->node[i][4] * 255.0,
                    net->node[i][5] * 255.0,
                    ss.str(), c0);
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, ss.str());
            }
        }
    } if (!edges_exist) {
        std::cerr << "[INFO] No edges to display." << std::endl;
    }
    edges_num = edges;

    // Start the viewer
    std::cerr << "[INFO] " << PointName << " is shown." << std::endl;
    std::cerr << "[INFO] Please window closed..." << std::endl;
    viewer->spin();
    // Update viewer_closed to true when the window is closed
}

RTC::ReturnCode_t Analyses::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("merge_PointCloud", m_merge_PointCloudIn);
  
  // Set OutPort buffer
  addOutPort("analyses_PointCloud", m_analyses_PointCloudOut);

  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>

  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Analyses::onFinalize()
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t Analyses::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t Analyses::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t Analyses::onActivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Activating Analyses...." << std::endl;
    
    merge_PC_flag        = true;
    merge_PC_1th_flag    = true;
    merge_PC_read_flag   = false;
    merge_PC_output_flag = false;
    PC_Read_flag         = false;

    InputCloud_times = 0;

    epoch = 1000;
    MaxEpoch = 10000;
    epoch_times = 0;
    edges_num = 0;

    merge_PointCloud_    = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    analyses_PointCloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (gng_net == NULL) {
        gng_net = init_gng(); //GNGの初期化
        gng_net->weight[0] = 1;//x
        gng_net->weight[1] = 1;//y
        gng_net->weight[2] = 1;//z
        gng_net->weight[3] = 0;//r
        gng_net->weight[4] = 0;//g
        gng_net->weight[5] = 0;//b
    }

    if(pointcloud_data == NULL){
        pointcloud_data = malloc2d_double(3000000, LDIM);
    }

    std::cerr << "[INFO] Activated Analyses OK!" << std::endl;

    return RTC::RTC_OK;
}


RTC::ReturnCode_t Analyses::onDeactivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Deactivating Analyses...." << std::endl;
    
    PC_Read_flag = false;
    merge_PointCloud_.reset();

    std::cerr << "[INFO] Deactivated Analyses OK!" << std::endl;

    return RTC::RTC_OK;
}


RTC::ReturnCode_t Analyses::onExecute(RTC::UniqueId /*ec_id*/)
{
    // 点群の読み込み /////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    if (merge_PC_flag) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // merge_PointCloudからデータを読み込む
        if (m_merge_PointCloudIn.isNew()) {
            m_merge_PointCloudIn.read();
            if (merge_PC_1th_flag) {
                std::cerr << "[INFO] \"merge_PointCloud\" received!" << std::endl;
                merge_PC_1th_flag = false;
            }
            InputCloud_times++;
            std::cerr << "[INFO] " << InputCloud_times << " \"merge_PointCloud\" is being read..." << std::endl;

            // RTCPCLからPCLに型変換
            cloud->is_dense = m_merge_PointCloud.is_dense;
            cloud->points.resize(m_merge_PointCloud.width * m_merge_PointCloud.height);
            float* src = (float*)m_merge_PointCloud.data.get_buffer();
            for (size_t i = 0; i < cloud->points.size(); i++) {
                cloud->points[i].x = src[0];
                cloud->points[i].y = src[1];
                cloud->points[i].z = src[2];
                src += 3;
            }
            merge_PC_read_flag = true;
        }

        // 現在時刻を取得
        auto currentTime = std::chrono::system_clock::now();

        if (merge_PC_read_flag) {
            // 最新データ受信時間の更新
            startTime = currentTime;
        }
        auto timeDifference = currentTime - startTime;

        // 一定時間以内に次の点群が受信された場合、merge_PointCloud_に追加
        if (timeDifference <= maxTimeDifference) {
            *merge_PointCloud_ += *cloud;
            merge_PC_read_flag = false;
            merge_PC_output_flag = true;
        }
        else if (merge_PC_output_flag) {
            // 指定した時間を超えたらループを終了
            merge_PC_flag = false;
            merge_PC_output_flag = false;
            PC_Read_flag = true;
        }
        cloud.reset();

        if (PC_Read_flag) {
            // 処理の追加 /////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
           
            int datasize = 0;
            for (size_t i = 0; i < merge_PointCloud_->points.size(); i++) {
                if (merge_PointCloud_->points[i].x == 0
                    && merge_PointCloud_->points[i].y == 0
                    && merge_PointCloud_->points[i].z == 0) {
                    continue;
                }

                pointcloud_data[datasize][0] = merge_PointCloud_->points[i].x;
                pointcloud_data[datasize][1] = merge_PointCloud_->points[i].y;
                pointcloud_data[datasize++][2] = merge_PointCloud_->points[i].z;
            }
            for (int i = 0; i < gng_net->node_n; i++) {
                gng_net->edge_ct[i] = 0;
                for (int j = 0; j < gng_net->node_n; j++) {
                    gng_net->edge[i][j] = 0;
                }
            }
            gng_net->edge[1][0] = 1;
            gng_net->edge[0][1] = 1;
            gng_net->edge_ct[0] = 1;
            gng_net->edge_ct[1] = 1;
            gng_net->node_n = 2;

            printf("# of data is %d\n", datasize);
            int maxnode = GNGN;
            if (datasize < GNGN) {
                maxnode = datasize / 2;
            }
			
            do {
                do {
                    for (int i = 0; i < epoch; i++) {
                        gng_main(gng_net, pointcloud_data, datasize);
                        epoch_times++;
                    }
                    if (epoch_times >= MaxEpoch) {
                        break;
                    }
                } while (gng_net->node_n < GNGN - 3);
                break;
            } while (epoch_times < MaxEpoch);

            filtered_analyses_PointCloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
            int a_currentsize = analyses_PointCloud_->size();
            analyses_PointCloud_->resize(a_currentsize + gng_net->node_n);
            for (int i = 0; i < gng_net->node_n; i++) {
                analyses_PointCloud_->points[a_currentsize + i].x = gng_net->node[i][0];
                analyses_PointCloud_->points[a_currentsize + i].y = gng_net->node[i][1];
                analyses_PointCloud_->points[a_currentsize + i].z = gng_net->node[i][2];
                analyses_PointCloud_->points[a_currentsize + i].r = 0;
                analyses_PointCloud_->points[a_currentsize + i].g = 0;
                analyses_PointCloud_->points[a_currentsize + i].b = 0;
                if (gng_net->node[i][7] != -10.0) {
                    double threshold = 0.1;
                    analyses_PointCloud_->points[a_currentsize + i].r = (int)(gng_net->node[i][7] / threshold * 255.0);

                    if (analyses_PointCloud_->points[a_currentsize + i].r > 100) {
                        pcl::PointXYZRGB point;
                        point.x = gng_net->node[i][0];
                        point.y = gng_net->node[i][1];
                        point.z = gng_net->node[i][2];
                        point.r = (int)(gng_net->node[i][7] / threshold * 255.0);
                        point.g = 0;
                        point.b = 0;
                        if (analyses_PointCloud_->points[a_currentsize + i].r > 255) {
                            analyses_PointCloud_->points[a_currentsize + i].r = 255;
                            point.r = 255;
                        }
                        filtered_analyses_PointCloud_->points.push_back(point);
                    }
                }
                else {
                    analyses_PointCloud_->points[a_currentsize + i].g = 255;
                }
            }
            filtered_analyses_PointCloud_->width = static_cast<uint32_t>(filtered_analyses_PointCloud_->points.size());
            filtered_analyses_PointCloud_->height = 1;
            filtered_analyses_PointCloud_->is_dense = true;

            pcl::io::savePLYFile("res.ply", *analyses_PointCloud_);
            analyses_PointCloud_.reset();
            analyses_PointCloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

            int f_currentsize = filtered_analyses_PointCloud_->size();
            analyses_PointCloud_->resize(f_currentsize);
            for (int i = 0; i < f_currentsize; i++) {
                analyses_PointCloud_->points[i].x = filtered_analyses_PointCloud_->points[i].x;
                analyses_PointCloud_->points[i].y = filtered_analyses_PointCloud_->points[i].y;
                analyses_PointCloud_->points[i].z = filtered_analyses_PointCloud_->points[i].z;
            }

            // 出力する点群が無い場合，データポートから0を出力
            if (analyses_PointCloud_->size() == 0) {
                analyses_PointCloud_->resize(1);
                analyses_PointCloud_->points[0].x = NULL;
                analyses_PointCloud_->points[0].y = NULL;
                analyses_PointCloud_->points[0].z = NULL;
            }

	    // GNGView(filtered_analyses_PointCloud_, gng_net, "GNG");
            std::cerr << "[INFO] Current number of epochs: " << epoch_times << std::endl;
            std::cerr << "[INFO] Current number of Nodes: "  << gng_net->node_n + 1 << std::endl;
            std::cerr << "[INFO] Current number of Edges: "  << edges_num << std::endl;

            int Process_OK = 0; // 関数等からの戻り値によって点群の出力を開始

            // 点群の出力 /////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////

            if (Process_OK == 0) {
                // 点群の配列数を取得
                size_t analyses_PC_size = analyses_PointCloud_->width * analyses_PointCloud_->height;

                // 点群を分割して送信
                std::cerr << "[INFO] \"analyses_PointCloud\" is divided..." << analyses_PointCloud_->size() << std::endl;
                int OutputCloud_times = 0;

                // 点群を分割して通信
                for (size_t j = 0; j < analyses_PC_size; j += MAX_PC_COMM) {
                    size_t endIdx = std::min(j + MAX_PC_COMM, analyses_PC_size);

                    // 部分点群の作成
                    PointCloudTypes::PointCloud subCloud;

                    // 点群データの有無の確認
                    subCloud.is_dense = (::CORBA::Boolean)analyses_PointCloud_->is_dense;
                    // subCloudのサイズを設定
                    subCloud.width  = endIdx - j;
                    subCloud.height = analyses_PointCloud_->height;

                    // subCloudのデータを格納するためにバッファを確保
                    subCloud.data.length(sizeof(float) * (endIdx - j) * 3);

                    // バッファへのポインタを取得
                    float* dest = (float*)subCloud.data.get_buffer();

                    // analyses_PointCloud_からsubCloudにデータをコピー
                    for (size_t i = j; i < endIdx; i++) {
                        dest[0] = analyses_PointCloud_->points[i].x;
                        dest[1] = analyses_PointCloud_->points[i].y;
                        dest[2] = analyses_PointCloud_->points[i].z;
                        dest += 3;
                    }

                    // OutPortデータへ代入
                    // 点群データの有無の確認
                    m_analyses_PointCloud.is_dense = subCloud.is_dense;
                    // subCloudのサイズを設定
                    m_analyses_PointCloud.width  = subCloud.width;
                    m_analyses_PointCloud.height = subCloud.height;
                    m_analyses_PointCloud.data   = subCloud.data;

                    // memory release for RTCPCL_Cloud
                    subCloud.data.release();

                    OutputCloud_times++;
                    std::cerr << "[INFO] " << OutputCloud_times << " \"analyses_PointCloud\" is output..." << std::endl;

                    // OutPortから出力する
                    while (!m_analyses_PointCloudOut.write()) {
                        RTC::DataPortStatusList stat = m_analyses_PointCloudOut.getStatusList();

                        for (size_t i(0), len(stat.size()); i < len; ++i) {
                            if (stat[i] != RTC::DataPortStatus::PORT_OK) {
                                std::cout << "[ERRO] Error in connector number " << i << " with status: " << RTC::toString(stat[i]) << std::endl;
                                Sleep(0.5 * 1000);
                            }
                        }
                    }
                    Sleep(0.5 * 1000);
                }
                analyses_PointCloud_.reset();
                analyses_PointCloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

                Process_OK = 1;

                merge_PC_flag        = true;
                merge_PC_1th_flag    = true;
                merge_PC_read_flag   = false;
                merge_PC_output_flag = false;
                PC_Read_flag         = false;

                InputCloud_times = 0;
                merge_PointCloud_.reset();
                filtered_analyses_PointCloud_.reset();
                merge_PointCloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            }
        }
    }
    return RTC::RTC_OK;
}


//RTC::ReturnCode_t Analyses::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t Analyses::onError(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Analyses::onReset(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t Analyses::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t Analyses::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}



extern "C"
{
 
  void AnalysesInit(RTC::Manager* manager)
  {
    coil::Properties profile(analyses_spec);
    manager->registerFactory(profile,
                             RTC::Create<Analyses>,
                             RTC::Delete<Analyses>);
  }
  
}
