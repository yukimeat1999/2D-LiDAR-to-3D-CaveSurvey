// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  MeasurementSystem.cpp
 * @brief Measurement System for 3D point cloud data creation component using 2D LiDAR and Dynamixel.
 *
 */
// </rtc-template>

#include "MeasurementSystem.h"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0    // Default Protocol version of DYNAMIXEL X series.

// Default setting
int BAUDRATE;           // Default Baudrate of DYNAMIXEL X series

// モータの現在位置を取得し、安全な運転をするかどうか選択する
#define CHECK 0         // 0: 現在位置を確認しない  1: 現在位置を確認する
#define START_POS 50    // モータの初期位置
#define RATE 1          // 1回毎のモータの移動数
#define SYS_ERR_WIDE 20 // 指示したモータ位置の許容誤差

dynamixel::PortHandler* portHandler;
dynamixel::PacketHandler* packetHandler;
uint8_t dxl_id = 1;     // モータのID
int dxl_comm_result = COMM_TX_FAIL;
uint8_t  dxl_error = 0;
uint32_t goal_position = 0;
uint32_t present_position = 0;

double MaxRange;
int motor_pos_;
double motor_deg_;
int encoder_resolution_;
bool turn_OK    = false;
bool conected   = false;
bool conected_l = false;
bool conected_m = false;
bool start_flag = false;
bool fin_flag   = false;

// 移動と回転のパラメータ
//pcl::PointXYZ point;
double moveX = 0;
double moveY = 0;
double moveZ = 0;
double roll  = 0;
double pitch = 0;
double yaw   = 0;

// 複数回スキャンする場合のスキャン回数を記録する変数
int Times = 0;
int Fin_Times = 0;

#define MAX_PC_COMM 170000

// Declarations about point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud;

void printTest(int a) {
    std::cerr << "////////////////////////////////////////" << std::endl;
    std::cerr << "[INFO] Print variable : " << a << std::endl;
    std::cerr << "conected_l: " << conected_l << std::endl;
    std::cerr << "conected_m: " << conected_m << std::endl;
    std::cerr << "conected  : " << conected << std::endl;
    std::cerr << std::endl;
    std::cerr << "start_flag: " << start_flag << std::endl;
    std::cerr << "turn_OK   : " << turn_OK << std::endl;
    std::cerr << "fin_flag  : " << fin_flag << std::endl;
    std::cerr << std::endl;
    std::cerr << "Times     : " << Times << std::endl;
    std::cerr << "Fin_Times : " << Fin_Times << std::endl;
    std::cerr << "////////////////////////////////////////" << std::endl;
}

// Module specification
// <rtc-template block="module_spec">
static const char* const measurementsystem_spec[] =
  {
    "implementation_id", "MeasurementSystem",
    "type_name",         "MeasurementSystem",
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
MeasurementSystem::MeasurementSystem(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_rangeIn("range", m_range),
    m_new_PointCloudOut("new_PointCloud", m_new_PointCloud)
    // </rtc-template>
{
    m_BAUDRATE = 0;
    m_encoder_resolution_ = 0;
}

/*!
 * @brief destructor
 */
MeasurementSystem::~MeasurementSystem()
{
}



RTC::ReturnCode_t MeasurementSystem::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("range", m_rangeIn);
  
  // Set OutPort buffer
  addOutPort("new_PointCloud", m_new_PointCloudOut);

  
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


int setupDynamixel(uint8_t dxl_id)
{
    std::cerr << "[INFO] setupDynamixel" << std::endl;
    // Use Position Control Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_OPERATING_MODE,
        3,
        &dxl_error
    );

    // Enable Torque of DYNAMIXEL
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        1,
        &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
        conected_m = false;
        conected   = false;
        std::cerr << "[ERRO] Failed to enable torque!\a" << std::endl;
        return 0;
    }
    else {
        conected_m = true;
        std::cerr << "[INFO] Succeeded to enable torque." << std::endl;
        return 1;
    }
}

void updateMotorPosition(int& motor_pos_temp) {
    std::cerr << "[INFO] updateMotorPosition" << std::endl;

    if ((!start_flag) || turn_OK) {
        // Start and Finish
        motor_pos_temp = START_POS;
    }
    else if (!turn_OK) {
        // Update the motor position (0 to 4095) and motor_deg_ (0.0 to 360.0)
        motor_pos_temp = (motor_pos_temp + RATE) % encoder_resolution_;
    }

    motor_pos_ = motor_pos_temp;
    motor_deg_ = (static_cast<double>(motor_pos_temp) / encoder_resolution_) * 360.0;

    std::cerr << "motor_pos_: " << motor_pos_ << std::endl;
    std::cerr << "motor_deg_: " << motor_deg_ << std::endl;

    uint8_t dxl_error = 0;
    uint32_t goal_position = (unsigned int)(motor_pos_temp % encoder_resolution_); // Convert int32 -> uint32
    present_position = 0;

    // CHECKの値でモータの現在位置を取得し、安全な運転をするかどうか選択する
    if (CHECK == 0) { // 0: 現在位置を確認しない

        // モータを指定位置まで回転
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            dxl_id,
            ADDR_GOAL_POSITION,
            goal_position,
            &dxl_error
        );
    }
    else if (CHECK == 1) { // 1: 現在位置を確認する

        // goal_positionまでモータが移動したことが確認できればwhile文を終了
        do {

            // モータを指定位置まで回転
            dxl_comm_result = packetHandler->write4ByteTxRx(
                portHandler,
                dxl_id,
                ADDR_GOAL_POSITION,
                goal_position,
                &dxl_error
            );

            // モータの現在位置を取得
            dxl_comm_result = packetHandler->read4ByteTxRx(
                portHandler,
                dxl_id,
                ADDR_PRESENT_POSITION,
                reinterpret_cast<uint32_t*>(&present_position),
                &dxl_error
            );

            present_position = (unsigned int)(present_position % encoder_resolution_);
            std::cerr << "[ERRO] present_position: " << present_position << std::endl;
            Sleep(0.01 * 1000);

            // チェックの余裕を持たせる
            if ((present_position >= (goal_position - SYS_ERR_WIDE)) && (present_position <= (goal_position + SYS_ERR_WIDE))) {
                break;
            }
        } while (goal_position != present_position);
    } // CHECK...

    Sleep(0.01 * 1000); // 0.01s

    // 初期位置への移動時は動きが変則的なため、停止まで待機
    if ((!start_flag) && (!turn_OK)) {
        // スタート時
        start_flag = true;
        Sleep(3 * 1000);
    }
    else if (start_flag && turn_OK) {
        // 初期位置へ帰還時
        Sleep(3 * 1000);
    };
}

// 座標を移動させ、回転させる関数
void transformPoint(pcl::PointXYZ& point, double moveX, double moveY, double moveZ, double roll, double pitch, double yaw) {
    // 移動行列
    Translation3d translation(moveX, moveY, moveZ);

    // 回転行列 (Z軸回り)
    AngleAxisd rollAngle(roll, Vector3d::UnitX());
    AngleAxisd pitchAngle(pitch, Vector3d::UnitY());
    AngleAxisd yawAngle(yaw, Vector3d::UnitZ());
    Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;
    Affine3d rotation = Affine3d(quaternion);

    // 移動と回転を組み合わせた変換行列
    Affine3d transform = translation * rotation;

    // 座標を変換
    Vector3d originalPoint(point.x, point.y, point.z);
    Vector3d transformedPoint = transform * originalPoint;

    point.x = transformedPoint.x();
    point.y = transformedPoint.y();
    point.z = transformedPoint.z();
}

double degToRad(double deg) {
    return deg * M_PI / 180.0;
}

void lidarCallback(const RTC::RangeData& m_range) {

    if (!start_flag) {
        updateMotorPosition(motor_pos_);
        Sleep(1 * 1000);

        // 移動と回転のパラメータ
        moveX = m_range.geometry.geometry.pose.position.x;
        moveY = m_range.geometry.geometry.pose.position.y;
        moveZ = m_range.geometry.geometry.pose.position.z;
        roll  = m_range.geometry.geometry.pose.orientation.r;
        pitch = m_range.geometry.geometry.pose.orientation.p;
        yaw   = m_range.geometry.geometry.pose.orientation.y;
    }
    else if (conected_m) {
        conected = true;
    }
    else {
        conected = false;
    }

    if ((!turn_OK) && conected) {
        // Process the 2D LiDAR data and convert it to 3D point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t i = 0; i < m_range.ranges.length(); ++i) {
            MaxRange = (m_range.config.maxRange / 1000.0);

            // Calculate 3D coordinates
            double range = (double)(m_range.ranges[i] / 1000.0); // センサからのデータの単位は[mm]で受信
            if (range > (MaxRange - 1.0)) {
                range = 0.0;
            }

            double angle = m_range.config.minAngle + (i * m_range.config.angularRes);

            pcl::PointXYZ inputPoint;

            inputPoint.x = range * std::cos(angle);
            inputPoint.y = range * std::sin(angle);
            inputPoint.z = 0.0;

            // 座標を移動させ、回転させる
            transformPoint(inputPoint, moveX, moveY, moveZ, roll, pitch, yaw);

            // Apply axis transformation
            double conv_x = -(inputPoint.z);
            double conv_y = inputPoint.y;
            double conv_z = inputPoint.x;

            // Apply axis rotation based on motor position
            double motor_rad = degToRad(motor_deg_);
            double rotated_x = conv_x * std::cos(motor_rad) - conv_y * std::sin(motor_rad);
            double rotated_y = conv_x * std::sin(motor_rad) + conv_y * std::cos(motor_rad);
            double rotated_z = conv_z;

            // Create a new PCL point
            pcl::PointXYZ point;
            point.x = rotated_x;
            point.y = rotated_y;
            point.z = rotated_z;

            new_cloud->points.push_back(point);
        }

        // Merge the new point cloud with the existing point cloud
        *pointcloud += *new_cloud;

        // Motor control
        updateMotorPosition(motor_pos_);

        // Check if the motor has completed half rotation
        if (motor_pos_ > ((encoder_resolution_ / 2) + START_POS)) {
            turn_OK = true;
            updateMotorPosition(motor_pos_);
        }
    }
    else if (turn_OK && (motor_pos_ == START_POS) && (!fin_flag)) {

        start_flag = false;
        fin_flag   = true;
        // 現在の日時を取得
        auto now = std::chrono::system_clock::now();
        std::time_t current_time = std::chrono::system_clock::to_time_t(now);

        // 日付と時刻のフォーマットを設定
        std::stringstream ss;
        ss << std::put_time(std::localtime(&current_time), "%Y%m%d_%H%M%S");

        // ファイル名を生成
        std::string filename = "pointcloud_" + ss.str() + ".ply";

        // PLYファイルとして保存します
        int save_flg = pcl::io::savePLYFileASCII(filename, *pointcloud);

        // 保存が成功したかどうかを確認
        if (save_flg == 0) {
            std::cout << "[INFO] 点群を " << filename << " として保存しました。" << std::endl;
        }
        else {
            std::cerr << "[ERRO] 点群の保存に失敗しました !\a" << std::endl;
        }
        Fin_Times++;
    }
}


RTC::ReturnCode_t MeasurementSystem::onFinalize()
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t MeasurementSystem::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t MeasurementSystem::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t MeasurementSystem::onActivated(RTC::UniqueId /*ec_id*/)
{
    std::cerr << "[INFO] Activating Dynamixel...." << std::endl;
    if (Times == 0) {
        // 変数設定
        BAUDRATE = m_BAUDRATE;
        encoder_resolution_ = m_encoder_resolution_;

        // 変数のリセット
        portHandler = dynamixel::PortHandler::getPortHandler(m_DEVICE_NAME.c_str());
        packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        // Open Serial Port
        dxl_comm_result = portHandler->openPort();
        if (dxl_comm_result == false) {
            std::cerr << "[ERRO] Failed to open the port!\a" << std::endl;
            return RTC::RTC_ERROR;
        }
        else {
            std::cerr << "[INFO] Succeeded to open the port." << std::endl;
        }
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
    if (dxl_comm_result == false) {
        std::cerr << "[ERRO] Failed to set the baudrate!\a" << std::endl;
        return RTC::RTC_ERROR;
    }
    else {
        std::cerr << "[INFO] Succeeded to set the baudrate." << std::endl;
    }

    int setupErro = setupDynamixel(BROADCAST_ID);
    if (!setupErro) {
        return RTC::RTC_ERROR;
    }

    pointcloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    std::cerr << "[INFO] Activating Dynamixel OK!" << std::endl;

    Times++;
    
    return RTC::RTC_OK;
}


RTC::ReturnCode_t MeasurementSystem::onDeactivated(RTC::UniqueId /*ec_id*/)
{
    // Disable Torque of DYNAMIXEL
    std::cerr << "[INFO] Deactivating Dynamixel....";
    conected_m = false;
    conected   = false;

    packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error
    );
    std::cerr << " OK!" << std::endl;
    return RTC::RTC_OK;
}


RTC::ReturnCode_t MeasurementSystem::onExecute(RTC::UniqueId /*ec_id*/)
{
    if ((Times != Fin_Times) || (Fin_Times == 0)) {
        // ループ
        if (m_rangeIn.isNew()) {
            conected_l = true;
            // InPortデータの読み込み
            m_rangeIn.read();

            // 関数呼び出し
            lidarCallback(m_range);
        }
        else {
            std::cerr << "[ERRO] Not found LiDAR range data." << std::endl;
            std::cerr << "\a";

            conected_l = false;
            conected   = false;
            //Sleep(1 * 1000);
        }
    }
    else if (Times == Fin_Times) {
        if (turn_OK && fin_flag) {

            // 点群の配列数を取得
            size_t new_PC_size = pointcloud->width * pointcloud->height;

            // 点群を分割して送信
            std::cerr << "[INFO] \"new_PointCloud\" is divided..." << std::endl;
            int OutputCloud_times = 0;

            // 点群を分割して通信
            for (size_t j = 0; j < new_PC_size; j += MAX_PC_COMM) {
                size_t endIdx = std::min(j + MAX_PC_COMM, new_PC_size);

                // 部分点群の作成
                PointCloudTypes::PointCloud subCloud;

                // 点群データの有無の確認
                subCloud.is_dense = (::CORBA::Boolean)pointcloud->is_dense;
                // subCloudのサイズを設定
                subCloud.width  = endIdx - j;
                subCloud.height = pointcloud->height;

                // subCloudのデータを格納するためにバッファを確保
                subCloud.data.length(sizeof(float) * (endIdx - j) * 3);

                // バッファへのポインタを取得
                float* dest = (float*)subCloud.data.get_buffer();

                // pointcloudからsubCloudにデータをコピー
                for (size_t i = j; i < endIdx; i++) {
                    dest[0] = pointcloud->points[i].x;
                    dest[1] = pointcloud->points[i].y;
                    dest[2] = pointcloud->points[i].z;
                    dest += 3;
                }

                // OutPortデータへ代入
                // 点群データの有無の確認
                m_new_PointCloud.is_dense = subCloud.is_dense;
                // subCloudのサイズを設定
                m_new_PointCloud.width  = subCloud.width;
                m_new_PointCloud.height = subCloud.height;
                m_new_PointCloud.data   = subCloud.data;

                // memory release for RTCPCL_Cloud
                subCloud.data.release();

                OutputCloud_times++;
                std::cerr << "[INFO] " << OutputCloud_times << " \"new_PointCloud\" is output..." << std::endl;

                // OutPortから出力する
                while (!m_new_PointCloudOut.write()) {
                    RTC::DataPortStatusList stat = m_new_PointCloudOut.getStatusList();

                    for (size_t i(0), len(stat.size()); i < len; ++i) {
                        if (stat[i] != RTC::DataPortStatus::PORT_OK) {
                            std::cout << "[ERRO] Error in connector number " << i << " with status: " << RTC::toString(stat[i]) << std::endl;
                            Sleep(0.5 * 1000);
                        }
                    }
                }
                Sleep(0.5 * 1000);
            }
            pointcloud.reset();

            // Reset bool
            conected_l = false;
            conected   = false;
            start_flag = false;
            turn_OK    = false;
            fin_flag   = false;

            std::cerr << "[INFO] Measurement is completed, please Deactivate." << std::endl;
        }
        std::cerr << "\a";
        Sleep(10 * 1000);
    }

    return RTC::RTC_OK;
}


//RTC::ReturnCode_t MeasurementSystem::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t MeasurementSystem::onError(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MeasurementSystem::onReset(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t MeasurementSystem::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t MeasurementSystem::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}



extern "C"
{
 
  void MeasurementSystemInit(RTC::Manager* manager)
  {
    coil::Properties profile(measurementsystem_spec);
    manager->registerFactory(profile,
                             RTC::Create<MeasurementSystem>,
                             RTC::Delete<MeasurementSystem>);
  }
  
}
