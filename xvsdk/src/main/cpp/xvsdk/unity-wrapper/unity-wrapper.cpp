#include "unity-wrapper.h"


#ifdef WIN32
                                                                                                                        #define NOMINMAX
#include <Windows.h>
#include <time.h>
#include <stdio.h>
#endif

#ifdef __linux__

#include <sys/types.h>
#include <unistd.h>

#endif

// #include <Eigen/Core>
// #include <Eigen/Geometry>
#include <iostream>
#include <sstream>
#include <future>
#include <cstring>
#include <string.h>
#include <map>
#include <vector>
#include <list>
#include <algorithm>
#include <iomanip>
#include <thread>
#include <queue>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>

#ifdef ANDROID

#include <pthread.h>
#include <libusb.h>
#include <jni.h>

#endif
/* #include <qiwei/aSeeVRSDK.h>
#include <qiwei/aSeeVRTypes.h> */
#define XSLAM_VENDOR 0x040e
#define XSLAM_PRODUCT 0xf408

// #define MNN_ENABLE

// #include <callVpuDual.h>
// #include <handskeleton.hpp>
// // zyb
// #include <MNN/ImageProcess.hpp>
// #include <MNN/Interpreter.hpp>
// #include <MNN/AutoTime.hpp>
#include <memory>
#include <string>
#include <stdexcept>
#include <xv-sdk-private.h>
#define LOG_TAG "xvxr#wrapper"
// 全局变量控制日志开关
static bool isLogEnabled = true;  // true 表示启用日志，false 表示禁用日志
#define LOG_DEBUG(level, tag, ...)                                                 \
    do                                                                    \
    {                                                                     \
        if (isLogEnabled)                                                 \
        {                                                                 \
            __android_log_print(level, tag, __VA_ARGS__);                             \
        }                                                                 \
    } while (false)
#define LOG_ERROR(level, tag, ...)                                            \
    do                                                                    \
    {                                                                     \
        if (isLogEnabled)                                                 \
        {                                                                 \
            __android_log_print(level,tag, __VA_ARGS__);                              \
        }                                                                 \
    } while (false)

static int m_fd = -1;
static xv::SlamStartMode s_startMode = xv::SlamStartMode::Normal;
static bool g_driver_only = false;
static std::shared_ptr<xv::Device> device;
static std::mutex s_poseMutex;
static std::shared_ptr<xv::Pose> s_slamPose = nullptr;

static std::mutex s_slamMapMutex;
static int s_MapId = -1;
static std::shared_ptr<const xv::SlamMap> slamMap = nullptr;

static std::mutex s_tofPlaneMutex;
static int s_tofPlaneId = -1;
static std::shared_ptr<const std::vector<xv::Plane>> s_tofPlane = nullptr;
static std::vector<unsigned char>  g_deviceStatus;
static std::mutex s_stereoPlaneMutex;
static int s_stereoPlaneId = -1;
static std::shared_ptr<const std::vector<xv::Plane>> s_stereoPlane = nullptr;

static int s_gesturePlatform = UnityWrapper::PLATFORM::ANDROID_NPU;
static bool s_gestureEgo = true;
static std::mutex s_initMutex;
static int s_components = UnityWrapper::COM_ALL;
static bool s_ready = false;
static bool isSetDeviceCb = false;
static UnityWrapper::SlamType s_type = UnityWrapper::SlamType::Mixed;
static fn_surface_callback surfacecb;
static std::mutex s_stereoImageMtx;
static std::shared_ptr<xv::FisheyeImages> s_stereoImage;
static bool hasTofPlane = false;
static std::mutex s_imuMutex;
static std::shared_ptr<xv::Imu> s_imu;

static std::mutex s_eventMutex;
static std::shared_ptr<xv::Event> s_event;

static std::mutex s_handMutex;
static std::shared_ptr<const std::vector<xv::keypoint>> s_keypoints;
static std::vector<xv::Pose> *s_handPoses;

static std::mutex s_GestureMutex;
static std::shared_ptr<const xv::GestureData> s_gestureData;

static std::mutex s_DynamicGestureMutex;
static std::shared_ptr<const xv::GestureData> s_DynamicgestureData;

static std::mutex s_slamHandMutex;
static std::shared_ptr<const std::vector<xv::keypoint>> s_slamKeypoints;
static std::shared_ptr<const std::vector<xv::Pose>> s_GesturePose;
static std::filebuf mapStream;
static std::string map_filename = "map.bin";
static std::string map_shared_filename = "map_shared.bin";
static std::atomic_int localized_on_reference_percent(0);

static std::mutex s_depthImageMtx;
static std::shared_ptr<xv::DepthImage> s_depthImage;

static std::mutex s_tofIRImageMtx;
static std::shared_ptr<const xv::GrayScaleImage> s_ir;

static std::mutex s_sgbmImageMtx;
static std::shared_ptr<xv::SgbmImage> s_sgbmImage;

static std::mutex s_depthColorImageMtx;
static std::shared_ptr<const xv::DepthColorImage> s_depthColorImage;

static std::mutex s_surfaceMtx;
static std::shared_ptr<const xv::ex::Surfaces> s_surfaces;

static std::mutex s_oriMutex;
static std::shared_ptr<xv::Orientation> s_orientation;

static std::mutex s_micDataMtx;
static std::shared_ptr<const xv::MicData> s_micData;

static std::mutex s_colorMutex;
static std::mutex s_color2Mutex;
static std::mutex s_thermalMutex;
static std::mutex s_irTrackingMutex;
static std::mutex s_rgblFustionThermalMutex;
static std::mutex s_rgbrFustionThermalMutex;
static std::shared_ptr<xv::ColorImage> s_color;
static std::shared_ptr<xv::ColorImage> s_color2;
static std::shared_ptr<xv::ThermalImage> s_thermal;
static std::shared_ptr<xv::IrTrackingImage> s_IrTrackingImage;
static std::shared_ptr<xv::ColorImage> s_rgblFustionThermal;
static std::shared_ptr<xv::ColorImage> s_rgbrFustionThermal;
static int imuId = -1;
static UnityWrapper::RgbSource s_rgbSource = UnityWrapper::RgbSource::VSC;
static UnityWrapper::RgbResolution s_rgbResolution = UnityWrapper::RgbResolution::UNDEF;
static std::mutex lastStereoImage_mtx;
static std::mutex s_objMutex;
static std::vector<xv::Object> s_objects;
#ifdef ANDROID
static xv::GazeCalibration calibration;
#endif
static std::shared_ptr<xv::Display> display;
static const char* gSvrConfigFilePath = "/data/misc/xr/xvxrapi_config.txt";
static const char* noRootxrConfigFilePath = "/sdcard/xv/xvxrapi_config.txt";
#ifdef XV_GPS
static GpsData s_gpsData;
static xv::GPSDistanceData s_gpsDistanceData;

static bool isSlamStart = false;
int s_gpsCallbackId = -1;
int s_gpsDistanceCallbackId = -1;
#endif

// zybdsa
#include <cmath>
#include <mutex>
#include <algorithm>
#include <queue>

#ifndef __linux__
                                                                                                                        #include <Eigen/Dense>
#include <Eigen/Core>

#include <MNN/ImageProcess.hpp>
#include <MNN/Interpreter.hpp>
#include <MNN/AutoTime.hpp>
#include <MNN/ImageProcess.hpp>
#endif

#include <limits>
#include <iostream>
#include <functional>
#include <xv-sdk.h>

const std::string model_name = "/sdcard/detnet_ceternet_sim.mnn";
const std::string model_name2 = "/sdcard/posenetv33_8x_sim.mnn";
const std::string model_name3 = "/sdcard/kinematicmodel_left.mnn";
static int s_gesture_callback_id;
static int s_cnn_callback_id;
// static HandSkeleton::handskeleton_wrapper detection = HandSkeleton::handskeleton_wrapper();
static bool time_init_flag = true;
static std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
static int s_cnnSource = 0xff;
static std::mutex keypoints_mtx;
static bool s_object_ui_show = true;
static bool s_uvc_rgb_runing = false;
static bool s_stereo_runing = false;
static std::atomic<long> fisheyesCallbackCount(0);
static std::atomic<long> keypointCount(0);
static device_status_callback callbackDeviceStatus;
static device_status_callback_ex callbackDeviceStatusEx = nullptr;
static device_status m_DeviceStatus;
static double calibra[2][11],calibraT[2][3],calibraR[2][9];
static bool isSetCalibra = false;
static std::string tagDetectorId = "";
static std::string tagRgbDetectorId = "";
//bool* isUpdateCalibra;
static std::shared_ptr< bool> isUpdateCalibra;
static bool hasFisheyesImagesUpdate = false;
#ifdef FE_RECTIFICATION

struct fe_images_info
{
   std::pair<xv::GrayScaleImage, xv::GrayScaleImage> fe;
   xv::Pose pose;
};

static std::atomic<bool> fe_running(false);
static std::atomic<bool> has_fe_images(false);
static std::mutex s_feImagesMtx;
static fe_images_info m_feImagesInfo;
static std::thread fisheyesRectificationThread;
static std::shared_ptr<xv::StereoRectificationMesh> stereoRectificationMesh;

#endif

template<typename... Args>
#define M_PI 3.14159265358979323846 // pi
std::string sformat(const std::string &format, Args... args) {
    size_t size = snprintf(nullptr, 0, format.c_str(), args...) + 1; // Extra space for '\0'
    if (size <= 0) {
        throw std::runtime_error("Error during formatting.");
    }
    std::unique_ptr<char[]> buf(new char[size]);
    snprintf(buf.get(), size, format.c_str(), args...);
    return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
}

#ifdef ANDROID

#include <android/log.h>

class androidout : public std::streambuf {
public:
    enum {
        bufsize = 128
    }; // ... or some other suitable buffer size
    androidout() { this->setp(buffer, buffer + bufsize - 1); }

private:
    int overflow(int c) {
        if (c == traits_type::eof()) {
            *this->pptr() = traits_type::to_char_type(c);
            this->sbumpc();
        }
        return this->sync() ? traits_type::eof() : traits_type::not_eof(c);
    }

    int sync() {
        int rc = 0;
        if (this->pbase() != this->pptr()) {
            char writebuf[bufsize + 1];
            memcpy(writebuf, this->pbase(), this->pptr() - this->pbase());
            writebuf[this->pptr() - this->pbase()] = '\0';

            rc = __android_log_write(ANDROID_LOG_INFO, "std", writebuf) > 0;
            this->setp(buffer, buffer + bufsize - 1);
        }
        return rc;
    }

    char buffer[bufsize];
};

class androiderr : public std::streambuf {
public:
    enum {
        bufsize = 128
    }; // ... or some other suitable buffer size
    androiderr() { this->setp(buffer, buffer + bufsize - 1); }

private:
    int overflow(int c) {
        if (c == traits_type::eof()) {
            *this->pptr() = traits_type::to_char_type(c);
            this->sbumpc();
        }
        return this->sync() ? traits_type::eof() : traits_type::not_eof(c);
    }

    int sync() {
        int rc = 0;
        if (this->pbase() != this->pptr()) {
            char writebuf[bufsize + 1];
            memcpy(writebuf, this->pbase(), this->pptr() - this->pbase());
            writebuf[this->pptr() - this->pbase()] = '\0';

            rc = __android_log_write(ANDROID_LOG_ERROR, "std", writebuf) > 0;
            this->setp(buffer, buffer + bufsize - 1);
        }
        return rc;
    }

    char buffer[bufsize];
};

static JavaVM *gJavaVM;

jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved) {
    gJavaVM = vm;
    LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy JNI_OnLoad 0");
    return JNI_VERSION_1_6;
}

#endif

std::string printable(const unsigned char *data, unsigned int size, bool join = true) {
    std::stringstream ss;
    ss << std::hex;
    if (join) {
        ss << "0x";
        for (unsigned int i = 0; i < size; i++) {
            ss << std::setfill('0') << std::setw(2) << int(data[i]);
        }
    } else {
        for (unsigned int i = 0; i < size; i++) {
            ss << "0x" << std::setfill('0') << std::setw(2) << int(data[i]);
            if (i < size - 1) {
                ss << ",";
            }
        }
    }
    ss << std::dec;
    return ss.str();
}

/** \cond */
struct Position {
    double x;
    double y;
    double z;

    Vector3 toVector() const {
        return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
    }
};

//struct Orientation {
//    double pitch;
//    double yaw;
//    double roll;
//
//    Vector3 toVector() const {
//        return {static_cast<float>(pitch),static_cast<float>(yaw),static_cast<float>(roll)};
//    }
//};
/** \endcond */

namespace UnityWrapper {

    static void xv_push_rgb_img(std::shared_ptr<xv::ColorImage> ptr);
    /**
 * \enum SlamType
 * SLAM source
 */

    /**
 * \var SlamType::Edge
 * On device SLAM
 *
 * \var SlamType::Mixed
 * On host SLAM
 */

    void xslam_setLogLevel(int level) {
        if(level == 6){
            isLogEnabled = false;
        } else {
            isLogEnabled = true;
        }
        xv::setLogLevel(static_cast<xv::LogLevel>(level));
    }

    void cslamSwitchedCallback(int map_quality);

    void cslamLocalizedCallback(float percent);

    static int eventID = -1;

    volatile int *FrameDValue = nullptr;
    static int *Frequency = nullptr;
    volatile long long *StaTime = nullptr;
    volatile char *IsWriteST = nullptr;
    volatile char *CurrentSync = nullptr;

    long long nano_time() {
        //	 #if defined(HAVE_POSIX_CLOCKS)

#ifdef ANDROID
        timespec now;

        clock_gettime(CLOCK_MONOTONIC, &now);

        return now.tv_sec * 1000000000LL + now.tv_nsec;
#elif WIN32
                                                                                                                                return GetTickCount();
#else
        return 0;
#endif
    }

    void setCalibra() {
        if(device->display()->calibration().size() == 2){
            for(int i = 0;i < 2;i++) {
                if(ISMTK) {
                    float sideLength = 1920.0f;
                    calibra[i][0] = sideLength/2.0f;
                    calibra[i][1] = sideLength/2.0f;
                    calibra[i][2] = sideLength/2.0f;
                    calibra[i][3] = sideLength/2.0f;
                    calibra[i][4] = 0.0f;
                    calibra[i][5] = 0.0f;
                    calibra[i][6] = 0.0f;
                    calibra[i][7] = 0.0f;
                    calibra[i][8] = 0.0f;
                    calibra[i][9] = sideLength;
                    calibra[i][10] = sideLength;
                }else {

                    calibra[i][0] = device->display()->calibration()[i].pdcm[0].fx;
                    calibra[i][1] = device->display()->calibration()[i].pdcm[0].fy;
                    calibra[i][2] = device->display()->calibration()[i].pdcm[0].u0;
                    calibra[i][3] = device->display()->calibration()[i].pdcm[0].v0;
                    calibra[i][4] = device->display()->calibration()[i].pdcm[0].distor[0];
                    calibra[i][5] = device->display()->calibration()[i].pdcm[0].distor[1];
                    calibra[i][6] = device->display()->calibration()[i].pdcm[0].distor[2];
                    calibra[i][7] = device->display()->calibration()[i].pdcm[0].distor[3];
                    calibra[i][8] = device->display()->calibration()[i].pdcm[0].distor[4];
                    calibra[i][9] = device->display()->calibration()[i].pdcm[0].w;
                    calibra[i][10] =device->display()->calibration()[i].pdcm[0].h;
                }

                std::memcpy(calibraR[i],
                            &((xv::Matrix3d) device->display()->calibration()[i].pose.rotation())[0],
                            sizeof(xv::Matrix3d));
                std::memcpy(calibraT[i],
                            &((xv::Vector3d) device->display()->calibration()[i].pose.translation())[0],
                            sizeof(xv::Vector3d));
            }
            if(calibra[1][10]>0){
                isSetCalibra = true;
            }
        }else{
            std::cout
                    << "set calibration error!"
                    << std::endl;
        }

    }

    void getCalibra(stereo_pdm_calibration *calib){
        if(device->display()->calibration().size() == 2){
            for (int i = 0; i < 2; i++) {
                calib->calibrations[i].intrinsic.K[0] = calibra[i][0];
                calib->calibrations[i].intrinsic.K[1] = calibra[i][1];
                calib->calibrations[i].intrinsic.K[2] = calibra[i][2];
                calib->calibrations[i].intrinsic.K[3] = calibra[i][3];
                calib->calibrations[i].intrinsic.K[4] = calibra[i][4];
                calib->calibrations[i].intrinsic.K[5] = calibra[i][5];
                calib->calibrations[i].intrinsic.K[6] = calibra[i][6];
                calib->calibrations[i].intrinsic.K[7] = calibra[i][7];
                calib->calibrations[i].intrinsic.K[8] = calibra[i][8];
                calib->calibrations[i].intrinsic.K[9] = calibra[i][9];
                calib->calibrations[i].intrinsic.K[10] = calibra[i][10];

                std::memcpy(calib->calibrations[i].extrinsic.rotation,
                            calibraR[i],
                            sizeof(xv::Matrix3d));
                std::memcpy(calib->calibrations[i].extrinsic.translation,
                            calibraT[i],
                            sizeof(xv::Vector3d));

                std::cout
                        << sformat("fisheye calibration,rotation size:%u,translation size:%u{",
                                   (unsigned int) sizeof(xv::Matrix3d),
                                   (unsigned int) sizeof(xv::Vector3d))
                        << std::endl;
                std::cout << sformat("T%d:%lf,%lf,%lf", i + 1,
                                     calib->calibrations[i].extrinsic.translation[0],
                                     calib->calibrations[i].extrinsic.translation[1],
                                     calib->calibrations[i].extrinsic.translation[2])
                          << std::endl;
                std::cout << sformat("R%d:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", i,
                                     calib->calibrations[i].extrinsic.rotation[0],
                                     calib->calibrations[i].extrinsic.rotation[1],
                                     calib->calibrations[i].extrinsic.rotation[2],
                                     calib->calibrations[i].extrinsic.rotation[3],
                                     calib->calibrations[i].extrinsic.rotation[4],
                                     calib->calibrations[i].extrinsic.rotation[5],
                                     calib->calibrations[i].extrinsic.rotation[6],
                                     calib->calibrations[i].extrinsic.rotation[7],
                                     calib->calibrations[i].extrinsic.rotation[8])
                          << std::endl;
                std::cout << sformat("K%d:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf}", i + 1,
                                     calib->calibrations[i].intrinsic.K[0],
                                     calib->calibrations[i].intrinsic.K[1],
                                     calib->calibrations[i].intrinsic.K[2],
                                     calib->calibrations[i].intrinsic.K[3],
                                     calib->calibrations[i].intrinsic.K[4],
                                     calib->calibrations[i].intrinsic.K[5],
                                     calib->calibrations[i].intrinsic.K[6],
                                     calib->calibrations[i].intrinsic.K[7],
                                     calib->calibrations[i].intrinsic.K[8],
                                     calib->calibrations[i].intrinsic.K[9],
                                     calib->calibrations[i].intrinsic.K[10])
                          << std::endl;
            }

        }else{
            std::cout
                    << "set calibration error!"
                    << std::endl;
        }
    }


    void eventCallback(xv::Event const &event) {
        if (event.type == 240 && StaTime != nullptr) {
            //            long long fTemp;
            //            while(*IsWriteST){}
            //            long long staTime = *StaTime;
            //            fTemp = (nano_time() - event.state) - staTime;
            //            if(fTemp<*Frequency/4&&fTemp>*Frequency/-4){
            //                *FrameDValue += (int)fTemp-*Frequency/5;
            //            }
            xslam_set_sync(event.state);
        }
    }

    /** \cond */
    void finalInit(int fd, int components) {
#ifdef __linux__
        std::cout << "PID: " << getpid() << std::endl;
#endif
        if (fd == -1) {
        #ifdef __XV_DRIVER_ONLY__
            auto devices = xv::getDevices(.0, "", nullptr,
                                          static_cast<xv::SlamStartMode>(gSlamStartMode), xv::DeviceSupport::ONLYDRIVER);
        #else
            auto devices = xv::getDevices(3.);
        #endif
            if (devices.empty()) {
                std::cout << "pose device is empty: " << std::endl;
                return;
            }
            device = devices.begin()->second;
            g_driver_only = true;
        } else {
            // device = xv::getDevice(fd);
       //     xv::setLogLevel(xv::LogLevel::debug);
      //      device = std::dynamic_pointer_cast<xv::Device>(xv::getDevice(fd));
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy getDevice slam startMode is %d", static_cast<xv::SlamStartMode>(gSlamStartMode));
#endif
           device = std::dynamic_pointer_cast<xv::Device>(xv::getDevice(fd,"", static_cast<xv::SlamStartMode>(gSlamStartMode)));
        }
        m_fd = fd;
        if (device) {
            setCalibra();
        }

#ifdef XV_JOYSTICK
        if (device && device->wirelessController()) {
            device->wirelessController()->registerSlam(device->slam());
        }
#endif

        if (g_driver_only) {
            xslam_start_imu();
        }
            xslam_start_stereo_stream();
        //   xslam_start_rgb_stream();
            device->slam()->registerCallback([](xv::Pose const &pose) {
            auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
            /*  std::cout << "SLAM pose callback : "
                                                       << " [timestamp=" << pose.hostTimestamp() << " x=" << pose.x() << " y=" << pose.y() << " z=" << pose.z()
                                                       << " pitch=" << pitchYawRoll[0] * 180. / M_PI << "°"
                                                       << " yaw=" << pitchYawRoll[1] * 180. / M_PI << "°"
                                                       << " roll=" << pitchYawRoll[2] * 180. / M_PI << "°"
                                                       << " confidence=" << pose.confidence() << std::endl;*/
            s_poseMutex.lock();
            s_slamPose = std::make_shared<xv::Pose>(pose);
            s_poseMutex.unlock();
        });

        long getPoseCount = 0, lastFrameId = 0;
        auto fisheyes = std::dynamic_pointer_cast<xv::FisheyeCameras>(device->fisheyeCameras());
        // Simulate a 60Hz loop to show the use of the `getPose` function.
        std::atomic<bool> stop(false);
        std::atomic<bool> enableStereoInput(false);

        if (device->colorCamera()) {
            device->colorCamera()->registerCallback([](xv::ColorImage const &rgb) {
                s_colorMutex.lock();
                s_color = std::make_shared<xv::ColorImage>(rgb);
                xv_push_rgb_img(s_color);
                s_colorMutex.unlock();
            });
        }

        if (device) {
            std::vector<unsigned char> command;
            command.push_back(0x02);
            command.push_back(0xfe);
            command.push_back(0x20);
            command.push_back(0x21);
            std::vector<unsigned char> result;
            bool res1 = device->hidWriteAndRead(command, result);
        }

        //        xslam_start_detect_plane_from_tof();
//        device->slam()->start(); //slamMode
        s_ready = true;
        std::cout << "xslam_init done" << std::endl;
    }

#ifdef ANDROID

    /** \endcond */
    std::shared_ptr<xv::Device> xslam_get_device() {
        if (s_ready)
            return device;
        return nullptr;
    }

#endif

    fn_surface_callback xslam_get_surface_cb() {
        if (s_ready && surfacecb)
            return surfacecb;
    }

    /**
 * Init Slam, this will open the device using VIP and PID
 * @param components Select components to enable
 */
    bool xslam_init_components(int components) {

        std::lock_guard<std::mutex> lock(s_initMutex);

        if (s_ready)
            return true;

        //freopen("output.txt", "w", stdout);
        //freopen("error.txt", "w", stderr);

#ifdef ANDROID
        static bool once = false;
        if (!once) {
            once = true;
            std::cout.rdbuf(new androidout);
            std::cerr.rdbuf(new androiderr);
        }
#endif

        std::cout << std::hex << "components: 0x" << components << std::dec << std::endl;

        try {
            finalInit(-1, components);
        }
        catch (std::exception &e) {
            std::cerr << e.what() << std::endl;
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "Failed to init (%s)", e.what());
#endif
            return false;
        }

        return true;
    }

    /**
 * Init Slam, this will open the device using given file descriptor
 * @param fd
 * @param components Select components to enable
 */
    bool xslam_init_components_with_fd(int fd, int components) {
        std::lock_guard<std::mutex> lock(s_initMutex);

        if (s_ready)
            return true;

#ifdef ANDROID
        static bool once = false;
        if (!once) {
            once = true;
            std::cout.rdbuf(new androidout);
            std::cerr.rdbuf(new androiderr);
        }
#endif

        std::cout << std::hex << "fd: " << fd << std::dec << std::endl;
        std::cout << std::hex << "components: 0x" << components << std::dec << std::endl;

        try {
            finalInit(fd, components);
        }
        catch (std::exception &e) {
            std::cerr << e.what() << std::endl;
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "Failed to init with fd=%d (%s)", fd,
                                e.what());
#endif
            return false;
        }

        return true;
    }

    /**
     * Init Slam, this will open the device using VIP and PID. This will open all channel and streams
     * @param components Select components to enable
     */
    bool xslam_init() {
        return xslam_init_components(COM_ALL);
    }

    void writeConfigFile(bool isroot){
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "eddy", "writeConfigFile variables file: %s", gSvrConfigFilePath);
#endif
        if(isroot)
            WriteVariableFile(gSvrConfigFilePath);
        else
            WriteVariableFile(noRootxrConfigFilePath);
    }

    int xslam_get_config_variable(const char* name){
        char value[kMaxVariableValueLength];
        GetVariable(name)->GetValue(value,kMaxVariableValueLength);

#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "GetVariable(name)->GetValue: %s", value);
#endif
        return atoi(value);
    }
    void xslam_register_config_param(char *name,int defaultVaule){
        RegisterVariable( name,(int *)defaultVaule, (int)defaultVaule, kVariablePermanent);
        writeConfigFile(m_fd == -1 ? true:false);
    }
    void xslam_set_debug_log_open(int mode,bool isroot) {
        gDebugLogOpen = mode;
        writeConfigFile(isroot);
    }

    int xslam_get_debug_log_open() {
        return static_cast<int>(gDebugLogOpen);;
    }
    void xslam_set_login_open(int mode,bool isroot) {
        gLoginOpen = mode;
        writeConfigFile(isroot);
    }

    int xslam_get_login_open() {
        return static_cast<int>(gLoginOpen);;
    }
    void xslam_set_start_mode(int mode,bool isroot) {
        gSlamStartMode = mode;
        writeConfigFile(isroot);
    }

    int xslam_get_start_mode() {
      return static_cast<int>(gSlamStartMode);;
    }
    void xslam_set_glass_ipd(int value,bool isroot) {
        gGlassIpd = value;
        writeConfigFile(isroot);
    }

    int xslam_get_glass_ipd() {
        return gGlassIpd;
    }
    void xslam_set_glass_ipd2(int value,bool isroot) {
        gGlassIpd2 = value;
        writeConfigFile(isroot);
    }

    int xslam_get_glass_ipd2() {
        return gGlassIpd2;
    }
    void xslam_set_glass_Light(int value,bool isroot) {
        gGlassLight = value;
        writeConfigFile(isroot);
    }

    int xslam_get_glass_Light() {
        return gGlassLight;
    }

    void xslam_set_box_channel(int value,bool isroot) {
        gChannel = value;
        writeConfigFile(isroot);
    }

    int xslam_get_box_channel() {
        return gChannel;
    }

    /**
     * Init Slam, this will open the device using given file descriptor. This will open all channel and streams
     * @param fd
     */
    bool xslam_init_with_fd(int fd) {
        return xslam_init_components_with_fd(fd, COM_ALL);
    }

    /**
     * Uninit Slam, this will close the device
     */
    bool xslam_uninit() {
        s_ready = false;

        try {
            if (device) {
                if (device->slam())
                    device->slam()->stop();
                device.reset();
                xv::detachDevice(m_fd);
            }
        }
        catch (std::exception &e) {
            std::cerr << e.what() << std::endl;
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "Failed to uninit: %s", e.what());
#endif
            return false;
        }
        return true;
    }

    /**
     * Set SLAM type
     * \see SlamType
     * @param type SLAM source
     */
    void xslam_slam_type(SlamType type) {

        if (s_type != type) {
            s_type = type;
        }
        if (device && device->slam()) {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "device->slam()->start mode (%d)",
                                s_type);
#endif
            device->slam()->stop();
            xv::Slam::Mode slamMode = xv::Slam::Mode::Mixed;
            if (s_type == UnityWrapper::SlamType::Edge)
                slamMode = xv::Slam::Mode::Edge;
            device->slam()->start(static_cast<xv::Slam::Mode>(s_type)); //slamMode
            isSlamStart = true;
        }
    }

    void xslam_stop_slam() {
        if (device && device->slam()) {
            device->slam()->stop(); //slamMode
        }
    }

    /**
     * Get euclidean metric
     * @param
     * @return
     */
    double
    calc_euc_distance(double x0, double y0, double x1, double y1) {
        double x_pow = pow(x1 - x0, 2);
        double y_pow = pow(y1 - y0, 2);
        return sqrt(x_pow + y_pow);
    }

    /**
     * TODO init version
     * Get hand landmark_xyz from hand landmark detection
     * @param landmarks_xyz world space coords in meter
     * @return
     */
    bool xslam_get_hand_landmark_xyz(Vector3 *result, int *type, Matrix4x4 *matrix4) {
        // std::vector<xv::Pose> gesturePos = device->gestureEX()->GetGesturePose();
        return true;
    }

    bool xslam_get_pose_at(double *poseData, double timestamp) {

        xv::Pose pose;

        if (isSlamStart && device->slam()->getPoseAt(pose, timestamp) == false) {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xslam_get_pose_at failed");
#endif
            return false;
        }

        auto r = pose.rotation();
        auto _quat = xv::rotationToQuaternion(r);
        poseData[0] = _quat[0];
        poseData[1] = _quat[1];
        poseData[2] = _quat[2];
        poseData[3] = _quat[3];

        poseData[4] = pose.x();
        poseData[5] = pose.y();
        poseData[6] = pose.z();
#ifdef ANDROID
        if(gDebugLogOpen){
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xslam_get_pose_at success %f", pose.x());
        }
#endif

        return true;
    }

    bool xslam_get_pose_prediction(double *poseData, long long *timestamp, double predictionTime) {

        xv::Pose pose;
        if(device && isSlamStart){

            if (device->slam()->getPose(pose, predictionTime) == false) {
#ifdef ANDROID
                if(gDebugLogOpen) {
                    LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xslam_get_pose_prediction failed");
                }

#else
                std::cout << "xvwrapper xslam_get_pose_prediction failed" << std::endl;
#endif
                return false;
            }

            auto r = pose.rotation();
            auto _quat = xv::rotationToQuaternion(r);
            poseData[0] = _quat[0];
            poseData[1] = _quat[1];
            poseData[2] = _quat[2];
            poseData[3] = _quat[3];

            poseData[4] = pose.x();
            poseData[5] = pose.y();
            poseData[6] = pose.z();
#ifdef ANDROID
            if(gDebugLogOpen) {
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xslam_get_pose_prediction success %f",
                                    pose.x());
            }
#else
            std::cout << "xvwrapper xslam_get_pose_prediction success(" << pose.x() << "," << pose.y() << "," << pose.z() << ")" << std::endl;

#endif
            if (timestamp) {
                *timestamp = (long long) (pose.hostTimestamp() * 1000000000);
            }

            return true;
        } else {
            return false;
        }


    }

    EXPORT_API bool xslam_get_pose_prediction_with_sensor(double *poseData, double *angularVelocity,
                                                          double *angularAcceleration,
                                                          long long *timestamp,
                                                          double predictionTime) {
        xv::Pose pose;
        if (isSlamStart && device->slam()->getPose(pose, predictionTime) == false) {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr",
                                "xslam_get_pose_prediction_with_sensor failed");
#endif
            return false;
        }

        auto r = pose.rotation();
        auto _quat = xv::rotationToQuaternion(r);
        auto _velocity = pose.angularVelocity();
        auto _acceleration = pose.angularAcceleration();
        poseData[0] = _quat[0];
        poseData[1] = _quat[1];
        poseData[2] = _quat[2];
        poseData[3] = _quat[3];

        poseData[4] = pose.x();
        poseData[5] = pose.y();
        poseData[6] = pose.z();

        angularVelocity[0] = _velocity[0];
        angularVelocity[1] = _velocity[1];
        angularVelocity[2] = _velocity[2];

        angularAcceleration[0] = _acceleration[0];
        angularAcceleration[1] = _acceleration[1];
        angularAcceleration[2] = _acceleration[2];

#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr",
                            "xslam_get_pose_prediction_with_sensor success %d", pose.x());
#endif
        if (timestamp) {
            *timestamp = (long long) (pose.hostTimestamp() * 1000000000);
        }

        return true;
    }
    bool xslam_get_pose_confidence(double *confidence) {
        std::shared_ptr<xv::Pose> pose;
        s_poseMutex.lock();
        pose = s_slamPose;
        s_poseMutex.unlock();
        if (pose == nullptr) {
            return false;
        }

        *confidence = pose->confidence();

        return true;
    }
    /**
     * Get 6 DOF from slam source, right-hand coordinate
     * @param position in meter
     * @param orientation Euler angle {pitch, yaw, roll}
     * @param timestamp in µs
     * @return
     */
    bool xslam_get_6dof(Vector3 *position, Vector3 *orientation, long long *timestamp) {
        std::shared_ptr<xv::Pose> pose;
        s_poseMutex.lock();
        pose = s_slamPose;
        s_poseMutex.unlock();
        if (pose == nullptr) {
            return false;
        }

        if (timestamp) {
            *timestamp = pose->edgeTimestampUs();
        }

        if (position) {
            position->x = (-1) * pose->x();
            position->y = (-1) * pose->y();
            position->z = (-1) * pose->z();
            std::cout << sformat("eddy 6dof position x = %d", position->x) << std::endl;
        }

        if (orientation) {
            auto pitchYawRoll = xv::rotationToPitchYawRoll(pose->rotation());
            orientation->x = pitchYawRoll[0];
            orientation->y = pitchYawRoll[1];
            orientation->z = pitchYawRoll[2];
            std::cout << sformat("eddy 6dof orientation x = %d", orientation->x) << std::endl;
        }

        return true;
    }

    bool xslam_get_transform_matrix(float *matrix, long long *timestamp, int *status) {
        if (matrix == nullptr) {
            std::cerr << "Matrix reference is null" << std::endl;
            return false;
        }

        std::shared_ptr<xv::Pose> pose;
        s_poseMutex.lock();
        pose = s_slamPose;
        s_poseMutex.unlock();
        if (!pose) {
            std::cout << "No pose for type " << s_type << std::endl;
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "No pose for type %d", s_type);
#endif
            return false;
        }

        if (timestamp) {
            *timestamp = pose->edgeTimestampUs();
        }

        if (status) {
            *status = 0;
        }

        // Set to identity
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                matrix[i * 4 + j] = (i == j) ? 1 : 0;
            }
        }

        // [  0  4  8 12
        //    1  5  9 13
        //    2  6 10 14
        //    3  7 11 15 ]

        // Change coordinate from right hand to left hand

        // Position
        matrix[12] = pose->x();
        matrix[13] = -pose->y();
        matrix[14] = pose->z();
#ifdef ANDROID

        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy pose result x: %f,y: %f, z: %f",
                            pose->x(), pose->y(), pose->z());
#endif
        // Rotation
#if USE_TR
                                                                                                                                matrix[0] = pose->rotation()[0];
        matrix[1] = pose->rotation()[1];
        matrix[2] = pose->rotation()[2];

        matrix[4] = pose->rotation()[3];
        matrix[5] = pose->rotation()[4];
        matrix[6] = pose->rotation()[5];

        matrix[8] = pose->rotation()[6];
        matrix[9] = pose->rotation()[7];
        matrix[10] = pose->rotation()[8];
#else
        matrix[0] = pose->rotation()[0];
        matrix[1] = pose->rotation()[3];
        matrix[2] = pose->rotation()[6];

        matrix[4] = pose->rotation()[1];
        matrix[5] = pose->rotation()[4];
        matrix[6] = pose->rotation()[7];

        matrix[8] = pose->rotation()[2];
        matrix[9] = pose->rotation()[5];
        matrix[10] = pose->rotation()[8];
#endif

        return true;
    }

    bool xslam_get_transform(Matrix4x4 *matrix, long long *timestamp, int *status) {
        if (matrix == nullptr) {
            std::cerr << "Matrix reference is null" << std::endl;
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "Matrix reference is null");
#endif
            return false;
        }

        return xslam_get_transform_matrix(matrix->m, timestamp, status);
    }

    bool xslam_ready() {
        return s_ready/* && isSlamStart*/;
    }

    /**
     * @brief Get the last image data in RGB format
     * @brief xslam_get_rgb_image_RGB Get the last image data in RGB format
     * @param data
     * @param width
     * @param height
     * @param timestamp pointer to timestamp, update to new image's timestamp. if new image's timestamp <= timestamp, function will return false
     * @return
     */
    bool xslam_get_rgb_image_RGB(unsigned char *data, int width, int height, double *timestamp) {
        //auto t1 = std::chrono::system_clock::now();

        if (!s_color)
            return false;

        if (data == nullptr)
            return false;

        if (s_color->hostTimestamp <= *timestamp)
            return false;

        if (!s_colorMutex.try_lock())
            return false;

        std::shared_ptr<xv::ColorImage> tmp = s_color;

        unsigned srcWidth = tmp->width;
        unsigned srcHeight = tmp->height;

        // To improve performance, release lock before really use the data.
        s_colorMutex.unlock();

        if (width <= 0) {
            width = srcWidth;
        }
        if (height <= 0) {
            height = srcHeight;
        }

#ifdef USE_FFMPEG

                                                                                                                                const auto &yuv_image = tmp->data.get();

        auto pFrameYUV = av_frame_alloc();
        auto pFrameBGR = av_frame_alloc();

        av_image_fill_arrays(pFrameYUV->data, pFrameYUV->linesize, yuv_image, AV_PIX_FMT_YUV420P, srcWidth, srcHeight, 1);
        av_image_fill_arrays(pFrameBGR->data, pFrameBGR->linesize, data, AV_PIX_FMT_RGB24, width, height, 1);

        auto imgCtx = sws_getContext(srcWidth, srcHeight, AV_PIX_FMT_YUV420P, width, height, AV_PIX_FMT_RGB24, SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (imgCtx != nullptr)
        {
            sws_scale(imgCtx, pFrameYUV->data, pFrameYUV->linesize, 0, srcHeight, pFrameBGR->data, pFrameBGR->linesize);
        }

        sws_freeContext(imgCtx);
        imgCtx = nullptr;
        av_frame_free(&pFrameYUV);
        av_frame_free(&pFrameBGR);

        cv::Mat mrgb(height, width, CV_8UC3, data);
        cv::flip(mrgb, mrgb, 1);

#else
        cv::Mat myuv(srcHeight * 3 / 2, srcWidth, CV_8UC1,
                     const_cast<unsigned char *>(tmp->data.get()));
        cv::Mat mrgb(height, width, CV_8UC3, data);

        if (width != srcWidth || height != srcHeight) {
            cv::Mat t;
            cv::cvtColor(myuv, t, cv::COLOR_YUV420p2BGR);
            cv::resize(t, mrgb, mrgb.size());
        } else {
            cv::cvtColor(myuv, mrgb, cv::COLOR_YUV420p2BGR);
        }

#endif

        *timestamp = tmp->hostTimestamp;

        //auto t2 = std::chrono::system_clock::now();
        //auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();

#ifdef ANDROID
        //LOG_DEBUG(ANDROID_LOG_INFO,    "XVisio", "RGB duration: %lld", dt);
#endif

        return true;
    }

    int frameCount = 0;

    /**
     * @brief xslam_get_rgb_image_RGBA Get the last image data in RGBA format
     * @param data
     * @param width
     * @param height
     * @param timestamp pointer to timestamp, update to new image's timestamp. if new image's timestamp <= timestamp, function will return false
     * @return
     */
    bool xslam_get_rgb_image_RGBA(unsigned char *data, int width, int height, double *timestamp) {
        //auto t1 = std::chrono::system_clock::now();

        if (!s_color){
            return false;
        }

        if (data == nullptr)
            return false;

        if (s_color->hostTimestamp <= *timestamp){
            return false;
        }


        if (!s_colorMutex.try_lock()){
#ifdef ANDROID
            if(gDebugLogOpen)
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy s_colorMutex lock failed!");
#endif
            return false;
        }



        std::shared_ptr<xv::ColorImage> tmp = s_color;
        unsigned srcWidth = tmp->width;
        unsigned srcHeight = tmp->height;
        s_colorMutex.unlock();

        std::vector<xv::Object> objects;

#ifdef MNN_ENABLE
                                                                                                                                frameCount++;
        // if (frameCount % 2 == 0)
        {
            auto resulthands = detection.run_one(s_color);

            for (auto &resulthand : resulthands)
            {
                xv::Object obj_temp;
                obj_temp.typeID = resulthand.gesture_index;
                obj_temp.shape = xv::Object::Shape::HandSkeleton;
                for (int iii = 0; iii < 21; iii++)
                {
                    xv::Object::keypoint k_temp;
                    k_temp.x = resulthand.keypoints[iii].x / 640.0f;
                    k_temp.y = resulthand.keypoints[iii].y / 480.0f;
                    k_temp.z = resulthand.keypoints[iii].z;
                    obj_temp.keypoints.push_back(k_temp);
                }
                objects.push_back(obj_temp);
            }
        }

        //    objects = inference_mnn_one(tmp);
#else
        s_objMutex.lock();
        if (s_cnnSource == 2)
            objects = s_objects;
        s_objMutex.unlock();
#endif
        if (width <= 0) {
            width = srcWidth;
        }
        if (height <= 0) {
            height = srcHeight;
        }

#ifdef USE_FFMPEG

                                                                                                                                const auto &yuv_image = tmp->data.get();

        auto pFrameYUV = av_frame_alloc();
        auto pFrameBGR = av_frame_alloc();

        av_image_fill_arrays(pFrameYUV->data, pFrameYUV->linesize, yuv_image, AV_PIX_FMT_YUV420P, srcWidth, srcHeight, 1);
        av_image_fill_arrays(pFrameBGR->data, pFrameBGR->linesize, data, AV_PIX_FMT_RGBA, width, height, 1);

        auto imgCtx = sws_getContext(srcWidth, srcHeight, AV_PIX_FMT_YUV420P, width, height, AV_PIX_FMT_RGBA, SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (imgCtx != nullptr)
        {
            sws_scale(imgCtx, pFrameYUV->data, pFrameYUV->linesize, 0, srcHeight, pFrameBGR->data, pFrameBGR->linesize);
        }

        sws_freeContext(imgCtx);
        imgCtx = nullptr;
        av_frame_free(&pFrameYUV);
        av_frame_free(&pFrameBGR);

        cv::Mat mrgb(height, width, CV_8UC4, data);
        cv::flip(mrgb, mrgb, 1);

#else
        if (!tmp->data.get()) {
            std::cerr << "Invalid RGB data!" << std::endl;
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy tmp->data.get()  failed!");
#endif
            return false;
        }

        if (tmp->codec == xv::ColorImage::Codec::YUV420p) {
//            std::cerr << "eddy YUV420p RGB data!" << std::endl;
            cv::Mat myuv(srcHeight * 3 / 2, srcWidth, CV_8UC1,
                         const_cast<unsigned char *>(tmp->data.get()));
            cv::Mat mrgb(height, width, CV_8UC4, data);

            if (width != srcWidth || height != srcHeight) {
                cv::Mat t;
                cv::cvtColor(myuv, t, cv::COLOR_YUV420p2BGRA);
                cv::resize(t, mrgb, mrgb.size());
            } else {
                cv::cvtColor(myuv, mrgb, cv::COLOR_YUV420p2BGRA);
            }
#endif
            double rw = static_cast<double>(width) / static_cast<double>(srcWidth);
            double rh = static_cast<double>(height) / static_cast<double>(srcHeight);

/*#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy RGB width: %d", width);
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy RGB objects size : %lu",
                                objects.size());
#endif*/
//            std::cout << sformat(".eddy..%d...", width) << std::endl;
            // std::cout << "eddy objects size =" << objects.size() << std::endl;
            for (unsigned int i = 0; i < objects.size(); i++) {
                std::cout << "eddy objects start draw" << std::endl;
                const auto obj = objects.at(i);
                // *type = obj.typeID;
/*#ifdef ANDROID
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy RGB typeId : %lu",
                                    obj.typeID);
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy RGB typshapeeId : %lu",
                                    obj.shape);
#endif*/
                if (obj.shape == xv::Object::Shape::BoundingBox && s_object_ui_show) {
                    //double rx = width - (rw * obj.x + rw * obj.width);
                    //double ry = height - (rh * obj.y + rh * obj.height);
                    double rx = rw * obj.x;
                    double ry = rh * obj.y;
                    const cv::Rect r(rx, ry, rw * obj.width, rh * obj.height);
                    cv::rectangle(mrgb, r, cv::Scalar(255, 255, 0));
                    std::stringstream stream;
                    stream << std::fixed << std::setprecision(2) << 100.0 * obj.confidence;
                    std::string str = obj.type + ":" + stream.str() + "%";
                    int baseline = 0;
                    cv::Size textSize = getTextSize(str, cv::FONT_HERSHEY_DUPLEX, 1.0, 1.0,
                                                    &baseline);
                    cv::putText(mrgb, str, (r.br() + r.tl()) * 0.5 -
                                           cv::Point(textSize.width, textSize.height) * 0.5,
                                cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 255, 0));
                } else if (obj.shape == xv::Object::Shape::HandSkeleton) {
#ifdef ANDROID
                    LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy entry HandSkeleton");
#endif
                    static std::vector<std::vector<int>> list_connections = {{0,  1,  2,  3,  4},
                                                                             {0,  5,  6,  7,  8},
                                                                             {5,  9,  10, 11, 12},
                                                                             {9,  13, 14, 15, 16},
                                                                             {13, 17},
                                                                             {0,  17, 18, 19, 20}};

                    for (auto obj_xy : obj.keypoints) {
                        cv::circle(mrgb,
                                   cv::Point2f(obj_xy.x * static_cast<double>(width),
                                               obj_xy.y * static_cast<double>(height)),
                                   6,
                                   cv::Scalar(255, 0, 0),
                                   -1);
                    }
                    int npts[] = {5, 5, 5, 5, 2, 5};

                    cv::Point points[6][5];
                    for (int ip = 0; ip < list_connections.size(); ip++) {
                        for (int jp = 0; jp < list_connections[ip].size(); jp++) {
                            int lm_index = list_connections[ip][jp];

                            points[ip][jp] = cv::Point(
                                    int(obj.keypoints[lm_index].x * static_cast<double>(width)),
                                    int(obj.keypoints[lm_index].y * static_cast<double>(height)));
                        }
                    }
                    const cv::Point *pts[] = {points[0], points[1], points[2], points[3], points[4],
                                              points[5]};

                    cv::polylines(mrgb, pts, npts, 6, false, cv::Scalar(0, 255, 0), 2, cv::LINE_AA,
                                  0);
                }
            }

            cv::flip(mrgb, mrgb, 1);
        } else if (tmp->codec == xv::ColorImage::Codec::YUYV) {
            std::cerr << "eddy YUYV RGB data!" << std::endl;
            cv::Mat myuv(srcHeight, srcWidth, CV_8UC2,
                         const_cast<unsigned char *>(tmp->data.get()));
            cv::Mat mrgb(height, width, CV_8UC4, data);

            if (width != srcWidth || height != srcHeight) {
                cv::Mat t;
                cv::cvtColor(myuv, t, cv::COLOR_YUV2BGRA_YUYV);
                cv::resize(t, mrgb, mrgb.size());
            } else {
                cv::cvtColor(myuv, mrgb, cv::COLOR_YUV2BGRA_YUYV);
            }
            cv::flip(mrgb, mrgb, 1);
        } /*  else if (tmp->codec == xv::ColorImage::Codec::TOF32) {
        memcpy(data, tmp->data.get(), tmp->dataSize);
    }  */
        else {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "Unsupport RGB codec: %d",
                                static_cast<int>(tmp->codec));
#endif
            return false;
        }

        *timestamp = tmp->hostTimestamp;

        //auto t2 = std::chrono::system_clock::now();
        //auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();

#ifdef ANDROID
        // LOG_DEBUG(ANDROID_LOG_INFO,    "XVisio", "RGB duration: %lld", dt);
#endif

        return true;
    }
    /**
        * @brief xslam_get_rgb_image_RGBA Get the last image data in RGBA format
        * @param data
        * @param width
        * @param height
        * @param timestamp pointer to timestamp, update to new image's timestamp. if new image's timestamp <= timestamp, function will return false
        * @return
        */
    bool xslam_get_rgb_image_RGBA_flip(unsigned char *data, int width, int height, double *timestamp) {
        //auto t1 = std::chrono::system_clock::now();

        if (!s_color){
            return false;
        }

        if (data == nullptr)
            return false;

        if (s_color->hostTimestamp <= *timestamp){
            return false;
        }


        if (!s_colorMutex.try_lock()){
#ifdef ANDROID
            if(gDebugLogOpen)
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy s_colorMutex lock failed!");
#endif
            return false;
        }



        std::shared_ptr<xv::ColorImage> tmp = s_color;
        unsigned srcWidth = tmp->width;
        unsigned srcHeight = tmp->height;
        s_colorMutex.unlock();

        std::vector<xv::Object> objects;

#ifdef MNN_ENABLE
        frameCount++;
        // if (frameCount % 2 == 0)
        {
            auto resulthands = detection.run_one(s_color);

            for (auto &resulthand : resulthands)
            {
                xv::Object obj_temp;
                obj_temp.typeID = resulthand.gesture_index;
                obj_temp.shape = xv::Object::Shape::HandSkeleton;
                for (int iii = 0; iii < 21; iii++)
                {
                    xv::Object::keypoint k_temp;
                    k_temp.x = resulthand.keypoints[iii].x / 640.0f;
                    k_temp.y = resulthand.keypoints[iii].y / 480.0f;
                    k_temp.z = resulthand.keypoints[iii].z;
                    obj_temp.keypoints.push_back(k_temp);
                }
                objects.push_back(obj_temp);
            }
        }

        //    objects = inference_mnn_one(tmp);
#else
        s_objMutex.lock();
        if (s_cnnSource == 2)
            objects = s_objects;
        s_objMutex.unlock();
#endif
        if (width <= 0) {
            width = srcWidth;
        }
        if (height <= 0) {
            height = srcHeight;
        }

#ifdef USE_FFMPEG

        const auto &yuv_image = tmp->data.get();

        auto pFrameYUV = av_frame_alloc();
        auto pFrameBGR = av_frame_alloc();

        av_image_fill_arrays(pFrameYUV->data, pFrameYUV->linesize, yuv_image, AV_PIX_FMT_YUV420P, srcWidth, srcHeight, 1);
        av_image_fill_arrays(pFrameBGR->data, pFrameBGR->linesize, data, AV_PIX_FMT_RGBA, width, height, 1);

        auto imgCtx = sws_getContext(srcWidth, srcHeight, AV_PIX_FMT_YUV420P, width, height, AV_PIX_FMT_RGBA, SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (imgCtx != nullptr)
        {
            sws_scale(imgCtx, pFrameYUV->data, pFrameYUV->linesize, 0, srcHeight, pFrameBGR->data, pFrameBGR->linesize);
        }

        sws_freeContext(imgCtx);
        imgCtx = nullptr;
        av_frame_free(&pFrameYUV);
        av_frame_free(&pFrameBGR);

        cv::Mat mrgb(height, width, CV_8UC4, data);
        cv::flip(mrgb, mrgb, 1);

#else
        if (!tmp->data.get()) {
            std::cerr << "Invalid RGB data!" << std::endl;
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy tmp->data.get()  failed!");
#endif
            return false;
        }

        if (tmp->codec == xv::ColorImage::Codec::YUV420p) {
//            std::cerr << "eddy YUV420p RGB data!" << std::endl;
            cv::Mat myuv(srcHeight * 3 / 2, srcWidth, CV_8UC1,
                         const_cast<unsigned char *>(tmp->data.get()));
            cv::Mat mrgb(height, width, CV_8UC4, data);

            if (width != srcWidth || height != srcHeight) {
                cv::Mat t;
                cv::cvtColor(myuv, t, cv::COLOR_YUV420p2BGRA);
                cv::resize(t, mrgb, mrgb.size());
            } else {
                cv::cvtColor(myuv, mrgb, cv::COLOR_YUV420p2BGRA);
            }
#endif
            double rw = static_cast<double>(width) / static_cast<double>(srcWidth);
            double rh = static_cast<double>(height) / static_cast<double>(srcHeight);

/*#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy RGB width: %d", width);
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy RGB objects size : %lu",
                                objects.size());
#endif*/
//            std::cout << sformat(".eddy..%d...", width) << std::endl;
            // std::cout << "eddy objects size =" << objects.size() << std::endl;
            for (unsigned int i = 0; i < objects.size(); i++) {
                std::cout << "eddy objects start draw" << std::endl;
                const auto obj = objects.at(i);
                // *type = obj.typeID;
/*#ifdef ANDROID
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy RGB typeId : %lu",
                                    obj.typeID);
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy RGB typshapeeId : %lu",
                                    obj.shape);
#endif*/
                if (obj.shape == xv::Object::Shape::BoundingBox && s_object_ui_show) {
                    //double rx = width - (rw * obj.x + rw * obj.width);
                    //double ry = height - (rh * obj.y + rh * obj.height);
                    double rx = rw * obj.x;
                    double ry = rh * obj.y;
                    const cv::Rect r(rx, ry, rw * obj.width, rh * obj.height);
                    cv::rectangle(mrgb, r, cv::Scalar(255, 255, 0));
                    std::stringstream stream;
                    stream << std::fixed << std::setprecision(2) << 100.0 * obj.confidence;
                    std::string str = obj.type + ":" + stream.str() + "%";
                    int baseline = 0;
                    cv::Size textSize = getTextSize(str, cv::FONT_HERSHEY_DUPLEX, 1.0, 1.0,
                                                    &baseline);
                    cv::putText(mrgb, str, (r.br() + r.tl()) * 0.5 -
                                           cv::Point(textSize.width, textSize.height) * 0.5,
                                cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 255, 0));
                } else if (obj.shape == xv::Object::Shape::HandSkeleton) {
#ifdef ANDROID
                    LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy entry HandSkeleton");
#endif
                    static std::vector<std::vector<int>> list_connections = {{0,  1,  2,  3,  4},
                                                                             {0,  5,  6,  7,  8},
                                                                             {5,  9,  10, 11, 12},
                                                                             {9,  13, 14, 15, 16},
                                                                             {13, 17},
                                                                             {0,  17, 18, 19, 20}};

                    for (auto obj_xy : obj.keypoints) {
                        cv::circle(mrgb,
                                   cv::Point2f(obj_xy.x * static_cast<double>(width),
                                               obj_xy.y * static_cast<double>(height)),
                                   6,
                                   cv::Scalar(255, 0, 0),
                                   -1);
                    }
                    int npts[] = {5, 5, 5, 5, 2, 5};

                    cv::Point points[6][5];
                    for (int ip = 0; ip < list_connections.size(); ip++) {
                        for (int jp = 0; jp < list_connections[ip].size(); jp++) {
                            int lm_index = list_connections[ip][jp];

                            points[ip][jp] = cv::Point(
                                    int(obj.keypoints[lm_index].x * static_cast<double>(width)),
                                    int(obj.keypoints[lm_index].y * static_cast<double>(height)));
                        }
                    }
                    const cv::Point *pts[] = {points[0], points[1], points[2], points[3], points[4],
                                              points[5]};

                    cv::polylines(mrgb, pts, npts, 6, false, cv::Scalar(0, 255, 0), 2, cv::LINE_AA,
                                  0);
                }
            }

            cv::flip(mrgb, mrgb, 0);
        } else if (tmp->codec == xv::ColorImage::Codec::YUYV) {
            std::cerr << "eddy YUYV RGB data!" << std::endl;
            cv::Mat myuv(srcHeight, srcWidth, CV_8UC2,
                         const_cast<unsigned char *>(tmp->data.get()));
            cv::Mat mrgb(height, width, CV_8UC4, data);

            if (width != srcWidth || height != srcHeight) {
                cv::Mat t;
                cv::cvtColor(myuv, t, cv::COLOR_YUV2BGRA_YUYV);
                cv::resize(t, mrgb, mrgb.size());
            } else {
                cv::cvtColor(myuv, mrgb, cv::COLOR_YUV2BGRA_YUYV);
            }
            cv::flip(mrgb, mrgb, 0);
        } /*  else if (tmp->codec == xv::ColorImage::Codec::TOF32) {
        memcpy(data, tmp->data.get(), tmp->dataSize);
    }  */
        else {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "Unsupport RGB codec: %d",
                      static_cast<int>(tmp->codec));
#endif
            return false;
        }

        *timestamp = tmp->hostTimestamp;

        //auto t2 = std::chrono::system_clock::now();
        //auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();

#ifdef ANDROID
        // LOG_DEBUG(ANDROID_LOG_INFO,    "XVisio", "RGB duration: %lld", dt);
#endif

        return true;
    }
/**
 * @brief Get the last image data in YUV format
 * @param data Should be 1.5 * height * width allocated unsigned char array
 * @param width pointer to destination width. if <=0 will set to origin width
 * @param height pointer to destination height. if <=0 will set to origin height
 * @param timestamp pointer to timestamp, update to new image's timestamp. if new image's timestamp <= timestamp, function will return false
 * @return
 */
    bool xslam_get_rgb_image_YUV(unsigned char *data, int width, int height, double *timestamp) {
        //auto t1 = std::chrono::system_clock::now();
        std::cout << sformat(".eddy.xslam_get_rgb_image_YUV.%d...", width) << std::endl;
        if (!s_color)
            return false;

        if (s_color->hostTimestamp <= *timestamp)
            return false;

        if (!s_colorMutex.try_lock())
            return false;

        std::shared_ptr<xv::ColorImage> tmp = s_color;
        unsigned srcWidth = tmp->width;
        unsigned srcHeight = tmp->height;

        s_colorMutex.unlock();

        if (width <= 0) {
            width = srcWidth;
        }
        if (height <= 0) {
            height = srcHeight;
        }

        if (width != srcWidth || height != srcHeight) {
#ifdef USE_FFMPEG
                                                                                                                                    const auto &yuv_image = tmp->data.get();

        auto pFrameYUV1 = av_frame_alloc();
        auto pFrameYUV2 = av_frame_alloc();

        av_image_fill_arrays(pFrameYUV1->data, pFrameYUV1->linesize, yuv_image, AV_PIX_FMT_YUV420P, srcWidth, srcHeight, 1);
        av_image_fill_arrays(pFrameYUV2->data, pFrameYUV2->linesize, data, AV_PIX_FMT_YUV420P, *width, *height, 1);

        auto imgCtx = sws_getContext(srcWidth, srcHeight, AV_PIX_FMT_YUV420P, *width, *height, AV_PIX_FMT_YUV420P, SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (imgCtx != nullptr)
        {
            sws_scale(imgCtx, pFrameYUV1->data, pFrameYUV1->linesize, 0, srcHeight, pFrameYUV2->data, pFrameYUV2->linesize);
        }

        sws_freeContext(imgCtx);
        imgCtx = nullptr;
        av_frame_free(&pFrameYUV1);
        av_frame_free(&pFrameYUV2);
#else
            // cv::Mat mgray(srcHeight, srcWidth, CV_8UC1, const_cast<unsigned char *>(tmp->images[0].data.get()));

            cv::Mat myuv1(srcHeight * 3 / 2, srcWidth, CV_8UC1,
                          const_cast<unsigned char *>(tmp->data.get()));
            cv::Mat myuv2(height * 3 / 2, width, CV_8UC1, data);
            cv::resize(myuv1, myuv2, myuv2.size());
#endif
        } else {
            std::memcpy(data, tmp->data.get(), tmp->dataSize);
        }

        //auto t2 = std::chrono::system_clock::now();
        //auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();

#ifdef ANDROID
        //LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "YUV duration: %lld", dt);
#endif

        *timestamp = tmp->hostTimestamp;

        return true;
    }

    int xslam_get_rgb_width() {
        if (!s_color)
            return 0;

        return s_color->width;
    }

    int xslam_get_rgb_height() {
        if (!s_color)
            return 0;

        return s_color->height;
    }

    void xslam_rgb_set_exposure(int aecMode, int exposureGain, float exposureTimeMs) {
        if (device->colorCamera()) {
            device->colorCamera()->setExposure(aecMode, exposureGain, exposureTimeMs);
        }
    }

    void xslam_rgb_set_brightness(int brightness) {
        if (device->colorCamera()) {
            device->colorCamera()->setBrightness(brightness);
        }
    }

/**
 * @brief xslam_get_left_image Get the last left image data in RGBA format
 * @param data
 * @param width
 * @param height
 * @return
 */
    bool xslam_get_left_image(unsigned char *data, int width, int height, double *timestamp) {
        if (!s_stereoImage)
            return false;

        if (data == nullptr)
            return false;

        if (!s_stereoImageMtx.try_lock())
            return false;

        std::shared_ptr<const xv::FisheyeImages> tmp = s_stereoImage;
        unsigned srcWidth = tmp->images[0].width;
        unsigned srcHeight = tmp->images[0].height;
        s_stereoImageMtx.unlock();

        std::vector<xv::Object> objects;
        s_objMutex.lock();
        if (s_cnnSource == 0)
            objects = s_objects;
        s_objMutex.unlock();

        if (width <= 0) {
            width = srcWidth;
        }
        if (height <= 0) {
            height = srcHeight;
        }

        std::shared_ptr<unsigned char> datatmp;
        // datatmp = std::static_pointer_cast<unsigned char>(tmp->images[0].data);

        cv::Mat mgray(srcHeight, srcWidth, CV_8UC1,
                      const_cast<unsigned char *>(tmp->images[0].data.get()));
        cv::Mat mrgb(height, width, CV_8UC4, data);

        if (width != srcWidth || height != srcHeight) {
            cv::Mat resizedMat(height, width, mgray.type());
            cv::resize(mgray, resizedMat, resizedMat.size());
            cv::cvtColor(resizedMat, mrgb, cv::COLOR_GRAY2BGRA);
        } else {
            cv::cvtColor(mgray, mrgb, cv::COLOR_GRAY2BGRA);
        }

        double rw = static_cast<double>(width) / static_cast<double>(srcWidth);
        double rh = static_cast<double>(height) / static_cast<double>(srcHeight);

        for (unsigned int i = 0; i < objects.size(); i++) {
            const auto obj = objects.at(i);
            //double rx = width - (rw * obj.x + rw * obj.width);
            //double ry = height - (rh * obj.y + rh * obj.height);
            double rx = rw * obj.x;
            double ry = rh * obj.y;
            const cv::Rect r(rx, ry, rw * obj.width, rh * obj.height);
            cv::rectangle(mrgb, r, cv::Scalar(255, 255, 0));
            std::stringstream stream;
            stream << std::fixed << std::setprecision(2) << 100.0 * obj.confidence;
            std::string str = obj.type + ":" + stream.str() + "%";
            cv::putText(mrgb, str, (r.br() + r.tl()) * 0.5, cv::FONT_HERSHEY_DUPLEX, 1.0,
                        cv::Scalar(255, 255, 0));
        }

        cv::flip(mrgb, mrgb, 1);

        if (timestamp) {
            *timestamp = tmp->hostTimestamp;
        }

        return true;
    }

/**
 * @brief xslam_get_right_image Get the last right image data in RGBA format
 * @param data
 * @param width
 * @param height
 * @return
 */
    bool xslam_get_right_image(unsigned char *data, int width, int height, double *timestamp) {
        if (!s_stereoImage)
            return false;

        if (data == nullptr)
            return false;

        if (!s_stereoImageMtx.try_lock())
            return false;

        std::shared_ptr<const xv::FisheyeImages> tmp = s_stereoImage;
        unsigned srcWidth = tmp->images[1].width;
        unsigned srcHeight = tmp->images[1].height;
        s_stereoImageMtx.unlock();

        std::vector<xv::Object> objects;
        s_objMutex.lock();
        if (s_cnnSource == 1)
            objects = s_objects;
        s_objMutex.unlock();

        if (width <= 0) {
            width = srcWidth;
        }
        if (height <= 0) {
            height = srcHeight;
        }

        std::shared_ptr<unsigned char> datatmp;
        // datatmp = std::static_pointer_cast<unsigned char>(tmp->images[0].data);

        cv::Mat mgray(srcHeight, srcWidth, CV_8UC1,
                      const_cast<unsigned char *>(tmp->images[1].data.get()));
        cv::Mat mrgb(height, width, CV_8UC4, data);

        if (width != srcWidth || height != srcHeight) {
            cv::Mat resizedMat(height, width, mgray.type());
            cv::resize(mgray, resizedMat, resizedMat.size());
            cv::cvtColor(resizedMat, mrgb, cv::COLOR_GRAY2BGRA);
        } else {
            cv::cvtColor(mgray, mrgb, cv::COLOR_GRAY2BGRA);
        }

        double rw = static_cast<double>(width) / static_cast<double>(srcWidth);
        double rh = static_cast<double>(height) / static_cast<double>(srcHeight);

        for (unsigned int i = 0; i < objects.size(); i++) {
            const auto obj = objects.at(i);
            //double rx = width - (rw * obj.x + rw * obj.width);
            //double ry = height - (rh * obj.y + rh * obj.height);
            double rx = rw * obj.x;
            double ry = rh * obj.y;
            const cv::Rect r(rx, ry, rw * obj.width, rh * obj.height);
            cv::rectangle(mrgb, r, cv::Scalar(255, 255, 0));
            std::stringstream stream;
            stream << std::fixed << std::setprecision(2) << 100.0 * obj.confidence;
            std::string str = obj.type + ":" + stream.str() + "%";
            cv::putText(mrgb, str, (r.br() + r.tl()) * 0.5, cv::FONT_HERSHEY_DUPLEX, 1.0,
                        cv::Scalar(255, 255, 0));
        }

        cv::flip(mrgb, mrgb, 1);

        if (timestamp) {
            *timestamp = tmp->hostTimestamp;
        }

        return true;
    }

    int xslam_get_stereo_width() {
        if (!s_stereoImage)
            return 0;

        return s_stereoImage->images[0].width;
    }

    int xslam_get_stereo_height() {
        if (!s_stereoImage)
            return 0;

        return s_stereoImage->images[0].height;
    }
    /**
     * @brief 设置双目鱼眼摄像头的帧率。
     *
     * 此函数用于为双目鱼眼摄像头配置帧率，以控制图像采集的速度。
     *
     * @param framerate 设置的帧率值，以帧每秒（FPS）为单位。
     */
    void xslam_stereo_set_framerate(float framerate) {
        if (device->fisheyeCameras()) {
            device->fisheyeCameras()->setFramerate(framerate);
        }
    }
    /**
     * @brief 设置双目鱼眼摄像头的曝光参数。
     *
     * 此函数为双目鱼眼摄像头配置曝光参数，包括自动曝光模式、曝光增益和曝光时间。
     *
     * @param aecMode 自动曝光模式
     * @param exposureGain 曝光增益，用于调节摄像头的灵敏度。
     * @param exposureTimeMs 曝光时间，以毫秒为单位。
     *
     * @note 只有当设备的鱼眼摄像头可用时，函数才会执行操作。
     */
    void xslam_stereo_set_exposure(int aecMode, int exposureGain, float exposureTimeMs) {
        if (device->fisheyeCameras()) {
            device->fisheyeCameras()->setExposure(aecMode, exposureGain, exposureTimeMs);
        }
    }
    /**
     * @brief 设置双目鱼眼摄像头的亮度。
     *
     * 此函数调节双目鱼眼摄像头的亮度参数。
     *
     * @param brightness 亮度值，用于调节摄像头的输出亮度，值的范围由具体实现决定。
     *
     * @note 只有当设备的鱼眼摄像头可用时，函数才会执行操作。
     */
    void xslam_stereo_set_brightness(int brightness) {
        if (device->fisheyeCameras()) {
            device->fisheyeCameras()->setBrightness(brightness);
        }
    }

    int xslam_get_stereo_max_points() {
        return 1024;
    }

    bool xslam_get_left_points(Vector2 *points, int *size) {
        return true;
    }

    bool xslam_get_right_points(Vector2 *points, int *size) {
        return true;
    }

    int xslam_get_tof_width() {
        if (s_depthImage) {
            return s_depthImage->width;
        }
        return 0;
    }

    int xslam_get_tof_height() {
        if (s_depthImage) {
            return s_depthImage->height;
        }
        return 0;
    }

    // 转换深度图像数据为浮点值
    void convertDepthDataToFloat(const void* depthData, float* f, unsigned width, unsigned height, float dmax) {
        const int16_t* depthArray = reinterpret_cast<const int16_t*>(depthData);

        for (unsigned int i = 0; i < width * height; ++i) {
            // 从深度数据中获取深度值
            float depthValue = static_cast<float>(depthArray[i]);
            // 归一化深度值到 [0.0, 1.0] 范围
            f[i] = std::min(1.0f, std::max(0.0f, depthValue / dmax));
        }
    }
/*    bool xslam_get_tofir_data(unsigned char *data, int width, int height) {
        std::shared_ptr<const xv::GrayScaleImage> tof;
        s_tofIRImageMtx.lock();
        tof = s_ir;
        s_tofIRImageMtx.unlock();

        if (tof) {
            memcpy(data, tof->data.get(), tof->dataSize);

            return true;
        }
        return false;

    }*/



void convertDepthToRGBAWithGamma(const unsigned short* tmp_d, int width, int height, unsigned short dmin, unsigned short dmax, double gamma, cv::Mat& rgbaImage) {
    // Step 1: 将 tmp_d 转换为单通道 16 位深度图 Mat
    cv::Mat depthImage(height, width, CV_16U, (void*)tmp_d);

    // Step 2: 归一化深度图数据，并转换为 8-bit 灰度图
    cv::Mat normalizedDepth;
    depthImage.convertTo(normalizedDepth, CV_32F, 1.0 / (dmax - dmin), -dmin * 1.0 / (dmax - dmin)); // 归一化到 [0, 1]

    // Step 3: 应用伽马校正
    cv::Mat gammaCorrected;
    cv::pow(normalizedDepth, 1.0 / gamma, gammaCorrected);  // 进行伽马校正

    // Step 4: 将伽马校正后的图像缩放到 [0, 255] 范围，并转换为 8-bit 灰度图
    gammaCorrected.convertTo(gammaCorrected, CV_8U, 255.0);

    // Step 5: 将单通道灰度图转换为四通道 RGBA 格式
    cv::cvtColor(gammaCorrected, rgbaImage, cv::COLOR_GRAY2BGRA);
}


    /**
 * @brief xslam_get_tofir_image Get the last left image data in RGBA format
 * @param data
 * @param width
 * @param height
 * @return
 */
    bool xslam_get_tofir_image(unsigned char *data, int width, int height) {

        if(!s_ir)
            return false;

        if (data == nullptr)
            return false;

        if (!s_tofIRImageMtx.try_lock())
            return false;
        std::shared_ptr<const xv::GrayScaleImage> tof_ir;

        tof_ir = s_ir;
        auto tmp_d = reinterpret_cast<unsigned short const *>(tof_ir->data.get());
        s_tofIRImageMtx.unlock();
        unsigned srcWidth = tof_ir->width;
        unsigned srcHeight = tof_ir->height;
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy xslam_get_tofir_image srcWidth = %d,srcHeight = %d,%d,%d" ,
                  srcWidth,srcHeight,width,height);
        std::vector<xv::Object> objects;
     //   float dmax = 2191/4;
        if (width <= 0) {
            width = srcWidth;
        }
        if (height <= 0) {
            height = srcHeight;
        }

        unsigned short dmin = tmp_d[0];
        unsigned short dmax = tmp_d[0];
        for (unsigned int i = 0; i < tof_ir->height * tof_ir->width; i++) {
            unsigned short d = tmp_d[i];
            if (d > dmax) {
                dmax = d;
            }

            if (d < dmin) {
                dmin = d;
            }
        }
        cv::Mat rgbaImage(height, width, CV_8UC4, data);
        convertDepthToRGBAWithGamma(tmp_d,width,height,dmin,dmax,2.8,rgbaImage);


        cv::flip(rgbaImage, rgbaImage, 1);


        return true;
    }
    bool xslam_get_tof_image(unsigned char *data, int width, int height) {
        if (!s_depthImage)
            return false;

        if (data == nullptr)
            return false;

        if (!s_depthImageMtx.try_lock())
            return false;

        std::shared_ptr<const xv::DepthImage> tmp = s_depthImage;
        unsigned srcWidth = tmp->width;
        unsigned srcHeight = tmp->height;
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy xslam_get_tof_image srcWidth = %d,srcHeight = %d,%d,%d" ,
                            srcWidth,srcHeight,width,height);
        double distance = 4.5;
        float *f = new float[srcWidth * srcHeight];
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "tmp->type  = %d", tmp->type );
        if (tmp->type == xv::DepthImage::Type::Depth_16) {
            const int16_t* depthData = reinterpret_cast<const int16_t*>(tmp->data.get());
            for (unsigned i = 0; i < srcWidth * srcHeight; ++i) {
                f[i] = (float)depthData[i] * 0.001;  // Convert millimeters to meters
#ifdef ANDROID
                if (i < 20 || (i >= srcWidth * srcHeight - 20)) {  // Debug the first and last few pixels
                    LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "Pixel %d: d = %d, f = %f", i, depthData[i], f[i]);
                }
#endif
            }

        }  else {
            xv::TofCamera::Manufacturer tofManu = device->tofCamera()->getManufacturer();
            if (tofManu == xv::TofCamera::Manufacturer::Sony) {
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "tmp->type  = %d", tmp->type );
                s_depthImageMtx.unlock();
                return false;
            } else {
                memcpy(f, tmp->data.get(), tmp->dataSize);
            }
        }

        cv::Mat m(srcHeight, srcWidth, CV_32FC1, f);

        s_depthImageMtx.unlock();

        std::vector<xv::Object> objects;
        s_objMutex.lock();
        if (s_cnnSource == 3)
            objects = s_objects;
        s_objMutex.unlock();

        if (width != srcWidth || height != srcHeight) {
            cv::resize(m, m, cv::Size(height, width));
        }

        cv::Mat map, mask, mask0, mask1, mask2;
        cv::threshold(m, map, 10.5, 0, cv::THRESH_TOZERO_INV);
        cv::threshold(m, mask, 0.2, 1.0, cv::THRESH_BINARY);
        mask.convertTo(mask0, CV_8UC1, 255, 0);
        //cv::threshold(m,mask, 9.5, 1.0, cv::THRESH_BINARY);
        //mask.convertTo(mask1, CV_8UC1, 255, 0);
        //cv::threshold(m,mask, 10.5, 1.0, cv::THRESH_BINARY);
        //mask.convertTo(mask2, CV_8UC1, 255, 0);
        //cv::add(mask0,mask1,mask);
        //cv::threshold(map, map, distance, distance, cv::THRESH_TRUNC);

        cv::Mat adjMap;

        map.convertTo(adjMap, CV_8UC1, 255.0 / std::max(0.01, distance), 0);
        cv::Mat falseColorsMap(m.rows, m.cols, CV_8UC3);
        cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);

        falseColorsMap.copyTo(adjMap, mask0);

        cv::Mat mrgb(height, width, CV_8UC4, data);
        cv::cvtColor(adjMap, mrgb, cv::COLOR_BGR2RGBA);
        double rw = static_cast<double>(width) / static_cast<double>(srcWidth);
        double rh = static_cast<double>(height) / static_cast<double>(srcHeight);

        for (unsigned int i = 0; i < objects.size(); i++) {
            const auto obj = objects.at(i);
            //double rx = width - (rw * obj.x + rw * obj.width);
            //double ry = height - (rh * obj.y + rh * obj.height);
            double rx = rw * obj.x;
            double ry = rh * obj.y;
            const cv::Rect r(rx, ry, rw * obj.width, rh * obj.height);
            cv::rectangle(mrgb, r, cv::Scalar(255, 255, 0));
            std::stringstream stream;
            stream << std::fixed << std::setprecision(2) << 100.0 * obj.confidence;
            std::string str = obj.type + ":" + stream.str() + "%";
            cv::putText(mrgb, str, (r.br() + r.tl()) * 0.5, cv::FONT_HERSHEY_DUPLEX, 1.0,
                        cv::Scalar(255, 255, 0));
        }

        cv::flip(mrgb, mrgb, 1);

        delete[] f;

        return true;
    }

    bool xslam_get_depth_data(float *data) {
        if (!s_depthImage)
            return false;

        if (data == nullptr)
            return false;

        if (!s_depthImageMtx.try_lock())
            return false;

        auto tmp = s_depthImage;

        s_depthImageMtx.unlock();

        memcpy(data, tmp->data.get(), tmp->dataSize);

        return true;
    }

    bool xslam_get_color_depth_data(float *data) {
        if (!s_depthColorImage)
            return false;

        if (data == nullptr)
            return false;

        if (!s_depthColorImageMtx.try_lock())
            return false;

        auto tmp = s_depthColorImage;

        s_depthColorImageMtx.unlock();

        memcpy(data, tmp->data.get(), tmp->width * tmp->height * 7);

        return true;
    }

    void xslam_tof_set_exposure(int aecMode, int exposureGain, float exposureTimeMs) {
        if (device->tofCamera()) {
            device->tofCamera()->setExposure(aecMode, exposureGain, exposureTimeMs);
        }
    }

    void xslam_tof_set_brightness(int brightness) {
        if (device->tofCamera()) {
            device->tofCamera()->setBrightness(brightness);
        }
    }

    int xslam_tof_set_steam_mode(int cmd) {
        bool bOk = false;
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "xslam_tof_set_steam_mode = %d",cmd);
        if (device->tofCamera()) {
            switch (cmd) {
                case 0:
                    bOk = device->tofCamera()->setStreamMode(xv::TofCamera::StreamMode::DepthOnly);
                    break;
                case 1:
                    bOk = device->tofCamera()->setStreamMode(xv::TofCamera::StreamMode::CloudOnly);
                    break;
                case 2:
                    bOk = device->tofCamera()->setStreamMode(
                            xv::TofCamera::StreamMode::DepthAndCloud);
                    break;
                case 3:
                    bOk = device->tofCamera()->setStreamMode(xv::TofCamera::StreamMode::None);
                    break;
                case 4:
                    bOk = device->tofCamera()->setStreamMode(
                            xv::TofCamera::StreamMode::CloudOnLeftHandSlam);
                    break;
                default:
                    break;
            }
            return bOk;
        } else {
            return false;
        }
    }

    void xslam_tof_set_framerate(float framerate) {
        if (device->tofCamera()) {
            device->tofCamera()->setFramerate(framerate);
        }
    }

    void xslam_tof_set_resolution(int resolution) {
        if (device->tofCamera()) {
            device->tofCamera()->setResolution(resolution);
        }
    }
    /**
     * @brief Set brightness level.
     *
     * @param[in] level display brightness level
     */
    void xslam_display_set_brightnesslevel(int level) {
        if (device->display()) {
            device->display()->setBrightnessLevel(level);
        }
    }
    /**
    * @brief 光学模组使能接口 暂时不支持
    */
    void xslam_display_open() {
        if (device->display()) {
            device->display()->open();
        }
    }
    /**
      * @brief 光学模组关闭接口 暂时不支持
      */
    void xslam_display_close() {
        if (device->display()) {
            device->display()->close();
        }
    }

    bool xslam_get_cloud_data_ex(Vector3 *cloud) {
        std::shared_ptr<xv::DepthImage> tof;
        s_depthImageMtx.lock();
        tof = s_depthImage;
        s_depthImageMtx.unlock();

        if (tof && tof->type != xv::DepthImage::Type::IR && tof->type != xv::DepthImage::Type::Depth_32) {
            float *cloud_t = const_cast<float *>(reinterpret_cast<const float *>(tof->data.get()));
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "xslam_get_cloud_data_ex entry width = %d,"
                                                          "height = %d tof type = %d",tof->width,tof->height,tof->type);
            int index = 0;
            for (int i = 0; i < tof->width * tof->height; ++i) {
                cloud[index].x = cloud_t[i * 3];
                cloud[index].y = cloud_t[i * 3 + 1];
                cloud[index].z = cloud_t[i * 3 + 2];
                ++index;
            }

            return true;
        } else {
            return false;
        }

    }

/**
 * @brief 获取点云数据。
 *
 * 该函数从设备中获取点云数据，并将结果存储到传入的 `Vector3` 数组中。点云数据通常用于三维空间的重建或环境感知。
 *
 * @param cloud 用于存储点云数据的指针。调用函数前需要确保该指针有效，且指向的内存空间足够存储点云数据。
 *
 * @return bool
 * - `true` 表示成功获取点云数据并填充到 `cloud` 中。
 * - `false` 表示获取点云数据失败（例如设备未初始化、设备不支持点云功能，或点云数据不可用）。
 */
    bool xslam_get_cloud_data(Vector3 *cloud) {
        if (s_depthImage == nullptr)
            return false;
        s_depthImageMtx.lock();
        auto depthImage_ = s_depthImage;
        s_depthImageMtx.unlock();
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "xslam_get_cloud_data entry 1");
        auto pointCloud = device->tofCamera()->depthImageToPointCloud(*depthImage_);
        if(pointCloud){
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "xslam_get_cloud_data entry 2");
            for (int i = 0; i < pointCloud->points.size(); i++) {
                cloud[i].x = pointCloud->points.at(i)[0];
                cloud[i].y = pointCloud->points.at(i)[1];
                cloud[i].z = pointCloud->points.at(i)[2];
            }
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "xslam_get_cloud_data first x = %f",
                                cloud[10000].x);
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "xslam_get_cloud_data first y = %f",
                                cloud[10000].y);
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "xslam_get_cloud_data first z = %f",
                                cloud[10000].z);
#endif
            return true;
        } else {
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "xslam_get_cloud_data entry 3");
            return false;
        }
    }

    bool xslam_get_imu(Vector3 *accel, Vector3 *gyro, Vector3 *magn, long long *timestamp) {
        return false;
    }

    bool xslam_get_imu_array(Vector3 *imu, double *timestamp) {
        if (s_imu) {
            s_imuMutex.lock();
            if (imu) {
                imu[0].x = s_imu->accel[0];
                imu[0].y = s_imu->accel[1];
                imu[0].z = s_imu->accel[2];
                imu[1].x = s_imu->gyro[0];
                imu[1].y = s_imu->gyro[1];
                imu[1].z = s_imu->gyro[2];
                imu[2].x = s_imu->magneto[0];
                imu[2].y = s_imu->magneto[1];
                imu[2].z = s_imu->magneto[2];
            }
            if (timestamp) {
                *timestamp = s_imu->hostTimestamp;
            }
            s_imuMutex.unlock();
            return true;
        }

        return false;
    }

    bool xslam_get_event(int *type, int *state, long long *timestamp) {
        s_eventMutex.lock();
        if(s_event){
            *type = s_event->type;
            *state = s_event->state;
            *timestamp = s_event->edgeTimestampUs;
        }
        s_eventMutex.unlock();
        return true;
    }

    bool xslam_get_3dof(Orientation *o) {
        if (s_orientation) {
            s_oriMutex.lock();
            memcpy(o, s_orientation.get(), sizeof(Orientation));
            s_oriMutex.unlock();
            return true;
        }
        return false;
    }
//    bool xslam_enable_surface_reconstruction(bool enable,int callbackId)
//    {
//        xv::SlamEx *slamEx = dynamic_cast<xv::SlamEx *>(device->slam().get());
//        if(enable){
//            return slamEx->startSurfaceReconstruction();
//        } else{
//            slamEx->unregisterSurfaceCallback(callbackId);
//            slamEx->setEnableSurfaceReconstruction(false);
//            device->tofCamera()->stop();
//            return slamEx->stopSurfaceReconstruction();
//        }
//    }
    bool xslam_enable_surface_reconstruction(bool enable,int callbackId)
    {
        xv::SlamEx *slamEx = dynamic_cast<xv::SlamEx *>(device->slam().get());
        if(enable){
            xv::TofCamera::Manufacturer tofManu = device->tofCamera()->getManufacturer();
            if (tofManu == xv::TofCamera::Manufacturer::Sony)
            {
                device->tofCamera()->setSonyTofSetting(xv::TofCamera::SonyTofLibMode::IQMIX_DF,
                                                       xv::TofCamera::Resolution::QVGA,
                                                       xv::TofCamera::Framerate::FPS_5);
            }
            else if (tofManu == xv::TofCamera::Manufacturer::Pmd)
            {
                //默认pmd tof 配置
                device->tofCamera()->setFramerate(5.);

                usleep(1000 * 1000);
                device->tofCamera()->setFramerate(15.);
                usleep(1000 * 1000);
                device->tofCamera()->setFramerate(5.);
            }
            bool useTof = device->tofCamera()->start();
            sleep(1);
            return slamEx->startSurfaceReconstruction();
        } else{
//            slamEx->unregisterSurfaceCallback(callbackId);
//            slamEx->setEnableSurfaceReconstruction(false);
            device->tofCamera()->stop();
            return slamEx->stopSurfaceReconstruction();
        }
    }

    int
    xslam_start_surface_callback(bool enableSuface, bool enableTexturing, fn_surface_callback cb) {
#ifdef ANDROID
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "registerSurfaceCallback 0");

#endif
        device->slam()->stop();
        if (device && device->tofCamera() && device->slam()) {

            xslam_tof_set_steam_mode(0);
            //     LOG_DEBUG("eddy useTof %d", useTof);
            /// Workaround for ToF lazer that do not power up
            xv::TofCamera::Manufacturer tofManu = device->tofCamera()->getManufacturer();
            if (tofManu == xv::TofCamera::Manufacturer::Sony)
            {
                device->tofCamera()->setSonyTofSetting(xv::TofCamera::SonyTofLibMode::IQMIX_DF,
                                                       xv::TofCamera::Resolution::QVGA,
                                                       xv::TofCamera::Framerate::FPS_5);
            }
            else if (tofManu == xv::TofCamera::Manufacturer::Pmd)
            {
                //默认pmd tof 配置
                device->tofCamera()->setFramerate(5.);

                usleep(1000 * 1000);
                device->tofCamera()->setFramerate(15.);
                usleep(1000 * 1000);
                device->tofCamera()->setFramerate(5.);
            }
            bool useTof = device->tofCamera()->start();
            surfacecb = cb;
            xv::SlamEx *slamEx = dynamic_cast<xv::SlamEx *>(device->slam().get());
            usleep(1000 * 1000);
            slamEx->setEnableSurfaceReconstruction(true);
            slamEx->setEnableSurfaceTexturing(false); //false
            slamEx->startSurfaceReconstruction();
//            device->slam()->start();
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "registerSurfaceCallback 1");

#endif
//            slamEx->setEnableSurfaceMultiResolutionMesh(true);//(args["surface-multi-resolution"].as<bool>());
//            slamEx->setEnableSurfaceMobileObjects(true);
            slamEx->setSurfaceUseFisheyeTexturing(false);
            // slamEx->setSurfaceMinVoxelSize(0.2);
            int calllbackId = slamEx->registerSurfaceCallback(
                    [](std::shared_ptr<const xv::ex::Surfaces> surfaces) {
#ifdef ANDROID
                        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                  "eddy registerSurfaceCallback entry");

#endif
                        s_surfaceMtx.lock();
                        std::map<std::uint64_t, xv::ex::Surface> s_surfaces_map = surfaces->surfaces;
                        s_surfaceMtx.unlock();
                        //    LOG_DEBUG("eddy registerSurfaceCallback size %d", surfaces->surfaces.size());
                        XslamSurface xsf[surfaces->surfaces.size()];
                        int surfaceId = 0;
                        if (surfaces->surfaces.size() > 0) {
                            //    ++pkgId;
                            std::size_t totalVertices = 0, totalTriangles = 0;
                            for (auto const &s : s_surfaces_map) {

                                if (surfaceId + 1 > s_surfaces_map.size())
                                    continue;
                                xsf[surfaceId].mapId = s.first;

                                xsf[surfaceId].id = s.second.id;
                                xsf[surfaceId].version = s.second.version;
                                xsf[surfaceId].verticesSize = s.second.verticesSize;
                                xsf[surfaceId].vertices = new Vector3[s.second.verticesSize];
                                xsf[surfaceId].vertexNormals = new Vector3[s.second.verticesSize];

                                for (int i = 0; i < s.second.verticesSize; ++i) {
                                    if (s.second.vertices && s.second.vertices.get()) {
                                        const auto &vertices = s.second.vertices.get()[i];

                                        xsf[surfaceId].vertices[i].x = vertices[0];
                                        xsf[surfaceId].vertices[i].y = vertices[1];
                                        xsf[surfaceId].vertices[i].z = vertices[2];

                                        if (s.second.vertexNormals.get()) {
                                            const auto &vertexNormals = s.second.vertexNormals.get()[i];
                                            xsf[surfaceId].vertexNormals[i].x = vertexNormals[0];
                                            xsf[surfaceId].vertexNormals[i].y = vertexNormals[1];
                                            xsf[surfaceId].vertexNormals[i].z = vertexNormals[2];
                                        }
                                    }

                                    /*     xsf[surfaceId].textureRgba = s.second.textureRgba.get();
                                                                           if (s.second.textureRgba)
                                                                          {

                                                                              const auto &textureCoordinates = s.second.textureCoordinates.get()[i];
#ifdef ANDROID
                                                                              LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy registerSurfaceCallback textureCoordinates x %d", textureCoordinates[0]);

#endif
                                                                              xsf[surfaceId].textureCoordinates[i].x = textureCoordinates[0];
                                                                              xsf[surfaceId].textureCoordinates[i].y = textureCoordinates[1];
                                                                          } */
                                }
                                xsf[surfaceId].trianglesSize = s.second.trianglesSize;
#ifdef ANDROID
                                LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                          "eddy registerSurfaceCallback trianglesSize = %d",
                                          s.second.trianglesSize);

#endif
                                xsf[surfaceId].triangles = new Vector3uint[s.second.trianglesSize];
                                for (int j = 0; j < s.second.trianglesSize; j++) {

                                    const auto &triangles = s.second.triangles.get()[j];

                                    xsf[surfaceId].triangles[j].x = triangles[0];
                                    xsf[surfaceId].triangles[j].y = triangles[1];
                                    xsf[surfaceId].triangles[j].z = triangles[2];
                                }
                                xsf[surfaceId].textureWidth = s.second.textureWidth;
                                xsf[surfaceId].textureHeight = s.second.textureHeight;
                                surfaceId++;
                            }
                            if (surfacecb && surfaces->surfaces.size() > 0) {
                                surfacecb(xsf, surfaces->surfaces.size());
                            }
                        }
                    });
            device->slam()->start();
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "registerSurfaceCallback 2");

#endif
            return calllbackId;
        } else {
            return -1;
        }
#else
        return -1;
#endif
    }
    void xslam_stop_surface_callback(int callbackId){
        xv::SlamEx *slamEx = dynamic_cast<xv::SlamEx *>(device->slam().get());
        slamEx->unregisterSurfaceCallback(callbackId);
        slamEx->setEnableSurfaceReconstruction(false);
        slamEx->stopSurfaceReconstruction();
        device->tofCamera()->stop();
    }
    void xslam_start_orientation_stream() {
        if (device->orientationStream()) {
            device->orientationStream()->start();
        }
    }

    void xslam_stop_orientation_stream() {
        if (device->orientationStream()) {
            device->orientationStream()->stop();
        }
    }


    bool xslam_reset_slam() {
        if (device && device->slam()) {
            isSlamStart = false;
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "xslam_reset_slam gSlamStartMode = %d",gSlamStartMode);
#endif
         /*   if(static_cast<xv::SlamStartMode>(gSlamStartMode) == xv::SlamStartMode::Normal){
                return device->slam()->reset();
            } else {*/

                device->slam()->stop();

                usleep(1000 * 1000);
#ifdef ANDROID
                LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "xslam_reset_slam other start");
#endif
                bool ret =  device->slam()->start();
                isSlamStart = true;
                return ret;
       //     }
        } else {
            return false;
        }
    }

    bool xslam_set_aec(int p1, int p2, int p3, int p4, int p5, int p6) {
        return false;
    }

    bool
    xslam_set_imu_configuration(int mode, int stereoOffsetUs, int edgePredUs, bool edgeImuFusion,
                                bool imuSyncedWithinEdgePacket) {
        /*   if(device->imuSensor()){
            return s_hid->setImuConfiguration(static_cast<XSlam::HID::ImuMode>(mode), stereoOffsetUs, edgePredUs, edgeImuFusion, imuSyncedWithinEdgePacket);
        } */
        return false;
    }

    bool xslam_set_flip(bool flip) {
        return false;
    }

    bool xslam_set_post_filter(bool enabled, float rotationParam, float translationParam) {
        return false;
    }

    bool xslam_set_imu_fusion(int imuFusionMode, bool synced, float delay, float prediction) {
        return false;
    }

    static fn_skeleton_callback skeleton_cb;
    static fn_gesture_callback gesture_cb;
    static fn_skeleton_debug_callback skeleton_debug_cb;
    int xslam_start_skeleton_with_cb(int type, fn_skeleton_callback cb) {
        if (cb == nullptr) {
            return -1;
        }
        skeleton_cb = cb;
#ifdef XV_GESTURE
        if (device->gesture()) {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "xslam_start_skeleton_with_cb 2");

#endif
            device->gesture()->start();
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "start_skeleton 3");

#endif
            if (type == 0) {
                return device->gesture()->registerKeypointsCallback(
                        [](std::shared_ptr<const std::vector<xv::keypoint>> keypoints) {
                            s_handMutex.lock();
                            s_keypoints = keypoints;
                            XslamSkeleton skelet = {0};

                            int count = s_keypoints->size();

#ifdef ANDROID
                            if (count != 0) {
                                LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                                    "KeypointsCallback count:%d,[0]:%f", count,
                                                    s_keypoints.get()->data()[0].x);
                            } else {
                                LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                                    "KeypointsCallback count:%d", count);
                            }

#endif
                            if (count != 0) {
                                if (count > 21) {
                                    skelet.size = 2;
                                } else {
                                    skelet.size = 1;
                                }
                                for (int i = 0; i < count; i++) {
                                    /*    if (i > 20)
                                                                            {
                                                                                skelet.joints[1][i - 21].x = s_keypoints.get()->data()[i].x;
                                                                                skelet.joints[1][i - 21].y = s_keypoints.get()->data()[i].y;
                                                                                skelet.joints[1][i - 21].z = s_keypoints.get()->data()[i].z;
                                                                            }
                                                                            else
                                                                            {
                                                                                skelet.joints[0][i].x = s_keypoints.get()->data()[i].x;
                                                                                skelet.joints[0][i].y = s_keypoints.get()->data()[i].y;
                                                                                skelet.joints[0][i].z = s_keypoints.get()->data()[i].z;
                                                                            }*/
                                }
                            }

                            skeleton_cb(skelet);
                            s_handMutex.unlock();
                        });
            } /*else if (type == 1) {
#ifdef ANDROID
                LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                    "SlamKeypointsCallback register start");

#endif
                 return device->gesture()->registerSlamKeypointsCallback([](std::shared_ptr<const xv::HandPose> handpose)
                                                                    {
                                                                        XslamSkeleton skelet = {0};
                                                                        int index = 0;
                                                                        skelet.size = handpose->pose.size();
                                                                        for (auto pose : handpose->pose)
                                                                        {
                                                                            skelet.joints_ex[index].x = pose.x();
                                                                            skelet.joints_ex[index].y = pose.y();
                                                                            skelet.joints_ex[index].z = pose.z();
                                                                            auto r = pose.rotation();
                                                                            auto _quat = xv::rotationToQuaternion(r);
                                                                            skelet.poseData[index].x = _quat[0];
                                                                            skelet.poseData[index].y = _quat[1];
                                                                            skelet.poseData[index].z = _quat[2];
                                                                            skelet.poseData[index].w = _quat[3];
                                                                            index++;
                                                                        }

#ifdef ANDROID
                                                                        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                                                                             "unity-wrapper gesture KeypointsCallback count:%d",
                                                                                             skelet.size);
                                                                         if (index > 0)
                                                                         {
                                                                             LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                                                                                 "unity-wrapper gesture pose[0].xyz:(%f,%f,%f),pose[index-1].xyz:(%f,%f,%f)",
                                                                                                 skelet.joints_ex[0].x, skelet.joints_ex[0].y,
                                                                                                 skelet.joints_ex[0].z,
                                                                                                 skelet.joints_ex[index - 1].x,
                                                                                                 skelet.joints_ex[index - 1].y,
                                                                                                 skelet.joints_ex[index - 1].z);
                                                                         }

#endif
                                                                         index = 0;
                                                                         for (const auto &item : handpose->scale){
                                                                             skelet.scale[index] = item;
                                                                             index ++;
                                                                         }
                                                                        skeleton_cb(skelet);
                                                                    });
            }*/
        }
#endif
        return -1;
    }

    int xslam_start_gesture_ex(const std::string &path) {
#ifdef ANDROID
        // if (device->gestureEX()) {
            //    device->gestureEX()->start(JVM, password);
        // }
#endif
        return 0;
    }

  /*  int xslam_start_skeleton_debug_with_cb(fn_skeleton_debug_callback cb) {

#ifdef ANDROID
        if (cb == nullptr) {
            return -1;
        }
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "unity-wrapper gesture xslam_start_skeleton_debug_with_cb start");

#endif
        skeleton_debug_cb = cb;
        void *JVM;
#ifdef XV_GESTURE
        if (device->gesture()) {
            device->gesture()->start();

            return  device->gesture()->registerSlamKeypointsDebugCallback([](std::shared_ptr<const xv::PoseDebug> posePtr)

                                                                     {
                                                                         XslamSkeletonDebug skelet = {0};
                                                                         int index = 0;
                                                                         skelet.size = posePtr->gesture_pose.size();

#ifdef ANDROID
                                                                         LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                                                                             "unity-wrapper gesture KeypointsCallback count:%d",
                                                                                             skelet.size);

#endif

                                                                         for (auto pose : posePtr->gesture_pose)
                                                                         {

                                                                             skelet.joints_ex[index].x = pose.x();
                                                                             skelet.joints_ex[index].y = pose.y();
                                                                             skelet.joints_ex[index].z = pose.z();
                                                                             auto r = pose.rotation();
                                                                             auto _quat = xv::rotationToQuaternion(r);
                                                                             skelet.poseData[index].x = _quat[0];
                                                                             skelet.poseData[index].y = _quat[1];
                                                                             skelet.poseData[index].z = _quat[2];
                                                                             skelet.poseData[index].w = _quat[3];
                                                                             index++;
                                                                         };
                                                                         memcpy(skelet.hand_uv_leftcam,posePtr->hand_uv_leftcam,sizeof(posePtr->hand_uv_leftcam));
                                                                         memcpy(skelet.hand_uv_rightcam,posePtr->hand_uv_rightcam,sizeof(posePtr->hand_uv_rightcam));
                                                                         memcpy(skelet.handpose_data,posePtr->handpose_data,sizeof(posePtr->handpose_data));
                                                                         skeleton_debug_cb(skelet);
                                                                     });
        } else {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                "unity-wrapper gesture xslam_start_skeleton_ex_with_cb device->gesture() return false");

#endif
        }
#else
        if (device->gestureEX()) {
            device->gestureEX()->start(gJavaVM, "");

            return device->gestureEX()->registerPosCallback(
                    [](std::shared_ptr<const std::vector<xv::Pose>> poses) {
                        XslamSkeleton skelet = {0};
                        int index = 0;
                        skelet.size = poses->size();
                        for (auto pose : *poses.get()) {

                            skelet.joints_ex[index].x = pose.x();
                            skelet.joints_ex[index].y = pose.y();
                            skelet.joints_ex[index].z = pose.z();
                            auto r = pose.rotation();
                            auto _quat = xv::rotationToQuaternion(r);
                            skelet.poseData[index].x = _quat[0];
                            skelet.poseData[index].y = _quat[1];
                            skelet.poseData[index].z = _quat[2];
                            skelet.poseData[index].w = _quat[3];
                            index++;
                        }
                        skeleton_cb(skelet);
                    });
        }
#endif
#endif
        return -1;
    }*/
    void xslam_set_gesture_platform(int platform){
            s_gesturePlatform = platform;
    }
    void xslam_set_gesture_ego(bool ego){
        s_gestureEgo = ego;
    }
    int xslam_start_skeleton_ex_with_cb(fn_skeleton_callback cb) {
#ifdef ANDROID
        if (cb == nullptr) {
            return -1;
        }
        if (!s_stereoImage){
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                "eddy stereo Image is null");

#endif
        }

#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "unity-wrapper gesture xslam_start_skeleton_ex_with_cb start");
#endif
        skeleton_cb = cb;
        void *JVM;
#ifdef XV_GESTURE
        if (device->gesture()) {
            device->gesture()->setPlatform(s_gesturePlatform,s_gestureEgo);
            bool res =  device->gesture()->start();
            if(res == false){
#ifdef ANDROID
                LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                    "unity-wrapper gesture xslam_start_skeleton_ex_with_cb start return false");
#endif
                device->gesture()->setPlatform(1,s_gestureEgo);
                device->gesture()->start();
            }
            return  device->gesture()->registerSlamKeypointsCallback([](std::shared_ptr<const xv::HandPose> handpose)
                                                                     {
                                                                         XslamSkeleton skelet = {0};
                                                                         int index = 0;
                                                                         skelet.size = handpose->pose.size();

                                                                         for (auto pose :  handpose->pose)
                                                                         {

                                                                             skelet.joints_ex[index].x = pose.x();
                                                                             skelet.joints_ex[index].y = pose.y();
                                                                             skelet.joints_ex[index].z = pose.z();
                                                                             auto r = pose.rotation();
                                                                             auto _quat = xv::rotationToQuaternion(r);
                                                                             skelet.poseData[index].x = _quat[0];
                                                                             skelet.poseData[index].y = _quat[1];
                                                                             skelet.poseData[index].z = _quat[2];
                                                                             skelet.poseData[index].w = _quat[3];
                                                                             index++;
                                                                         }
                                                                         index = 0;
                                                                      /*   for (const float item : handpose->scale){
                                                                             skelet.scale[index] = item;
                                                                             index ++;
                                                                         }*/
                                                                         skelet.fisheye_timestamp = handpose->fisheye_timestamp;
                                                                         for (int i = 0; i < 2; ++i) {
                                                                             skelet.scale[i] = handpose->scale[i];
                                                                             skelet.timestamp[i] = handpose->timestamp[i];
                                                                             skelet.status[i] = handpose->status[i];
                                                                         }


                                                                         skeleton_cb(skelet);
                                                                     });
        } else {
            #ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "unity-wrapper gesture xslam_start_skeleton_ex_with_cb device->gesture() return false");

#endif
        }
#else
        if (device->gestureEX()) {
            device->gestureEX()->start(gJavaVM, "");

            return device->gestureEX()->registerPosCallback(
                    [](std::shared_ptr<const std::vector<xv::Pose>> poses) {
                        XslamSkeleton skelet = {0};
                        int index = 0;
                        skelet.size = poses->size();
                        for (auto pose : *poses.get()) {

                            skelet.joints_ex[index].x = pose.x();
                            skelet.joints_ex[index].y = pose.y();
                            skelet.joints_ex[index].z = pose.z();
                            auto r = pose.rotation();
                            auto _quat = xv::rotationToQuaternion(r);
                            skelet.poseData[index].x = _quat[0];
                            skelet.poseData[index].y = _quat[1];
                            skelet.poseData[index].z = _quat[2];
                            skelet.poseData[index].w = _quat[3];
                            index++;
                        }
                        skeleton_cb(skelet);
                    });
        }
#endif
#endif
        return -1;
    }

    int xslam_start_gesture_ex_with_cb(fn_gesture_callback cb) {
        if (cb == nullptr) {
            return -1;
        }
#ifdef ANDROID
        gesture_cb = cb;
        void *JVM;
        if (true) {
            //  device->gestureEX()->start();

//            return device->gestureEX()->registerCallback([](xv::GestureData const &gesture) {
//                for (int i = 0; i < 2; i++) {
//                    if (gesture.index[i] != -1) {
//                        std::cout << "gesture host timestamp = " << gesture.hostTimestamp
//                                  << std::endl;
//                        std::cout << "gesture edge timestamp = " << gesture.edgeTimestampUs
//                                  << std::endl;
//                        std::cout << "gesture index = " << gesture.index[i] << std::endl;
//                    }
//                }
//                if (gesture_cb) {
//                    gesture_cb(gesture);
//                }
//            });
        }
#endif
        return -1;
    }

    bool xslam_stop_slam_skeleton_with_cb(int type, int id) {
        if (type == 0) {
            return device->gesture()->unregisterKeypointsCallback(id);
        } else if (type == 1) {
            return device->gesture()->unregisterSlamKeypointsCallback(id);
        }
        return false;
    }

    int xslam_start_gesture_with_cb(int type, fn_gesture_callback cb) {
        if (cb == nullptr) {
            return -1;
        }
        gesture_cb = cb;
        if (device->gesture()) {
            device->gesture()->start();

            if (type == 0) {
                return device->gesture()->registerCallback([](xv::GestureData const &gesture) {
                    s_GestureMutex.lock();
                    s_gestureData = std::make_shared<xv::GestureData>(gesture);
                    /*          XslamGesture xgesture = {0};
                                                           xgesture.index[0] = gesture.index[0];
                                                           xgesture.index[1] = gesture.index[1];
                                                           xgesture.position[0] = gesture.position[0];
                                                           xgesture.position[1] = gesture.position[1];
                                                           xgesture.hostTimestamp = gesture.hostTimestamp;
                                                           xgesture.edgeTimestampUs = gesture.edgeTimestampUs; */
                    gesture_cb(gesture);
                    s_GestureMutex.unlock();
                });
            } else if (type == 1) {
#ifdef ANDROID
                LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                    "DynamicGestureCallback register start");

#endif
                return device->gesture()->registerDynamicGestureCallback(
                        [](xv::GestureData const &gesture) {
                            s_GestureMutex.lock();
                            s_gestureData = std::make_shared<xv::GestureData>(gesture);
                            /*       XslamGesture xgesture = {0};
                                                                         xgesture.index[0] = gesture.index[0];
                                                                         xgesture.index[1] = gesture.index[1];
                                                                         xgesture.slamPosition[0] = gesture.slamPosition[0];
                                                                         xgesture.slamPosition[1] = gesture.slamPosition[1];
                                                                         xgesture.dataFetchTimeMs = gesture.dataFetchTimeMs;
                                                                         xgesture.dataTimeStampMs = gesture.dataTimeStampMs; */
                            gesture_cb(gesture);
                            s_GestureMutex.unlock();
                        });
            }
        }
        return -1;
    }

    int xslam_start_skeleton() {
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "xslam_start_skeleton 1");

#endif
        if (device->gesture()) {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "xslam_start_skeleton 2");

#endif
            device->gesture()->start();
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "xslam_start_skeleton 3");

#endif
            return device->gesture()->registerKeypointsCallback(
                    [](std::shared_ptr<const std::vector<xv::keypoint>> keypoints) {
                        s_handMutex.lock();
                        s_keypoints = keypoints;
#ifdef ANDROID
                        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                            "eddy s_keypoints size (%d)", s_keypoints->size());
                        if (s_keypoints->size() != 0) {
                            for (int i = 0; i < 21; i++) {
                                LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                                    "eddy s_keypoints size (%d)",
                                                    s_keypoints->size());
                                LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                                    "eddy s_keypoints (%d)",
                                                    s_keypoints.get()->data()[i].x);
                            }
                        }

                        //    LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy s_keypoints (%d)", s_keypoints.get()->data()[0].x);
#endif
                        s_handMutex.unlock();
                    });
        }
    }

    bool xslam_stop_skeleton(int id) {
        return device->gesture()->unregisterKeypointsCallback(id);
    }

    int xslam_start_slam_skeleton() {
        /*    if (device->gesture())
    {

        return device->gesture()->registerSlamKeypointsCallback([](std::shared_ptr<const std::vector<xv::keypoint>> keypoints)
                                                                {
#ifdef ANDROID
                                                                    LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "SlamKeypointsCallback xslam_start_slam_skeleton start");

#endif
                                                                    s_slamHandMutex.lock();
                                                                    s_slamKeypoints = keypoints;
                                                                    s_slamHandMutex.unlock();
                                                                });
    }*/
        return 0;
    }

    bool xslam_stop_slam_skeleton(int id) {
        return device->gesture()->unregisterSlamKeypointsCallback(id);
    }

    int xslam_start_gesture() {
        if (device->gesture()) {
            s_gesture_callback_id = device->gesture()->registerCallback(
                    [](xv::GestureData const &gesture) {
                        s_GestureMutex.lock();
                        s_gestureData = std::make_shared<xv::GestureData>(gesture);
#ifdef ANDROID
                        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                            "eddy s_gestureData index (%d)",
                                            s_gestureData->index[0]);
                        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                            "eddy s_gestureData position (%d)",
                                            s_gestureData->position[0].x);
                        //    LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy s_keypoints (%d)", s_keypoints.get()->data()[0].x);
#endif
                        s_GestureMutex.unlock();
                    });
            return s_gesture_callback_id;
        }
    }

    void xslam_stop_gesture() {
        device->gesture()->unregisterCallback(s_gesture_callback_id);
        device->gesture()->stop();
    }

    int xslam_start_dynamic_gesture() {
        if (device->gesture()) {

            return device->gesture()->registerDynamicGestureCallback(
                    [](xv::GestureData const &gesture) {
                        s_DynamicGestureMutex.lock();
                        s_DynamicgestureData = std::make_shared<xv::GestureData>(gesture);
                        s_DynamicGestureMutex.unlock();
                    });
        }
    }

    bool xslam_stop_dynamic_gesture(int id) {
        return device->gesture()->UnregisterDynamicGestureCallback(id);
    }

//https://blog.csdn.net/su317/article/details/2109124 返回结构体数组
    ObjectData *xslam_get_objects(int *num) {

#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy xslam_get_objects start");
#endif
        std::vector<xv::Object> objects;
        s_objMutex.lock();
        objects = s_objects;

        s_objMutex.unlock();
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy xslam_get_objects size : %d",
                            objects.size());
        if (objects.size() > 0) {
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy xslam_get_objects typeId : %lu",
                                objects.at(0).typeID);
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr",
                                "eddy xslam_get_objects typshapeeId : %s",
                                objects.at(0).type.c_str());
        }

#endif
        if (objects.size() > 0) {
            ObjectData *objectsPtr = new ObjectData[objects.size()];
            for (unsigned int i = 0; i < objects.size(); i++) {
                objectsPtr[i].x = objects.at(i).x;
                objectsPtr[i].y = objects.at(i).y;
                // objectsPtr[i].blobIndex = objects.at(i).blobIndex;
                objectsPtr[i].width = objects.at(i).width;
                objectsPtr[i].height = objects.at(i).height;
                objectsPtr[i].confidence = objects.at(i).confidence;
                objectsPtr[i].type = (char *) objects.at(i).type.c_str();
                objectsPtr[i].keypoints = new Vector3[objects.at(i).keypoints.size()];
                objectsPtr[i].pointsSize = objects.at(i).keypoints.size();
                for (int j = 0; j < objects.at(i).keypoints.size(); j++) {
                    objectsPtr[i].keypoints[j].x = objects.at(i).keypoints[j].x;
                    objectsPtr[i].keypoints[j].y = objects.at(i).keypoints[j].y;
                    objectsPtr[i].keypoints[j].z = objects.at(i).keypoints[j].z;
#ifdef ANDROID
                    LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy objects keypoints x  %f",
                                        objectsPtr[i].keypoints[j].x);
                    LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy objects keypoints y %f",
                                        objectsPtr[i].keypoints[j].y);
                    LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy objects keypoints z %f",
                                        objectsPtr[i].keypoints[j].z);
#endif
                }

#ifdef ANDROID
                //    LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy xslam_get_objects blobIndex : %s", objectsPtr[i].blobIndex);

                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr",
                                    "eddy xslam_get_objects typeId : %s", objectsPtr[i].type);
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr",
                                    "eddy xslam_get_objects confidence : %f",
                                    objectsPtr[i].confidence);
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr",
                                    "eddy xslam_get_objects keypoints.size : %d",
                                    objects.at(i).keypoints.size());
#endif
            }
            *num = (int) objects.size();

            return objectsPtr;
        } else {
            *num = 0;
            return nullptr;
        }
    }

    bool xslam_get_hand_keypoints(hand_keypoints *keypoints, int type) {
        // memcpy(keypoints,&s_keypoints.get()->data()[0], 21);
        if (type == 0) {
            if (s_keypoints) {
                s_handMutex.lock();
                if (s_keypoints->size() != 0) {
                    for (int i = 0; i < 21; i++) {
                        keypoints->point[i].x = s_keypoints.get()->data()[i].x;
                        keypoints->point[i].y = s_keypoints.get()->data()[i].y;
                        keypoints->point[i].z = s_keypoints.get()->data()[i].z;
                    }
                }

                s_handMutex.unlock();
                return true;
            } else
                return false;
        } else {
            if (s_slamKeypoints) {
                s_slamHandMutex.lock();
                if (s_slamKeypoints->size() != 0) {
                    for (int i = 0; i < 21; i++) {
                        keypoints->point[i].x = s_slamKeypoints.get()->data()[i].x;
                        keypoints->point[i].y = s_slamKeypoints.get()->data()[i].y;
                        keypoints->point[i].z = s_slamKeypoints.get()->data()[i].z;
                    }
                }

                s_slamHandMutex.unlock();
                return true;
            } else
                return false;
        }
    }

    bool xslam_get_gesture(GestureData *gesture) {
        if (s_gestureData) {
            s_GestureMutex.lock();
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy xslam_get_gesture entry");
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy xslam_get_gesture index (%d)",
                                s_gestureData->index[0]);
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy xslam_get_gesture position (%d)",
                                s_gestureData->position[0].x);
            //    LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy s_keypoints (%d)", s_keypoints.get()->data()[0].x);
#endif
            for (int i = 0; i < 2; i++) {
                gesture->index[i] = s_gestureData->index[i];
                gesture->position[i].x = s_gestureData->position[i].x;
                gesture->position[i].y = s_gestureData->position[i].y;
                gesture->position[i].z = s_gestureData->position[i].z;
                gesture->slamPosition[i].x = s_gestureData->slamPosition[i].x;
                gesture->slamPosition[i].y = s_gestureData->slamPosition[i].y;
                gesture->slamPosition[i].z = s_gestureData->slamPosition[i].z;
            }
            gesture->confidence = s_gestureData->confidence;
            gesture->hostTimestamp = s_gestureData->hostTimestamp;
            gesture->edgeTimestampUs = s_gestureData->edgeTimestampUs;
            s_GestureMutex.unlock();
            return true;
        } else {
            return false;
        }
    }

    bool xslam_get_dynamic_gesture(GestureData *gesture) {

        if (s_DynamicgestureData)
            return false;
        if (s_DynamicgestureData) {
            s_DynamicGestureMutex.lock();

            for (int i = 0; i < 2; i++) {
                gesture->index[i] = s_DynamicgestureData->index[i];
                gesture->position[i].x = s_DynamicgestureData->position[i].x;
                gesture->position[i].y = s_DynamicgestureData->position[i].y;
                gesture->position[i].z = s_DynamicgestureData->position[i].z;
                gesture->slamPosition[i].x = s_DynamicgestureData->slamPosition[i].x;
                gesture->slamPosition[i].y = s_DynamicgestureData->slamPosition[i].y;
                gesture->slamPosition[i].z = s_DynamicgestureData->slamPosition[i].z;
            }
            gesture->confidence = s_DynamicgestureData->confidence;
            gesture->hostTimestamp = s_DynamicgestureData->hostTimestamp;
            gesture->edgeTimestampUs = s_DynamicgestureData->edgeTimestampUs;
            s_DynamicGestureMutex.unlock();
        }
        return true;
    }

    bool xslam_set_rgb_source(RgbSource source) {
        s_rgbSource = source;
        return true;
    }
    /**
     * @brief 设置RGB彩色相机的分辨率。
     *
     * 此函数根据指定的分辨率调整RGB相机的分辨率，如果设备包含彩色相机。
     * 支持多种预定义分辨率。
     *
     * @param res 需要设置的分辨率，类型为枚举值 RgbResolution。
     *            支持的值包括：
     *            - RGB_1920x1080: 分辨率为 1920x1080
     *            - RGB_1280x720: 分辨率为 1280x720
     *            - RGB_640x480: 分辨率为 640x480
     *            - RGB_320x240: 分辨率为 320x240
     *            - RGB_2560x1920: 分辨率为 2560x1920
     *
     * @return 如果分辨率设置成功，返回 true；如果设备没有彩色相机或操作失败，返回 false。
     */
    bool xslam_set_rgb_resolution(RgbResolution res) {
        if (device->colorCamera()) {
            switch (res) {
                case RGB_1920x1080:
                    return device->colorCamera()->setResolution(
                            xv::ColorCamera::Resolution::RGB_1920x1080);
                    break;
                case RGB_1280x720:
                    return device->colorCamera()->setResolution(
                            xv::ColorCamera::Resolution::RGB_1280x720);
                    break;
                case RGB_640x480:
                    return device->colorCamera()->setResolution(
                            xv::ColorCamera::Resolution::RGB_640x480);
                    break;
                case RGB_320x240:
                    return device->colorCamera()->setResolution(
                            xv::ColorCamera::Resolution::RGB_320x240);
                    break;
                case RGB_2560x1920:
                    return device->colorCamera()->setResolution(
                            xv::ColorCamera::Resolution::RGB_2560x1920);
                    break;
            }
        } else {
            return false;
        }
    }

    void xslam_start_imu() {
        if (device->imuSensor()) {
            device->imuSensor()->registerCallback([](xv::Imu const &imu) {
                s_imuMutex.lock();
                s_imu = std::make_shared<xv::Imu>(imu);
                s_imuMutex.unlock();
            });
            device->imuSensor()->start();
        }
    }

    void xslam_stop_imu() {
        if (device)
            device->imuSensor()->stop();
    }

    void xslam_start_rgb_stream() {
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy xslam_start_rgb_stream entry");

#endif

        // VSC may not support rgb, so must use if
        if (device && device->colorCamera()) {

            device->colorCamera()->start();
        }
        s_uvc_rgb_runing = true;
    }

    void xslam_stop_rgb_stream() {

        if (device && device->colorCamera()) {
            device->colorCamera()->stop();
        }

        s_uvc_rgb_runing = false;
    }
    void xslam_start_sony_tof_stream(int libmode,int resulution,int fps) {
        if (device && device->tofCamera()) {

            device->tofCamera()->registerCallback([](xv::DepthImage const &im) {
                s_depthImageMtx.lock();
                s_depthImage = std::make_shared<xv::DepthImage>(im);
                s_depthImageMtx.unlock();
            });
            xv::TofCamera::Manufacturer tofManu = device->tofCamera()->getManufacturer();
            if (tofManu == xv::TofCamera::Manufacturer::Sony){
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy xslam_start_tof_stream tofManu is sony");
                          device->tofCamera()->setSonyTofSetting(
                                  static_cast<xv::TofCamera::SonyTofLibMode>(libmode),
                                  static_cast<xv::TofCamera::Resolution>(resulution),
                                  static_cast<xv::TofCamera::Framerate>(fps));

            }

            device->tofCamera()->start();
        }
    }
    void xslam_start_tof_stream() {
        if (device && device->tofCamera()) {

            device->tofCamera()->registerCallback([](xv::DepthImage const &im) {
                if (im.type != xv::DepthImage::Type::IR) {
                    s_depthImageMtx.lock();
                    s_depthImage = std::make_shared<xv::DepthImage>(im);
                    s_depthImageMtx.unlock();
                }
            });
            xv::TofCamera::Manufacturer tofManu = device->tofCamera()->getManufacturer();
            if (tofManu == xv::TofCamera::Manufacturer::Sony){
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy xslam_start_tof_stream tofManu is sony");
                device->tofCamera()->setSonyTofSetting(xv::TofCamera::SonyTofLibMode::IQMIX_DF,
                                                       xv::TofCamera::Resolution::VGA,
                                                       xv::TofCamera::Framerate::FPS_10);

            }

            device->tofCamera()->start();
        }
    }

    void xslam_stop_tof_stream() {
        if (device && device->tofCamera()) {
            device->tofCamera()->stop();
        }
    }

/*    cv::Mat raw_to_opencv( std::shared_ptr<const xv::GrayScaleImage> tof_ir) {
        cv::Mat out;
        if( tof_ir ){
            out = cv::Mat::zeros(tof_ir->height, tof_ir->width, CV_8UC3);

            float dmax = 2191/4;

            auto tmp_d = reinterpret_cast<short const*>(tof_ir->data.get());

            for (unsigned int i=0; i< tof_ir->height*tof_ir->width; i++) {

                short d = tmp_d[i];

                unsigned int u = static_cast<unsigned int>( std::max(0.0f, std::min(255.0f,  d * 255.0f / dmax )));
                if( u < 15 )
                    u = 0;
                const auto &cc = colors.at(u);
                out.at<cv::Vec3b>( i/tof_ir->width, i%tof_ir->width ) = cv::Vec3b(cc.at(2), cc.at(1),cc.at(0) );

                *//*short u = tof->ir.get()[i];
                    cvtof.at<cv::Vec3b>( i/tof->width, i%tof->width ) = cv::Vec3s(u*255/2191,u*255/2191,u*255/2191);*//*
            }
        }
        return out;
    }*/

    void xslam_start_tofir_stream(){
        device->tofCamera()->enableTofIr(true);
        device->tofCamera()->registerCallback([](xv::DepthImage const & tof){
            if (tof.type == xv::DepthImage::Type::IR) {
                auto ir = std::make_shared<xv::GrayScaleImage>();
                ir->width = tof.width;
                ir->height = tof.height;
                ir->data = tof.data;
                s_tofIRImageMtx.lock();
                s_ir = ir;
                s_tofIRImageMtx.unlock();
            }
        });
        device->tofCamera()->start();
    }

    static device_beidou_gps_callback callbackGpsStream;
    int xslam_start_beidou_stream(device_beidou_gps_callback cb,int mode) {
        callbackGpsStream = cb;
        BeiDouGPSDataEx tmp;
        tmp.data_ready_flag = 1;
        tmp.lat_data = 3120.996844;
        tmp.latdir = 1;
        tmp.lon_data = 12117.542215;
        tmp.londir = 2;
        tmp.satellite_num = 5;
        tmp.mode = 1;
     //   callbackGpsStream(tmp);
        if (device && device->beiDouGPS()) {
            device->beiDouGPS()->setMode((xv::BeiDouGPSMode) mode);
            device->beiDouGPS()->start();
            return  device->beiDouGPS()->registerCallback([](xv::BeiDouGPSData const &data) {
#ifdef ANDROID
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy BeiDouGPSData entry");
#endif
                BeiDouGPSDataEx tmp;
                tmp.data_ready_flag = (int)data.data_ready_flag;
                tmp.lat_data = data.lat_data;
                tmp.latdir = (int)data.latdir;
                tmp.lon_data = data.lon_data;
                tmp.londir = (int)data.londir;
                tmp.satellite_num =(int)data.satellite_num;
                tmp.mode =(int)data.mode;
                callbackGpsStream(tmp);
              //  callbackGpsStream(data);
            });
        } else {
            return -1;
        }
    }
    bool xslam_prime_lens(){
        if(device){
            std::vector<unsigned char> write;
            std::vector<unsigned char> vecRead;
            bool ret;
            write.push_back(0x02);
            write.push_back(0xab);
            write.push_back(0xee);
            write.push_back(0x00);

            return device->hidWriteAndRead(write, vecRead);
        } else {
            return false;
        }
    }
    bool xslam_zoom_lens(){
        if(device){
            std::vector<unsigned char> write;
            std::vector<unsigned char> vecRead;
            bool ret;
            write.push_back(0x02);
            write.push_back(0xab);
            write.push_back(0xee);
            write.push_back(0x03);

            return device->hidWriteAndRead(write, vecRead);
        } else {
            return false;
        }
    }
    bool
    xslam_start_light_preception(){
        if(device){
            std::vector<unsigned char> write;
            std::vector<unsigned char> vecRead;
            bool ret;
            write.push_back(0x02);
            write.push_back(0xFD);
            write.push_back(0x67);
            write.push_back(0x01);

            return device->hidWriteAndRead(write, vecRead);
        } else {
            return false;
        }
    }

    bool xslam_stop_light_preception(){
        if(device) {
            std::vector<unsigned char> write;
            std::vector<unsigned char> vecRead;
            bool ret;
            write.push_back(0x02);
            write.push_back(0xFD);
            write.push_back(0x67);
            write.push_back(0x00);
            return device->hidWriteAndRead(write, vecRead);
        } else {
            return false;
        }
    }
    static device_stream_callback callbackDeviceStream;
    int xslam_start_event_stream(device_stream_callback cb) {
        callbackDeviceStream = cb;
        if (device && device->eventStream()) {
          int status =  device->eventStream()->registerCallback([](xv::Event const &event){
                callbackDeviceStream(event);
            });
            device->eventStream()->start();
            return  status;
        } else {
            return -1;
        }
    }

    void xslam_stop_event_stream() {
        if (device && device->eventStream()) {
            device->eventStream()->stop();
        }
    }


    int xslam_register_device_status_callback(device_status_callback cb) {
        callbackDeviceStatus = cb;
        if (device && device->deviceStatus()) {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy xslam_register_device_status_callback entry");

#endif
            std::vector<unsigned char> command;
            command.push_back(0x02);
            command.push_back(0xfe);
            command.push_back(0x36);
            command.push_back(0x1);
            std::vector<unsigned char> result;
            bool res1 = device->hidWriteAndRead(command, result);
            return  device->deviceStatus()->registerCallback([](const std::vector<unsigned char>& deviceStatus){
                if(deviceStatus.size()>0){

                    int length =  deviceStatus.size();
                    callbackDeviceStatus(&deviceStatus[0],length);
                }

            });
        } else {
            return -1;
        }
    }
    int xv_get_glass_tem(){
        if(g_deviceStatus.size()>0){
            LOG_DEBUG(ANDROID_LOG_INFO,"xvxr","eddy xv_get_glass_tem = %d", (int)g_deviceStatus[17]);
            return  (int)g_deviceStatus[17];
        } else {
            return -1;
        }
    }
    /**
     * @brief 启动双目摄像头的流数据处理。
     *
     * 此函数启动设备的双目鱼眼摄像头数据流，并注册一个回调函数来处理接收到的图像数据。
     * 如果设备包含鱼眼摄像头，回调函数会在数据更新时执行，将接收到的原始图像存储到共享变量中。
     *
     * @note 如果设备或其鱼眼摄像头不可用，函数将不会执行任何操作。
     */
    void xslam_start_stereo_stream() {
        if (device && device->fisheyeCameras()) {
            int ret = device->fisheyeCameras()->registerCallback(
                    [](xv::FisheyeImages const &images) {
                        if (s_stereo_runing) {
                            s_stereoImageMtx.lock();
                            s_stereoImage = std::make_shared<xv::FisheyeImages>(images);
                            hasFisheyesImagesUpdate = true;
                            s_stereoImageMtx.unlock();
                        }

                        ++fisheyesCallbackCount;
                    });
            std::cerr << sformat("FE WHOLE ret = %d", ret) << std::endl;
            device->fisheyeCameras()->start();
        }
        s_stereo_runing = true;
    }

    void xslam_stop_stereo_stream() {
        if (device && device->fisheyeCameras())
            device->fisheyeCameras()->stop();
        s_stereo_runing = false;
    }

    void xslam_start_speaker_stream() {
        if (device->speaker()) {
            device->speaker()->enable();
        }
    }

    void xslam_stop_speaker_stream() {
        if (device->speaker()) {
            device->speaker()->disable();
        }
    }

    void xslam_start_speaker_send(unsigned char *data, int len) {
        if (device->speaker()) {
            device->speaker()->send(data, len);
        }
    }

    bool xslam_start_speaker_isPlaying() {
        if (device->speaker()) {
            return device->speaker()->isPlaying();
        }
        return false;
    }

    void xslam_start_sgbm_stream(const char *config) {
        if (device->sgbmCamera()) {
            device->sgbmCamera()->start(config);
        }
    }

    void xslam_set_sgbm_config(const char *config) {
        if (device->sgbmCamera()) {
            device->sgbmCamera()->setConfig(config);
        }
    }

    void xslam_set_thermal_model(int mode) {
        if (device->thermalCamera()) {
            device->thermalCamera()->setMode(static_cast<xv::ThermalCamera::Mode>(mode));
        }
    }

    bool xslam_control_device(int m) {
        xv::DeviceSetting setting = {0x0E030007};
        setting.args.val[0] = m;
        return device->control(setting);
    }

    bool
    xslam_SyncSet(int *frameDValue, int *Freq, long long *staTime, char *isWriteST, char *CSync) {
        FrameDValue = frameDValue;
        Frequency = Freq;
        StaTime = staTime;
        IsWriteST = isWriteST;
        CurrentSync = CSync;
        return true;
    }

    bool xslam_setStatus(bool *isUDCalibra){
        isUpdateCalibra = std::make_shared<bool>(isUDCalibra);
        return true;
    }

    bool xslam_GetDevicesCalibration(double devices[2][6],double translation[2][3],double rotation[2][9]){
        if(device->display()->calibration().size() == 2){
            if(isSetCalibra == false){
                setCalibra();
            }
            for(int i = 0;i<2;i++){
                devices[i][0] = calibra[i][0];
                devices[i][1] = calibra[i][1];
                devices[i][2] = calibra[i][2];
                devices[i][3] = calibra[i][3];
                devices[i][4] = calibra[i][9];
                devices[i][5] = calibra[i][10];
                std::memcpy(translation[i], calibraT[i], sizeof(xv::Vector3d));
                std::memcpy(rotation[i], calibraR[i], sizeof(xv::Matrix3d));
            }
            return true;
        }else{
            std::cout
                    << "set calibration error!"
                    << std::endl;
            return false;
        }

    }
    bool xslam_set_sync(int Interval) {

        long long curTime = nano_time();
        long long fTemp;
        while (*IsWriteST != 0) {
        };
        long long staTime = *StaTime;

        int HidFST = 0;
        fTemp = (curTime - Interval) - staTime;
        if (*CurrentSync != 0) {
            if (fTemp < -*Frequency / 2 - *Frequency) {
                HidFST += fTemp + *Frequency * 3 - 500000;
            } else {
                HidFST += fTemp + *Frequency - 500000;
            }
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr",
                                "HID timing I___________________________");
#endif
        } else {
            if (fTemp < -*Frequency / 2 - *Frequency) {
                HidFST += fTemp + *Frequency * 2 - 500000;
            } else if (fTemp < -*Frequency / 2) {
                HidFST += fTemp + *Frequency - 500000;
                //        }else if(fTemp<*Frequency/2){
                //            HidFST += fTemp-2000000;
            } else {
                HidFST += fTemp - 500000;
            }
        }

        //if(HidFST < *Frequency/2&&HidFST > *Frequency/-2&&HidFST!=0){
        *FrameDValue = (int) HidFST;
        //}

#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr",
                            "HID timing Interval:%d,fTemp:%lld,Frequency:%d *FrameDValue:%d",
                            Interval, fTemp, *Frequency, *FrameDValue);
#endif

        return true;
    }

    void sendDeviceStatusCallbackEvent(int index,int status){
        if(callbackDeviceStatusEx == nullptr){
            std::cout << "callbackDeviceStatusEx is null " << std::endl;
        }
        if(isSetDeviceCb){
            switch (index) {
                case 0:            //0: pose confidence status callback
                    if(m_DeviceStatus.status[index] != status){
                        m_DeviceStatus.status[index] = status;
#ifdef ANDROID
                        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr",
                                            "sendDeviceStatusCallbackEvent index:%d status:%d",
                                            index,status);
#endif
                        callbackDeviceStatusEx(m_DeviceStatus);
                    }
                    break;
                default:

                    break;
            }
        }
    }
    bool xslam_get_double_pose_old(xv::Pose &poseData1, xv::Pose &poseData2, double predictionTime) {

        double addTime;
        if(ISMTK == 0){
            addTime= 0.01666666;
        }else{
            addTime= 0.01103791;
        }
        if (!device->slam()->getPose(poseData2, predictionTime + addTime)) {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "XVisio",
                                "xslam_get_pose_prediction_with_sensor failed");
#endif
            return false;
        }

        if (!device->slam()->getPose(poseData1, predictionTime)) {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "XVisio",
                                "xslam_get_pose_prediction_with_sensor failed");
#endif
            return false;
        }
        return true;
    }
    bool xslam_get_double_pose(xv::Pose &poseData1, xv::Pose &poseData2, double predictionTime,double syncDelTime) {

//        double addTime;
//        if(ISMTK == 0){
//            addTime= 0.01666666;
//        }else{
//            addTime= 0.01103791;
//        }
        if(device)
        {
            if (!device->slam()->getPose(poseData2, predictionTime + syncDelTime)) {
#ifdef ANDROID
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr",
                                    "xslam_get_pose_prediction_with_sensor failed");
#endif
                sendDeviceStatusCallbackEvent(0,0);
                return false;
            }

            if (!device->slam()->getPose(poseData1, predictionTime)) {
#ifdef ANDROID
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr",
                                    "xslam_get_pose_prediction_with_sensor failed");
#endif
                return false;
            }
            if(poseData1.confidence() > 0){
                sendDeviceStatusCallbackEvent(0,1);
            } else {
                sendDeviceStatusCallbackEvent(0,0);
            }
            return true;
        } else {
            return false;
        }

    }

    bool xslam_get_pose_angVal(double *poseData, double *angVel, double predictionTime) {
        xv::Pose pose;
        if(device && isSlamStart){
            if (!device->slam()->getPose(pose, predictionTime)) {
#ifdef ANDROID
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr",
                                    "xslam_get_pose_prediction_with_sensor failed");
#endif

                sendDeviceStatusCallbackEvent(0,0);
                return false;
            }
            if(pose.confidence() > 0){
                sendDeviceStatusCallbackEvent(0,1);
            } else {
                sendDeviceStatusCallbackEvent(0,0);
            }

            auto _velocity = pose.angularVelocity();
            auto r = pose.rotation();
            auto _quat = xv::rotationToQuaternion(r);
            poseData[0] = _quat[0];
            poseData[1] = _quat[1];
            poseData[2] = _quat[2];
            poseData[3] = _quat[3];

            poseData[4] = pose.x();
            poseData[5] = pose.y();
            poseData[6] = pose.z();

            angVel[0] = _velocity[0];
            angVel[1] = _velocity[1];
            angVel[2] = _velocity[2];
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "quat_look %f ,%f ,%f .%f ", _quat[0],
                                _quat[1], _quat[2], _quat[3]);
#endif
            //
            //
            //
            //        xv::Pose pose0;
            //        if (device->slam()->getPose(pose0, 0) == false)
            //        {
            //#ifdef ANDROID
            //            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xslam_get_pose_prediction_with_sensor failed");
            //#endif
            //            return false;
            //        }
            //        double x  =  pose.x() -  pose0.x();
            //        double y  =  pose.y() -  pose0.y();
            //
            //        double x1  =  _quat[1];

            //        std::vector<xv::Calibration> clib;
            //
            //        clib = display->calibration();
            //#ifdef ANDROID
            //            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "Calibration size:%d,mode:%d",clib,);
            //#endif
            //        getPixelShift(,,,);

            return true;
        } else {
            return false;
        }

    }

    bool
    xslam_hidWriteAndRead(std::vector<unsigned char> &command, std::vector<unsigned char> &result) {
        if (device) {
            bool res = device->hidWriteAndRead(command, result);
            return res;
        }
        return false;
    }

    bool xslam_write_hid(unsigned char *wdata, int wlen) {
        int rlen = 63;
        uint8_t buf[63];
        return xslam_write_and_read_hid(wdata, wlen, buf, rlen);
    }

    bool xslam_write_and_read_hid(unsigned char *wdata, int wlen, unsigned char *rdata, int rlen) {
        const std::vector<unsigned char> com = std::vector<unsigned char>(wdata, wdata + wlen);
        std::vector<unsigned char> result = std::vector<unsigned char>(rdata, rdata + rlen);
        if (device->hidWriteAndRead(com, result)) {
            int size = rlen > result.size() ? result.size() : rlen;
            memcpy(rdata, result.data(), sizeof(unsigned char) * size);
            return true;
        } else {
            return false;
        }
    }

    bool xslam_write_and_read_uvc(unsigned char *wdata, int wlen, unsigned char *rdata, int rlen) {
        const std::vector<unsigned char> com = std::vector<unsigned char>(wdata, wdata + wlen);
        std::vector<unsigned char> result = std::vector<unsigned char>(rdata, rdata + rlen);
        if (device->uvcWriteAndRead(com, result)) {
            return true;
        } else {
            return false;
        }
    }

    bool xslam_write_and_read_vsc(unsigned char *wdata, int wlen, unsigned char *rdata, int rlen) {
        const std::vector<unsigned char> com = std::vector<unsigned char>(wdata, wdata + wlen);
        std::vector<unsigned char> result = std::vector<unsigned char>(rdata, rdata + rlen);
        if (device->vscWriteAndRead(com, result)) {
            return true;
        } else {
            return false;
        }
    }

    bool readIMUBias(imu_bias *bias) {
        auto imu = device->imuSensor();
        if (imu) {

            /*
            bool ret = s_hid->readIMUBias(o);
            std::memcpy(bias, &o, sizeof(imu_bias));
            return ret; */
        }
        return false;
    }
    /**
    * @brief 读取立体鱼眼相机的标定信息。
    *
    * 该函数通过设备读取鱼眼相机的标定信息，并将标定结果存储到 m_FECalibration 中。如果成功读取标定信息并且包含有效数据，返回 true；否则返回 false。
    *
    * @param m_FECalibration 存储标定信息的容器
    * @return bool 标定是否成功
    */
    bool xslam_readStereoFisheyesCalibration( std::vector<xv::CalibrationEx> &m_FECalibration) {
        if (device) {
            LOG_DEBUG(ANDROID_LOG_WARN, "zhiyuan",
                                "eddy xvReadStereoFisheyesCalibration 1");
            auto fisheyeCamerasEx = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(
                    device->fisheyeCameras());

            double timestampDiff = 0.;

            std::dynamic_pointer_cast<xv::DeviceEx>(device)->getFisheyeCalibration(m_FECalibration,timestampDiff);
            if (m_FECalibration.size() > 0 && m_FECalibration[0].seucm.size() > 0){
                    return true;
                } else {
                    return false;
                }
            }

            return false;
    }
/**
 * @brief 读取立体鱼眼相机的标定信息并存储在 calib 中。
 *
 * 该函数读取设备的鱼眼相机标定信息，并将其保存到传入的 calib 结构中。返回标定是否成功。
 *
 * @param calib 存储标定信息的结构体
 * @param imu_fisheye_shift_us 存储 IMU 与鱼眼相机时间偏移的值
 * @return bool 标定是否成功
 */
    bool readStereoFisheyesCalibration(stereo_fisheyes *calib, int *imu_fisheye_shift_us) {

#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "start---- readStereoFisheyesCalibration");
#else
        std::cout << sformat("...%s...", __FUNCTION__) << std::endl;
#endif

        auto fisheye = device->fisheyeCameras();
        *imu_fisheye_shift_us = 0;
        //  std::cout << "Fisheye calibration:" << std::endl;
        // std::cout << device->fisheyeCameras()->calibration() << std::endl;
        if (fisheye && fisheye->calibration().size() == 2) {

#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                "readStereoFisheyesCalibration sucess.");
#else
                                                                                                                                    std::cout << sformat("...%s...,read fisheye cablibration sucess.", __FUNCTION__) << std::endl;
            //   calib->calibrations[0].intrinsic.K[0] = 0;
            std::cout << sformat("...%f...,read fisheye cablibration sucess.", fisheye->calibration()[0].ucm[0].fx) << std::endl;

#endif
            // std::cout << sformat("...%f...,read fisheye cablibration sucess.", fisheye->calibration()[0].ucm[1].fy) << std::endl;

            //    std::cout << sformat("...%s...,read fisheye cablibration sucess.", fisheye->calibration()->calibration()) << std::endl;
            //2 eyes
            for (int i = 0; i < 2; i++) {

                calib->calibrations[i].intrinsic.K[0] = fisheye->calibration()[i].ucm[0].fx;
                calib->calibrations[i].intrinsic.K[1] = fisheye->calibration()[i].ucm[0].fy;
                calib->calibrations[i].intrinsic.K[2] = fisheye->calibration()[i].ucm[0].u0;
                calib->calibrations[i].intrinsic.K[3] = fisheye->calibration()[i].ucm[0].v0;
                calib->calibrations[i].intrinsic.K[4] = fisheye->calibration()[i].ucm[0].xi;
                calib->calibrations[i].intrinsic.K[5] = fisheye->calibration()[i].ucm[0].w;
                calib->calibrations[i].intrinsic.K[6] = fisheye->calibration()[i].ucm[0].h;

                std::memcpy(calib->calibrations[i].extrinsic.rotation,
                            &((xv::Matrix3d) fisheye->calibration()[i].pose.rotation())[0],
                            sizeof(xv::Matrix3d));
                std::memcpy(calib->calibrations[i].extrinsic.translation,
                            &((xv::Vector3d) fisheye->calibration()[i].pose.translation())[0],
                            sizeof(xv::Vector3d));

#ifdef ANDROID

                LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                    "eddy readStereoFisheyesCalibration fy =....%f...,",
                                    calib->calibrations[i].intrinsic.K[1]);

                LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                    "eddy readStereoFisheyesCalibration R%d:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                                    i,
                                    calib->calibrations[i].extrinsic.rotation[0],
                                    calib->calibrations[i].extrinsic.rotation[1],
                                    calib->calibrations[i].extrinsic.rotation[2],
                                    calib->calibrations[i].extrinsic.rotation[3],
                                    calib->calibrations[i].extrinsic.rotation[4],
                                    calib->calibrations[i].extrinsic.rotation[5],
                                    calib->calibrations[i].extrinsic.rotation[6],
                                    calib->calibrations[i].extrinsic.rotation[7],
                                    calib->calibrations[i].extrinsic.rotation[8]);
                LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                    "eddy readStereoFisheyesCalibration K%d:%lf,%lf,%lf,%lf,%lf,%lf,%lf}",
                                    i + 1,
                                    calib->calibrations[i].intrinsic.K[0],
                                    calib->calibrations[i].intrinsic.K[1],
                                    calib->calibrations[i].intrinsic.K[2],
                                    calib->calibrations[i].intrinsic.K[3],
                                    calib->calibrations[i].intrinsic.K[4],
                                    calib->calibrations[i].intrinsic.K[5],
                                    calib->calibrations[i].intrinsic.K[6]);

#else

                                                                                                                                        std::cout << sformat("read fisheye cablibration fy =....%f...,", calib->calibrations[i].intrinsic.K[1]) << std::endl;

                std::cout << sformat("fisheye calibration,rotation size:%u,translation size:%u{",
                                     (unsigned int)sizeof(xv::Matrix3d), (unsigned int)sizeof(xv::Vector3d))
                          << std::endl;
                std::cout << sformat("eddy T%d:%lf,%lf,%lf", i + 1,
                                     calib->calibrations[i].extrinsic.translation[0], calib->calibrations[i].extrinsic.translation[1], calib->calibrations[i].extrinsic.translation[2])
                          << std::endl;
                std::cout << sformat("eddy R%d:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", i,
                                     calib->calibrations[i].extrinsic.rotation[0], calib->calibrations[i].extrinsic.rotation[1], calib->calibrations[i].extrinsic.rotation[2],
                                     calib->calibrations[i].extrinsic.rotation[3], calib->calibrations[i].extrinsic.rotation[4], calib->calibrations[i].extrinsic.rotation[5],
                                     calib->calibrations[i].extrinsic.rotation[6], calib->calibrations[i].extrinsic.rotation[7], calib->calibrations[i].extrinsic.rotation[8])
                          << std::endl;
                std::cout << sformat("eddy K%d:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf}", i + 1,
                                     calib->calibrations[i].intrinsic.K[0], calib->calibrations[i].intrinsic.K[1], calib->calibrations[i].intrinsic.K[2],
                                     calib->calibrations[i].intrinsic.K[3], calib->calibrations[i].intrinsic.K[4], calib->calibrations[i].intrinsic.K[5],
                                     calib->calibrations[i].intrinsic.K[6], calib->calibrations[i].intrinsic.K[7], calib->calibrations[i].intrinsic.K[8],
                                     calib->calibrations[i].intrinsic.K[9], calib->calibrations[i].intrinsic.K[10])
                          << std::endl;
#endif
            }
            return true;
        }

        return false;
    }
/**
 * @brief 读取显示器的标定信息。
 *
 * 该函数读取设备的显示器标定信息，并将标定结果保存到 calib 结构体中。
 *
 * @param calib 存储显示器标定信息的结构体
 * @return bool 标定是否成功
 */
    bool readDisplayCalibration(pdm_calibration *calib) {
        std::cout << sformat("...%s...", __FUNCTION__) << std::endl;

        auto display = device->display();
        if (display) {
            std::cout << sformat("...%s...,read display cablibration sucess.", __FUNCTION__)
                      << std::endl;
            for (int i = 0; i < 2; i++) {
                calib->intrinsic.K[0] = display->calibration()[i].pdcm[0].fx;
                calib->intrinsic.K[1] = display->calibration()[i].pdcm[0].fy;
                calib->intrinsic.K[2] = display->calibration()[i].pdcm[0].u0;
                calib->intrinsic.K[3] = display->calibration()[i].pdcm[0].v0;
                calib->intrinsic.K[4] = display->calibration()[i].pdcm[0].distor[0];
                calib->intrinsic.K[5] = display->calibration()[i].pdcm[0].distor[1];
                calib->intrinsic.K[6] = display->calibration()[i].pdcm[0].distor[2];
                calib->intrinsic.K[7] = display->calibration()[i].pdcm[0].distor[3];
                calib->intrinsic.K[8] = display->calibration()[i].pdcm[0].distor[4];
                calib->intrinsic.K[9] = display->calibration()[i].pdcm[0].w;
                calib->intrinsic.K[10] = display->calibration()[i].pdcm[0].h;

                std::memcpy(calib->extrinsic.rotation,
                            &((xv::Matrix3d) display->calibration()[i].pose.rotation())[0],
                            sizeof(xv::Matrix3d));
                std::memcpy(calib->extrinsic.translation,
                            &((xv::Vector3d) display->calibration()[i].pose.translation())[0],
                            sizeof(xv::Vector3d));
            }

            std::cout << sformat("display calibration,rotation size:%u,translation size:%u{",
                                 (unsigned int) sizeof(xv::Matrix3d),
                                 (unsigned int) sizeof(xv::Vector3d))
                      << std::endl;
            std::cout << sformat("T%d:%lf,%lf,%lf", 0 + 1,
                                 calib->extrinsic.translation[0], calib->extrinsic.translation[1],
                                 calib->extrinsic.translation[2])
                      << std::endl;
            std::cout << sformat("R%d:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", 0,
                                 calib->extrinsic.rotation[0], calib->extrinsic.rotation[1],
                                 calib->extrinsic.rotation[2],
                                 calib->extrinsic.rotation[3], calib->extrinsic.rotation[4],
                                 calib->extrinsic.rotation[5],
                                 calib->extrinsic.rotation[6], calib->extrinsic.rotation[7],
                                 calib->extrinsic.rotation[8])
                      << std::endl;
            std::cout << sformat("K%d:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf}", 0 + 1,
                                 calib->intrinsic.K[0], calib->intrinsic.K[1],
                                 calib->intrinsic.K[2],
                                 calib->intrinsic.K[3], calib->intrinsic.K[4],
                                 calib->intrinsic.K[5],
                                 calib->intrinsic.K[6], calib->intrinsic.K[7],
                                 calib->intrinsic.K[8],
                                 calib->intrinsic.K[9], calib->intrinsic.K[10])
                      << std::endl;
            return true;
        }

        return false;
    }
    /**
     * @brief 读取 ToF 相机的标定信息。
     *
     * 该函数读取 ToF 相机的标定信息，并将结果保存到 calib 结构体中。
     *
     * @param calib 存储 ToF 相机标定信息的结构体
     * @return bool 标定是否成功
     */
    bool readToFCalibration(pdm_calibration *calib) {
        std::cout << sformat("...%s...", __FUNCTION__) << std::endl;

        auto tof = device->tofCamera();
        if (tof) {
            std::cout << sformat("...%s...,read tof cablibration sucess.", __FUNCTION__)
                      << std::endl;
            calib->intrinsic.K[0] = tof->calibration()[0].pdcm[0].fx;
            calib->intrinsic.K[1] = tof->calibration()[0].pdcm[0].fy;
            calib->intrinsic.K[2] = tof->calibration()[0].pdcm[0].u0;
            calib->intrinsic.K[3] = tof->calibration()[0].pdcm[0].v0;
            calib->intrinsic.K[4] = tof->calibration()[0].pdcm[0].distor[0];
            calib->intrinsic.K[5] = tof->calibration()[0].pdcm[0].distor[1];
            calib->intrinsic.K[6] = tof->calibration()[0].pdcm[0].distor[2];
            calib->intrinsic.K[7] = tof->calibration()[0].pdcm[0].distor[3];
            calib->intrinsic.K[8] = tof->calibration()[0].pdcm[0].distor[4];
            calib->intrinsic.K[9] = tof->calibration()[0].pdcm[0].w;
            calib->intrinsic.K[10] = tof->calibration()[0].pdcm[0].h;

            std::memcpy(calib->extrinsic.rotation,
                        &((xv::Matrix3d) tof->calibration()[0].pose.rotation())[0],
                        sizeof(xv::Matrix3d));
            std::memcpy(calib->extrinsic.translation,
                        &((xv::Vector3d) tof->calibration()[0].pose.translation())[0],
                        sizeof(xv::Vector3d));

            return true;
        }

        return false;
    }

    bool readRGBCalibration(rgb_calibration *calib) {
        std::cout << sformat("...%s...", __FUNCTION__) << std::endl;

        auto color = device->colorCamera();
        if (color) {
            std::cout << sformat("...%s...,read color cablibration sucess.", __FUNCTION__)
                      << std::endl;
            calib->intrinsic480.K[0] = color->calibration()[0].pdcm[0].fx;
            calib->intrinsic480.K[1] = color->calibration()[0].pdcm[0].fy;
            calib->intrinsic480.K[2] = color->calibration()[0].pdcm[0].u0;
            calib->intrinsic480.K[3] = color->calibration()[0].pdcm[0].v0;
            calib->intrinsic480.K[4] = color->calibration()[0].pdcm[0].distor[0];
            calib->intrinsic480.K[5] = color->calibration()[0].pdcm[0].distor[1];
            calib->intrinsic480.K[6] = color->calibration()[0].pdcm[0].distor[2];
            calib->intrinsic480.K[7] = color->calibration()[0].pdcm[0].distor[3];
            calib->intrinsic480.K[8] = color->calibration()[0].pdcm[0].distor[4];
            calib->intrinsic480.K[9] = color->calibration()[0].pdcm[0].w;
            calib->intrinsic480.K[10] = color->calibration()[0].pdcm[0].h;

            calib->intrinsic720.K[0] = color->calibration()[0].pdcm[1].fx;
            calib->intrinsic720.K[1] = color->calibration()[0].pdcm[1].fy;
            calib->intrinsic720.K[2] = color->calibration()[0].pdcm[1].u0;
            calib->intrinsic720.K[3] = color->calibration()[0].pdcm[1].v0;
            calib->intrinsic720.K[4] = color->calibration()[0].pdcm[1].distor[0];
            calib->intrinsic720.K[5] = color->calibration()[0].pdcm[1].distor[1];
            calib->intrinsic720.K[6] = color->calibration()[0].pdcm[1].distor[2];
            calib->intrinsic720.K[7] = color->calibration()[0].pdcm[1].distor[3];
            calib->intrinsic720.K[8] = color->calibration()[0].pdcm[1].distor[4];
            calib->intrinsic720.K[9] = color->calibration()[0].pdcm[1].w;
            calib->intrinsic720.K[10] = color->calibration()[0].pdcm[1].h;

            calib->intrinsic1080.K[0] = color->calibration()[0].pdcm[2].fx;
            calib->intrinsic1080.K[1] = color->calibration()[0].pdcm[2].fy;
            calib->intrinsic1080.K[2] = color->calibration()[0].pdcm[2].u0;
            calib->intrinsic1080.K[3] = color->calibration()[0].pdcm[2].v0;
            calib->intrinsic1080.K[4] = color->calibration()[0].pdcm[2].distor[0];
            calib->intrinsic1080.K[5] = color->calibration()[0].pdcm[2].distor[1];
            calib->intrinsic1080.K[6] = color->calibration()[0].pdcm[2].distor[2];
            calib->intrinsic1080.K[7] = color->calibration()[0].pdcm[2].distor[3];
            calib->intrinsic1080.K[8] = color->calibration()[0].pdcm[2].distor[4];
            calib->intrinsic1080.K[9] = color->calibration()[0].pdcm[2].w;
            calib->intrinsic1080.K[10] = color->calibration()[0].pdcm[2].h;
            std::memcpy(calib->extrinsic.rotation,
                        &((xv::Matrix3d) color->calibration()[0].pose.rotation())[0],
                        sizeof(xv::Matrix3d));
            std::memcpy(calib->extrinsic.translation,
                        &((xv::Vector3d) color->calibration()[0].pose.translation())[0],
                        sizeof(xv::Vector3d));

            return true;
        }

        return false;
    }

    bool readStereoFisheyesPDMCalibration(stereo_pdm_calibration *calib) {
        std::cout << sformat("...%s...", __FUNCTION__) << std::endl;

        auto fisheye = device->fisheyeCameras();
        if (fisheye && fisheye->calibration().size() == 2) {
            std::cout << sformat("...%s...,read fisheye cablibration sucess.", __FUNCTION__)
                      << std::endl;
            //2 eyes
            for (int i = 0; i < 2; i++) {
                calib->calibrations[i].intrinsic.K[0] = fisheye->calibration()[i].pdcm[0].fx;
                calib->calibrations[i].intrinsic.K[1] = fisheye->calibration()[i].pdcm[0].fy;
                calib->calibrations[i].intrinsic.K[2] = fisheye->calibration()[i].pdcm[0].u0;
                calib->calibrations[i].intrinsic.K[3] = fisheye->calibration()[i].pdcm[0].v0;
                calib->calibrations[i].intrinsic.K[4] = fisheye->calibration()[i].pdcm[0].distor[0];
                calib->calibrations[i].intrinsic.K[5] = fisheye->calibration()[i].pdcm[0].distor[1];
                calib->calibrations[i].intrinsic.K[6] = fisheye->calibration()[i].pdcm[0].distor[2];
                calib->calibrations[i].intrinsic.K[7] = fisheye->calibration()[i].pdcm[0].distor[3];
                calib->calibrations[i].intrinsic.K[8] = fisheye->calibration()[i].pdcm[0].distor[4];
                calib->calibrations[i].intrinsic.K[9] = fisheye->calibration()[i].pdcm[0].w;
                calib->calibrations[i].intrinsic.K[10] = fisheye->calibration()[i].pdcm[0].h;

                std::memcpy(calib->calibrations[i].extrinsic.rotation,
                            &((xv::Matrix3d) fisheye->calibration()[i].pose.rotation())[0],
                            sizeof(xv::Matrix3d));
                std::memcpy(calib->calibrations[i].extrinsic.translation,
                            &((xv::Vector3d) fisheye->calibration()[i].pose.translation())[0],
                            sizeof(xv::Vector3d));
                std::cout << sformat("fisheye calibration,rotation size:%u,translation size:%u{",
                                     (unsigned int) sizeof(xv::Matrix3d),
                                     (unsigned int) sizeof(xv::Vector3d))
                          << std::endl;
                std::cout << sformat("T%d:%lf,%lf,%lf", i + 1,
                                     calib->calibrations[i].extrinsic.translation[0],
                                     calib->calibrations[i].extrinsic.translation[1],
                                     calib->calibrations[i].extrinsic.translation[2])
                          << std::endl;
                std::cout << sformat("R%d:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", i,
                                     calib->calibrations[i].extrinsic.rotation[0],
                                     calib->calibrations[i].extrinsic.rotation[1],
                                     calib->calibrations[i].extrinsic.rotation[2],
                                     calib->calibrations[i].extrinsic.rotation[3],
                                     calib->calibrations[i].extrinsic.rotation[4],
                                     calib->calibrations[i].extrinsic.rotation[5],
                                     calib->calibrations[i].extrinsic.rotation[6],
                                     calib->calibrations[i].extrinsic.rotation[7],
                                     calib->calibrations[i].extrinsic.rotation[8])
                          << std::endl;
                std::cout << sformat("K%d:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf}", i + 1,
                                     calib->calibrations[i].intrinsic.K[0],
                                     calib->calibrations[i].intrinsic.K[1],
                                     calib->calibrations[i].intrinsic.K[2],
                                     calib->calibrations[i].intrinsic.K[3],
                                     calib->calibrations[i].intrinsic.K[4],
                                     calib->calibrations[i].intrinsic.K[5],
                                     calib->calibrations[i].intrinsic.K[6],
                                     calib->calibrations[i].intrinsic.K[7],
                                     calib->calibrations[i].intrinsic.K[8],
                                     calib->calibrations[i].intrinsic.K[9],
                                     calib->calibrations[i].intrinsic.K[10])
                          << std::endl;
            }
            return true;
        }

        return false;
    }

    bool readStereoDisplayCalibration(stereo_pdm_calibration *calib) {
        std::cout << sformat("...%s...", __FUNCTION__) << std::endl;
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy readStereoDisplayCalibration: 0");
#endif
        if (device) {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy readStereoDisplayCalibration: 1");
#endif
            auto fisheye = device->display();

            if (fisheye) {
#ifdef ANDROID
                LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                    "eddy readStereoDisplayCalibration size: %d",
                                    fisheye->calibration().size());
#endif
            }
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy readStereoDisplayCalibration: 2");
#endif
            if (fisheye && fisheye->calibration().size() == 2) {
#ifdef ANDROID
                LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                    "eddy readStereoDisplayCalibration: 3");
#endif
                std::cout << sformat("...%s...,read display cablibration sucess.", __FUNCTION__)
                          << std::endl;

                if(isSetCalibra == false){
                    return false;
                }

                getCalibra(calib);
                //2 eyes
                return true;
            } else {
                return false;
            }
        }
        return false;
    }
    bool xvisio_start(){
        return true;
    }

    bool xvisio_stop(){
        return true;
    }
    bool updateCalibra(double distance) {
        if(isSetCalibra == true) {
            double dis = calibraT[1][0] - calibraT[0][0];
            double cha = (dis - distance*0.01)*0.5;
            calibraT[0][0] = calibraT[0][0] + cha;
            calibraT[1][0] = calibraT[1][0] - cha;
            isUpdateCalibra =  std::make_shared<bool>(true);
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr",
                                "calibraT:%f,%f distance:",calibraT[0][0],
            calibraT[1][0],distance);
#endif
            return true;
        } else {
            return false;
        }


    }

    void xslam_set_object_ui_on(bool isshow) {
        s_object_ui_show = isshow;

        if (device && device->objectDetector()) {
            if (isshow) {
                device->objectDetector()->start();
            } else {
                device->objectDetector()->stop();
            }
        }
    }

    bool xslam_set_hand_config_path_s(const char *path) {
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy xslam_set_hand_config_path_s: %s",
                            path);
        // LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy xslam_set_hand_config_path_s: %s", *path);
#endif

        if (device->gesture()) {
            return true;
        }
        return false;
    }

    int xslam_start_cnn() {
        if (device->objectDetector()) {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy xslam_start_cnn");

#endif
            s_cnn_callback_id = device->objectDetector()->registerCallback(
                    [](const std::vector<xv::Object> objects) {
#ifdef ANDROID
                        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr",
                                            "eddy objectDetector callback");

#endif
                        s_objMutex.lock();
                        std::cerr << "eddy objectDetector callback" << std::endl;
                        s_objects = objects;
                        if (time_init_flag) {
                            start_time = std::chrono::high_resolution_clock::now();
                            time_init_flag = false;
                        } else {
                            auto now = std::chrono::high_resolution_clock::now();
                            auto cost = std::chrono::duration_cast<std::chrono::microseconds>(
                                    now - start_time).count();
                            start_time = now;
                            std::cout << "handskeleton cost time micro second is " << cost
                                      << std::endl;
#ifdef ANDROID
                            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr",
                                                "eddy handskeleton cost time micro second is %lld",
                                                cost);

#endif
                        }
                        s_objMutex.unlock();
                    });
            device->objectDetector()->start();
            return s_cnn_callback_id;
        }
        return -1;
    }

    bool xslam_set_cnn_model_s(const std::string &path) {

        if (device->objectDetector()) {

            /*  {
            s_cnn_callback_id = device->objectDetector()->registerCallback([](const std::vector<xv::Object> objects)
                                                                           {
                                                                               s_objMutex.lock();
                                                                               std::cerr << "eddy objectDetector callback" << std::endl;
                                                                               s_objects = objects;
                                                                               if (time_init_flag)
                                                                               {
                                                                                   start_time = std::chrono::high_resolution_clock::now();
                                                                                   time_init_flag = false;
                                                                               }
                                                                               else
                                                                               {
                                                                                   auto now = std::chrono::high_resolution_clock::now();
                                                                                   auto cost = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count();
                                                                                   start_time = now;
                                                                                   std::cout << "handskeleton cost time micro second is " << cost << std::endl;
#ifdef ANDROID
                                                                                   LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy handskeleton cost time micro second is %lld", cost);

#endif
                                                                               }
                                                                               s_objMutex.unlock();
                                                                           });
            device->objectDetector()->start();
        } */
            return device->objectDetector()->setModel(
                    path); //"/sdcard/palm_detection_fp32_reverse.blob /sdcard/hand_landmark_r14_5.blob"
        }
        return false;
    }

    bool xslam_set_cnn_descriptor_s(const std::string &path) {
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy xslam_set_cnn_descriptor_s");
        // LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy xslam_set_hand_config_path_s: %s", *path);
#endif
        if (device->objectDetector()) {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                "eddy xslam_set_cnn_descriptor_s entry 1: %s", path.c_str());
            // LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy xslam_set_hand_config_path_s: %s", *path);
#endif
            return device->objectDetector()->setDescriptor(path); //"/sdcard/config_tensorflow.json"
        }
        return false;
    }

    bool xslam_set_cnn_source(int source) {
        //  source = 2;
        if (device->objectDetector()) {
            s_cnnSource = source; //source;
            return device->objectDetector()->setSource(
                    static_cast<xv::ObjectDetector::Source>(source));
        }
        return false;
    }

    int xslam_get_cnn_source() {
        if (device->objectDetector()) {
            return static_cast<int>(device->objectDetector()->getSource());
        }
        return -1;
    }

    void xslam_stop_cnn() {
        if (device->objectDetector()) {
            device->objectDetector()->unregisterCallback(s_cnn_callback_id);
            device->objectDetector()->stop();
        }
    }

    bool xslam_set_cnn_model(const char *path) {
        return xslam_set_cnn_model_s(path);
    }

    bool xslam_set_cnn_descriptor(const char *path) {
        return xslam_set_cnn_descriptor_s(path);
    }

    bool xslam_get_cnn_descriptor(xv::ObjectDescriptor *desc) {
        if (device->objectDetector()) {
            desc->type = device->objectDetector()->getDescriptor().type;
            desc->classes = device->objectDetector()->getDescriptor().classes;
        }
        return false;
    }
    static rknn_objectdetect_callback rknn_cb;

    bool setRkNNModelPath(const std::string &path){
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "XVisio", "setRkNNModelPath path is  %s",
                            path.c_str());
#endif
        if(device && device->objectDetectorRKNN3588()){
            return device->objectDetectorRKNN3588()->setModel(path);
        } else {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "XVisio", "setRkNNModelPath device is null",
                                path.c_str());
#endif
            return false;
        }
    }
    bool xslam_set_rknn_model(const char *path) {
        return setRkNNModelPath(path);
    }

    void objectRKNN3588Callback(const std::vector<xv::Det2dObject> & res){
        ObjectData *objectsPtr = new ObjectData[res.size()];
        for (int i = 0; i < res.size(); ++i){
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "XVisio", "objectRKNN3588Callback item.name = %s,score = %f,width = %f,height = %f",
                                res[i].name.c_str(),res[i].score,res[i].width,res[i].height);
#endif
            objectsPtr[i].type = (char *)res[i].name.c_str();
            objectsPtr[i].confidence = res[i].score;
            objectsPtr[i].x = res[i].left;
            objectsPtr[i].y = res[i].top;
            objectsPtr[i].width = res[i].width;
            objectsPtr[i].height = res[i].height;
            for (int j = 0; j < res[i].keypoints.size(); j++) {
                objectsPtr[i].keypoints[j].x = res[i].keypoints[j].x;
                objectsPtr[i].keypoints[j].y = res[i].keypoints[j].y;
                objectsPtr[i].keypoints[j].z = res[i].keypoints[j].z;
#ifdef ANDROID
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy objects keypoints x  %f",
                                    objectsPtr[i].keypoints[j].x);
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy objects keypoints y %f",
                                    objectsPtr[i].keypoints[j].y);
                LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy objects keypoints z %f",
                                    objectsPtr[i].keypoints[j].z);
#endif
            }

        }
        rknn_cb(objectsPtr,res.size());

    }

    int startObjectDetectRkNN(rknn_objectdetect_callback cb) {
        if (cb == nullptr) {
            return -1;
        }
        rknn_cb = cb;
        device->objectDetectorRKNN3588()->start();
        return device->objectDetectorRKNN3588()->registerCallback(objectRKNN3588Callback);
    }
    void stopObjectDetectRkNN(int id){
        device->objectDetectorRKNN3588()->unregisterCallback(id);
    }

    int xslam_transfer_speaker_buffer(const unsigned char *data, int len) {
        return -1;
    }

    bool xslam_play_sound(const unsigned char *data, int len) {
        if (device->speaker()) {
            return device->speaker()->play(data, len);
        }
        return false;
    }

    bool xslam_play_sound_file(const char *path) {
        if (device->speaker()) {
            return device->speaker()->play(path);
        }
        return false;
    }

    bool xslam_is_playing() {
        if (device && device->speaker()) {
            return device->speaker()->isPlaying();
        }
        return false;
    }

    void xslam_stop_play() {
        if (device && device->speaker()) {
            device->speaker()->disable();
        }
    }

    std::queue<std::shared_ptr<xv::MicData>> qmic;
    std::thread mic_thread;
    std::mutex mq;
    static cb_data mic_cb;
    static int mic_cb_id = -1;

    bool xslam_set_mic_callback(cb_data cb) {
        if (true) {
            mic_cb = cb;
            if (!device->mic()->start()) {
                std::cerr << "startAudioStreaming failed" << std::endl;
                return false;
            }
            if (!mic_thread.joinable()) {
                mic_thread = std::thread([&] {
                    while (true) {
                        if (!qmic.empty()) {
                            s_micDataMtx.lock();
                            auto audio = qmic.front();
                            qmic.pop();
                            s_micDataMtx.unlock();
                            audio->data.get();
                            mic_cb(const_cast<unsigned char *>(audio->data.get()), audio->dataSize);
                        } else {
                            std::this_thread::sleep_for(std::chrono::milliseconds(20));
                        }
                    }
                });
            }

            if (device->mic()) {
                mic_cb_id = device->mic()->registerCallback([](const xv::MicData micData) {
                    s_micDataMtx.lock();
                    s_micData = std::make_shared<xv::MicData>(micData);
                    s_micDataMtx.unlock();
                });
            }
            return true;
        }
        return false;
    }

    void xslam_unset_mic_callback() {
        if (device && device->mic()) {
            device->mic()->unregisterCallback(mic_cb_id);
            device->mic()->stop();
        }
    }

    void xslam_mic_start() {
        if (device && device->mic()) {
            device->mic()->start();
        }
    }

    bool xslam_switch_rgb() {
        std::vector<unsigned char> command;
        command.push_back(0x02);
        command.push_back(0xfe);
        command.push_back(0x20);
        command.push_back(0x24);
        command.push_back(0x02);
        std::vector<unsigned char> result;
        return device->hidWriteAndRead(command, result);
     }
    bool xslam_switch_display() {
        std::vector<unsigned char> command;
        command.push_back(0x02);
        command.push_back(0xfe);
        command.push_back(0x20);
        command.push_back(0x24);
        command.push_back(0x01);
        std::vector<unsigned char> result;
        return device->hidWriteAndRead(command, result);
    }

    bool xslam_switch_audio(bool status) {
        std::vector<unsigned char> command;
        command.push_back(0x02);
        command.push_back(0xFE);
        command.push_back(0x27);
        if(status) {
            command.push_back(0x01);
        } else {
            command.push_back(0x00);
        }
        std::vector<unsigned char> result;
        return device->hidWriteAndRead(command, result);
    }
    void serializePlane(const std::vector<xv::Plane> &planes, unsigned char *data, int *len) {
        if (data == nullptr || *len < 64) {
            std::cerr << "bad plane out memory" << std::endl;
            return;
        }

        int maxlen = *len;

        *((int *) data) = planes.size();
        data += 4;
        *len = 4;

        for (int i = 0; i < planes.size(); i++) {
            //Vector3d: std::array<double,3>
            //struct Plane {
            //    std::vector<Vector3d> points;
            //    Vector3d normal;
            //    double d;
            //    std::string id;
            //};
            auto &plane = planes[i];
            int alen = 4 + plane.points.size() * 3 * sizeof(double) + 3 * sizeof(double) +
                       sizeof(double) + 4 + plane.id.size();
            if (*len + alen > maxlen) {
                std::cerr << sformat(
                        "Plane too big. curr plane id:%d, plane count:%zu, (max len,used len,curr_plane_data_len):(%d,%d,%d) curr points count:%zu",
                        i, planes.size(), maxlen, *len, alen, planes[i].points.size()) << std::endl;
                return;
            }
            int a = plane.points.size();
            for (int i = 00; i < 4; i++) {
                data[i] = (char) a;

                a = a >> 8;
            }
            //   *((int *)data) = plane.points.size();
            data += 4;
            if (plane.points.size() > 0) {
                std::memcpy(data, &plane.points[0], plane.points.size() * 3 * sizeof(double));
                data += plane.points.size() * 3 * sizeof(double);
            }
            std::memcpy(data, &plane.normal, 3 * sizeof(double));
            data += 3 * sizeof(double);
            std::memcpy(data, &plane.d, sizeof(double));
            data += sizeof(double);
            *((int *) data) = plane.id.size();
            data += 4;
            std::memcpy(data, plane.id.c_str(), plane.id.size());
            data += plane.id.size();
            *len += alen;
        }
    }

    xplan_package xslam_get_plane_from_tof_ex(int index) {
        //     std::shared_ptr<const std::vector<xv::Plane>> planes;
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy xslam_get_plane_from_tof_ex entry s_tofPlane size = %d",
                            s_tofPlane->size());

#endif

#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy xslam_get_plane_from_tof_ex entry 1");

#endif
        auto plane = s_tofPlane->at(index);
        xplan_package xplanPackage;

        xplanPackage.distance = plane.d;
        //  xplanPackage.idStr = plane.id.c_str();
        strcpy(xplanPackage.idStr, plane.id.c_str());
        xplanPackage.normal.x = plane.normal[0];
        xplanPackage.normal.y = plane.normal[1];
        xplanPackage.normal.z = plane.normal[2];
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_DEBUG, "eddy xslam_get_plane_from_tof_ex",
                            " Plane[%d]: Distance=%lf; Normal: %lf %lf %lf id:%s",
                            index, xplanPackage.distance, plane.normal[0], plane.normal[1],
                            plane.normal[2], xplanPackage.idStr);
#endif
        /*    xplanPackage.pointsSize = plane.points.size();
                    for (uint32_t j = 0; j <  plane.points.size(); ++j)
                    {
                        const auto points = plane.points[j];
                        xplanPackage.points[j].x  = points[0];
                        xplanPackage.points[j].y  = points[1];
                        xplanPackage.points[j].z  = points[2];
                    }*/
        xplanPackage.verticesSize = plane.vertices.size();
        xplanPackage.vertices = new Vector3[xplanPackage.verticesSize];
        for (uint32_t l = 0; l < plane.vertices.size(); ++l) {
            const auto vertices = plane.vertices[l];
            xplanPackage.vertices[l].x = vertices[0];
            xplanPackage.vertices[l].y = vertices[1];
            xplanPackage.vertices[l].z = vertices[2];
        }
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_DEBUG, "eddy xslam_get_plane_from_tof_ex",
                            "vertices 0: %lf %lf %lf", xplanPackage.vertices[0].x,
                            xplanPackage.vertices[0].y,
                            xplanPackage.vertices[0].z);
#endif
        xplanPackage.trianglesSize = plane.triangles.size();
        xplanPackage.triangles = new Vector3uint[xplanPackage.trianglesSize];
        for (uint32_t k = 0; k < plane.triangles.size(); ++k) {
            const auto triangles = plane.triangles[k];
            xplanPackage.triangles[k].x = triangles[0];
            xplanPackage.triangles[k].y = triangles[1];
            xplanPackage.triangles[k].z = triangles[2];
        }
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_DEBUG, "eddy xslam_get_plane_from_tof_ex",
                            "triangles 0: %d %d %d", xplanPackage.triangles[0].x,
                            xplanPackage.triangles[0].y,
                            xplanPackage.triangles[0].z);
#endif
        return xplanPackage;
    }

    static bool hasTofPlane = false;
    static fn_tof_plan_callback tofPlanCallback;
    bool xslam_start_detect_plane_from_tof_nosurface()
    {
        if (device->slam() && device->tofCamera())
        {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy xslam_start_detect_plane_from_tof_nosurface entry");

#endif

            xslam_tof_set_steam_mode(0);

            auto slam = device->slam();
            auto slamEx_a = dynamic_cast<xv::SlamEx*>(slam.get());
//            slamEx_a->setEnableSurface(true);
            slamEx_a->setEnableSurfaceReconstruction(true);

            xv::TofCamera::Manufacturer tofManu = device->tofCamera()->getManufacturer();
            if (tofManu == xv::TofCamera::Manufacturer::Sony)
            {
        /*        device->tofCamera()->setSonyTofSetting(xv::TofCamera::SonyTofLibMode::IQMIX_SF,
                                                       xv::TofCamera::Resolution::QVGA,
                                                       xv::TofCamera::Framerate::FPS_5);*/
            }
            else if (tofManu == xv::TofCamera::Manufacturer::Pmd)
            {
                //默认pmd tof 配置
                device->tofCamera()->setFramerate(5.);
            }


            device->tofCamera()->start();
            s_tofPlaneId = device->slam()->registerTofPlanesCallback([](std::shared_ptr<const std::vector<xv::Plane>> plane)
                                                                     {
#ifdef ANDROID
                                                                         LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy TofPlanesCallback callback");

#endif
                                                                         if (plane->size() > 0)
                                                                         {
                                                                             std::cerr << "tof got plane, id: " << plane->at(0).id << ", [" << plane->at(0).normal[0] << "," << plane->at(0).normal[1] << "," << plane->at(0).normal[2] << "]" << std::endl;
                                                                             s_tofPlaneMutex.lock();
                                                                             s_tofPlane = plane;
                                                                             s_tofPlaneMutex.unlock();
                                                                         }
                                                                     });
            device->slam()->start(xv::Slam::Mode::Mixed); //slamMode
            return true;
        }
        return false;
    }
    bool xslam_start_detect_plane_from_tof(fn_tof_plan_callback cb) {
        if (cb == nullptr) {
            return false;
        }
        tofPlanCallback = cb;
        if (device->slam() && device->tofCamera()) {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                "eddy xslam_start_detect_plane_from_tof entry");

#endif
            xslam_tof_set_steam_mode(0);
            auto slam = device->slam();
            auto slamEx_a = dynamic_cast<xv::SlamEx*>(slam.get());
//            slamEx_a->setEnableSurface(true);
            slamEx_a->setEnableSurfaceReconstruction(true);
            slamEx_a->setEnableSurfacePlanes(true);
            xv::TofCamera::Manufacturer tofManu = device->tofCamera()->getManufacturer();
            if (tofManu == xv::TofCamera::Manufacturer::Sony)
            {
            /*    device->tofCamera()->setSonyTofSetting(xv::TofCamera::SonyTofLibMode::IQMIX_SF,
                                                       xv::TofCamera::Resolution::QVGA,
                                                       xv::TofCamera::Framerate::FPS_5);*/
            }
            else if (tofManu == xv::TofCamera::Manufacturer::Pmd)
            {
                //默认pmd tof 配置
                device->tofCamera()->setFramerate(5.);
            }


            device->tofCamera()->start();
            s_tofPlaneId = device->slam()->registerTofPlanesCallback(
                    [](std::shared_ptr<const std::vector<xv::Plane>> plane) {
#ifdef ANDROID
                        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                            "eddy TofPlanesCallback callback");
#endif
                        if (plane->size() > 0) {
                            hasTofPlane = true;
                            std::cerr << "c, id: " << plane->at(0).id << ", ["
                                      << plane->at(0).normal[0] << "," << plane->at(0).normal[1]
                                      << "," << plane->at(0).normal[2] << "]" << std::endl;
                            s_tofPlaneMutex.lock();
                            s_tofPlane = plane;
                            int planSize = 0;
#ifdef ANDROID
                            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                                "eddy TofPlanesCallback callback size  = %d",
                                                plane->size());
                            xplan_package plans[plane->size()];
#endif
                            for (int i = 0; i < plane->size(); i++) {
                                plans[i] = xslam_get_plane_from_tof_ex(i);
                            }

                            tofPlanCallback(plans, plane->size());
                            s_tofPlaneMutex.unlock();
                        }
                    });
            device->slam()->start(xv::Slam::Mode::Mixed); //slamMode

            return true;
        }
        return false;
    }

    bool xslam_stop_detect_plane_from_tof() {
        if (device->slam() && device->tofCamera()) {
            device->slam()->unregisterTofPlanesCallback(s_tofPlaneId);
            device->tofCamera()->stop();
            return true;
        }
        return false;
    }

    slammap *xslam_get_slam_map(int *len) {
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy xslam_get_slam_map entry 1");

#endif
        if (slamMap) {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy xslam_get_slam_map entry 2");

#endif

            s_slamMapMutex.lock();
            *len = slamMap->vertices.size();
            slammap *maps = new slammap[slamMap->vertices.size()];
            for (int i = 0; i < slamMap->vertices.size(); i++) {
                maps[i].vertice.x = slamMap->vertices[i][0];
                maps[i].vertice.y = slamMap->vertices[i][1];
                maps[i].vertice.z = slamMap->vertices[i][2];
            }
            //
            s_slamMapMutex.unlock();

            return maps;
        }
        return nullptr;
    }

    bool xslam_get_plane_from_tof(unsigned char *data, int *len) {
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy xslam_get_plane_from_tof entry 1");

#endif
        if (s_tofPlane) {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                "eddy xslam_get_plane_from_tof entry 2");

#endif
            std::shared_ptr<const std::vector<xv::Plane>> planes;
            s_tofPlaneMutex.lock();
            planes = s_tofPlane;
            s_tofPlaneMutex.unlock();
            serializePlane(*planes.get(), data, len);
            return true;
        }
        return false;
    }

    bool xslam_start_detect_plane_from_stereo() {
        if (device->slam()) {
            auto slam = device->slam();
            auto slamEx = dynamic_cast<xv::SlamEx*>(slam.get());
            slamEx->setSurfaceUseFisheyes(true);
            slamEx->setSurfaceMinVoxelSize(0.06);
            s_stereoPlaneId = device->slam()->registerStereoPlanesCallback(
                    [](std::shared_ptr<const std::vector<xv::Plane>> plane) {
                        if (plane->size() > 0) {
                            std::cerr << "stereo got plane, id: " << plane->at(0).id << ", ["
                                      << plane->at(0).normal[0] << "," << plane->at(0).normal[1]
                                      << "," << plane->at(0).normal[2] << "]" << std::endl;
                            s_stereoPlaneMutex.lock();
                            s_stereoPlane = plane;
                            s_stereoPlaneMutex.unlock();
                        }
                    });
            return true;
        }
        return false;
    }

    bool xslam_stop_detect_plane_from_stereo() {
        if (device->slam()) {
            device->slam()->unregisterStereoPlanesCallback(s_stereoPlaneId);
            return true;
        }
        return false;
    }

    bool xslam_get_plane_from_stereo(unsigned char *data, int *len) {
        if (s_stereoPlane) {
            std::shared_ptr<const std::vector<xv::Plane>> planes;
            s_stereoPlaneMutex.lock();
            planes = s_stereoPlane;
            s_stereoPlaneMutex.unlock();
            serializePlane(*planes.get(), data, len);
            return true;
        }
        return false;
    }

    void cslamSavedCallback(int status_of_saved_map, int map_quality) {
        std::cout << " Save map (quality is " << map_quality << "/100) and switch to CSlam:";
        switch (status_of_saved_map) {
            case 2:
                std::cout << " Map well saved. " << std::endl;
                break;
            case -1:
                std::cout << " Map cannot be saved, an error occured when trying to save it."
                          << std::endl;
                break;
            default:
                std::cout << " Unrecognized status of saved map " << std::endl;
                break;
        }
        mapStream.close();
    }

    void cslamSwitchedCallback(int map_quality) {
        std::cout << " map (quality is " << map_quality << "/100) and switch to CSlam:";
        mapStream.close();
    }

    void cslamLocalizedCallback(float percent) {
        static int k = 0;
        if (k++ % 100 == 0) {
            localized_on_reference_percent = static_cast<int>(percent * 100);
            std::cout << "localized: " << localized_on_reference_percent << "%" << std::endl;
        }
    }

    void xslam_read_version(unsigned char *version) {
        std::memcpy(version, xv::version().toString().c_str(), xv::Version().toString().size());
    }

    void xslam_read_device_version(unsigned char *version) {
        if(device && !g_driver_only){
            std::memcpy(version,  device->info().at("version").c_str(), device->info().at("version").size());
        }
    }
    /**
     * @brief 启动SLAM地图生成并注册回调函数
     *
     * 该函数用于启动SLAM地图生成并注册一个回调函数，在每次地图更新时被调用。
     * 回调函数会获取更新后的SLAM地图，并将其存储在 `slamMap` 中，供后续使用。
     *
     * @return bool 返回操作是否成功。返回 `true` 表示成功启动SLAM并注册回调，`false` 表示启动失败。
     */
    bool xslam_start_map() {

        if (device->slam()) {
            s_MapId = device->slam()->registerMapCallback(
                    [&](std::shared_ptr<const xv::SlamMap> map) {
                        std::cout << " NB 3D points " << map->vertices.size() << std::endl;
                        s_slamMapMutex.lock();
                        /**
                         * @brief 稀疏SLAM地图，包含3D点云数据
                         *
                         * 该结构体用于表示通过SLAM算法生成的稀疏地图，地图数据包含若干3D点。
                         * 这些点表示环境中的特征点，通常用于后续的定位、建图、地图优化等任务。
                         */
                        slamMap = map;
                        s_slamMapMutex.unlock();
                    });

            return true;
        }
        return false;
    }

    bool xslam_stop_map() {

        if (device->slam()) {
            s_MapId = device->slam()->unregisterMapCallback(s_MapId);

            return true;
        }
        return false;
    }

    bool xslam_load_map_and_switch_to_cslam(const char *mapPath, cslam_switched_callback csc,
                                            cslam_localized_callback clc) {
        std::cout << "load cslam map and switch to cslam" << std::endl;
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy xslam_load_map_and_switch_to_cslam entry 0");

#endif
        if (mapStream.is_open()) {
            mapStream.close();
        }

        if (mapStream.open(mapPath, std::ios::binary | std::ios::in) == nullptr) {
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                                "eddy xslam_load_map_and_switch_to_cslam entry 1");

#endif
            std::cout << "open " << map_filename << " failed." << std::endl;
            return false;
        }
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy xslam_load_map_and_switch_to_cslam entry 2");

#endif
        return device->slam()->loadMapAndSwitchToCslam(
                mapStream,
                csc,
                clc);
    }

    bool xslam_save_map_and_switch_to_cslam(const char *mapPath, cslam_saved_callback csc,
                                            cslam_localized_callback clc) {
        if (mapStream.is_open()) {
            mapStream.close();
        }
        if (mapStream.open(mapPath, std::ios::binary | std::ios::out | std::ios::trunc) ==
            nullptr) {
            // std::cout << "open " << map_filename << " failed." << std::endl;
            return false;
        }
        return device->slam()->saveMapAndSwitchToCslam(mapStream, csc, clc);
    }

/*   bool xslam_device_attach(int fd)
    {
        return xv::attach(fd);
    }

    bool xslam_device_detach(int fd)
    {
        return xv::detach(fd);
    } */

    int xslam_registerPlugEventCallback(const std::function<void(std::shared_ptr<xv::Device> device,
                                                                 xv::PlugEventType type)> &Callback) {

        return -1;
    }

    bool xslam_unregisterHotplugCallback(int callbackID) {
        // xv::unregisterHotplugCallback(callbackID);
        return false;
    }

    bool xv_device_init() {
        std::cout << "xvsdk version: " << xv::version() << std::endl;
        auto devices = xv::getDevices(5.);

        if (devices.empty()) {
            std::cout << "Timeout: no device found\n";
            return false;
        }

        device = devices.begin()->second;

        if (device->imuSensor()) {
            device->imuSensor()->registerCallback([](xv::Imu const &imu) {
                s_imuMutex.lock();
                s_imu = std::make_shared<xv::Imu>(imu);
                s_imuMutex.unlock();
            });
        }

        if (device->fisheyeCameras()) {
            device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const &images) {
                s_stereoImageMtx.lock();
                s_stereoImage = std::make_shared<xv::FisheyeImages>(images);
                s_stereoImageMtx.unlock();
            });
        }

        if (device->slam()) {
            device->slam()->registerCallback([](xv::Pose const &pose) {
                s_poseMutex.lock();
                s_slamPose = std::make_shared<xv::Pose>(pose);
                s_poseMutex.unlock();
            });
        }

        if (device->colorCamera()) {
            device->colorCamera()->registerCallback([](xv::ColorImage const &rgb) {
                s_colorMutex.lock();
                s_color = std::make_shared<xv::ColorImage>(rgb);
                s_colorMutex.unlock();
            });
        }

        if (device->tofCamera()) {
            device->tofCamera()->registerCallback([](xv::DepthImage const &tof) {
                s_depthImageMtx.lock();
                s_depthImage = std::make_shared<xv::DepthImage>(tof);
                s_depthImageMtx.unlock();
            });
        }

        if (device->sgbmCamera()) {
            device->sgbmCamera()->registerCallback([](xv::SgbmImage const &sgbm) {
                s_sgbmImageMtx.lock();
                s_sgbmImage = std::make_shared<xv::SgbmImage>(sgbm);
                s_sgbmImageMtx.unlock();
            });
        }
        s_ready = true;
        return true;
    }

    bool xv_start_slam() {
        bool bResult = false;
        if (device->slam()) {
            bResult = device->slam()->start();
        }
        return bResult;
    }

    bool xv_stop_slam() {
        bool bResult = false;
        if (device->slam()) {
            bResult = device->slam()->stop();
        }
        return bResult;
    }

    bool xv_get_6dof(Vector3 *position, Vector3 *orientation, Vector4 *quaternion,
                         long long *edgeTimestamp, double *hostTimestamp, double *confidence) {
        std::shared_ptr<xv::Pose> pose;
        s_poseMutex.lock();
        pose = s_slamPose;
        s_poseMutex.unlock();

        if (pose) {
            *edgeTimestamp = pose->edgeTimestampUs();
            *hostTimestamp = pose->hostTimestamp();

            position->x = (-1) * pose->x();
            position->y = (-1) * pose->y();
            position->z = (-1) * pose->z();

            auto pitchYawRoll = xv::rotationToPitchYawRoll(pose->rotation());
            orientation->x = pitchYawRoll[0];
            orientation->y = pitchYawRoll[1];
            orientation->z = pitchYawRoll[2];

            auto q = xv::rotationToQuaternion(pose->rotation());
            quaternion->x = q[0];
            quaternion->y = q[1];
            quaternion->z = q[2];
            quaternion->w = q[3];

            *confidence = pose->confidence();

            return true;
        }
        return false;
    }

    bool xv_get_6dof_prediction(Vector3 *position, Vector3 *orientation, Vector4 *quaternion,
                                    long long *edgeTimestamp, double *hostTimestamp,
                                    double *confidence, double prediction) {
        xv::Pose pose;
        if (device->slam()) {
            device->slam()->getPose(pose, prediction);
            *edgeTimestamp = pose.edgeTimestampUs();
            *hostTimestamp = pose.hostTimestamp();

            position->x = (-1) * pose.x();
            position->y = (-1) * pose.y();
            position->z = (-1) * pose.z();

            auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
            orientation->x = pitchYawRoll[0];
            orientation->y = pitchYawRoll[1];
            orientation->z = pitchYawRoll[2];

            auto q = xv::rotationToQuaternion(pose.rotation());
            quaternion->x = q[0];
            quaternion->y = q[1];
            quaternion->z = q[2];
            quaternion->w = q[3];

            *confidence = pose.confidence();

            return true;
        }
        return false;
    }

    bool xv_start_stereo() {
        bool bResult = false;
        if (device->fisheyeCameras()) {
            bResult = device->fisheyeCameras()->start();
        }
        return bResult;
    }

    bool
    xv_get_stereo_info(int *width, int *height, long long *edgeTimestamp, double *hostTimestamp,
                           unsigned int *dataSize) {
        std::shared_ptr<xv::FisheyeImages> stereo;
        s_stereoImageMtx.lock();
        stereo = s_stereoImage;
        s_stereoImageMtx.unlock();

        if (stereo) {
            *edgeTimestamp = stereo->edgeTimestampUs;
            *hostTimestamp = stereo->hostTimestamp;

            *width = stereo->images[0].width;
            *height = stereo->images[0].height;

            *dataSize = stereo->images[0].width * stereo->images[0].height;

            return true;
        }
        return false;
    }

    bool xv_get_stereo_image(unsigned char *left, unsigned char *right) {
        std::shared_ptr<xv::FisheyeImages> stereo;
        s_stereoImageMtx.lock();
        stereo = s_stereoImage;
        s_stereoImageMtx.unlock();

        if (stereo) {
            memcpy(left, stereo->images[0].data.get(),
                   sizeof(unsigned char) * stereo->images[0].width * stereo->images[0].height);
            memcpy(right, stereo->images[1].data.get(),
                   sizeof(unsigned char) * stereo->images[1].width * stereo->images[1].height);

            return true;
        }
        return false;
    }

    bool xv_start_imu() {
        bool bResult = false;
        if (device->imuSensor()) {
            bResult = device->imuSensor()->start();
        }
        return bResult;
    }

    bool
    xv_get_imu(Vector3 *accel, Vector3 *gyro, long long *edgeTimestamp, double *hostTimestamp) {
        std::shared_ptr<xv::Imu> imu;
        s_imuMutex.lock();
        imu = s_imu;
        s_imuMutex.unlock();

        if (imu) {
            accel->x = imu->accel[0];
            accel->y = imu->accel[1];
            accel->z = imu->accel[2];

            gyro->x = imu->gyro[0];
            gyro->y = imu->gyro[1];
            gyro->z = imu->gyro[2];

            *edgeTimestamp = imu->edgeTimestampUs;
            *hostTimestamp = imu->hostTimestamp;

            return true;
        }
        return false;
    }

    bool xv_start_rgb() {
        bool bResult = false;
        if (device->colorCamera()) {
            bResult = device->colorCamera()->start();
        }
        return bResult;
    }
    void get_boot_time() {
        timespec ts;
        uint64_t m_uptime;
        if( clock_gettime( CLOCK_BOOTTIME, &ts ) == 0 ){
            m_uptime = ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
        }
    }
    bool
    xv_get_rgb_info(int *width, int *height, long long *edgeTimestamp, double *hostTimestamp,
                        unsigned int *dataSize) {
        std::shared_ptr<xv::ColorImage> rgb;
        s_colorMutex.lock();
        rgb = s_color;
        s_colorMutex.unlock();

        if (rgb) {
            *edgeTimestamp = rgb->edgeTimestampUs;
            *hostTimestamp = rgb->hostTimestamp;

            *width = rgb->width;
            *height = rgb->height;

            *dataSize = rgb->dataSize;

            return true;
        }
        return false;
    }

    bool xv_get_rgb_image(unsigned char *data) {
        std::shared_ptr<xv::ColorImage> rgb;
        s_colorMutex.lock();
        rgb = s_color;
        s_colorMutex.unlock();

        if (rgb) {
            memcpy(data, rgb->data.get(), sizeof(unsigned char) * rgb->width * rgb->height);

            return true;
        }
        return false;
    }

    bool xv_start_tof() {
        bool bResult = false;
        if (device->tofCamera()) {
            bResult = device->tofCamera()->start();
        }
        return bResult;
    }

    bool
    xv_get_tof_info(int *width, int *height, long long *edgeTimestamp, double *hostTimestamp,
                        unsigned int *dataSize) {
        std::shared_ptr<xv::DepthImage> tof;
        s_depthImageMtx.lock();
        tof = s_depthImage;
        s_depthImageMtx.unlock();

        if (tof) {
            *edgeTimestamp = tof->edgeTimestampUs;
            *hostTimestamp = tof->hostTimestamp;

            *width = tof->width;
            *height = tof->height;

            *dataSize = tof->dataSize;

            return true;
        }
        return false;
    }

    bool xv_get_tof_image(unsigned char *data) {
        std::shared_ptr<xv::DepthImage> tof;
        s_depthImageMtx.lock();
        tof = s_depthImage;
        s_depthImageMtx.unlock();

        if (tof) {
            memcpy(data, tof->data.get(), tof->dataSize);

            return true;
        }
        return false;
    }

    bool xv_start_sgbm() {
        bool bResult = false;
#ifndef ANDROID
                                                                                                                                static struct xv::sgbm_config global_config = {
        0,                                             //enable_dewarp
        3.5,                                           //dewarp_zoom_factor
        1,                                             //enable_disparity
        1,                                             //enable_depth
        0,                                             //enable_point_cloud
        0.11285,                                       //baseline
        69,                                            //fov
        255,                                           //disparity_confidence_threshold
        {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, //homography
        0,                                             //enable_gamma
        2.2,                                           //gamma_value
        0,                                             //enable_gaussian
        0,                                             //mode
        5000,                                          //max_distance
        100,                                           //min_distance
    };

    if (device->sgbmCamera())
    {
        bResult = device->sgbmCamera()->start(global_config);
    }
#endif
        return bResult;
    }

    bool
    xv_get_sgbm_info(int *width, int *height, long long *edgeTimestamp, double *hostTimestamp,
                         unsigned int *dataSize) {
        std::shared_ptr<xv::SgbmImage> sgbm;
        s_sgbmImageMtx.lock();
        sgbm = s_sgbmImage;
        s_sgbmImageMtx.unlock();

        if (sgbm) {
            *edgeTimestamp = sgbm->edgeTimestampUs;
            *hostTimestamp = sgbm->hostTimestamp;

            *width = sgbm->width;
            *height = sgbm->height;

            *dataSize = sgbm->dataSize;

            return true;
        }
        return false;
    }

    bool xv_get_sgbm_image(unsigned char *data) {
        std::shared_ptr<xv::SgbmImage> sgbm;
        s_sgbmImageMtx.lock();
        sgbm = s_sgbmImage;
        s_sgbmImageMtx.unlock();

        if (sgbm) {
            memcpy(data, sgbm->data.get(), sizeof(unsigned char) * sgbm->width * sgbm->height);

            return true;
        }
        return false;
    }

    void saveImages(const char *name, const unsigned char *data, int len) {
        auto f = std::ofstream(name);
        if (f.is_open()) {
            f.write((const char *) data, len);
        } else {
            std::cout << "cannot write to " << name << std::endl;
        }
    }
    void xv_get_sn(char* sn, int bufferSize) {
        std::string serialNumber;
        if (device) {
            serialNumber = device->id();
        }
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xv_get_sn = %s",serialNumber.c_str());
#endif
        if (sn != nullptr && bufferSize > 0) {
            strncpy(sn, serialNumber.c_str(), bufferSize - 1);
//            sn[bufferSize - 1] = '\0';  // 确保字符串以null结尾
        }
    }
    void xv_get_sn(std::string &sn) {
        if (device) {
            sn = device->id();
        }
    }

    void xv_get_fe_camera_intrinsics_param(int *trans_size, int *ucm_size) {
        std::vector<xv::Calibration> calibrations = device->fisheyeCameras()->calibration();
        *trans_size = calibrations.size();
        *ucm_size = calibrations.at(0).ucm.size() * calibrations.size();
    }

    void xv_get_fe_camera_intrinsics(transform *trans, xv::UnifiedCameraModel *ucm) {
        std::vector<xv::Calibration> calibrations;
        std::vector<xv::UnifiedCameraModel> v_ucm;
        std::vector<transform> v_trans;
        int ucm_size = 0, trans_size = 0;
        calibrations = device->fisheyeCameras()->calibration();
        for (int i = 0; i < calibrations.size(); i++) {
            auto rotation = calibrations.at(i).pose.rotation();
            transform t;
            t.rotation[0] = rotation.at(0);
            t.rotation[1] = rotation.at(1);
            t.rotation[2] = rotation.at(2);
            t.rotation[3] = rotation.at(3);
            t.rotation[4] = rotation.at(4);
            t.rotation[5] = rotation.at(5);
            t.rotation[6] = rotation.at(6);
            t.rotation[7] = rotation.at(7);
            t.rotation[8] = rotation.at(8);
            auto translation = calibrations.at(i).pose.translation();
            t.translation[0] = translation.at(0);
            t.translation[1] = translation.at(1);
            t.translation[2] = translation.at(2);
            v_trans.push_back(t);
            trans_size++;
            for (int j = 0; j < calibrations.at(i).ucm.size(); j++) {
                v_ucm.push_back(calibrations.at(i).ucm.at(j));
                ucm_size++;
            }
        }
        memcpy(trans, v_trans.data(), sizeof(transform) * trans_size);
        memcpy(ucm, v_ucm.data(), sizeof(xv::UnifiedCameraModel) * ucm_size);
    }

    void xv_get_rgb_camera_intrinsics_param(int *trans_size, int *pdcm_size) {
        std::vector<xv::Calibration> calibrations = device->colorCamera()->calibration();
        *trans_size = calibrations.size();
        *pdcm_size = calibrations.at(0).pdcm.size() * calibrations.size();
    }

    void
    xv_get_rgb_camera_intrinsics(transform *trans, xv::PolynomialDistortionCameraModel *pdcm) {
        std::vector<xv::Calibration> calibrations;
        std::vector<xv::PolynomialDistortionCameraModel> v_pdcm;
        std::vector<transform> v_trans;
        int pdcm_size = 0, trans_size = 0;
        calibrations = device->colorCamera()->calibration();
        for (int i = 0; i < calibrations.size(); i++) {
            auto rotation = calibrations.at(i).pose.rotation();
            transform t;
            t.rotation[0] = rotation.at(0);
            t.rotation[1] = rotation.at(1);
            t.rotation[2] = rotation.at(2);
            t.rotation[3] = rotation.at(3);
            t.rotation[4] = rotation.at(4);
            t.rotation[5] = rotation.at(5);
            t.rotation[6] = rotation.at(6);
            t.rotation[7] = rotation.at(7);
            t.rotation[8] = rotation.at(8);
            auto translation = calibrations.at(i).pose.translation();
            t.translation[0] = translation.at(0);
            t.translation[1] = translation.at(1);
            t.translation[2] = translation.at(2);
            v_trans.push_back(t);
            trans_size++;
            for (int j = 0; j < calibrations.at(i).pdcm.size(); j++) {
                v_pdcm.push_back(calibrations.at(i).pdcm.at(j));
                pdcm_size++;
            }
        }
        memcpy(trans, v_trans.data(), sizeof(transform) * trans_size);
        memcpy(pdcm, v_pdcm.data(), sizeof(xv::PolynomialDistortionCameraModel) * pdcm_size);
    }

    void xv_get_tof_camera_intrinsics_param(int *trans_size, int *pdcm_size) {
        std::vector<xv::Calibration> calibrations = device->tofCamera()->calibration();
        *trans_size = calibrations.size();
        *pdcm_size = calibrations.at(0).pdcm.size() * calibrations.size();
    }

    void
    xv_get_tof_camera_intrinsics(transform *trans, xv::PolynomialDistortionCameraModel *pdcm) {
        std::vector<xv::Calibration> calibrations;
        std::vector<xv::PolynomialDistortionCameraModel> v_pdcm;
        std::vector<transform> v_trans;
        int pdcm_size = 0, trans_size = 0;
        calibrations = device->tofCamera()->calibration();
        for (int i = 0; i < calibrations.size(); i++) {
            auto rotation = calibrations.at(i).pose.rotation();
            transform t;
            t.rotation[0] = rotation.at(0);
            t.rotation[1] = rotation.at(1);
            t.rotation[2] = rotation.at(2);
            t.rotation[3] = rotation.at(3);
            t.rotation[4] = rotation.at(4);
            t.rotation[5] = rotation.at(5);
            t.rotation[6] = rotation.at(6);
            t.rotation[7] = rotation.at(7);
            t.rotation[8] = rotation.at(8);
            auto translation = calibrations.at(i).pose.translation();
            t.translation[0] = translation.at(0);
            t.translation[1] = translation.at(1);
            t.translation[2] = translation.at(2);
            v_trans.push_back(t);
            trans_size++;
            for (int j = 0; j < calibrations.at(i).pdcm.size(); j++) {
                v_pdcm.push_back(calibrations.at(i).pdcm.at(j));
                pdcm_size++;
            }
        }
        memcpy(trans, v_trans.data(), sizeof(transform) * trans_size);
        memcpy(pdcm, v_pdcm.data(), sizeof(xv::PolynomialDistortionCameraModel) * pdcm_size);
    }

    void xv_set_rgb_camera_resolution(xv::ColorCamera::Resolution resolution) {
        device->colorCamera()->setResolution(resolution);
    }

    void xv_start_fe_tag_detector(std::string &tagDetectorId) {
        if (device->fisheyeCameras()) {
            tagDetectorId = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(
                    device->fisheyeCameras())->startTagDetector(device->slam(), "36h11", 0.16, 50.);
        }
    }

    std::map<int, xv::Pose> detections;

    void xv_get_fe_tag_size(std::string &tagDetectorId, int *tagSize) {
        if (device->fisheyeCameras()) {
            detections = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(
                    device->fisheyeCameras())->getTagDetections(tagDetectorId);
            *tagSize = detections.size();
        }
    }

    void xv_get_fe_tag_detection(TagData *tags) {
        std::vector<TagData> v_tags;
        if (!detections.empty()) {
            for (auto const &d : detections) {
                TagData tag;
                tag.tagID = d.first;
                tag.confidence = d.second.confidence();

                tag.edgeTimestamp = d.second.edgeTimestampUs();
                tag.hostTimestamp = d.second.hostTimestamp();

                auto pitchYawRoll = xv::rotationToPitchYawRoll(d.second.rotation());
                tag.orientation.x = pitchYawRoll[0];
                tag.orientation.y = pitchYawRoll[1];
                tag.orientation.z = pitchYawRoll[2];

                auto q = xv::rotationToQuaternion(d.second.rotation());
                tag.quaternion.x = q[0];
                tag.quaternion.y = q[1];
                tag.quaternion.z = q[2];
                tag.quaternion.w = q[3];

                v_tags.push_back(tag);
            }
            memcpy(tags, v_tags.data(), sizeof(TagData) * detections.size());
        }
    }



xv::Matrix3d rotMultiply(xv::Matrix3d const& r1, xv::Matrix3d const& r2) {
    xv::Matrix3d ret;
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            ret[3*i+j] = r1[3*i]*r2[j] + r1[3*i+1]*r2[j+3] + r1[3*i+2]*r2[j+6];
        }
    }
    return ret;
}

xv::Matrix3d to_unity_rotation(xv::Matrix3d const& r)
{
    xv::Matrix3d s = 
    {
        1., 0., 0.,
        0., -1., 0.,
        0., 0., 1.
    }; 

    auto tmp = rotMultiply(s, r);
    return rotMultiply(tmp, s);
}

int xslam_detect_tags_internal2(const char *tagFamily, double size, TagArray *tagsArray, int arraySize)
{
    int tagsSize = 0;
    if (!device->fisheyeCameras())
    {
        return 0;
    }

    auto feDevice = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(std::dynamic_pointer_cast<xv::DeviceEx>(device)->fisheyeCameras());
    std::vector<std::pair<int,xv::Pose>> tags = feDevice->detectTags(device->slam(),tagFamily,size);
    if (tags.size() <= 0) {
        return 0;
    }

    int index = 0;
    tagsSize = tags.size() < arraySize ? tags.size() : arraySize;
    for (auto const &d : tags)
    {
        if (index >= tagsSize)
        {
            break;
        }

        DetectData& tag = tagsArray->detect[index];
        tag.tagID = d.first;
        tag.confidence = d.second.confidence();

        tag.edgeTimestamp = d.second.edgeTimestampUs();
        tag.hostTimestamp = d.second.hostTimestamp();

        tag.position.x = d.second.translation()[0];
        tag.position.y = d.second.translation()[1] * -1.;
        tag.position.z = d.second.translation()[2];

        auto unityRotation = to_unity_rotation(d.second.rotation());
        auto pitchYawRoll = xv::rotationToPitchYawRoll(unityRotation);
        tag.orientation.x = pitchYawRoll[0];
        tag.orientation.y = pitchYawRoll[1];
        tag.orientation.z = pitchYawRoll[2];

        auto q = xv::rotationToQuaternion(unityRotation);
        tag.quaternion.x = q[0];
        tag.quaternion.y = q[1];
        tag.quaternion.z = q[2];
        tag.quaternion.w = q[3];

#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xslam_detect_tags tag:%d conf:%f, (%f, %f, %f), (%f, %f, %f), (%f, %f, %f, %f)", tag.tagID, tag.confidence, tag.position.x, tag.position.y, tag.position.z,
                            tag.orientation.x, tag.orientation.y, tag.orientation.z, tag.quaternion.x, tag.quaternion.y, tag.quaternion.z, tag.quaternion.w);
#endif
        index ++;
    }

    return tagsSize;
}

static xv::AprilTagDetector* g_apriltag_detector = nullptr;
int xslam_detect_tags_internal4(const char *tagFamily, double size, TagArray *tagsArray, int arraySize)
{
    int tagsSize = 0;
    if (!device->fisheyeCameras())
    {
        return 0;
    }

    if (g_apriltag_detector == nullptr) {
        auto c = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->calibrationEx();
        std::vector<xv::CalibrationEx> calib;
        calib.push_back(c[0]);
        calib.push_back(c[1]);
        g_apriltag_detector = new xv::AprilTagDetector(calib, tagFamily);
        device->fisheyeCameras()->start();
        LOG_DEBUG(ANDROID_LOG_INFO, "apriltag", "detect_tags_internal4 start tagFamily:%s, size:%f", tagFamily, size);
        auto pitchYawRoll0 = xv::rotationToPitchYawRoll(calib[0].pose.rotation());
        auto pitchYawRoll1 = xv::rotationToPitchYawRoll(calib[1].pose.rotation());
        LOG_DEBUG(ANDROID_LOG_INFO, "apriltag", "detect_tags_internal4 calib0 trans:(%.3f, %.3f, %.3f) rotation(%.3f, %.3f, %.3f)",
                            calib[0].pose.x(), calib[0].pose.y(), calib[0].pose.z(), pitchYawRoll0[0], pitchYawRoll0[1], pitchYawRoll0[2]);
        LOG_DEBUG(ANDROID_LOG_INFO, "apriltag", "detect_tags_internal4 calib1 trans:(%.3f, %.3f, %.3f) rotation(%.3f, %.3f, %.3f)",
                            calib[1].pose.x(), calib[1].pose.y(), calib[1].pose.z(), pitchYawRoll1[0], pitchYawRoll1[1], pitchYawRoll1[2]);
    }

    xv::FisheyeImages frame;
    {
        std::lock_guard<std::mutex> lock(s_stereoImageMtx);
        if (s_stereoImage == nullptr) {
            LOG_DEBUG(ANDROID_LOG_INFO, "apriltag", "detect_tags_internal4 ERROR image");
            return 0;
        }
        frame.edgeTimestampUs = s_stereoImage->edgeTimestampUs;
        frame.hostTimestamp = s_stereoImage->hostTimestamp;
        frame.id = s_stereoImage->id;
        frame.images.push_back(s_stereoImage->images[0]);
        frame.images.push_back(s_stereoImage->images[1]);
    }
    xv::Pose pose;
    if(!device->slam()->getPoseAt(pose, frame.hostTimestamp)) {
        LOG_DEBUG(ANDROID_LOG_INFO, "apriltag", "getPoseAt FAIL");
        return 0;
    }

    std::vector<xv::TagPose> tags = g_apriltag_detector->detect(frame, size);
    LOG_DEBUG(ANDROID_LOG_INFO, "apriltag", "detect_tags_internal4 tagFamily:%s, size:%f, tags:%d", tagFamily, size, tags.size());
    tagsSize = tags.size() < arraySize ? tags.size() : arraySize;
    int index = 0;
    for (auto const &d: tags) {
        if (index >= tagsSize) {
            break;
        }

        DetectData &tag = tagsArray->detect[index];
        tag.tagID = d.tagId;
        tag.confidence = d.confidence;

        tag.edgeTimestamp = frame.edgeTimestampUs;
        tag.hostTimestamp = frame.hostTimestamp;

        auto p = pose * d.transform;
        tag.position.x = p.translation()[0];
        tag.position.y = p.translation()[1] * -1.;
        tag.position.z = p.translation()[2];

        auto unityRotation = to_unity_rotation(p.rotation());
        auto pitchYawRoll = xv::rotationToPitchYawRoll(unityRotation);
        tag.orientation.x = pitchYawRoll[0];
        tag.orientation.y = pitchYawRoll[1];
        tag.orientation.z = pitchYawRoll[2];

        auto q = xv::rotationToQuaternion(unityRotation);
        tag.quaternion.x = q[0];
        tag.quaternion.y = q[1];
        tag.quaternion.z = q[2];
        tag.quaternion.w = q[3];
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_INFO, "apriltag", "detect_tags_internal4 tag:%d conf:%f, (%f, %f, %f), (%f, %f, %f), (%f, %f, %f, %f)",
                            tag.tagID, tag.confidence, tag.position.x, tag.position.y, tag.position.z,
                            tag.orientation.x, tag.orientation.y, tag.orientation.z, tag.quaternion.x,
                            tag.quaternion.y, tag.quaternion.z, tag.quaternion.w);
#endif
        index ++;
    }

    return tagsSize;
}

int xslam_detect_tags(const char *tagFamily, double size, TagArray *tagsArray, int arraySize)
{
    if (!device->fisheyeCameras())
    {
        return 0;
    }

    int ret = 0;
    auto c = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->calibrationEx();
    if (std::strcmp(tagFamily,"qr-code") == 0) {
        xslam_start_detect_tags(tagFamily, size, tagsArray, arraySize);
    } else if (c.size() == 4) {
        ret = xslam_detect_tags_internal4(tagFamily, size, tagsArray, arraySize);
    } else if(c.size() == 2) {
        ret = xslam_detect_tags_internal2(tagFamily, size, tagsArray, arraySize);
    } else {
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xslam_start_detect_tags error calib:%d", c.size());
    }

    return ret;
}


int xslam_start_detect_tags(const char *tagFamily, double size, TagArray *tagsArray, int arraySize)
{
    int tagsSize = 0;
    if (!device->fisheyeCameras())
    {
        return 0;
    }

    if(tagDetectorId.empty())
    {
        device->fisheyeCameras()->start();
        tagDetectorId = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->startTagDetector(device->slam(), tagFamily, size, 50.);
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xslam_start_detect_tags tagDetectorId:%s", tagDetectorId.c_str());
#endif
    }
    else
    {
        {

            std::map<int, xv::Pose> detections = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->getTagDetections(tagDetectorId);
#ifdef ANDROID
            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xslam_start_detect_tags detections:%d", detections.size());

#endif

            tagsSize = detections.size() < arraySize ? detections.size() : arraySize;
            int index = 0;
            if (!detections.empty())
            {
                for (auto const &d : detections)
                {
                    if (index >= tagsSize)
                    {
                        break;
                    }

                    DetectData& tag = tagsArray->detect[index];
                    tag.tagID = d.first;
                    tag.confidence = d.second.confidence();

                    tag.edgeTimestamp = d.second.edgeTimestampUs();
                    tag.hostTimestamp = d.second.hostTimestamp();

                    tag.position.x = d.second.translation()[0];
                    tag.position.y = d.second.translation()[1] * -1.;
                    tag.position.z = d.second.translation()[2];

                    auto unityRotation = to_unity_rotation(d.second.rotation());
                    auto pitchYawRoll = xv::rotationToPitchYawRoll(unityRotation);
                    tag.orientation.x = pitchYawRoll[0];
                    tag.orientation.y = pitchYawRoll[1];
                    tag.orientation.z = pitchYawRoll[2];

                    auto q = xv::rotationToQuaternion(unityRotation);
                    tag.quaternion.x = q[0];
                    tag.quaternion.y = q[1];
                    tag.quaternion.z = q[2];
                    tag.quaternion.w = q[3];
#ifdef QRCODETAG
                    std::string codeStr = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->getCode(tagDetectorId,tag.tagID);
                    std::strcpy(tag.qrcode,codeStr.c_str());
#endif

#ifdef ANDROID
                    LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xslam_start_detect_tags tag:%d conf:%f, (%f, %f, %f), (%f, %f, %f), (%f, %f, %f, %f) qrcode is %s", tag.tagID, tag.confidence, tag.position.x, tag.position.y, tag.position.z,
                                        tag.orientation.x, tag.orientation.y, tag.orientation.z, tag.quaternion.x, tag.quaternion.y, tag.quaternion.z, tag.quaternion.w,tag.qrcode);
#endif
                    index ++;
                }

                // std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->stopTagDetector(tagDetectorId);
                // tagDetectorId = "";
                // device->fisheyeCameras()->stop();
            }
        }

    }

    return tagsSize;
}

void xslam_stop_detect_tags()
{
    device->fisheyeCameras()->stop();
    if (!tagDetectorId.empty())
    {
        std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->stopTagDetector(tagDetectorId);
    }
    tagDetectorId = "";
    if (g_apriltag_detector != nullptr) {
        g_apriltag_detector = nullptr;
    }
}

int xslam_start_rgb_detect_tags(const char *tagFamily, double size, TagArray *tagsArray, int arraySize)
{ 
    int tagsSize = 0;
    if (!device->colorCamera())
    {
        return 0;
    }

    if(tagRgbDetectorId.empty())
    {
        tagRgbDetectorId = std::dynamic_pointer_cast<xv::ColorCameraEx>(device->colorCamera())->startTagDetector(device->slam(), tagFamily, size, 50.);
    #ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xslam_start_rgb_detect_tags tagRgbDetectorId:%s", tagRgbDetectorId.c_str());
    #endif
    }
    else
    {
        std::map<int, xv::Pose> detections = std::dynamic_pointer_cast<xv::ColorCameraEx>(device->colorCamera())->getTagDetections(tagRgbDetectorId);
    #ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xslam_start_rgb_detect_tags detections:%d", detections.size());
    #endif

        tagsSize = detections.size() < arraySize ? detections.size() : arraySize;
        int index = 0;
        if (!detections.empty())
        {
            for (auto const &d : detections)
            {
                if (index >= tagsSize) 
                {
                    break;
                }


                DetectData& tag = tagsArray->detect[index];

                tag.tagID = d.first;
                tag.confidence = d.second.confidence();

                tag.edgeTimestamp = d.second.edgeTimestampUs();
                tag.hostTimestamp = d.second.hostTimestamp();
                    
                tag.position.x = d.second.translation()[0];
                tag.position.y = d.second.translation()[1] * -1.;
                tag.position.z = d.second.translation()[2]; 

                auto unityRotation = to_unity_rotation(d.second.rotation());
                auto pitchYawRoll = xv::rotationToPitchYawRoll(unityRotation);
                tag.orientation.x = pitchYawRoll[0];
                tag.orientation.y = pitchYawRoll[1];
                tag.orientation.z = pitchYawRoll[2];

                auto q = xv::rotationToQuaternion(unityRotation);
                tag.quaternion.x = q[0];
                tag.quaternion.y = q[1];
                tag.quaternion.z = q[2];
                tag.quaternion.w = q[3];
#ifdef QRCODETAG
                std::string codeStr = std::dynamic_pointer_cast<xv::ColorCameraEx>(
                        device->colorCamera())->getCode(tagRgbDetectorId,tag.tagID);
                std::strcpy(tag.qrcode,codeStr.c_str());
#endif
#ifdef ANDROID
    LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xslam_start_detect_tags tag:%d conf:%f, "
                                                  "(%f, %f, %f), (%f, %f, %f), (%f, %f, %f, %f) qrcode is %s",
                                                  tag.tagID, tag.confidence, tag.position.x, tag.position.y, tag.position.z,
                            tag.orientation.x, tag.orientation.y, tag.orientation.z, tag.quaternion.x,
                            tag.quaternion.y, tag.quaternion.z, tag.quaternion.w,tag.qrcode);
#endif

                index ++;
            }

            std::dynamic_pointer_cast<xv::ColorCameraEx>(device->colorCamera())->stopTagDetector(tagRgbDetectorId);
            tagRgbDetectorId = "";
        }
    }

    return tagsSize;
}

void xslam_stop_rgb_detect_tags()
{
    if (!tagRgbDetectorId.empty())
    {
        std::dynamic_pointer_cast<xv::ColorCameraEx>(device->colorCamera())->stopTagDetector(tagRgbDetectorId);
        device->colorCamera()->stop();
    }
    tagRgbDetectorId = "";
}



#ifdef XV_GAZE

//gaze

    static int gazeId = -1;

    static fn_gaze_callback callbackGaze;

    void gazeCallback(xv::XV_ET_EYE_DATA_EX const &eyedata) {

#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.timestamp = %lld", eyedata.timestamp);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy gazeCallback eyedata.recommend = %d",
                            eyedata.recommend);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.recomGaze.gazePoint.x = %f，y = %f,z = %f",
                            eyedata.recomGaze.gazePoint.x, eyedata.recomGaze.gazePoint.y,
                            eyedata.recomGaze.gazePoint.z);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.recomGaze.rawPoint.x = %f，y = %f,z = %f",
                            eyedata.recomGaze.rawPoint.x, eyedata.recomGaze.rawPoint.y,
                            eyedata.recomGaze.rawPoint.z);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.recomGaze.smoothPoint.x = %f，y = %f,z = %f",
                            eyedata.recomGaze.smoothPoint.x, eyedata.recomGaze.smoothPoint.y,
                            eyedata.recomGaze.smoothPoint.z);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.leftGaze.gazePoint.x = %f，y = %f,z = %f",
                            eyedata.leftGaze.gazePoint.x, eyedata.leftGaze.gazePoint.y,
                            eyedata.leftGaze.gazePoint.z);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.leftGaze.rawPoint.x = %f，y = %f,z = %f",
                            eyedata.leftGaze.rawPoint.x, eyedata.leftGaze.rawPoint.y,
                            eyedata.leftGaze.rawPoint.z);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.leftGaze.smoothPoint.x = %f，y = %f,z = %f",
                            eyedata.leftGaze.smoothPoint.x, eyedata.leftGaze.smoothPoint.y,
                            eyedata.leftGaze.smoothPoint.z);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.rightGaze.gazePoint.x = %f，y = %f,z = %f",
                            eyedata.rightGaze.gazePoint.x, eyedata.rightGaze.gazePoint.y,
                            eyedata.rightGaze.gazePoint.z);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.rightGaze.rawPoint.x = %f，y = %f,z = %f",
                            eyedata.rightGaze.rawPoint.x, eyedata.rightGaze.rawPoint.y,
                            eyedata.rightGaze.rawPoint.z);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.rightGaze.smoothPoint.x = %f，y = %f,z = %f",
                            eyedata.rightGaze.smoothPoint.x, eyedata.rightGaze.smoothPoint.y,
                            eyedata.rightGaze.smoothPoint.z);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.rightGaze.smoothPoint.x = %f",
                            eyedata.rightGaze.smoothPoint.x);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.rightGaze.smoothPoint.y = %f",
                            eyedata.rightGaze.smoothPoint.y);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.rightGaze.exDataBitMask = %d",
                            eyedata.rightGaze.exDataBitMask);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.rightGaze.rawPoint.y = %f",
                            eyedata.rightGaze.rawPoint.y);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.rightGaze.gazeDirection.y = %f",
                            eyedata.leftGaze.gazeDirection.y);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.leftPupil.pupilBitMask = %d",
                            eyedata.leftPupil.pupilBitMask);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.leftPupil.pupilDistance = %f",
                            eyedata.leftPupil.pupilDistance);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.leftPupil.pupilCenter.x = %f",
                            eyedata.leftPupil.pupilCenter.x);
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr",
                            "eddy gazeCallback eyedata.leftPupil.pupilCenter.y = %f",
                            eyedata.leftPupil.pupilCenter.y);
#endif

        if (callbackGaze) {
            callbackGaze(eyedata);
        }
    }
    std::shared_ptr<xv::CameraModel> rgbCameraModel;
    bool xslam_get_gaze2rgb_xy(double* gaze, double* p2d)
    {
        if (device->colorCamera() && !rgbCameraModel)
            rgbCameraModel = std::dynamic_pointer_cast<xv::CameraEx>(device->colorCamera())->cameraModel();
        if (!rgbCameraModel) {

            return false;
        }
        xv::Transform pRgbImu; // IMU pose in RGB
        xv::Transform pImudisplay; // display pose in imu

        pRgbImu = device->colorCamera()->calibration()[0].pose.inverse(); // IMU pose in RGB
        pImudisplay = device->display()->calibration()[0].pose; //display pose in imu

        pImudisplay.setTranslation({pImudisplay.x()+0.032,pImudisplay.y(),pImudisplay.z()});

        auto pRgbeye_c = pRgbImu * pImudisplay;

        xv::Vector3d p3dRgb = pRgbeye_c * xv::Vector3d({gaze[0],gaze[1],gaze[2]});

        if (rgbCameraModel->project(p3dRgb.data(), p2d))
        {
            return true;
        }
        return false;
    }
    bool xslam_start_gaze() {
        if (!s_ready)
            return false;
        return device->gaze()->start();
        //  return true;
    }

    bool xslam_stop_gaze() {
        if (!s_ready)
            return false;
        return device->gaze()->stop();
    }
/**
 * @brief 设置眼动追踪的回调函数。
 *
 * 该函数用于设置一个回调函数，用于处理眼动追踪的 gaze 数据。当设备产生新的 gaze 数据时，
 * 会通过回调函数将数据传递给调用者。
 *
 * @param cb 一个用户定义的回调函数，其类型为 `fn_gaze_callback`，用于接收 gaze 数据。
 *           如果传入空指针（nullptr），则会移除当前的回调。
 *
 * @return int
 * - `0` 表示设置成功。
 * - 非 `0` 表示设置失败（例如设备未初始化或回调函数无效）。
 */
    int xslam_set_gaze_callback(fn_gaze_callback cb) {
        if (s_ready) {
            callbackGaze = cb;
            return gazeId = device->gaze()->registerCallback(gazeCallback);
        } else
            return -1;
    }

    bool xslam_unset_gaze_callback() {
        if (!s_ready)
            return -1;
        return device->gaze()->unregisterCallback(gazeId);
    }

    void xslam_gaze_set_config_path(const char *path) {
        if (!s_ready)
            return;
        std::string s(path);
        return device->gaze()->setConfigPath(s);
    }
    void xslam_set_usr_eye_ready() {
        if (!s_ready)
            return;
        return device->gaze()->setUsrEyeReady();
    }
    bool xslam_get_gaze_status() {
        if (!s_ready)
            return -1;
        return device->gaze()->getGazeStatus();
    }

    void xslam_gaze_enable_dump(bool enable) {
        if (!s_ready)
            return;
         device->gaze()->enableDump(enable);
    }
    void xslam_set_gaze_configs(int width,int height,float ipdDist,int srValue/*,int etWidth,int etHeight*/) {
        if (!s_ready)
            return;
        xv::GazeConfigs config;
        config.screenWidth = width;
        config.screenHeight = height;
        config.ipdDist = ipdDist;
        config.srValue = srValue;
 /*       config.etWidth = etWidth;
        config.etHeight = etHeight;*/
        return device->gaze()->setGazeConfigs(config);
    }
//#ifndef __linux__
    int xslam_start_gaze_calibration(int points) {
        return calibration.StartCalibration(points);
    }

    int xslam_start_calibration_point(int eye, int index, const xv::XV_ET_POINT_2D *point,
                                      xv::xv_ET_point_process_callback cb1, void *context1,
                                      xv::xv_ET_point_finish_callback cb2, void *context2) {
        return calibration.StartCalibrationPoints(eye, index, point, cb1, context1, cb2, context2);
    }

    int xslam_compute_calibration(int eye, xv::XV_ET_COEFFICIENT *out_coe) {
        return calibration.ComputeCalibration(eye, out_coe);
    }

    int xslam_complete_calibration() {
        return calibration.CompleteCalibration();
    }

    int xslam_cancel_gaze_calibration(int eye) {
        return calibration.CancelCalibration(eye);
    }

    int xslam_set_default_calibration(int eye, float minX, float maxX, float minY, float maxY,
                                      const xv::XV_ET_COEFFICIENT *coe) {

        return calibration.SetDefaultCalibration(eye, minX, maxX, minY, maxY, coe);
    }
//#endif
#endif
/**
 * @brief 设置左右眼摄像头的曝光参数。
 *
 * 该函数用于为眼动追踪设备的左右眼摄像头分别设置增益值（Gain）和曝光时间（曝光时长）。
 *
 * @param leftGain 左眼摄像头的增益值，取值范围[0, 255]
 * @param leftTimeMs 左眼摄像头的曝光时间（单位：毫秒），取值范围应符合设备规格。
 * @param rightGain 右眼摄像头的增益值，取值范围[0, 255]
 * @param rightTimeMs 右眼摄像头的曝光时间（单位：毫秒），取值范围应符合设备规格。
 *
 * @return bool
 * - `true` 表示曝光参数设置成功。
 * - `false` 表示设置失败（例如设备未准备好或设备不支持眼动追踪功能）。
 */
bool xslam_set_exposure(int leftGain, float leftTimeMs, int rightGain, float rightTimeMs) {
    if (!s_ready || !device || !device->eyetracking()) {
        return false;
    }

    return device->eyetracking()->setExposure(leftGain, leftTimeMs, rightGain, rightTimeMs);
}
/**
 * @brief 设置眼动追踪设备中指定 LED 的亮度。
 *
 * 该函数用于为眼动追踪设备的指定眼部和 LED 设置亮度值，以控制其发光强度。
 *
 * @param eye  0:left, 1:right, 2:both
 * @param led [0,7]:led index, 8:all
 * @param brightness [0,255], 0 is off
 *
 * @return bool
 * - `true` 表示亮度设置成功。
 * - `false` 表示设置失败（例如设备未准备好、设备不支持眼动追踪功能，或参数超出有效范围）。
 */
bool xslam_set_bright(int eye, int led, int brightness) {
    if (!s_ready || !device || !device->eyetracking()) {
        return false;
    }
    return device->eyetracking()->setLedBrighness(eye, led, brightness);
}

void xslam_set_device_status_callback(device_status_callback_ex cb) {
    if (s_ready) {
        callbackDeviceStatusEx = cb;
        memset(m_DeviceStatus.status,-1,sizeof(m_DeviceStatus.status));
        isSetDeviceCb = true;
    }
}

#ifdef XV_EYE_TRACK
#include "xv_pub_main.h"

#define LOG_TAG "xv#vuelosophy"
#define LOG_DEBUG(...)                                                         \
  do {                                                                         \
    LOG_DEBUG(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__);              \
  } while (false)

static int eyetrackingId = -1;
static XvPubData s_pub_data[2];
static unsigned char* s_pub_dbg_data0 = NULL;
static unsigned char* s_pub_dbg_data1 = NULL;
static bool s_get_dbg_img = false;

void on_eyetracking_callback(xv::EyetrackingImage const &eyetracking_image) {
  if (eyetracking_image.images.empty()) {
    LOG_DEBUG("on_eyetracking_callback no eyetracking image avaiable");
    return;
  }


  auto images = eyetracking_image.images;
  int w = images[0].width;
  int h = images[0].height;
  LOG_DEBUG("on_eyetracking_callback %d, %d", w, h);
  xv_pub_set_img((uint8_t *)images[0].data.get(),
                 (uint8_t *)images[1].data.get(), w * h,
                 eyetracking_image.edgeTimestampUs);
}

void on_pub_callback(int etIndex, float gaze_x, float gaze_y, float pupl_x, float pupl_y, int64_t ts) {
  if (etIndex != 0 && etIndex != 1) {
    return;
  }

  if (gaze_x < 0 || gaze_x < 0) {
    return;
  }

  if (pupl_x < 0 || pupl_y < 0) {
    return;
  }

  s_pub_data[etIndex].gaze_x = gaze_x;
  s_pub_data[etIndex].gaze_y = gaze_y;
  s_pub_data[etIndex].pupl_x = pupl_x;
  s_pub_data[etIndex].pupl_y = pupl_y;
  s_pub_data[etIndex].ts = ts;
}

void on_pub_dbg_callback(int etIndex, void *data, int total) {
  if (etIndex != 0 && etIndex != 1) {
    return;
  }

  if (data == NULL || total <= 0) {
    return;
  }

  if (etIndex == 0) {
    if (s_pub_dbg_data0 == NULL) {
        s_pub_dbg_data0 = (unsigned char*)malloc(total);
        std::memset(s_pub_dbg_data0, 0, total);
    }
    std::memcpy(s_pub_dbg_data0, data, total);
  } else if (etIndex == 1) {
    if (s_pub_dbg_data1 == NULL) {
        s_pub_dbg_data1 = (unsigned char*)malloc(total);
        std::memset(s_pub_dbg_data1, 0, total);
    }
    std::memcpy(s_pub_dbg_data1, data, total);
  }
  s_get_dbg_img = true;
}

void xslam_stop_pub_eyetrack() {
  if (!device || !device->eyetracking()) {
    return;
  }

  device->eyetracking()->stop();
  if (eyetrackingId != -1) {
    device->eyetracking()->unregisterCallback(eyetrackingId);
    eyetrackingId = -1;
  }
  xv_pub_dispose();
  xv_pub_register(NULL);
}

void xslam_start_pub_eyetrack(int width, int height) {
  LOG_DEBUG("start_eyetracking %d, %d", width, height);
  if (!device || !device->eyetracking()) {
    return;
  }

  xslam_stop_pub_eyetrack();
  device->eyetracking()->start();
  eyetrackingId =
      device->eyetracking()->registerCallback(on_eyetracking_callback);
  xv_pub_init(2, width, height, width, height, 50);
  xv_pub_register(on_pub_callback);
  xv_pub_register_dbg(on_pub_dbg_callback);
}

void xslam_get_pub_eyetrack(int eye, XvPubData *pub_data) {
  if (eye != 0 && eye != 1) {
    return;
  }

  if (pub_data) {
    pub_data->etIndex = eye;
    pub_data->gaze_x = s_pub_data[eye].gaze_x;
    pub_data->gaze_y = s_pub_data[eye].gaze_y;
    pub_data->pupl_x = s_pub_data[eye].pupl_x;
    pub_data->pupl_y = s_pub_data[eye].pupl_y;
    pub_data->ts = s_pub_data[eye].ts;
  }
}

bool xslam_get_pub_dbg(int eye, unsigned char * data, int width, int height) {
    if (eye != 0 && eye != 1) {
        return false;
    }

    if (!s_pub_dbg_data0 || !s_pub_dbg_data1) {
        return false;
    }

    if (!data || !s_get_dbg_img) {
        return false;
    }

    unsigned char* gray = eye == 0 ? s_pub_dbg_data0 : s_pub_dbg_data1;
    cv::Mat mgray(height, width, CV_8UC1, gray);
    cv::Mat mrgb(height, width, CV_8UC4, data);
    cv::cvtColor(mgray, mrgb, cv::COLOR_GRAY2BGRA);
    return true;
}

bool xslam_pub_calibration(int eye, int index, float x, float y) {
    if (eye != 0 && eye != 1) {
        return false;
    }

    int ret = xv_pub_set_calibration(eye, index, x, y);
    return ret == 1 ? true : false;
}

#endif

xv::RgbPixelPoseWithTof* g_rgb_pixel_pointing = nullptr;

void xv_start_rgb_pixel_pose() {
    if (!device || !device->tofCamera()) {
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xv_start_rgb_pixel_pose error tof camera");
        return;
    }

    device->tofCamera()->start();

    if (g_rgb_pixel_pointing == nullptr) {
        g_rgb_pixel_pointing = new xv::RgbPixelPoseWithTof(device);
    }
}

void xv_stop_rgb_pixel_pose() {
    if (!device || !device->tofCamera()) {
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xv_stop_rgb_pixel_pose error tof camera");
        return;
    }

    device->tofCamera()->stop();
}

bool xslam_start_get_rgb_pixel_buff3d_pose(pointer_3dpose* pointerPose, Vector2* rgbPixelPoint, int arraySize,double hostTimestamp, float radius){
    bool ret = false;
    std::vector<xv::Vector2d> click;
    for (int i = 0; i < arraySize; ++i){
        click.push_back({rgbPixelPoint[i].x,rgbPixelPoint[i].y});
    }

    std::vector<std::pair<xv::Vector2d,xv::Vector3d>> t_pointerPos;

    bool  markerPoseOk = g_rgb_pixel_pointing->getRgbPixelbuff3dPoseAt(t_pointerPos, hostTimestamp, click, radius);
    for (const auto& element : click) {
        std::cout << "x: " << element.data()[0] << ", y: " << element.data()[1] << std::endl;
    }
//    *pointsSize = t_pointerPos.size();
    for (int i = 0; i < t_pointerPos.size(); ++i) {
        pointerPose[i].pointerPose.x = t_pointerPos.at(i).second.data()[0];
        pointerPose[i].pointerPose.y = t_pointerPos.at(i).second.data()[1];
        pointerPose[i].pointerPose.z = t_pointerPos.at(i).second.data()[2];
        pointerPose[i].rgbPixelPoint.x = t_pointerPos.at(i).first.data()[0];
        pointerPose[i].rgbPixelPoint.y = t_pointerPos.at(i).first.data()[1];
        if(pointerPose[i].pointerPose.x == 0 && pointerPose[i].pointerPose.y == 0 && pointerPose[i].pointerPose.z == 0)
            pointerPose[i].isValid = false;
        else
            pointerPose[i].isValid = true;
        ret = true;
        std::cout << "isValid: (" << pointerPose[i].isValid << "), ";
        std::cout << "Key: (" << t_pointerPos.at(i).first.data()[0] << ", " << t_pointerPos.at(i).first.data()[1]  << "), ";
        std::cout << "Value: (" << t_pointerPos.at(i).second.data()[0] << ", " << t_pointerPos.at(i).second.data()[1] << ", " << t_pointerPos.at(i).second.data()[2] << ")" << std::endl;

    }

    return ret;
}
bool xv_get_rgb_pixel_pose(Vector3* pointerPose, Vector2* rgbPixelPoint, double hostTimestamp, float radius) {
    bool ret = false;
    if (pointerPose == nullptr || rgbPixelPoint == nullptr) {
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xv_get_rgb_pixel_pose error para");
        return false;
    }

    if (!g_rgb_pixel_pointing) {
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xv_get_rgb_pixel_pose error start FAIL");
        return false;
    }

    xv::Vector3d markerPose;
    xv::Vector2d rgbPose = {rgbPixelPoint->x,rgbPixelPoint->y};

    LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xv_get_rgb_pixel_pose rgbPose start");

    auto start = std::chrono::steady_clock::now();
    bool markerOK = g_rgb_pixel_pointing->getRgbPixel3dPoseAt(markerPose, hostTimestamp, rgbPose, radius);

    // 记录结束时间点
    auto end = std::chrono::steady_clock::now();

    // 计算执行时间
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
#ifdef ANDROID
    LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "Rectification takes {%f} ms", std::chrono::duration_cast<std::chrono::microseconds>(end-start).count()*1e-3);
#endif
    LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xv_get_rgb_pixel_pose rgbPose(%f, %f) %f==>%d (%f, %f, %f)",

                        rgbPose[0], rgbPose[1], radius, ret, markerPose[0], markerPose[1], markerPose[2]);
    if (markerOK) {
        pointerPose->x = markerPose[0];
        pointerPose->y = markerPose[1];
        pointerPose->z = markerPose[2];
        ret = true;
    }
    return ret;
}

bool xv_get_rgb_pixel_3dpose(Vector3* pointer3dPose, Vector2* rgbPixelPoint, float radius) {
    bool ret = false;
    if (pointer3dPose == nullptr || rgbPixelPoint == nullptr) {
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xv_get_rgb_pixel_3dpose error para");
        return false;
    }

    device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_1280x720);
    // get extrinsics between IMU and ToF and IMU and RGB camera
    xv::details::Transform_<double> pImuTof = device->tofCamera()->calibration()[0].pose; // ToF pose in IMU
    xv::details::Transform_<double> pRgbImu = device->colorCamera()->calibration()[0].pose.inverse(); // IMU pose in RGB

    // This camera model will be intialized after started the color camera to be sure to have a camera model with the good resolution (if change the resolution, need to get again the camera model).
    std::shared_ptr<xv::CameraModel>  cameraModel = std::dynamic_pointer_cast<xv::CameraEx>(device->colorCamera())->cameraModel();
    // Get rgb and Tof images copies
    xv::ColorImage rgb;
    {
        std::lock_guard<std::mutex> l(s_colorMutex);
        rgb = *(s_color.get());
    }

    xv::DepthImage tof;
    {
        std::lock_guard<std::mutex> l(s_depthImageMtx);
        tof = *(s_depthImage.get());
    }
    std::vector<xv::Vector3d> marker3dPoints;
    std::vector<float> depths;
    if (rgb.dataSize<=0 || !tof.data || !cameraModel) {
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xv_get_rgb_pixel_3dpose error rgb tof camera data");
        return false;
    }

    auto pointCloud = device->tofCamera()->depthImageToPointCloud(tof);
    if (!pointCloud) {
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xv_get_rgb_pixel_3dpose error pointCloud data");
        return false;
    }

    xv::Pose poseAtTof, poseAtRgb;
    // if VSLAM is running, get the IMU poses at RGB image and ToF image timestamps to make the 3D point synchronised ...
    if (!device->slam() || !device->slam()->getPoseAt(poseAtTof, tof.hostTimestamp) || !device->slam()->getPoseAt(poseAtRgb, rgb.hostTimestamp)) {
        poseAtTof = poseAtRgb = xv::Pose::Identity();
    }

    xv::details::Transform_<double> const& pWorldImuAtTof = poseAtTof;
    xv::details::Transform_<double> pImuWorldAtRgb = poseAtRgb.inverse();

    // for each point of the point cloud ...
    for (auto const& p3d : pointCloud->points) {
        if (p3d[2] > 9.999 && p3d[2]< 11.001) {
            continue;
        }
        else
        {
            // ... convert the point in world coordinates ...
            xv::Vector3d p3dW = pWorldImuAtTof * pImuTof * xv::Vector3d({p3d[0],p3d[1],p3d[2]});

            // ... convert the point in RGB camera coordinates
            xv::Vector3d p3dRgb = pRgbImu * pImuWorldAtRgb * p3dW;

            // ... project the 3D point in 2D point in RGB camera pixels coordinates ...
            xv::Vector2d p2d;
            // ... if the point in image is inside the detection ROI, then push it to the vector of detection depths
            if (cameraModel->project(p3dRgb.data(), p2d.data()) && std::abs(rgbPixelPoint->x-p2d[0])<radius && std::abs(rgbPixelPoint->y-p2d[1])<radius) {
                    marker3dPoints.push_back(p3dRgb);
                    depths.push_back(p3dRgb[2]);
            }
        }
    }


    // if enough 3D points, then compute the median of depth to get the corresponding 3D point
    if (marker3dPoints.size()>5) {
        // compute the median point based on depth (z) value
        auto m = marker3dPoints.begin() + marker3dPoints.size() / 2;
        std::nth_element(marker3dPoints.begin(), m, marker3dPoints.end(), [&](xv::Vector3d const& a, xv::Vector3d const& b){
            return a[2]<b[2];
        });
        xv::Vector3d pointerPose = poseAtRgb*pRgbImu.inverse() * marker3dPoints[marker3dPoints.size()/2];
        pointer3dPose->x = pointerPose[0];
        pointer3dPose->y = pointerPose[1];
        pointer3dPose->z = pointerPose[2];
        ret = true;
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xv_get_rgb_pixel_3dpose pointerPose--%f,%f,%f", pointerPose[0],pointerPose[1],pointerPose[2]);
    } else {
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "xv_get_rgb_pixel_3dpose No ToF points in the RGB pixels area, cannot provide 3D position of RGB pixel point");
    }

    return ret;
}

#ifdef FE_RECTIFICATION

void write_debug_file(const char* path, char* data, int size) {
    FILE* fp = NULL;
    fp = fopen(path, "w");
    if (fp != NULL) {
        fwrite(data, 1, size, fp);
        fclose(fp);
    }
}

void fisheyes_rectification_thread(){
    {

        while (fe_running)
        {

            auto t0 = std::chrono::steady_clock::now();
            if(!(s_ready &&device!=nullptr)){
                std::this_thread::sleep_until(t0+std::chrono::milliseconds(10));
                continue;
            }
            xv::FisheyeImages frame;
            frame.hostTimestamp = -1e9;
            {
                std::lock_guard<std::mutex> lock(s_stereoImageMtx);
                if (hasFisheyesImagesUpdate) {
                   frame = *s_stereoImage;
                   hasFisheyesImagesUpdate = false;
                }
            }

            if(frame.hostTimestamp>0 && s_stereo_runing){

                if (!stereoRectificationMesh) {
#ifdef FE_RECTIFICATION_SEUCM_UPDATE
                    const std::vector<xv::Calibration>& calibs = device->fisheyeCameras()->calibration();
                    std::vector<xv::Calibration> calibs_para;
                    if (calibs.size() == 4) {
                        calibs_para.push_back(calibs[2]);
                        calibs_para.push_back(calibs[3]);
                    } else {
                        calibs_para.push_back(calibs[0]);
                        calibs_para.push_back(calibs[1]);
                    }
                    stereoRectificationMesh = std::make_shared<xv::StereoRectificationMesh>(calibs_para, frame.images[0].width, frame.images[0].height);
                    // stereoRectificationMesh = std::make_shared<xv::StereoRectificationMesh>(device->display()->calibration());

                    double timestampDiff = 0.;
                    std::vector<xv::CalibrationEx> seucm;
                    std::dynamic_pointer_cast<xv::DeviceEx>(device)->getFisheyeCalibration(seucm,timestampDiff);

                    for (auto const& c: seucm) {
                        for (auto const& s : c.seucm) {
                            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "Rectification SEUCM: %.4f,%.4f %dx%d", s.fx, s.fy, s.w, s.h);
                        }
                        for (auto const& s : c.camerasModel) {
                            LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "Rectification CM: %dx%d", s->width(), s->height());
                        }

                    }
#else
                     stereoRectificationMesh = std::make_shared<xv::StereoRectificationMesh>(device->fisheyeCameras()->calibration());
                    // stereoRectificationMesh = std::make_shared<xv::StereoRectificationMesh>(device->display()->calibration());
#endif
                }
                auto t1 = std::chrono::steady_clock::now();
                int left = frame.images.size()==4 ? 2 : 0;
                int right = frame.images.size()==4 ? 3 : 1;
                auto fe = stereoRectificationMesh->rectify(frame.images[left], frame.images[right]);
                auto t2 = std::chrono::steady_clock::now();

                static int cpt = 0;
                if (++cpt>10) {
#ifdef ANDROID
                    LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "Rectification takes {%f} ms", std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count()*1e-3);
#endif

                    cpt = 0;
                }
                xv::Pose pose;
                if ( device->slam()->getPoseAt(pose, frame.hostTimestamp))

                {
                    {
                        std::lock_guard<std::mutex> lock(s_feImagesMtx);
                        has_fe_images = true;
                        m_feImagesInfo.fe = fe;
                        m_feImagesInfo.pose = pose;
                    }

                }

            }else{
                 std::this_thread::sleep_until(t0+std::chrono::milliseconds(10));
            }

        }


    }
}

// a thread to do FE rectification
void xslam_start_fisheyes_rectification_thread(){
    if(fe_running){
        return;
    }
    fe_running = true;
    fisheyesRectificationThread = std::thread(fisheyes_rectification_thread);

}

void xslam_stop_fisheyes_rectification_thread(){
    if(fe_running){
        fe_running = false;
        fisheyesRectificationThread.join();
    }

}
bool xslam_get_fisheyes_rectification_thread()
{
    return fe_running;
}
bool xslam_get_fe_images_data(int *width,int *height,unsigned char* left,unsigned char* right,double* poseData){

    if(has_fe_images){
        {
            std::lock_guard<std::mutex> lock(s_feImagesMtx);
            has_fe_images = false;
            int w = m_feImagesInfo.fe.first.width;
            int h = m_feImagesInfo.fe.first.height;
            if(left!=NULL){
                memcpy(left,m_feImagesInfo.fe.first.data.get(),w*h);
            }
            if(right!=NULL){
                memcpy(right,m_feImagesInfo.fe.second.data.get(),w*h);
            }

            *width = w;
            *height = h;
            xv::Pose pose = m_feImagesInfo.pose;
            auto r = pose.rotation();
            auto _quat = xv::rotationToQuaternion(r);
            poseData[0] = _quat[0];
            poseData[1] = _quat[1];
            poseData[2] = _quat[2];
            poseData[3] = _quat[3];

            poseData[4] = pose.x();
            poseData[5] = pose.y();
            poseData[6] = pose.z();

        }
        return true;
    }
    return false;
}

bool xslam_get_fe_mesh_params(double *focal, double *baseline, int *camerasModelWidth,int *camerasModelHeight,double *leftPose, double  *rightPose)
{
    if (s_stereo_runing&&stereoRectificationMesh){
        *focal = stereoRectificationMesh->focal();
        *baseline = stereoRectificationMesh->baseline();
        if (g_driver_only) {
            std::vector<xv::CalibrationEx> feCalib;
            double imuFeTimeOffset;
            bool ret = (std::dynamic_pointer_cast<xv::DeviceEx>(device))->getFisheyeCalibration(feCalib, imuFeTimeOffset);
            if (ret && feCalib.size() > 0) {
                *camerasModelWidth = feCalib[0].seucm[0].w;
                *camerasModelHeight = feCalib[0].seucm[0].h;
            }
        } else {
            xv::Calibration cali =  device->fisheyeCameras()->calibration()[0];
            *camerasModelWidth = cali.camerasModel[0]->width();
            *camerasModelHeight = cali.camerasModel[0]->height();
        }
        xv::Transform left = stereoRectificationMesh->leftVirtualPose();
        xv::Transform right = stereoRectificationMesh->rightVirtualPose();
        auto rLeft = left.rotation();
        auto qLeft = xv::rotationToQuaternion(rLeft);
        leftPose[0] = qLeft[0];
        leftPose[1] = qLeft[1];
        leftPose[2] = qLeft[2];
        leftPose[3] = qLeft[3];
        leftPose[4] = left.x();
        leftPose[5] = left.y();
        leftPose[6] = left.z();
        auto rRight = left.rotation();
        auto qRight = xv::rotationToQuaternion(rRight);
        rightPose[0] = qRight[0];
        rightPose[1] = qRight[1];
        rightPose[2] = qRight[2];
        rightPose[3] = qRight[3];
        rightPose[4] = right.x();
        rightPose[5] = right.y();
        rightPose[6] = right.z();
        return true;
    }
    return false;
}

#endif

#ifdef XV_GPS
    bool set_rgb_source(RgbSource source) {
        s_rgbSource = source;
        return true;
    }
    bool set_rgb_resolution(RgbResolution res) {
        if (device->colorCamera()) {
            switch (res) {
                case RGB_1920x1080:
                    return device->colorCamera()->setResolution(
                            xv::ColorCamera::Resolution::RGB_1920x1080);
                    break;
                case RGB_1280x720:
                    return device->colorCamera()->setResolution(
                            xv::ColorCamera::Resolution::RGB_1280x720);
                    break;
                case RGB_640x480:
                    return device->colorCamera()->setResolution(
                            xv::ColorCamera::Resolution::RGB_640x480);
                    break;
                case RGB_320x240:
                    return device->colorCamera()->setResolution(
                            xv::ColorCamera::Resolution::RGB_320x240);
                    break;
                case RGB_2560x1920:
                    return device->colorCamera()->setResolution(
                            xv::ColorCamera::Resolution::RGB_2560x1920);
                    break;
            }
        } else {
            return false;
        }
    }


    void start_rgb_stream() {
#ifdef ANDROID
        LOG_DEBUG(ANDROID_LOG_WARN, "xvxr", "eddy xslam_start_rgb_stream entry");

#endif

        // VSC may not support rgb, so must use if
        if (device && device->colorCamera()) {

            device->colorCamera()->start();
        }

        s_uvc_rgb_runing = true;
    }

    void stop_rgb_stream() {

        if (device && device->colorCamera()) {
            device->colorCamera()->stop();
        }

        s_uvc_rgb_runing = false;
    }

    int get_rgb_width() {
        if (!s_color)
            return 0;

        return s_color->width;
    }

    int get_rgb_height() {
        if (!s_color)
            return 0;

        return s_color->height;
    }

    bool get_rgb_image_RGBA_Byte(unsigned char *data, int width, int height, double *timestamp) {
        //auto t1 = std::chrono::system_clock::now();

        if (!s_color)
            return false;

        if (data == nullptr)
            return false;

        if (s_color->hostTimestamp <= *timestamp)
            return false;

        if (!s_colorMutex.try_lock())
            return false;

        std::shared_ptr<xv::ColorImage> tmp = s_color;
        unsigned srcWidth = tmp->width;
        unsigned srcHeight = tmp->height;
        s_colorMutex.unlock();

        std::vector<xv::Object> objects;

#ifdef MNN_ENABLE
        frameCount++;
        // if (frameCount % 2 == 0)
        {
            auto resulthands = detection.run_one(s_color);

            for (auto &resulthand : resulthands)
            {
                xv::Object obj_temp;
                obj_temp.typeID = resulthand.gesture_index;
                obj_temp.shape = xv::Object::Shape::HandSkeleton;
                for (int iii = 0; iii < 21; iii++)
                {
                    xv::Object::keypoint k_temp;
                    k_temp.x = resulthand.keypoints[iii].x / 640.0f;
                    k_temp.y = resulthand.keypoints[iii].y / 480.0f;
                    k_temp.z = resulthand.keypoints[iii].z;
                    obj_temp.keypoints.push_back(k_temp);
                }
                objects.push_back(obj_temp);
            }
        }

        //    objects = inference_mnn_one(tmp);
#else
        s_objMutex.lock();
        if (s_cnnSource == 2)
            objects = s_objects;
        s_objMutex.unlock();
#endif
        if (width <= 0) {
            width = srcWidth;
        }
        if (height <= 0) {
            height = srcHeight;
        }

#ifdef USE_FFMPEG

        const auto &yuv_image = tmp->data.get();

        auto pFrameYUV = av_frame_alloc();
        auto pFrameBGR = av_frame_alloc();

        av_image_fill_arrays(pFrameYUV->data, pFrameYUV->linesize, yuv_image, AV_PIX_FMT_YUV420P, srcWidth, srcHeight, 1);
        av_image_fill_arrays(pFrameBGR->data, pFrameBGR->linesize, data, AV_PIX_FMT_RGBA, width, height, 1);

        auto imgCtx = sws_getContext(srcWidth, srcHeight, AV_PIX_FMT_YUV420P, width, height, AV_PIX_FMT_RGBA, SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (imgCtx != nullptr)
        {
            sws_scale(imgCtx, pFrameYUV->data, pFrameYUV->linesize, 0, srcHeight, pFrameBGR->data, pFrameBGR->linesize);
        }

        sws_freeContext(imgCtx);
        imgCtx = nullptr;
        av_frame_free(&pFrameYUV);
        av_frame_free(&pFrameBGR);

        cv::Mat mrgb(height, width, CV_8UC4, data);
        cv::flip(mrgb, mrgb, 1);

#else
        if (!tmp->data.get()) {
            std::cerr << "Invalid RGB data!" << std::endl;
            return false;
        }

        if (tmp->codec == xv::ColorImage::Codec::YUV420p) {
//            std::cerr << "eddy YUV420p RGB data!" << std::endl;
            cv::Mat myuv(srcHeight * 3 / 2, srcWidth, CV_8UC1,
                         const_cast<unsigned char *>(tmp->data.get()));
            cv::Mat mrgb(height, width, CV_8UC4, data);

            if (width != srcWidth || height != srcHeight) {
                cv::Mat t;
                cv::cvtColor(myuv, t, cv::COLOR_YUV420p2BGRA);
                cv::resize(t, mrgb, mrgb.size());
            } else {
                cv::cvtColor(myuv, mrgb, cv::COLOR_YUV420p2BGRA);
            }
#endif
            double rw = static_cast<double>(width) / static_cast<double>(srcWidth);
            double rh = static_cast<double>(height) / static_cast<double>(srcHeight);

            for (unsigned int i = 0; i < objects.size(); i++) {
                const auto obj = objects.at(i);
                // *type = obj.typeID;

                if (obj.shape == xv::Object::Shape::BoundingBox && s_object_ui_show) {
                    //double rx = width - (rw * obj.x + rw * obj.width);
                    //double ry = height - (rh * obj.y + rh * obj.height);
                    double rx = rw * obj.x;
                    double ry = rh * obj.y;
                    const cv::Rect r(rx, ry, rw * obj.width, rh * obj.height);
                    cv::rectangle(mrgb, r, cv::Scalar(255, 255, 0));
                    std::stringstream stream;
                    stream << std::fixed << std::setprecision(2) << 100.0 * obj.confidence;
                    std::string str = obj.type + ":" + stream.str() + "%";
                    int baseline = 0;
                    cv::Size textSize = getTextSize(str, cv::FONT_HERSHEY_DUPLEX, 1.0, 1.0,
                                                    &baseline);
                    cv::putText(mrgb, str, (r.br() + r.tl()) * 0.5 -
                                           cv::Point(textSize.width, textSize.height) * 0.5,
                                cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 255, 0));
                } else if (obj.shape == xv::Object::Shape::HandSkeleton) {
#ifdef ANDROID
                    LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "eddy entry HandSkeleton");
#endif
                    static std::vector<std::vector<int>> list_connections = {{0,  1,  2,  3,  4},
                                                                             {0,  5,  6,  7,  8},
                                                                             {5,  9,  10, 11, 12},
                                                                             {9,  13, 14, 15, 16},
                                                                             {13, 17},
                                                                             {0,  17, 18, 19, 20}};

                    for (auto obj_xy : obj.keypoints) {
                        cv::circle(mrgb,
                                   cv::Point2f(obj_xy.x * static_cast<double>(width),
                                               obj_xy.y * static_cast<double>(height)),
                                   6,
                                   cv::Scalar(255, 0, 0),
                                   -1);
                    }
                    int npts[] = {5, 5, 5, 5, 2, 5};

                    cv::Point points[6][5];
                    for (int ip = 0; ip < list_connections.size(); ip++) {
                        for (int jp = 0; jp < list_connections[ip].size(); jp++) {
                            int lm_index = list_connections[ip][jp];

                            points[ip][jp] = cv::Point(
                                    int(obj.keypoints[lm_index].x * static_cast<double>(width)),
                                    int(obj.keypoints[lm_index].y * static_cast<double>(height)));
                        }
                    }
                    const cv::Point *pts[] = {points[0], points[1], points[2], points[3], points[4],
                                              points[5]};

                    cv::polylines(mrgb, pts, npts, 6, false, cv::Scalar(0, 255, 0), 2, cv::LINE_AA,
                                  0);
                }
            }

            cv::flip(mrgb, mrgb, 1);
        } else if (tmp->codec == xv::ColorImage::Codec::YUYV) {
            cv::Mat myuv(srcHeight, srcWidth, CV_8UC2,
                         const_cast<unsigned char *>(tmp->data.get()));
            cv::Mat mrgb(height, width, CV_8UC4, data);

            if (width != srcWidth || height != srcHeight) {
                cv::Mat t;
                cv::cvtColor(myuv, t, cv::COLOR_YUV2BGRA_YUYV);
                cv::resize(t, mrgb, mrgb.size());
            } else {
                cv::cvtColor(myuv, mrgb, cv::COLOR_YUV2BGRA_YUYV);
            }
            cv::flip(mrgb, mrgb, 1);
        }
        else {
            return false;
        }
        *timestamp = tmp->hostTimestamp;

        return true;
    }
void start_bdStream()
{
    if(device && device->gpsModule())
    {
        device->gpsModule()->start();
        s_gpsCallbackId = device->gpsModule()->registerCallback([](const std::vector<unsigned char>& gpsData) {
            for (int i = 0; i < 80; i++)
            {
                s_gpsData.gpsData[i] = gpsData[i];
            }
        });
    }
}
void stop_bdStream()
{
    if(device && device->gpsModule())
    {
        device->gpsModule()->unregisterCallback(s_gpsCallbackId);
        device->gpsModule()->stop();
    }
}
void get_gpsStream(GpsData* gps)
{
    *gps = s_gpsData;
}
void start_laserDistanceStream()
{
    if(device && device->gpsDistanceModule())
    {
        device->gpsDistanceModule()->start();
        s_gpsDistanceCallbackId = device->gpsDistanceModule()->registerCallback([](const xv::GPSDistanceData &gpsDistanceData) {
            s_gpsDistanceData = gpsDistanceData;
        });
    }
}
void stop_laserDistanceStream()
{
    if(device && device->gpsDistanceModule())
    {
        device->gpsDistanceModule()->unregisterCallback(s_gpsDistanceCallbackId);
        device->gpsDistanceModule()->stop();
    }
}
void get_laserDistanceStream(xv::GPSDistanceData* gpsDistance)
{
    *gpsDistance = s_gpsDistanceData;
}

#endif

#ifdef XV_JOYSTICK
static wireless_pose_callback g_wireless_pose_callback = nullptr;
static wireless_scan_callback g_wireless_scan_callback = nullptr;
static wireless_upload_callback g_wireless_upload_callback = nullptr;
static wireless_state_callback g_wireless_state_callback = nullptr;
static xv::WirelessControllerDataType g_wireless_type = xv::WirelessControllerDataType::LEFT;

void xv_wireless_start() {
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_start");
    if(device && device->wirelessController()) {
        device->wirelessController()->start();
    } else {
        LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_start FAIL");
    }
}

void xv_wireless_stop() {
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_stop");
    if(device && device->wirelessController()) {
        device->wirelessController()->stop();
    } else {
        LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_stop FAIL");
    }
}

void xv_wireless_register(wireless_pose_callback callback) {
    g_wireless_pose_callback = callback;
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_register");
    if(device && device->wirelessController()) {
        device->wirelessController()->registerWirelessControllerDataCallback([](const xv::WirelessControllerData& data) {
            LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_register %f", data.pose.confidence());
            ControllerPos pose;
            g_wireless_type = data.type;
            pose.type = (char)data.type;
            pose.position.x = data.pose.translation()[0];
            pose.position.y = data.pose.translation()[1];
            pose.position.z = data.pose.translation()[2];
            pose.quaternion.x = data.pose.quaternion()[0];
            pose.quaternion.y = data.pose.quaternion()[1];
            pose.quaternion.z = data.pose.quaternion()[2];
            pose.quaternion.w = data.pose.quaternion()[3];
            pose.confidence = data.pose.confidence();
            pose.keyTrigger = data.keyTrigger;
            pose.keySide = data.keySide;
            pose.rocker_x = data.rocker_x;
            pose.rocker_y = data.rocker_y;
            LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless",
                                "xv_wireless_register rockerx = %d,rocky = %d ,data.key = %d",pose.rocker_x, pose.rocker_y,data.key );
//            pose.key = data.key;
            if(data.key == 16){
                pose.keyA = 1;
                pose.keyB = 0;
            } else if(data.key == 32){
                pose.keyA = 0;
                pose.keyB = 1;
            } else if(data.key == 48){
                pose.keyA = 1;
                pose.keyB = 1;
            } else {
                pose.keyA = 0;
                pose.keyB = 0;
            }

            if (g_wireless_pose_callback != nullptr) {
                g_wireless_pose_callback(&pose);
            }
        });
    } else {
        LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_register FAIL");
    }
}
void xv_wireless_get_device_info(WirelessControllerDeviceInfo *device_info,int device_type) {
    if(device && device->wirelessController()) {
        xv::WirelessControllerDeviceInformation info;
        xv::WirelessControllerDataType type;
        if(device_type == 1) {
            type =xv::WirelessControllerDataType::LEFT;
        } else {
            type =xv::WirelessControllerDataType::RIGHT;
        }
        device->wirelessController()->getWirelessControllerDeviceInformation(info,(xv::WirelessControllerDataType)type);
        device_info->battery = info.battery;
        device_info->temp = info.temp;
    }

}
void xv_wireless_register_state(wireless_state_callback stateCallback) {
    g_wireless_state_callback = stateCallback;
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_register_state");
    if(device && device->wirelessController()) {
        device->wirelessController()->registerWirelessControllerStateCallback([](const xv::WirelessControllerState& data) {
            LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_register_state %d", data.state);
            if (g_wireless_state_callback != nullptr) {
                    g_wireless_state_callback(data.name.c_str(), data.mac.c_str(), data.state);
            }
        });
    } else {
        LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_register_state FAIL");
    }
}

void xv_wireless_scan(wireless_scan_callback callback) {
    g_wireless_scan_callback = callback;
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_scan");
    if(device && device->wirelessController()) {
        device->wirelessController()->scanBleDevices([](std::map<std::string,std::string> map) {
            LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_scan %d", map.size());
            for(auto& pair : map)
            {
                auto name = pair.first;
                auto mac = pair.second;
                if (g_wireless_scan_callback != nullptr) {
                    g_wireless_scan_callback(name.c_str(), mac.c_str());
                }
            }
        });
    } else {
        LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_scan FAIL");
    }
}

void xv_wireless_connect(const char* name, const char* mac) {
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_connect name:%s, mac:%s", name, mac);
    if(device && device->wirelessController()) {
        device->wirelessController()->connectBleDevice(std::string(name), std::string(mac));
    } else {
        LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_connect FAIL");
    }
}

void xv_wireless_disconnect(const char* name, const char* mac) {
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_disconnect name:%s, mac:%s", name, mac);
    if(device && device->wirelessController()) {
        device->wirelessController()->disconnectBleDevice(std::string(name), std::string(mac));
    } else {
        LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_disconnect FAIL");
    }
}

void xv_wireless_upload_map(const char* path, wireless_upload_callback callback) {
    g_wireless_upload_callback = callback;
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_upload_map path:%s", path);
    if(device && device->wirelessController()) {
        device->wirelessController()->uploadMap(std::string(path), g_wireless_type, [](bool ret) {
            LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_upload_map %d", ret);
        });
    } else {
        LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_upload_map FAIL");
    }
}

bool xv_wireless_pair(int type, const char* name, const char* mac) {
    bool ret = false;
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_pair type:%02x name:%s, mac:%s", type, name, mac);
    if(device && device->wirelessController()) {
        if (type == 0) {
            ret = device->wirelessController()->pairingLeftWirelessController(std::string(name), std::string(mac));
        } else if(type == 1) {
            ret = device->wirelessController()->pairingRightWirelessController(std::string(name), std::string(mac));
        }
    } else {
        LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_pair FAIL");
    }
    return ret;
}

bool xv_wireless_set_slam_type(int type, int mode) {
    bool ret = false;
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_set_slam_type type:%d mode:%d", type, mode);
    if(device && device->wirelessController()) {
        xv::WirelessControllerSlamType slamType = (xv::WirelessControllerSlamType) mode;
        if (type == 0) {
            ret = device->wirelessController()->changeSlamType(slamType, xv::WirelessControllerDataType::LEFT);
        } else if(type == 1) {
            ret = device->wirelessController()->changeSlamType(slamType, xv::WirelessControllerDataType::RIGHT);
        }
    } else {
        LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_set_slam_type FAIL");
    }
    return ret;
}

int xv_wireless_get_slam_type(int type) {
    xv::WirelessControllerSlamType mode = xv::WirelessControllerSlamType::VIO;
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_get_slam_type type:%d", type);
    if(device && device->wirelessController()) {
        if (type == 0) {
            mode = device->wirelessController()->getSlamType(xv::WirelessControllerDataType::LEFT);
        } else if(type == 1) {
            mode = device->wirelessController()->getSlamType(xv::WirelessControllerDataType::RIGHT);
        }
    } else {
        LOG_DEBUG(ANDROID_LOG_INFO, "xv-wireless", "xv_wireless_get_slam_type FAIL");
    }
    return mode;
}

#endif
#ifdef ZHIYUAN_CAMERAS

bool xslam_switch_fusionCameras_state(bool isOpen) {
    if(device){
        std::vector<unsigned char> write;
        std::vector<unsigned char> vecRead;
        bool ret;
        write.push_back(0xfb);
        write.push_back(0xa1);
        if(isOpen) {
            write.push_back(0x01);
        } else {
            write.push_back(0x00);
        }
        return device->hidWriteAndRead(write, vecRead);
    } else {
        return false;
    }
}
bool xslam_switch_fusionCamerasLR(int state) {
    if(device){
        std::vector<unsigned char> write;
        std::vector<unsigned char> vecRead;
        bool ret;
        write.push_back(0xfb);
        write.push_back(0xa2);
        if(state == 0) {
            write.push_back(0x00);
        } else {
            write.push_back(0x01);
        }
        return device->hidWriteAndRead(write, vecRead);
    } else {
        return false;
    }
}
bool xslam_color_img_to_rgba(int width, int height, cv::Mat &mrgb, std::shared_ptr<xv::ColorImage> color_img) {

    if(color_img == nullptr) {
        return false;
    }

    std::size_t w = color_img->width;
    std::size_t h = color_img->height;

    if (!color_img->data.get()) {
        return false;
    }

    if (color_img->codec == xv::ColorImage::Codec::YUV420p) {
        cv::Mat myuv(h * 3 / 2, w, CV_8UC1, const_cast<unsigned char *>(color_img->data.get()));
        if (width != w || height != h) {
            cv::Mat t;
            cv::cvtColor(myuv, t, cv::COLOR_YUV420p2BGRA);
            cv::resize(t, mrgb, mrgb.size());
        } else {
            cv::cvtColor(myuv, mrgb, cv::COLOR_YUV420p2BGRA);
        }
         cv::flip(mrgb, mrgb, 1);
    } else if (color_img->codec == xv::ColorImage::Codec::YUYV) {
        cv::Mat myuv(h, w, CV_8UC2, const_cast<unsigned char *>(color_img->data.get()));
        if (width != w || height != h) {
            cv::Mat t;
            cv::cvtColor(myuv, t, cv::COLOR_YUV2BGRA_YUYV);
            cv::resize(t, mrgb, mrgb.size());
        } else {
            cv::cvtColor(myuv, mrgb, cv::COLOR_YUV2BGRA_YUYV);
        }
         cv::flip(mrgb, mrgb, 1);
    } else {
        return false;
    }

    return true;
}
bool xslam_device_get_rgbfusitionCamera(int width, int height, unsigned char *rgbldata, unsigned char *rgbrdata) {
    bool ret1 = false;
    bool ret2 = false;

    if (rgbldata != nullptr && s_rgblFustionThermal != nullptr && s_rgblFustionThermalMutex.try_lock()) {
        cv::Mat mrgb1(height, width, CV_8UC4, rgbldata);
        ret1 = xslam_color_img_to_rgba(width, height, mrgb1, s_rgblFustionThermal);
        s_rgblFustionThermalMutex.unlock();
    }

    if (rgbrdata != nullptr && s_rgbrFustionThermal != nullptr && s_rgbrFustionThermalMutex.try_lock()) {
        cv::Mat mrgb2(height, width, CV_8UC4, rgbrdata);
        ret2 = xslam_color_img_to_rgba(width, height, mrgb2, s_rgbrFustionThermal);
        s_rgbrFustionThermalMutex.unlock();
    }

    return ret1 || ret2;
}
int xslam_start_RGB_L_ThermalFusionCamera() {
    auto deviceEX = std::dynamic_pointer_cast<xv::DeviceEx>(device);
    deviceEX->RGB_L_ThermalFusionCamera()->start();
    return deviceEX->RGB_L_ThermalFusionCamera()->registerCallback([](xv::ColorImage const &rgb) {
        s_rgblFustionThermalMutex.lock();
        s_rgblFustionThermal = std::make_shared<xv::ColorImage>(rgb);
        s_rgblFustionThermalMutex.unlock();
    });
}
void xslam_stop_RGB_L_thermalFusionCamera(int id) {
    auto deviceEX = std::dynamic_pointer_cast<xv::DeviceEx>(device);
    deviceEX->RGB_L_ThermalFusionCamera()->unregisterCallback(id);
    deviceEX->RGB_L_ThermalFusionCamera()->stop();
}
int xslam_start_RGB_R_ThermalFusionCamera() {
    auto deviceEX = std::dynamic_pointer_cast<xv::DeviceEx>(device);
    deviceEX->RGB_R_ThermalFusionCamera()->start();
    return deviceEX->RGB_R_ThermalFusionCamera()->registerCallback([](xv::ColorImage const &rgb) {
        s_rgbrFustionThermalMutex.lock();
        s_rgbrFustionThermal = std::make_shared<xv::ColorImage>(rgb);
        s_rgbrFustionThermalMutex.unlock();
    });
}
    void xslam_stop_RGB_R_thermalFusionCamera(int id) {
    auto deviceEX = std::dynamic_pointer_cast<xv::DeviceEx>(device);
    deviceEX->RGB_R_ThermalFusionCamera()->unregisterCallback(id);
    deviceEX->RGB_R_ThermalFusionCamera()->stop();
}

int xslam_start_colorCamera2() {
   int cbId =  device->colorCamera()->registerCam2Callback([](xv::ColorImage const &rgb) {
        s_color2Mutex.lock();
        s_color2 = std::make_shared<xv::ColorImage>(rgb);
        s_color2Mutex.unlock();
    });
    device->colorCamera()->startCameras();
    return cbId;
}
void xslam_stop_colorCamera2(int id) {
    device->colorCamera()->unregisterCam2Callback(id);
    device->colorCamera()->stopCameras();
}

void xslam_stop_thermalCamera(int id) {
    device->thermalCamera()->unregisterCallback(id);
    device->thermalCamera()->stop();
}
int xslam_start_thermalCamera() {
    int cbId =  device->thermalCamera()->registerCallback([](xv::ThermalImage const &image) {
        s_thermalMutex.lock();
        s_thermal = std::make_shared<xv::ThermalImage>(image);
        s_thermalMutex.unlock();
    });
    //test
    device->thermalCamera()->start();
    return cbId;
}
int xslam_start_irTrackingCamera() {
    int cbId =  device->irTrackingCamera()->registerCallback([](xv::IrTrackingImage const &image) {
        s_irTrackingMutex.lock();
        s_IrTrackingImage = std::make_shared<xv::IrTrackingImage>(image);
        s_irTrackingMutex.unlock();
    });
    //test
    device->irTrackingCamera()->start();
    return cbId;
}
void xslam_stop_irTrackingCamera(int id) {
    device->irTrackingCamera()->unregisterCallback(id);
    device->irTrackingCamera()->stop();
}
bool xslam_thermalImage_img_to_rgba(int width, int height,cv::Mat &mrgb, std::shared_ptr<xv::ThermalImage> color_img) {

    if(color_img == nullptr) {
        return false;
    }

    std::size_t w = color_img->width;
    std::size_t h = color_img->height;

    if (!color_img->data.get()) {
        return false;
    }

    if (color_img->codec == xv::ThermalImage::Codec::UYVY) {
        cv::Mat myuv(h, w, CV_8UC2, const_cast<unsigned char *>(color_img->data.get()));
    //    cv::Mat mrgb(height, width, CV_8UC4, data);

        if (width != w || height != h) {
            cv::Mat t;
            cv::cvtColor(myuv, t, cv::COLOR_YUV2BGRA_UYVY);
            cv::resize(t, mrgb, mrgb.size());
        } else {
            cv::cvtColor(myuv, mrgb, cv::COLOR_YUV2BGRA_UYVY);
        }
        // cv::flip(mrgb, mrgb, 1);
    } else {
        return false;
    }

    return true;
}
    bool xslam_irTrackingImage_img_to_rgba(int width, int height,cv::Mat &mrgb, std::shared_ptr<xv::IrTrackingImage> color_img) {

        if(color_img == nullptr) {
            return false;
        }

        std::size_t w = color_img->width;
        std::size_t h = color_img->height;

        if (!color_img->data.get()) {
            return false;
        }

        cv::Mat myuv(h, w, CV_8UC2, const_cast<unsigned char *>(color_img->data.get()));
        //    cv::Mat mrgb(height, width, CV_8UC4, data);

        if (width != w || height != h) {
            cv::Mat t;
            cv::cvtColor(myuv, t, cv::COLOR_YUV2BGRA_UYVY);
            cv::resize(t, mrgb, mrgb.size());
        } else {
            cv::cvtColor(myuv, mrgb, cv::COLOR_YUV2BGRA_UYVY);
        }
        return true;
    }
bool xslam_device_get_irTracking(int width, int height, unsigned char *data1) {
    bool ret1 = false;
    bool ret2 = false;

    if (data1 != nullptr && s_IrTrackingImage != nullptr && s_irTrackingMutex.try_lock()) {
        cv::Mat mrgb1(height, width, CV_8UC4, data1);
        ret1 = xslam_irTrackingImage_img_to_rgba(width, height, mrgb1, s_IrTrackingImage);
        s_irTrackingMutex.unlock();
    }
    return ret1 ;
}
bool xslam_device_get_thermal(int width, int height, unsigned char *data1) {
    bool ret1 = false;
    bool ret2 = false;

    if (data1 != nullptr && s_thermal != nullptr && s_thermalMutex.try_lock()) {
        cv::Mat mrgb1(height, width, CV_8UC4, data1);
        ret1 = xslam_thermalImage_img_to_rgba(width, height, mrgb1, s_thermal);
        s_thermalMutex.unlock();
    }


    return ret1 ;
}
bool xslam_device_get_rgba2(int width, int height,  unsigned char *data2) {
    bool ret1 = false;
    bool ret2 = false;

    /*if (data1 != nullptr && s_color != nullptr && s_colorMutex.try_lock()) {
        cv::Mat mrgb1(height, width, CV_8UC4, data1);
        ret1 = xslam_color_img_to_rgba(width, height, mrgb1, s_color);
        s_colorMutex.unlock();
    }*/

    if (data2 != nullptr && s_color2 != nullptr && s_color2Mutex.try_lock()) {
        cv::Mat mrgb2(height, width, CV_8UC4, data2);
        ret2 = xslam_color_img_to_rgba(width, height, mrgb2, s_color2);
        s_color2Mutex.unlock();
    }

    return  ret2;
}
#endif
#ifdef XV_MULTY_DEVICE_RGB
static std::shared_ptr<xv::Device> g_rgb_device = nullptr;
static std::mutex g_rgb_color_mutex1, g_rgb_color_mutex2;
static std::shared_ptr<xv::ColorImage> g_rgb_color_img1 = nullptr;
static std::shared_ptr<xv::ColorImage> g_rgb_color_img2 = nullptr;
void xv_rgb_device_init(int fd) {
    if (g_rgb_device != nullptr) {
        return;
    }

    LOG_DEBUG(ANDROID_LOG_INFO, "xv-rgb", "xv_rgb_device_init %d", fd);
    std::shared_ptr<xv::Device> d = xv::getDevice(fd);
    if (d && d->deviceType() == xv::DeviceType::DoubleRGB) {
        g_rgb_device = d;
        LOG_DEBUG(ANDROID_LOG_INFO, "xv-rgb", "xv_rgb_device_init");
        g_rgb_device->colorCamera()->registerCallback([](xv::ColorImage const &rgb) {
            LOG_DEBUG(ANDROID_LOG_INFO, "xv-rgb", "xv_rgb_device_init rgbCallback1 %dX%d", rgb.width, rgb.height);
            g_rgb_color_mutex1.lock();
            g_rgb_color_img1 = std::make_shared<xv::ColorImage>(rgb);
            g_rgb_color_mutex1.unlock();
        });

        g_rgb_device->colorCamera()->registerCam2Callback([](xv::ColorImage const &rgb) {
            LOG_DEBUG(ANDROID_LOG_INFO, "xv-rgb", "xv_rgb_device_init rgbCallback2 %dX%d", rgb.width, rgb.height);
            g_rgb_color_mutex2.lock();
            g_rgb_color_img2 = std::make_shared<xv::ColorImage>(rgb);
            g_rgb_color_mutex2.unlock();
        });

        g_rgb_device->colorCamera()->start();
        g_rgb_device->colorCamera()->startCameras();
    }
}

bool xv_rgb_device_start_camera() {
    if (!g_rgb_device || !g_rgb_device->colorCamera() || g_rgb_device->deviceType() != xv::DeviceType::DoubleRGB) {
        LOG_DEBUG(ANDROID_LOG_INFO, "xv-rgb", "xv_rgb_device_start_camera FAIL");
        return false;
    }

    LOG_DEBUG(ANDROID_LOG_INFO, "xv-rgb", "xv_rgb_device_start_camera");
    g_rgb_device->colorCamera()->registerCallback([](xv::ColorImage const &rgb) {
        g_rgb_color_mutex1.lock();
        g_rgb_color_img1 = std::make_shared<xv::ColorImage>(rgb);
        g_rgb_color_mutex1.unlock();
    });

    g_rgb_device->colorCamera()->registerCam2Callback([](xv::ColorImage const &rgb) {
        g_rgb_color_mutex2.lock();
        g_rgb_color_img2 = std::make_shared<xv::ColorImage>(rgb);
        g_rgb_color_mutex2.unlock();
    });

    g_rgb_device->colorCamera()->start();
    g_rgb_device->colorCamera()->startCameras();
    return true;
}

bool xv_rgb_device_stop_camera() {
    if (!g_rgb_device || !g_rgb_device->colorCamera() || g_rgb_device->deviceType() != xv::DeviceType::DoubleRGB) {
        LOG_DEBUG(ANDROID_LOG_INFO, "xv-rgb", "xv_rgb_device_stop_camera FAIL");
        return false;
    }
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-rgb", "xv_rgb_device_stop_camera");
    g_rgb_device->colorCamera()->stop();
    g_rgb_device->colorCamera()->stopCameras();
    return true;
}

bool xv_rgb_device_get_rgba(int width, int height, unsigned char *data1, unsigned char *data2) {
    bool ret1 = false;
    bool ret2 = false;

    if (data1 != nullptr && g_rgb_color_img1 != nullptr && g_rgb_color_mutex1.try_lock()) {
        cv::Mat mrgb1(height, width, CV_8UC4, data1);
        ret1 = xv_color_img_to_rgba(width, height, mrgb1, g_rgb_color_img1);
        g_rgb_color_mutex1.unlock();
    }

    if (data2 != nullptr && g_rgb_color_img2 != nullptr && g_rgb_color_mutex2.try_lock()) {
        cv::Mat mrgb2(height, width, CV_8UC4, data2);
        ret2 = xv_color_img_to_rgba(width, height, mrgb2, g_rgb_color_img2);
        g_rgb_color_mutex2.unlock();
    }

    return ret1 && ret2;
}

#endif
#ifdef XV_GAZE_CALIB_NEW
int xslam_gaze_calibration_begin(int et_idx) {
    int ret = calibration.pubCalibBegin(et_idx);
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-gaze", "xslam_gaze_calibration_begin status:%d", ret);
    return ret;
}
int xslam_gaze_set_conf_gaze(int et_idx, int s_idx, float *conf_gaze_o ) {
    int ret = calibration.pubSetConfGaze(et_idx,s_idx,conf_gaze_o);
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-gaze", "xslam_gaze_set_conf_gaze status:%d", ret);
    return ret;
}

int xslam_gaze_calibration_end(int et_idx) {
    int ret = calibration.pubCalibEnd(et_idx);
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-gaze", "xslam_gaze_calibration_end status:%d", ret);
    return ret;
}


#endif
#ifdef XV_GAZE_CALIB
static xv::GazeCalibrationData g_gaze_calib_data;
static xv::GazeCalibrationData* g_gaze_calib_data_p = &g_gaze_calib_data;
static xv::GazeCalibrationData g_gaze_init_data;

int xslam_gaze_calibration_enter() {
    xv::GazeStatus ret = calibration.CalibrationEnter();
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-gaze", "xslam_gaze_calibration_enter %d", ret);
    return ret;
}

int xslam_gaze_calibration_leave() {
    int result = 0;
    xv::GazeStatus ret = calibration.CalibrationLeave(&result);
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-gaze", "xslam_gaze_calibration_leave status:%d, result:%d", ret, result);
    return result;
}

int xslam_gaze_calibration_collect(float x, float y, float z, int index) {
    int result = 0;
    xv::GazeStatus ret = calibration.CalibrationCollect(x, y, z, index, &result);
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-gaze", "xslam_gaze_calibration_collect status:%d, result:%d", ret, result);
    return result;
}

int xslam_gaze_calibration_retrieve(const char* file) {
    xv::GazeStatus ret = calibration.CalibrationRetrieve(&g_gaze_calib_data_p);
    int result = g_gaze_calib_data_p->size;
    if (result > 0) {
        FILE* fp = fopen(file, "wb");
        if (fp != NULL) {
            fwrite(g_gaze_calib_data_p->data, 1, g_gaze_calib_data_p->size, fp);
            fclose(fp);
        }
    }
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-gaze", "xslam_gaze_calibration_retrieve status:%d, size:%d", ret, result);
    return ret;
}

int xslam_gaze_calibration_apply(const char* file) {
    FILE* fp = fopen(file, "rb");
    if (fp == NULL) {
        return -1;
    }

    if (fseek(fp, 0, SEEK_END) != 0) {
        fclose(fp);
        return -1;
    }

    g_gaze_init_data.size = ftell(fp);
    if (g_gaze_init_data.size <= 0) {
        fclose(fp);
        return -1;
    }

    if (fseek(fp, 0, SEEK_SET) != 0) {
        fclose(fp);
        return -1;
    }

    g_gaze_init_data.data = (char*)malloc((size_t)g_gaze_init_data.size);
    if (g_gaze_init_data.data == NULL) {
        fclose(fp);
        return -1;
    }

    int read_len = fread(g_gaze_init_data.data, sizeof(char), (size_t)g_gaze_init_data.size, fp);
    if ((size_t)read_len != g_gaze_init_data.size) {
        fclose(fp);
        return -1;
    }

    xv::GazeStatus ret = calibration.CalibrationApply(&g_gaze_init_data);
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-gaze", "xslam_gaze_calibration_apply status:%d, size:%d", ret, g_gaze_init_data.size);
    return ret;
}

int xslam_gaze_calibration_reset() {
    xv::GazeStatus ret = calibration.CalibrationReset();
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-gaze", "xslam_gaze_calibration_reset status:%d", ret);
    return ret;
}

int xslam_gaze_calibration_compute_apply() {
    xv::GazeStatus ret = calibration.CalibrationComputeApply();
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-gaze", "xslam_gaze_calibration_compute_apply status:%d", ret);
    return ret;
}

int xslam_gaze_calibration_setup() {
    xv::GazeStatus ret = calibration.CalibrationSetup();
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-gaze", "xslam_gaze_calibration_setup status:%d", ret);
    return ret;
}

int xslam_gaze_calibration_query_status(GazeCalibStatus* status) {
    xv::CalibrationStatus result;
    xv::GazeStatus ret = calibration.CalibrationQueryStatus(&result);
    if (status) {
        status->enter_status = result.enter_status;
        status->collect_status = result.collect_status;
        status->setup_status = result.setup_status;
        status->compute_apply_status = result.compute_apply_status;
        status->leave_status = result.leave_status;
        status->reset_status = result.reset_status;
    }
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-gaze", "xslam_gaze_calibration_query_status status:%d", ret);
    return ret;
}
#endif

#ifdef XV_IRIS
static int g_iris_enroll_id = -1;
static int g_iris_identity_id = -1;
static fn_iris_callback g_iris_register_callback = nullptr;
static fn_iris_identity_callback g_iris_identity_callback = nullptr;
static unsigned char* g_iris_int_data = nullptr;
static JavaVM* g_iris_jvm = nullptr;
static jobject g_iris_context = nullptr;
static std::string g_iris_init_licence = "";
#define IRIS_USER_ID "7d68ffa5t78cct4c8eta91at3b22b2196653"
#define IRIS_SECRET "B4HH/Zh68GQAhFZdLZ2SmjqlEqtJKYgjksnreB/KpdxZKvzY60OJn+IOe4mxfiL9V+SwUSi0EkR9gZfERiKm960RbWENiQb90AeqWLUKppZxNN6r4JPuSWYmd0hlq+0FD/n22IssE53yb3qKYzNqIRiqTrF+qvVdBBNKXwZtZfycV3MLphm41OSZuJT84oYQg5cdMyk0vhaJJSSZ58M9FxdjpLRscludJReAJgyTbcKiW17g/NlyMdqRfYUW8zW+KzDISM+LdzkT+FUq5hNg/88KLuRWwJzn4ISwOfmb83s="
static std::string g_iris_offlineS = "";
/**
 * @brief 初始化虹膜识别模块。
 *
 * 该函数初始化虹膜识别模块，获取 JVM 和上下文对象的引用，并存储初始许可证信息。
 *
 * @param env JNI 环境指针。
 * @param context Java 上下文对象。
 * @param initLicence 初始许可证字符串。
 */
void xv_iris_init(JNIEnv *env, jobject context, jstring initLicence) {
    env->GetJavaVM(&g_iris_jvm);
    g_iris_context = env->NewGlobalRef(context);
    const char* init = env->GetStringUTFChars(initLicence, 0);
    g_iris_init_licence = std::string(init);
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-iris", "init initLicence:%s", g_iris_init_licence.c_str());
}
/**
 * @brief 激活虹膜识别模块。
 *
 * 该函数调用设备的激活方法，将虹膜识别模块进行在线激活。
 *
 * @return 激活结果字符串。如果失败，返回空字符串。
 */
const char* xv_iris_active() {
    if (!device || !device->iris() || !g_iris_jvm) {
        return "";
    }
    JNIEnv *env = nullptr;
    g_iris_jvm->AttachCurrentThread(&env, NULL);
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-iris", "active env:%p, context:%p", env, g_iris_context);
    int activeResult = 0;
    const char* ret = device->iris()->onlineActive(env, g_iris_context, g_iris_init_licence.c_str(), IRIS_USER_ID, IRIS_SECRET, activeResult);
    std::string result = "";
    if (ret != nullptr) {
        result = std::string(ret);
    }
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-iris", "active licence:%s, result:%d", result.c_str(), activeResult);
    return result.c_str();
}
/**
 * @brief 设置离线许可证。
 *
 * 该函数存储提供的离线许可证信息。
 *
 * @param licence 离线许可证字符串。
 */
void xv_iris_init_licence(const char* licence) {
    g_iris_offlineS = std::string(licence);
}
/**
 * @brief 注册虹膜特征提取的回调。
 *
 * 该函数注册一个用户及其特征提取的回调，并启动虹膜识别模块。
 *
 * @param name 用户名。
 * @param callback 回调函数，用于处理提取到的虹膜特征数据。
 */
void xv_iris_register(const char* name, fn_iris_callback callback) {
    if (!device || !device->iris() || !g_iris_jvm) {
        return;
    }

    JNIEnv *env = nullptr;
    g_iris_jvm->AttachCurrentThread(&env, NULL);
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-iris", "register :%s", name);
    g_iris_register_callback = callback;
    device->iris()->start(env, g_iris_context, g_iris_offlineS);
    std::string s(name);
    device->iris()->setUserName(s);
    g_iris_enroll_id = device->iris()->registerEnrollCallback([](xv::XV_IRIS_DATA const &data) {
        if(g_iris_register_callback != nullptr) {
            std::string n = std::string(data.name.data());
            LOG_DEBUG(ANDROID_LOG_INFO, "xv-iris", "register callback name:%s, size:%d", n.c_str(), data.size);
            IrisFeature feature;
            if (data.size == IRIS_FEATURE_DATA_LEN) {
                feature.size = data.size;
                memcpy(&feature.data[0], data.feature.data(), data.size);
                g_iris_register_callback(n.c_str(), feature);
            }
        }
    });
}
/**
 * @brief 停止虹膜注册过程。
 *
 * 停止虹膜模块并取消注册回调。
 */
void xv_iris_stop() {
    if (!device || !device->iris()) {
        return;
    }
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-iris", "stop");
    if (g_iris_enroll_id != -1) {
        device->iris()->UnregisterEnrollCallback(g_iris_enroll_id);
    }
    g_iris_enroll_id = -1;
    device->iris()->stop();
}
/**
 * @brief 启动虹膜身份验证。
 *
 * 该函数加载虹膜数据并注册一个回调，用于处理身份验证的结果。
 *
 * @param data 虹膜特征数据。
 * @param size 数据大小。
 * @param callback 回调函数，用于处理身份验证结果。
 */
void xv_iris_start_identity(unsigned char *data, int size, fn_iris_identity_callback callback) {
    if (!device || !device->iris() || !g_iris_jvm) {
        return;
    }
    JNIEnv *env = nullptr;
    g_iris_jvm->AttachCurrentThread(&env, NULL);
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-iris", "start_identity size:%d", size);
    g_iris_int_data = (unsigned char*)malloc(size);
    memcpy(g_iris_int_data, data, size);
    g_iris_identity_callback = callback;
    device->iris()->start(env, g_iris_context, g_iris_offlineS);
    device->iris()->loadIrisInfo(g_iris_int_data, size / IRIS_FEATURE_DATA_LEN);
    g_iris_identity_id = device->iris()->registerIdentifyCallback([](xv::XV_IRIS_DATA const &iris_data) {
        if (g_iris_identity_callback != nullptr) {
             std::string n = "";
             if (iris_data.name.size() > 0) {
                 n = std::string(iris_data.name.data());
             }
             LOG_DEBUG(ANDROID_LOG_INFO, "xv-iris", "start_identity callback name:%s", n.c_str());
             g_iris_identity_callback(n.c_str());
        }
    });
}
/**
 * @brief 停止虹膜身份验证。
 *
 * 停止虹膜模块并取消身份验证的回调。
 */
void xv_iris_stop_identity() {
    if (!device || !device->iris()) {
        return;
    }

    LOG_DEBUG(ANDROID_LOG_INFO, "xv-iris", "stop_identity");
    if (g_iris_identity_id != -1) {
        device->iris()->UnregisterIdentifyCallback(g_iris_identity_id);
    }
    g_iris_identity_id = -1;
    device->iris()->stop();
}
/**
 * @brief 设置配置文件路径。
 *
 * 该函数为虹膜模块配置所需的路径信息。
 *
 * @param path 配置文件的路径字符串。
 */
void xv_iris_setConfigPath(const char* path) {
    if (!device || !device->iris()) {
        return;
    }
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-iris", "setConfigPath :%s", path);
    std::string s(path);
    device->iris()->setConfigPath(s);
}
#endif

static int g_eyetracking_id;
static std::mutex g_eyetracking_mutex;
static std::shared_ptr<xv::EyetrackingImage> g_eyetracking_img = nullptr;

bool xv_eyetracking_support() {
    return device && device->eyetracking();
}
/**
 * @brief 启动眼动追踪功能。
 *
 * 该函数检查设备的眼动追踪模块是否可用，并启动眼动追踪功能。如果启动成功，
 * 会注册回调函数，用于接收眼动追踪图像数据，并将数据存储到全局变量中。
 *
 * @return bool
 * - `true` 表示眼动追踪启动成功。
 * - `false` 表示启动失败（设备未初始化或眼动追踪模块不可用）。
 *
 */

bool xv_eyetracking_start() {
    if (!device || !device->eyetracking()) {
        return false;
    }

    bool ret = device->eyetracking()->start();
    g_eyetracking_id = device->eyetracking()->registerCallback([](xv::EyetrackingImage const &eyetracking_image) {
        g_eyetracking_mutex.lock();
        g_eyetracking_img = std::make_shared<xv::EyetrackingImage>(eyetracking_image);
        g_eyetracking_mutex.unlock();
    });
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-eyetracking", "start :%d", ret);
    return ret;
}

bool xv_eyetracking_stop() {
    if (!device || !device->eyetracking()) {
        return false;
    }

    bool ret = device->eyetracking()->stop();
    if (g_eyetracking_id != -1) {
        device->eyetracking()->unregisterCallback(g_eyetracking_id);
        g_eyetracking_id = -1;
    }
    LOG_DEBUG(ANDROID_LOG_INFO, "xv-eyetracking", "stop :%d", ret);
    return ret;
}
  /*  bool xv_eyetracking_get_rgba(unsigned char * left, unsigned char * right) {
        if (g_eyetracking_img == nullptr) {
            return false;
        }

        if (g_eyetracking_mutex.try_lock()) {
            int width = g_eyetracking_img->images[0].width;
            int height = g_eyetracking_img->images[0].height;
            LOG_DEBUG(ANDROID_LOG_INFO, "xv-eyetracking", "get_rgba width:%d, height:%d", width, height);
            if (left != nullptr) {
                cv::Mat mgray_l(height, width, CV_8UC1, const_cast<unsigned char *>(g_eyetracking_img->images[0].data.get()));
                cv::Mat mrgb_l(height, width, CV_8UC4, left);
                cv::cvtColor(mgray_l, mrgb_l, cv::COLOR_GRAY2BGRA);
            }

            if (right != nullptr) {
                cv::Mat mgray_r(height, width, CV_8UC1, const_cast<unsigned char *>(g_eyetracking_img->images[1].data.get()));
                cv::Mat mrgb_r(height, width, CV_8UC4, right);
                cv::cvtColor(mgray_r, mrgb_r, cv::COLOR_GRAY2BGRA);
            }

            g_eyetracking_mutex.unlock();
        }
        return true;
    }*/

bool xv_eyetracking_get_rgba(unsigned char * left, unsigned char * right,int *width_,int *height_) {
    if (g_eyetracking_img == nullptr) {
        return false;
    }

    if (g_eyetracking_mutex.try_lock()) {
        int width = g_eyetracking_img->images[0].width;
        int height = g_eyetracking_img->images[0].height;
        *width_=width;
        *height_=height;

     LOG_DEBUG(ANDROID_LOG_INFO, "xv-eyetracking", "get_rgba width:%d, height:%d", width, height);
        if (left != nullptr) {
            cv::Mat mgray_l(height, width, CV_8UC1, const_cast<unsigned char *>(g_eyetracking_img->images[0].data.get()));
            cv::Mat mrgb_l(height, width, CV_8UC4, left);
            cv::cvtColor(mgray_l, mrgb_l, cv::COLOR_GRAY2BGRA);
        }

        if (right != nullptr) {
            cv::Mat mgray_r(height, width, CV_8UC1, const_cast<unsigned char *>(g_eyetracking_img->images[1].data.get()));
            cv::Mat mrgb_r(height, width, CV_8UC4, right);
            cv::cvtColor(mgray_r, mrgb_r, cv::COLOR_GRAY2BGRA);
        }

        g_eyetracking_mutex.unlock();
    }
    return true;
}
#ifdef XV_STM

int STMDataId = -1;
static std::mutex g_stm_mutex;
static std::shared_ptr<xv::TerrestrialMagnetismData> g_stmData = nullptr;
static xv::TerrestrialMagnetismData s_stmData;
void STMDataCallback(const xv::TerrestrialMagnetismData &stmData)
{
    std::cout << "STM offset data: " << stmData.offset[0] << ", " << stmData.offset[1] << ", " << stmData.offset[2] << std::endl;
    std::cout << "STM angles data: " << stmData.angles[0] << ", " << stmData.angles[1] << ", " << stmData.angles[2] << std::endl;
    std::cout << "STM magnetic data: " << stmData.magnetic[0] << ", " << stmData.magnetic[1] << ", " << stmData.magnetic[2] << std::endl;
    std::cout << "STM level: " << stmData.level << std::endl;
    g_stm_mutex.lock();
    g_stmData = std::make_shared<xv::TerrestrialMagnetismData>(stmData);
    g_stm_mutex.unlock();
}
    void orientationCallback(xv::Orientation const& o)
    {

    }

bool xv_stm_start() {
    if(device->terrestrialMagnetismModule()) {

        device->orientationStream()->start();
        device->orientationStream()->registerCallback(orientationCallback);
        STMDataId = device->terrestrialMagnetismModule()->registerCallback(STMDataCallback);
        device->terrestrialMagnetismModule()->start();

    }
    return true;
}
 bool xv_stm_stop() {
     device->terrestrialMagnetismModule()->unregisterCallback(STMDataId);
     device->terrestrialMagnetismModule()->stop();
     return true;
 }
 bool xv_stm_get_stream(xv::TerrestrialMagnetismData* data) {

    if(!g_stmData)
        return false;
    if(data == nullptr)
        return false;
    if(!g_stm_mutex.try_lock())
        return false;

    std::shared_ptr<xv::TerrestrialMagnetismData>  tmp = g_stmData;

     g_stm_mutex.unlock();
     memcpy(data->angles,tmp->angles,sizeof(float )*3);
     memcpy(data->offset,tmp->offset,sizeof(float )*3);
     memcpy(data->magnetic,tmp->magnetic,sizeof(float )*3);

     data->level = tmp->level;
     LOG_DEBUG(ANDROID_LOG_INFO, "xv_stm_get_stream",
                         "data->angles :%f,data->offset :%f,data->magnetic :%f", data->angles[0],data->offset[0],data->magnetic[0]);
     return true;
 }
#endif

static const int COLOR_IMAGE_CACHE_SIZE = 5;
static std::shared_ptr<xv::ColorImage> g_color_img_list[COLOR_IMAGE_CACHE_SIZE];
static int g_color_img_index = 0;
static void xv_push_rgb_img(std::shared_ptr<xv::ColorImage> ptr) {
    int index = g_color_img_index % COLOR_IMAGE_CACHE_SIZE;
    g_color_img_list[index] = ptr;
    g_color_img_index++;
}

bool xv_get_sync_rgb(unsigned char *data, int width, int height, double ts, double *timestamp) {
    std::shared_ptr<xv::ColorImage> target = nullptr;
    double minDiff = 1.0f;
    for (int i=0; i < COLOR_IMAGE_CACHE_SIZE; i ++) {
        std::shared_ptr<xv::ColorImage> ptr = g_color_img_list[i];
        if (ptr == nullptr) {
            continue;
        }

        double diff = abs(ptr->hostTimestamp - ts);
        if (diff < minDiff) {
            minDiff = diff;
            target = ptr;
        }
    }

    if (target == nullptr) {
        return false;
    }

    if (width <= 0) {
        width = target->width;
    }
    if (height <= 0) {
        height = target->height;
    }

    if (!target->data.get()) {
        return false;
    }

    if (target->codec == xv::ColorImage::Codec::YUV420p) {
        cv::Mat myuv(target->height * 3 / 2, target->width, CV_8UC1,
                const_cast<unsigned char *>(target->data.get()));
        cv::Mat mrgb(height, width, CV_8UC4, data);

        if (width != target->width || height != target->height) {
            cv::Mat t;
            cv::cvtColor(myuv, t, cv::COLOR_YUV420p2BGRA);
            cv::resize(t, mrgb, mrgb.size());
        } else {
            cv::cvtColor(myuv, mrgb, cv::COLOR_YUV420p2BGRA);
        }

        cv::flip(mrgb, mrgb, 1);
    } else if (target->codec == xv::ColorImage::Codec::YUYV) {
        cv::Mat myuv(target->height, target->width, CV_8UC2,
                     const_cast<unsigned char *>(target->data.get()));
        cv::Mat mrgb(height, width, CV_8UC4, data);

        if (width != target->width || height != target->height) {
            cv::Mat t;
            cv::cvtColor(myuv, t, cv::COLOR_YUV2BGRA_YUYV);
            cv::resize(t, mrgb, mrgb.size());
        } else {
            cv::cvtColor(myuv, mrgb, cv::COLOR_YUV2BGRA_YUYV);
        }
        cv::flip(mrgb, mrgb, 1);
    } else {
        LOG_DEBUG(ANDROID_LOG_INFO, "xvxr", "Unsupport RGB codec: %d", static_cast<int>(target->codec));
        return false;
    }

    *timestamp = target->hostTimestamp;
    return true;
}

bool xv_slam_set_tags(const char *tagFamily, double size, TagArray *tagsArray, int arraySize) {
    if (!s_ready || !device || tagsArray == nullptr || arraySize == 0) {
        return false;
    }

    std::vector<int> tagIds;
    std::vector<xv::Transform> tagPoses;

    for (int i=0; i < arraySize;i ++) {
        DetectData& tag = tagsArray->detect[i];
        tagIds.push_back(tag.tagID);
        xv::Vector3d t = {tag.position.x,tag.position.y,tag.position.z};
        xv::Vector4d q = {tag.quaternion.x, tag.quaternion.y, tag.quaternion.z, tag.quaternion.w};
        xv::Transform pose = xv::Transform::Identity();
        pose.setTranslation(t);
        pose.setRotation(xv::quaternionToRotation(q));
        tagPoses.push_back(pose);
    }

    std::dynamic_pointer_cast<xv::SlamEx>(device->slam())->setTagsMap(tagFamily, size, tagIds, tagPoses);
    return true;
}

bool xv_slam_get_pose_in_tags(Vector3* position, Vector4* quaternion, double* confidence) {
    if (!s_ready || !device || position == nullptr || quaternion == nullptr) {
        return false;
    }

    xv::Pose p;
    if (std::static_pointer_cast<xv::SlamEx>(device->slam())->getPoseInTagsMap(p, 0.)) {
        position->x = p.x();
        position->y = p.y();
        position->z = p.z();
        auto q = xv::rotationToQuaternion(p.rotation());
        quaternion->x = q[0];
        quaternion->y = q[1];
        quaternion->z = q[2];
        quaternion->w = q[3];
        *confidence = p.confidence();
        return true;
    }
    return false;
}


} // namespace UnityWra