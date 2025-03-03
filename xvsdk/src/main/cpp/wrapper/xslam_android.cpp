#include <jni.h>
#include <memory>
#include <vector>
#include <time.h>
#include <string>
#include <regex>
#include <sstream>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <mutex>
#include <cmath>
#include "customer/test.h"
#include "xv-wrapper.h"
#include <android/log.h>
#include <thread>
#include <chrono>
#include <math.h>
#include <xv-sdk.h>
#include "xv-sdk-ex.h"
#include "unity-wrapper.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "fps_count.hpp"

#define LOG_TAG "xslam#wrapper"
#define LOG_DEBUG(...)                                                \
    do                                                                \
    {                                                                 \
        __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__); \
    } while (false)

#define LOG_ERROR(...)                                                \
    do                                                                \
    {                                                                 \
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__); \
    } while (false)

#define PI acos(-1)

enum Resolution {
    RGB_1920x1080 = 0,  ///< RGB 1080p
    RGB_1280x720 = 1,  ///< RGB 720p
    RGB_640x480 = 2,  ///< RGB 480p
    RGB_320x240 = 3,  ///< RGB QVGA (not supported now)
    RGB_2560x1920 = 4,  ///< RGB 5m (not supported now)
    RGB_3840x2160 = 5,
};

struct __attribute__((pack)) RgbaStruct {
    unsigned char R, G, B, A;
};

extern std::vector<std::vector<unsigned char>> rgb_colors;

static std::shared_ptr<xv::Device> device;
static int slamId = -1;
static int rgbId = -1;
static int tofId = -1;
static int stereoId = -1;
static int imuId = -1;
static int sgmbId = -1;

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

void yuv2rgb(unsigned char *yuyv_image, int *rgb_image, int width, int height);

static bool m_ready = false;

static JavaVM *jvm = 0;
static jclass s_XCameraClass = nullptr;

static jmethodID s_imuCallback = nullptr;
static jmethodID s_tofCallback = nullptr;
static jmethodID s_tofIrCallback = nullptr;

static jmethodID s_stereoCallback = nullptr;
static jmethodID s_sgbmCallback = nullptr;

static jmethodID s_rgbCallback = nullptr;
static jmethodID s_rgbFpsCallback = nullptr;

static jmethodID s_poseCallback = nullptr;
static jmethodID s_poseCallbackEx = nullptr;

static const xv::sgbm_config sgbm_config
        {
                1,
                1.f,
                0,
                1,
                0,
//  .11285f,
                0.08f,
                96.f,
                255,
                {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0},
                1,
                2.2,
                0,
                0,//1.standard 2.lrcheck 3.extended 4.subpixel
                8000,
                100,
        };

xv::RgbPixelPoseWithTof* g_rgb_pixel_pointing = nullptr;


void xv_stop_rgb_pixel_pose() {
    if (!device || !device->tofCamera()) {
        __android_log_print(ANDROID_LOG_INFO, "xvxr", "xv_stop_rgb_pixel_pose error tof camera");
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
void onImuCallback(xv::Imu const &imu) {
    JNIEnv *jniEnv;
    jvm->AttachCurrentThread(&jniEnv, NULL);
    if (!jniEnv || !s_XCameraClass || !s_imuCallback) {
        return;
    }

    static FpsCount fc;
    static int cnt = 0;

    fc.tic();
    if (cnt++ % 500 == 0) {
        LOG_DEBUG("onImuStream fps = %d", int(fc.fps()));
    }

    jniEnv->CallStaticVoidMethod(s_XCameraClass, s_imuCallback, static_cast<double>(imu.accel[0]),
                                 static_cast<double>(imu.accel[1]),
                                 static_cast<double>(imu.accel[2]));
}

void stopImuStream() {
    if (!device || !device->imuSensor()) {
        return;
    }

    if (imuId != -1) {
        device->imuSensor()->unregisterCallback(imuId);
    }
    device->imuSensor()->stop();
}

void startImuStream() {
    if (!device || !device->imuSensor()) {
        return;
    }

    imuId = device->imuSensor()->registerCallback(onImuCallback);
    device->imuSensor()->start();
}

void onSlamCallback(xv::Pose const &pose) {
    JNIEnv *jniEnv;
    jvm->AttachCurrentThread(&jniEnv, NULL);
    if (!s_XCameraClass || !jniEnv) {
        return;
    }

    if (s_poseCallback) {
        auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
        jniEnv->CallStaticVoidMethod(s_XCameraClass, s_poseCallback,
                                     static_cast<double>(pose.x()),
                                     static_cast<double>(pose.y()),
                                     static_cast<double>(pose.z()),
                                     static_cast<double>(pitchYawRoll[0]*180/M_PI),
                                     static_cast<double>(pitchYawRoll[1]*180/M_PI),
                                     static_cast<double>(pitchYawRoll[2]*180/M_PI));
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
static std::string tagDetectorId = "";
int xslam_start_rgb_detect_tags(const char *tagFamily, double size, TagArray *tagsArray, int arraySize)
{
    int tagsSize = 0;
    if (!device->colorCamera())
    {
        return 0;
    }

    if(tagDetectorId.empty())
    {
        tagDetectorId = std::dynamic_pointer_cast<xv::ColorCameraEx>(device->colorCamera())->startTagDetector(device->slam(), tagFamily, size, 50.);
#ifdef ANDROID
        LOG_DEBUG("xslam_start_rgb_detect_tags tagDetectorId:%s,tagfamily is %s", tagDetectorId.c_str(),tagFamily);
#endif
    }
    else
    {
        std::map<int, xv::Pose> detections = std::dynamic_pointer_cast<xv::ColorCameraEx>(device->colorCamera())->getTagDetections(tagDetectorId);
#ifdef ANDROID
        LOG_DEBUG("xslam_start_rgb_detect_tags detections:%d", detections.size());
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
                        device->colorCamera())->getCode(tagDetectorId,tag.tagID);
                std::strcpy(tag.qrcode,codeStr.c_str());
#endif
#ifdef ANDROID
                LOG_DEBUG("xslam_start_detect_tags tag:%d conf:%f, "
                          "(%f, %f, %f), (%f, %f, %f), (%f, %f, %f, %f) qrcode is %s",
                          tag.tagID, tag.confidence, tag.position.x, tag.position.y, tag.position.z,
                          tag.orientation.x, tag.orientation.y, tag.orientation.z, tag.quaternion.x,
                          tag.quaternion.y, tag.quaternion.z, tag.quaternion.w,tag.qrcode);
#endif
                index ++;
            }

            std::dynamic_pointer_cast<xv::ColorCameraEx>(device->colorCamera())->stopTagDetector(tagDetectorId);
            tagDetectorId = "";
        }
    }

    return tagsSize;
}
void stopSlamStream() {
    if (!device || !device->slam()) {
        return;
    }

    if (slamId != -1) {
        device->slam()->unregisterCallback(slamId);
    }
    device->slam()->stop();
}

void startSlamStream() {
    if (!device || !device->slam()) {
        return;
    }

    slamId = device->slam()->registerCallback(onSlamCallback);
    device->slam()->start(xv::Slam::Mode::Mixed);
}
pointer_3dpose pointer3Dpose[10];
Vector2 rgbPixelPoint[10] ;
int arraySize = 10;
int len = 10; // 假设的数组长度
Vector2 vector2s[10]; // 固定大小的数组
xv::ColorImage m_Rgb;
void onRgbCallback(xv::ColorImage const &rgb) {
    JNIEnv *jniEnv;
    LOG_DEBUG("onRgbCallback entry ");
    jvm->AttachCurrentThread(&jniEnv, NULL);
    if (!s_XCameraClass || !jniEnv) {
        return;
    }
    m_Rgb = rgb;
    if (s_rgbCallback) {
        int w = rgb.width;
        int h = rgb.height;
        int s = w * h;

        auto d = rgb.data.get();
        LOG_DEBUG("onRgbCallback hostTimestamp = %f,edgeTimestampUs = %f", rgb.hostTimestamp,rgb.edgeTimestampUs);
        jintArray data = jniEnv->NewIntArray(s);
        jint *body = jniEnv->GetIntArrayElements(data, 0);
//        TagArray tagsArray[2];
//        xslam_start_rgb_detect_tags("qr-code",0.12,tagsArray,64);
        yuv2rgb((unsigned char *) d, body, w, h);

        jniEnv->CallStaticVoidMethod(s_XCameraClass, s_rgbCallback, w, h, data);
        jniEnv->DeleteLocalRef(data);
    }

    if (s_rgbFpsCallback) {
        static FpsCount fc;
        static int cnt = 0;
        fc.tic();

        if (cnt++ % 100 == 0) {
            int fps = std::round(fc.fps());
            jniEnv->CallStaticVoidMethod(s_XCameraClass, s_rgbFpsCallback, fps);
        }
    }
}

int xslam_tof_set_steam_mode(int cmd) {
    bool bOk = false;
    __android_log_print(ANDROID_LOG_WARN, "xvxr", "xslam_tof_set_steam_mode = %d",cmd);
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
void stopRgbStream() {
    if (!device || !device->colorCamera()) {
        return;
    }

    if (rgbId != -1) {
        device->colorCamera()->unregisterCallback(rgbId);
    }
    device->colorCamera()->stop();
}
std::atomic<bool> running(true); // 用于控制线程的运行状态
void startTimer() {
    while (running) {
        if(m_Rgb.data){
            xslam_start_get_rgb_pixel_buff3d_pose(pointer3Dpose, vector2s,  10,m_Rgb.hostTimestamp, 30);
            LOG_DEBUG("xslam_start_get_rgb_pixel_buff3d_pose pointerPose.x = %f,%f,%f",pointer3Dpose[4].pointerPose.x,pointer3Dpose[4].pointerPose.y,pointer3Dpose[4].pointerPose.z);
        }


        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 暂停 20 毫秒
    }
}


extern "C"
JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_nTestFuncs(JNIEnv *env, jclass clazz) {
    // TODO: implement nTestFuncs()
    __android_log_print(ANDROID_LOG_WARN, "xv#wrapper",
                        "test func entry");
    XvWrapper::xv_save_ir_tracking_image();
}
void testXvWrapper(){
//    XvWrapper::setXvDevice(device);
    XvWrapper::initXvDevice();
    stereo_pdm_calibration fisheyes;
    int imuUs;
    XvWrapper::xv_start_irTrackingCamera2();
    XvWrapper::xv_set_ir_tracking_exposureTime(2000);
    XvWrapper::xv_read_irTracking_calibrationEx(true);
    XvWrapper::xv_read_irTracking_calibrationEx(false);
    XvWrapper::xv_get_irTrackingCamera_params();
    XvWrapper::xv_register_device_status_callback();
    XvWrapper::xv_getCPUTemperature();
    XvWrapper::xv_rgb_set_exposure(1,20,300);
//    XvWrapper::xvReadStereoFisheyesCalibration();
   // XvWrapper::startRgbStream();
//    XvWrapper::xv_start_skeleton_ex_with_cb();
   // XvWrapper::start_et_gaze_callback();
//    XvWrapper::stm_start();
    // 启动定时线程
  //  std::thread timerThread(startTimer);

    // 主线程保持运行，等待信号
 //   std::cout << "Timer thread is running. Press Ctrl+C to stop." << std::endl;
  //  timerThread.join(); // 等待线程结束
 //   XvWrapper::xv_start_RGB_L_ThermalFusionCamera();
 //   int clid =   XvWrapper::xv_start_RGB_R_ThermalFusionCamera();

    //  XvWrapper::xv_stop_RGB_L_thermalFusionCamera(clid);
 //
}

void onTofCallback(xv::DepthImage const &im) {
    JNIEnv *jniEnv;
    jvm->AttachCurrentThread(&jniEnv, NULL);
    if (!jniEnv || !s_XCameraClass) {
        return;
    }

    std::shared_ptr<const xv::DepthImage> tmp = std::make_shared<xv::DepthImage>(im);;
    unsigned w = tmp->width;
    unsigned h = tmp->height;
    double distance = 4.5;
    LOG_DEBUG("onTofStream type: %d, width = %d,height = %d", im.type, w, h);

    int s = w * h;
    auto d = const_cast<unsigned char *>(tmp->data.get());

    std::vector<RgbaStruct> rgbVectors;
    rgbVectors.resize(s);

    if (tmp->type == xv::DepthImage::Type::Depth_16) {
        float dmax = 2494.0; // maybe 7494,2494,1498,1249 see mode_manage.h in sony toflib
        const auto tmp_d = reinterpret_cast<int16_t const *>(tmp->data.get());
        for (unsigned int i = 0; i < tmp->height * tmp->width; i++) {
            unsigned short d = tmp_d[i];

            auto max = std::min(255.0f, d * 255.0f / dmax);
            unsigned int u = static_cast<unsigned int>( std::max(0.0f, max));

            const auto &cc = rgb_colors.at(u);
            rgbVectors[i] = RgbaStruct{cc.at(0), cc.at(1), cc.at(2), 255};
        }
    }else if (tmp->type == xv::DepthImage::Type::Cloud) {
        std::shared_ptr<xv::PointCloud> pointCould = device->tofCamera()->formatPmdCloudToPointCloudStruct(im);
        if(pointCould){
            //测试第一个像素点的相机坐标系下 xyz
            float x =  pointCould->points[100][0];
            float y =  pointCould->points[100][1];
            float z =  pointCould->points[100][1];
#ifdef ANDROID
            __android_log_print(ANDROID_LOG_WARN, "zhiyuan",
                                "size is %d,pointCould[0].x = %f",pointCould->points.size(), x );
#endif
        }
    } else if (tmp->type == xv::DepthImage::Type::IR) {
        // float dmax = 2494.0; // maybe 7494,2494,1498,1249 see mode_manage.h in sony toflib
        auto tmp_d = reinterpret_cast<unsigned short const *>(tmp->data.get());
        unsigned short dmin = tmp_d[0];
        unsigned short dmax = tmp_d[0];
        for (unsigned int i = 0; i < tmp->height * tmp->width; i++) {
            unsigned short d = tmp_d[i];
            if (d > dmax) {
                dmax = d;
            }

            if (d < dmin) {
                dmin = d;
            }
        }

        double dFactor = 255.0 / (double) (dmax - dmin);
        LOG_DEBUG("onTofCallback IR dmin=%d, dmax=%d, dFactor1=%f", dmin, dmax, dFactor);
        double gamma = 2.8;
        for (unsigned int i = 0; i < tmp->height * tmp->width; i++) {
            unsigned short d = tmp_d[i];
            // int dv = (int) ((d - dmin) * dFactor);
            int dv = (int) (255.0 * pow((1.0 * d) / (dmax - dmin), 1 / gamma));
            unsigned char pixel = (unsigned char) dv;

            rgbVectors[i] = RgbaStruct{pixel, pixel, pixel, 255};
        }

    } else if (tmp->type == xv::DepthImage::Type::Depth_32) {
        float dmax = 7.5;
        const auto tmp_d = reinterpret_cast<float const *>(tmp->data.get());
        for (unsigned int i = 0; i < s; i++) {
            const auto &d = tmp_d[i];
            if (d < 0.01 || d > 9.9) {
                rgbVectors[i] = RgbaStruct{0, 0, 0, 255};
            } else {
                auto max = std::min(255.0f, d * 255.0f / dmax);
                unsigned int u = static_cast<unsigned int>(std::max(0.0f, max));
                const auto &cc = rgb_colors.at(u);
                rgbVectors[i] = RgbaStruct{cc.at(0), cc.at(1), cc.at(2), 255};
                //   LOG_DEBUG("onTofStream d = %f,u = %d", d,u);
            }

        }
    }

    jintArray data = jniEnv->NewIntArray(s);
    jint *body = jniEnv->GetIntArrayElements(data, 0);
    memcpy(body, rgbVectors.data(), s * sizeof(RgbaStruct));

    if (tmp->type == xv::DepthImage::Type::IR && s_tofIrCallback) {
        jniEnv->CallStaticVoidMethod(s_XCameraClass, s_tofIrCallback, w, h, data);
    } else if (tmp->type == xv::DepthImage::Type::Depth_16 && s_tofCallback) {
        jniEnv->CallStaticVoidMethod(s_XCameraClass, s_tofCallback, w, h, data);
    } else if (tmp->type == xv::DepthImage::Type::Depth_32 && s_tofCallback) {
        jniEnv->CallStaticVoidMethod(s_XCameraClass, s_tofCallback, w, h, data);
    }

    jniEnv->DeleteLocalRef(data);
}
void xv_start_rgb_pixel_pose() {
    if (!device || !device->tofCamera()) {
        __android_log_print(ANDROID_LOG_INFO, "xvxr", "xv_start_rgb_pixel_pose error tof camera");
        return;
    }
    tofId = device->tofCamera()->registerCallback(onTofCallback);
    device->tofCamera()->setStreamMode(xv::TofCamera::StreamMode::DepthOnly);
    device->tofCamera()->start();

    if (g_rgb_pixel_pointing == nullptr) {
        g_rgb_pixel_pointing = new xv::RgbPixelPoseWithTof(device);
    }
}
int getRandomValue(int min, int max) {
    return rand() % (max - min + 1) + min; // 生成 [min, max] 范围的随机数
}
void startRgbStream() {
   // XvWrapper::xv_switch_audio(true);
    LOG_DEBUG("startRgbStream");
    if (!device || !device->colorCamera()) {
        return;
    }

    stopRgbStream();
    rgbId = device->colorCamera()->registerCallback(onRgbCallback);
    device->colorCamera()->start();

    std::srand(static_cast<unsigned int>(std::time(0))); // 设置随机种子


    double imuUs = 0;
    XvWrapper::xvReadStereoFisheyesCalibration();

    for (int i = 0; i < len; i++) {
        vector2s[i].x = getRandomValue(480.0f, 1440.0f);
        vector2s[i].y = getRandomValue(270.0f, 810.0f);
    }
    xv_start_rgb_pixel_pose();
    xslam_tof_set_steam_mode(4);
}


void stopTofStream() {
    if (!device || !device->tofCamera()) {
        return;
    }

    if (tofId != -1) {
        device->tofCamera()->unregisterCallback(tofId);
    }
    device->tofCamera()->stop();
}

bool setPmdTofIRFunction()
{
    bool  ret = false;
    std::vector<unsigned char> result(63);
    bool bOK = device->hidWriteAndRead({0x02,0x10,0xf5,0x02,0x01}, result);
    if(bOK)
    {
        std::cout << "Enable IR successfully" << std::endl;
        ret = true;
    }
    else
    {
        std::cout << "Enable IR failed" << std::endl;
        ret = false;
    }
    return ret;
}

void start_tofir_stream(){


    device->tofCamera()->setStreamMode(xv::TofCamera::StreamMode::DepthAndCloud);

    xv::TofCamera::Manufacturer manufacturer = device->tofCamera()->getManufacturer();
    if(manufacturer == xv::TofCamera::Manufacturer::Pmd)
    {
        bool bOK = device->tofCamera()->enableTofIr(true);
        if(bOK)
            std::cout << "Enable IR successfully" << std::endl;
        else
            std::cout << "Enable IR failed" << std::endl;
    }
    device->tofCamera()->registerCallback(onTofCallback);
    device->tofCamera()->start();
}

void startTofStream() {
    LOG_DEBUG("startTofStream");
    if (!device || !device->tofCamera()) {
        return;
    }
    setPmdTofIRFunction();
    tofId = device->tofCamera()->registerCallback(onTofCallback);
    device->tofCamera()->setSonyTofSetting(xv::TofCamera::SonyTofLibMode::LABELIZE_SF,
                                           xv::TofCamera::Resolution::VGA,
                                           xv::TofCamera::Framerate::FPS_30);
    device->tofCamera()->start();


}

void onStrereoCallback(xv::FisheyeImages const &stereo) {
    JNIEnv *jniEnv;
    jvm->AttachCurrentThread(&jniEnv, NULL);
    if (!s_XCameraClass || !jniEnv) {
        return;
    }

    jvm->AttachCurrentThread(&jniEnv, NULL);

    if (s_stereoCallback) {
        int w = stereo.images[0].width;
        int h = stereo.images[0].height;
        int s = w * h;

        auto d = stereo.images[0].data.get();
        LOG_DEBUG("onStrereoCallback hostTimestamp = %f,edgeTimestampUs = %f", stereo.hostTimestamp,stereo.edgeTimestampUs);
        if (stereo.images.empty() || !stereo.images[0].data || d == nullptr) {
            LOG_DEBUG("onStrereoCallback no fisheyes avaiable");
            return;
        }

        LOG_DEBUG("onStrereoCallback w = %d, h=%d", w, h);
        jintArray data = jniEnv->NewIntArray(s);
        jint *body = jniEnv->GetIntArrayElements(data, 0);
        for (int i = 0; i < w; i++) {
            for (int j = 0; j < h; j++) {
                auto v = d[i + j * w];
                body[i + j * w] =
                        0xFF000000 + (v << 16 & 0xFF0000) + (v << 8 & 0xFF00) + (v & 0xFF);
            }
        }
        jniEnv->CallStaticVoidMethod(s_XCameraClass, s_stereoCallback, w, h, data);
        jniEnv->DeleteLocalRef(data);
    }
}

void stopStereoStream() {
    if (!device || !device->fisheyeCameras()) {
        return;
    }

    if (stereoId != -1) {
        device->fisheyeCameras()->unregisterCallback(stereoId);
    }
    device->fisheyeCameras()->stop();
}

void startStereoStream() {
    LOG_DEBUG("startStereoStream");
    if (!device || !device->fisheyeCameras()) {
        return;
    }

    stopStereoStream();
    stereoId = device->fisheyeCameras()->registerCallback(onStrereoCallback);
    device->fisheyeCameras()->start();
}

void onSgbmCallback(xv::SgbmImage const &sgbm_image) {
    JNIEnv *jniEnv;
    jvm->AttachCurrentThread(&jniEnv, NULL);
    if (!s_XCameraClass || !jniEnv) {
        return;
    }

    jvm->AttachCurrentThread(&jniEnv, NULL);

    if (s_sgbmCallback) {
        if (sgbm_image.type == xv::SgbmImage::Type::Depth) {
            int w = sgbm_image.width;
            int h = sgbm_image.height;
            int s = w * h;

            LOG_DEBUG("onSgbmCallback w = %d, h=%d", w, h);
            std::vector<RgbaStruct> rgbVectors;
            rgbVectors.resize(s);

            double focal_length =
                    sgbm_image.width / (2.f * tan(sgbm_config.fov / 2 / 180.f * M_PI));
            double max_distance_m = (focal_length * sgbm_config.baseline / 1);
            double min_distance_m = sgbm_config.min_distance / 1000.0;
            max_distance_m = std::min(max_distance_m, sgbm_config.max_distance / 1000.0);

            float dmax = max_distance_m;
            const auto tmp_d = reinterpret_cast<int16_t const *>(sgbm_image.data.get());
            for (unsigned int i = 0; i < h * w; i++) {
                unsigned short d = tmp_d[i];
                auto max = std::min(255.0f, d * 255.0f / dmax);
                unsigned int u = static_cast<unsigned int>( std::max(0.0f, max));
                const auto &cc = rgb_colors.at(u);
                rgbVectors[i] = RgbaStruct{cc.at(0), cc.at(1), cc.at(2), 255};
            }

            jintArray data = jniEnv->NewIntArray(s);
            jint *body = jniEnv->GetIntArrayElements(data, 0);
            memcpy(body, rgbVectors.data(), s * sizeof(RgbaStruct));

            jniEnv->CallStaticVoidMethod(s_XCameraClass, s_sgbmCallback, w, h, data);
            jniEnv->DeleteLocalRef(data);
        } else if (sgbm_image.type == xv::SgbmImage::Type::PointCloud) {

        }
    }
}

void stopSgbmStream() {
    if (!device || !device->sgbmCamera()) {
        return;
    }

    if (sgmbId != -1) {
        device->sgbmCamera()->unregisterCallback(sgmbId);
    }
    device->sgbmCamera()->stop();
}

void startSgbmStream() {
    LOG_DEBUG("startSgbmStream");
    if (!device || !device->sgbmCamera()) {
        return;
    }

    stopSgbmStream();
    device->sgbmCamera()->setSgbmResolution(xv::SgbmCamera::Resolution::SGBM_640x480);
    sgmbId = device->sgbmCamera()->registerCallback(onSgbmCallback);
    device->sgbmCamera()->start(sgbm_config);
}



extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_nAddUsbDevice(JNIEnv
                                            *env,
                                            jclass type, jstring
                                            deviceName_,
                                            jint fileDescriptor
) {
    static bool firstCall = true;
    if (firstCall) {
        firstCall = false;
        std::cout.rdbuf(new androidout);
        std::cerr.rdbuf(new androiderr);
        env->GetJavaVM(&jvm);
        std::cout << "Initialized" << std::endl;
    }

    int fd = fileDescriptor;
    LOG_DEBUG("nAddUsbDevice fd: %d", fd);
// 增加确认值是否一致
    std::cout << "Initial fd before xslam_init_with_fd: " << fd << std::endl;
    UnityWrapper::xslam_init_with_fd(fd);
    device = UnityWrapper::xslam_get_device();
   // device = xv::getDevice(fd);
    xv::setLogLevel(xv::LogLevel(0));
    if (!device) {
        LOG_DEBUG("nAddUsbDevice getDevice FAIL");
        return;
    }

    m_ready = true;
    usleep(2000 * 1000);
    if (device) {
        LOG_DEBUG("eddy nAddUsbDevice device version: %s", device->info().at("version").c_str());
        LOG_DEBUG("eddy nAddUsbDevice sdk version: %s", xv::Version().toString().c_str());
        testXvWrapper();
    }
    LOG_DEBUG("nAddUsbDevice inited");
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_nSetSlamMode(JNIEnv *env, jclass type,
                                           jint mode) {
    if (!device || !device->slam()) {
        return;
    }

    LOG_DEBUG("switch to mode %d", mode);
    device->slam()->stop();
    if (mode == 0) {
        device->slam()->start(xv::Slam::Mode::Mixed);
    } else if (mode == 1) {
        device->slam()->start(xv::Slam::Mode::Edge);
    } else if (mode == 2) {
        device->slam()->start(xv::Slam::Mode::EdgeFusionOnHost);
    }
}


extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_nSetRgbSolution(JNIEnv *env, jclass type, jint mode) {
    if (!device || !device->colorCamera()) {
        return;
    }

    LOG_DEBUG("nSetRgbSolution %d", mode);
    switch (mode) {
        case RGB_1920x1080:
            device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_1920x1080);
            break;
        case RGB_1280x720:
            device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_1280x720);
            break;
        case RGB_640x480:
            device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_640x480);
            break;
        case RGB_320x240:
            device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_320x240);
            break;
        case RGB_2560x1920:
            device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_2560x1920);
            break;
        default:
            break;
    }
    LOG_DEBUG("nSetRgbSolution end");
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_nRemoveUsbDevice(JNIEnv *env, jclass type,
                                               jint fileDescriptor) {

}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_initCallbacks(JNIEnv *env, jclass type) {
    s_XCameraClass = reinterpret_cast<jclass>(env->NewGlobalRef(type));
    s_imuCallback = env->GetStaticMethodID(s_XCameraClass, "imuCallback", "(DDD)V");
    s_tofCallback = env->GetStaticMethodID(s_XCameraClass, "tofCallback", "(II[I)V");
    s_tofIrCallback = env->GetStaticMethodID(s_XCameraClass, "tofIrCallback", "(II[I)V");
    s_stereoCallback = env->GetStaticMethodID(s_XCameraClass, "stereoCallback", "(II[I)V");
    s_sgbmCallback = env->GetStaticMethodID(s_XCameraClass, "sgbmCallback", "(II[I)V");
    s_rgbCallback = env->GetStaticMethodID(s_XCameraClass, "rgbCallback", "(II[I)V");
    s_rgbFpsCallback = env->GetStaticMethodID(s_XCameraClass, "rgbFpsCallback", "(I)V");
    s_poseCallback = env->GetStaticMethodID(s_XCameraClass, "poseCallback", "(DDDDDD)V");
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_startSlamStream(JNIEnv *env, jclass type) {
    startSlamStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_stopSlamStream(JNIEnv *env, jclass type) {
    stopSlamStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_startImuStream(JNIEnv *env, jclass type) {
    startImuStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_stopImuStream(JNIEnv *env, jclass type) {
    stopImuStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_startRgbStream(JNIEnv *env, jclass type) {
    startRgbStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_stopRgbStream(JNIEnv *env, jclass type) {
    stopRgbStream();
}


extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_startTofStream(JNIEnv *env, jclass type) {
  //  startTofStream();
    start_tofir_stream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_stopTofStream(JNIEnv *env, jclass type) {
    stopTofStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_startStereoStream(JNIEnv *env, jclass type) {
    startStereoStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_stopStereoStream(JNIEnv *env, jclass type) {
    stopStereoStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_startSgbmStream(JNIEnv *env, jclass type) {
    startSgbmStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_stopSgbmStream(JNIEnv *env, jclass type) {
    stopSgbmStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_stopCallbacks(JNIEnv *env, jclass type) {
    stopSlamStream();
    stopImuStream();
    stopRgbStream();
    stopTofStream();
    stopStereoStream();
    stopSgbmStream();
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_org_xvisio_xvsdk_XCamera_getPose(JNIEnv *env, jclass type) {
    const int size = 6;
    jdoubleArray result = env->NewDoubleArray(size);
    if(device && device->slam()) {
        xv::Pose pose;
        if(device->slam()->getPose(pose)) {
            jdouble array[size];
            array[0] = pose.x();
            array[1] = pose.y();
            array[2] = pose.z();
            auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
            array[3] = pitchYawRoll[0]*180/M_PI;
            array[4] = pitchYawRoll[1]*180/M_PI;
            array[5] = pitchYawRoll[2]*180/M_PI;
            env->SetDoubleArrayRegion(result, 0, size, array);
        }
    }
    return result;
}

