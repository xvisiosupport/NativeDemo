
#include <memory>
#include <vector>
#include <time.h>
#include <string>
#include <regex>
#include <sstream>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <cmath>
#include <android/log.h>
#include <thread>
#include <chrono>
#include <math.h>
#include <xv-sdk.h>
#include "xv-sdk-ex.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <cstdint>
#include <algorithm>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "fps_count.hpp"
#include "xv-wrapper.h"
#include <opencv2/opencv.hpp>
#include <dirent.h>

#define LOG_TAG "xv#wrapper"
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
using namespace UnityWrapper;

static int rgbId = -1;
static int rgb2Id = -1;
static int thermalId = -1;
static int irTrackingId = -1;
static int irTrackingId2 = -1;
static int tofId = -1;
static int rgbLThermalFusionCallbackID = -1;
static int rgbRThermalFusionCallbackID = -1;
static std::shared_ptr<xv::Device> device;
static int s_gesturePlatform = UnityWrapper::PLATFORM::ANDROID_NPU;
//为true 是第一人称 false 是第三人称
static bool s_gestureEgo = true;
static fn_gaze_callback callbackGaze;
static int gazeId = -1;
static JavaVM *jvm = 0;
static jclass s_XvInterfaceClass = nullptr;
static jmethodID s_startRecognizerFromPath = nullptr;
static jmethodID s_startRecognizerFromByte = nullptr;
static jmethodID s_switchRecognizeSource = nullptr;
static jmethodID s_getVolumeCurrMethod = nullptr;
static jmethodID s_getVolumeMaxMethod = nullptr;
static jmethodID s_adjustVolumeMethod = nullptr;
static jmethodID s_playAudioMethod = nullptr;
static jmethodID s_stopAudioMethod = nullptr;

int STMDataId = -1;
static std::mutex g_stm_mutex;
static std::shared_ptr<xv::TerrestrialMagnetismData> g_stmData = nullptr;
static xv::TerrestrialMagnetismData s_stmData;
static std::shared_ptr<xv::Imu> s_imu;
static std::mutex s_tofIRImageMtx;
static std::shared_ptr<const xv::GrayScaleImage> s_ir;
// 用于存储上次回调时间和帧计数
static std::atomic<int> frame_count(0);
static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
namespace XvWrapper {
    bool xv_test() {
        LOG_DEBUG("xv_test start");
        return true;
    }

    //获取xvisio眼镜设备指针
    void initXvDevice() {

        device = xslam_get_device();
    }

    void setXvDevice(std::shared_ptr<xv::Device> dev) {
        device = dev;
    }

    //test
    bool xv_test_get_6dof(double *poseData, long long *timestamp, double predictionTime) {
        //XSlam::pose_ptr pose;

        xv::Pose pose;

        if (xslam_get_pose_prediction(poseData, timestamp, predictionTime) == false) {
            LOG_DEBUG("...%s...,getpose failed111111", __FUNCTION__);
            return false;
        } else {
            LOG_DEBUG("...%s...,getpose success", __FUNCTION__);
        }


        return true;
    }

    /**
  * @brief 地磁数据 callback
  */
    void STMDataCallback(const xv::TerrestrialMagnetismData &stmData) {
        g_stm_mutex.lock();
        g_stmData = std::make_shared<xv::TerrestrialMagnetismData>(stmData);
        __android_log_print(ANDROID_LOG_INFO, "xv#wrapper",
                            "data->angles :%f,data->offset :%f,data->magnetic :%f",
                            g_stmData->angles[0],
                            g_stmData->offset[0], g_stmData->magnetic[0]);
        g_stm_mutex.unlock();
    }
    void orientationCallback(xv::Orientation const& o)
    {

    }
    /**
    * @brief 开启地磁数据获取
    */
    bool stm_start() {
        if (device && device->terrestrialMagnetismModule()) {
            device->orientationStream()->start();
            device->orientationStream()->registerCallback(orientationCallback);
            STMDataId = device->terrestrialMagnetismModule()->registerCallback(STMDataCallback);
            device->terrestrialMagnetismModule()->start();
            __android_log_print(ANDROID_LOG_INFO, "xv#wrapper",
                                "stm_start %d", STMDataId);
            return true;
        }
        return false;
    }

    /**
    * @brief 停止地磁数据获取
    */
    bool stm_stop() {
        device->terrestrialMagnetismModule()->unregisterCallback(STMDataId);
        return device->terrestrialMagnetismModule()->stop();

    }

/**
 * @brief 读取显示器的标定信息。
 *
 * 该函数读取设备的显示器标定信息，并将标定结果保存到 calib 结构体中。
 *
 * @param calib 存储显示器标定信息的结构体
 * @return bool 标定是否成功
 */
    bool xv_readDisplayCalibration(pdm_calibration *calib) {
        auto display = device->display();
        if (display) {
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
                LOG_DEBUG("xv#wrapper xv_readDisplayCalibration index is %d K[0] = %f,K[1] = %f",i, calib->intrinsic.K[0], calib->intrinsic.K[1]);
                LOG_DEBUG("xv#wrapper xv_readDisplayCalibration  index is %d  T%d:%lf,%lf,%lf", i,calib->extrinsic.translation[0], calib->extrinsic.translation[1],
                          calib->extrinsic.translation[2]);
            }

            return true;
        }

        return false;
    }
    /**
     * @brief rgb callback 默认格式为YUYV
     */
    void onRgbCallback(xv::ColorImage const &rgb) {

        int w = rgb.width;
        int h = rgb.height;
        auto d = rgb.data.get();
        LOG_DEBUG("xv#wrapper onRgbCallback w = %d", w);
    }

    /**
      * @brief 停止rgb 数据流
      */
    void stopRgbStream() {
        if (!device || !device->colorCamera()) {
            return;
        }

        if (rgbId != -1) {
            device->colorCamera()->unregisterCallback(rgbId);
        }
        device->colorCamera()->stop();
    }

    /**
     * @brief 开启rgb 数据流
     */
    void startRgbStream() {
        LOG_DEBUG("startRgbStream");

        if (!device || !device->colorCamera()) {
            return;
        }
        stopRgbStream();
        //设置 rgb 分辨率
        xslam_set_rgb_resolution(UnityWrapper::RgbResolution::RGB_1920x1080);
        rgbId = device->colorCamera()->registerCallback(onRgbCallback);
        device->colorCamera()->start();
    }
    // 将UYVY格式的图像数据转换为RGB
    void UYVY_to_RGB(const uint8_t* uyvy_data, int width, int height, std::vector<uint8_t>& rgb_data) {
        int image_size = width * height;
        rgb_data.resize(image_size * 3);  // 每个像素需要3个字节来存储RGB值

        for (int i = 0, j = 0; i < image_size; i++) {
            int y1 = uyvy_data[2 * i];     // UYVY格式中的Y分量
            int u  = uyvy_data[2 * i + 1]; // U
            int y2 = uyvy_data[2 * i + 2]; // V
            int v  = uyvy_data[2 * i + 3]; // V

            // 将YUV转换为RGB
            int r = static_cast<int>(y1 + 1.402 * (v - 128));
            int g = static_cast<int>(y1 - 0.344136 * (u - 128) - 0.714136 * (v - 128));
            int b = static_cast<int>(y1 + 1.772 * (u - 128));

            // 限制RGB的值在[0, 255]范围内
            rgb_data[j++] = std::min(255, std::max(0, r));
            rgb_data[j++] = std::min(255, std::max(0, g));
            rgb_data[j++] = std::min(255, std::max(0, b));

            r = static_cast<int>(y2 + 1.402 * (v - 128));
            g = static_cast<int>(y2 - 0.344136 * (u - 128) - 0.714136 * (v - 128));
            b = static_cast<int>(y2 + 1.772 * (u - 128));

            rgb_data[j++] = std::min(255, std::max(0, r));
            rgb_data[j++] = std::min(255, std::max(0, g));
            rgb_data[j++] = std::min(255, std::max(0, b));
        }
    }



    void save_bmp(const std::string &filename, int *rgb_data, int width, int height) {
        std::ofstream file(filename, std::ios::binary);

        // BMP Header
        uint8_t header[54] = {
                0x42, 0x4D,  // Signature "BM"
                0, 0, 0, 0,  // Size of the file (to be filled later)
                0, 0, 0, 0,  // Reserved
                54, 0, 0, 0, // Offset to pixel data
                40, 0, 0, 0, // Info header size
                0, 0, 0, 0,  // Width (to be filled later)
                0, 0, 0, 0,  // Height (to be filled later)
                1, 0,        // Planes (must be 1)
                24, 0,       // Bits per pixel (24 for RGB)
                0, 0, 0, 0,  // Compression (none)
                0, 0, 0, 0,  // Image size (to be filled later)
                0x13, 0x0B, 0, 0, // Horizontal resolution (2835 pixels/meter)
                0x13, 0x0B, 0, 0, // Vertical resolution (2835 pixels/meter)
                0, 0, 0, 0,  // Colors in palette
                0, 0, 0, 0   // Important colors
        };

        int row_size = (width * 3 + 3) & (~3); // Row size must be multiple of 4 bytes
        int image_size = row_size * height;
        int file_size = 54 + image_size;

        // Fill in size fields
        header[2] = (file_size) & 0xFF;
        header[3] = (file_size >> 8) & 0xFF;
        header[4] = (file_size >> 16) & 0xFF;
        header[5] = (file_size >> 24) & 0xFF;

        header[18] = (width) & 0xFF;
        header[19] = (width >> 8) & 0xFF;
        header[20] = (width >> 16) & 0xFF;
        header[21] = (width >> 24) & 0xFF;

        header[22] = (height) & 0xFF;
        header[23] = (height >> 8) & 0xFF;
        header[24] = (height >> 16) & 0xFF;
        header[25] = (height >> 24) & 0xFF;

        header[34] = (image_size) & 0xFF;
        header[35] = (image_size >> 8) & 0xFF;
        header[36] = (image_size >> 16) & 0xFF;
        header[37] = (image_size >> 24) & 0xFF;

        // Write the header
        file.write(reinterpret_cast<char*>(header), sizeof(header));

        // Write pixel data
        uint8_t *row = new uint8_t[row_size];

        for (int y = height - 1; y >= 0; --y) {
            uint8_t *row_ptr = row;
            for (int x = 0; x < width; ++x) {
                int pixel = rgb_data[y * width + x];
                int r = (pixel >> 16) & 0xFF;
                int g = (pixel >> 8) & 0xFF;
                int b = pixel & 0xFF;

                *row_ptr++ = b; // BMP format stores pixels as BGR
                *row_ptr++ = g;
                *row_ptr++ = r;
            }

            // Fill the rest of the row with padding bytes (if necessary)
            for (int i = width * 3; i < row_size; ++i) {
                row_ptr[i] = 0;
            }

            file.write(reinterpret_cast<char*>(row), row_size);
        }

        delete[] row;
        file.close();
    }

    /**
    * @brief 开启rgb2 数据流
    */
    int xv_start_colorCamera2() {
       rgb2Id = device->colorCamera()->registerCam2Callback([](xv::ColorImage const &rgb) {
            int w = rgb.width;
            int h = rgb.height;
            auto d = rgb.data.get();

            LOG_DEBUG("xv#wrapper colorCamera2 callback w = %d", w);
        });
        device->colorCamera()->startCameras();
        return rgb2Id;
    }
    std::atomic<int> frame_count(0);  // 使用原子计数器保证线程安全
    /**
     * @brief 开启红外数据流获取
     */
    int xv_start_ThermalCamera() {
        thermalId = device->thermalCamera()->registerCallback([](xv::ThermalImage const &image) {
            int w = image.width;
            int h = image.height;
            auto d = image.data.get();

        });
        device->thermalCamera()->start();
        return thermalId;
    }
    std::shared_ptr<xv::IrTrackingImage> s_IrTrackingImage;
    static std::mutex s_irTrackingMutex;
    /**
   * @brief xv_get_left_image Get the last left image data in RGBA format
   * @param data
   * @param width
   * @param height
   * @return
   */
    bool xv_save_ir_tracking_image() {
        if (!s_IrTrackingImage)
            return false;

        if (!s_irTrackingMutex.try_lock())
            return false;

        std::shared_ptr<const xv::IrTrackingImage> tmp = s_IrTrackingImage;
        unsigned srcWidth = tmp->width;
        unsigned srcHeight = tmp->height;
        s_irTrackingMutex.unlock();

        int width = s_IrTrackingImage->width;
        int height = s_IrTrackingImage->height;
        std::shared_ptr<unsigned char> datatmp;
        if (!tmp->data) {
            LOG_DEBUG("xv#wrapper irTrackingCamera tmp->data is null");
            return false;  // 或者其他错误处理
        }

        cv::Mat mgray(srcHeight, srcWidth, CV_8UC1,
                      const_cast<unsigned char *>(tmp->data.get()));
        unsigned char* data = new unsigned char[width * height * 4];
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

        cv::flip(mrgb, mrgb, 1);

        std::ostringstream file_name;
        // 获取帧计数并递增
        int current_frame = frame_count++;
        // 在 Android 中构建完整路径，路径应以 "/sdcard/" 开头
        file_name << "/xvisio/frame_" << current_frame << ".tif";
        std::string path = "/storage/emulated/0/Download/frame_" + std::to_string(current_frame) + ".tif";


        // 确保保存路径存在
        std::string dir = path.substr(0, path.find_last_of("/"));
        struct stat st = {0};
        if (stat(dir.c_str(), &st) == -1) {
            mkdir(dir.c_str(), 0700);  // 创建目录
        }
        bool result = cv::imwrite(path, mrgb);
        return true;
    }

    /**
  * @brief 开启K1红外数据camera1流获取
  */
    int xv_start_irTrackingCamera() {
        irTrackingId = device->irTrackingCamera()->registerCallback([](xv::IrTrackingImage const &image) {
            int w = image.width;
            int h = image.height;
            auto d = image.data.get();
    // 增加帧计数
            frame_count++;

            // 获取当前时间
            auto current_time = std::chrono::steady_clock::now();

            // 计算时间差（以秒为单位）
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_time).count();
            double seconds = duration / 1e6;

            // 每秒更新一次帧率
            static double fps = 0.0;
            if (seconds >= 1.0) { // 每秒计算一次
                fps = frame_count / seconds;
                LOG_DEBUG("xv#wrapper irTrackingCamera callback w = %d, FPS = %.2f", w, fps);

                // 重置计数和时间
                frame_count = 0;
                last_time = current_time;
            }
            // 示例：使用一个 YUYV 图片
         /*   if(frame_count++%10 == 0){
//                int *rgb_image = new int[w * h];  // 32-bit color per pixel (ARGB)
                // 转换为RGB格式
                // 创建一个 RGB 图像（例如，全白图像）
                std::vector<uint8_t> rgb_data;
                UYVY_to_RGB((unsigned char *) d, w, h,rgb_data);
                // 将 RGB 数据转换为 OpenCV 的 Mat 格式
                cv::Mat image(h, w, CV_8UC3, rgb_data.data());  // CV_8UC3表示每个像素3个字节，RGB格式
                // 保存图像为 PNG 格式
                std::ostringstream file_name;
                // 获取帧计数并递增
                int current_frame = frame_count++;
                // 在 Android 中构建完整路径，路径应以 "/sdcard/" 开头
                file_name << "/xvisio/frame_" << current_frame << ".png";
                std::string path = std::string("/storage/emulated/0") + file_name.str();  // 使用绝对路径

                // 确保保存路径存在
                std::string dir = path.substr(0, path.find_last_of("/"));
                struct stat st = {0};
                if (stat(dir.c_str(), &st) == -1) {
                    mkdir(dir.c_str(), 0700);  // 创建目录
                }
                bool result = cv::imwrite(path, image);
            }*/
            LOG_DEBUG("xv#wrapper irTrackingCamera callback w = %d", w);
        });
        device->irTrackingCamera()->start();

        return irTrackingId;
    }
    /**
    * @brief 开启K1红外数据camera2流获取
    */

    int xv_start_irTrackingCamera2() {
        irTrackingId2 = device->irTrackingCamera()->registerCamera2Callback([](xv::IrTrackingImage const &image) {
            int w = image.width;
            int h = image.height;
            auto d = image.data.get();
            s_irTrackingMutex.lock();
            s_IrTrackingImage = std::make_shared<xv::IrTrackingImage>(image);
            s_irTrackingMutex.unlock();
            // 示例：使用一个 YUYV 图片
        /*    if(frame_count++%10 == 0){
                int *rgb_image = new int[w * h];  // 32-bit color per pixel (ARGB)
                yuv2rgb((unsigned char *) d, rgb_image, w, h);
                save_bmp("output.bmp", rgb_image, w, h);
                LOG_DEBUG("xv#wrapper irTrackingCamera callback w = %d", w);
            }*/
            LOG_DEBUG("xv#wrapper irTrackingCamera callback w = %d", w);
        });


        return irTrackingId2;
    }
    bool xv_stop_irTrackingCamera2() {
        LOG_DEBUG("xv#wrapper xv_stop_thermalCamera");
        device->irTrackingCamera()->unregisterCamera2Callback(irTrackingId2);
        return device->irTrackingCamera()->stopCamera2();
    }

    /**
   * @brief 开启K1红外参数
   */
    void xv_get_irTrackingCamera_params(){
        int fps = device->irTrackingCamera()->getFrameRate();
        LOG_DEBUG("xv#wrapper ir tracking camera fps = %d", fps);
        xv::ResolutionParam param;
        bool ret = device->irTrackingCamera()->getResolution(param);
        if(ret)
        {
            LOG_DEBUG("xv#wrapper ir tracking camera width = %d,height = %d", param.width, param.height);
        }
        xv::RoiParam roiParam;
        ret = device->irTrackingCamera()->getROI(roiParam);
        if(ret)
        {
            LOG_DEBUG("xv#wrapper ir tracking roi width = %d,roiParam.x = %d", roiParam.width,roiParam.x);
        }

        xv::IrTrackingTemperature temperature;
        ret = device->irTrackingCamera()->getTemperature(temperature);
        if(ret)
        {
            LOG_DEBUG("xv#wrapper ir tracking temperature one= %d,temperature.two = %d", temperature.one,temperature.two);
        }

        int time = device->irTrackingCamera()->getExposureTime();
        if(time != -1)
        {
            LOG_DEBUG("xv#wrapper ir tracking exposure time = %d",time);
        }

    }

    /**
    * @brief 关闭rgb2 数据流
    */
    void xv_stop_colorCamera2() {
        LOG_DEBUG("xv#wrapper xv_stop_colorCamera2");
        device->colorCamera()->unregisterCam2Callback(rgb2Id);
        device->colorCamera()->stopCameras();
    }
    /**
    * @brief 关闭红外数据流
    */
    bool xv_stop_thermalCamera() {
        LOG_DEBUG("xv#wrapper xv_stop_thermalCamera");
        device->thermalCamera()->unregisterCallback(thermalId);
        return device->thermalCamera()->stop();
    }

    /**
  * @brief 关闭红外数据流
  */
    bool xv_stop_irTrackingCamera() {
        LOG_DEBUG("xv#wrapper xv_stop_thermalCamera");
        device->irTrackingCamera()->unregisterCallback(irTrackingId);
        return  device->irTrackingCamera()->stop();
    }
    /**
     * @brief Exposure setting.
     * @param[in] aecMode 0:auto exposure 1:manual exposure
     * @param[in] exposureGain Only valid in manual exposure mode, [0,255]
     * @param[in] exposureTimeMs Only valid in manual exposure mode, in milliseconds
     */
    void xv_rgb_set_exposure(int aecMode, int exposureGain, float exposureTimeMs) {
        if (device->colorCamera()) {
            device->colorCamera()->setExposure(aecMode, exposureGain, exposureTimeMs);
        }
        LOG_DEBUG("xv#wrapper rgb_set_exposure: %d, %d, %d", aecMode, exposureGain, exposureTimeMs);
    }

    /**
    * @brief tof 回调函数
    * @param[in] im tof回调数据
    */
    void onTofCallback(xv::DepthImage const &im) {
        std::shared_ptr<const xv::DepthImage> tmp = std::make_shared<xv::DepthImage>(im);;
        //tof宽高
        unsigned w = tmp->width;
        unsigned h = tmp->height;

        LOG_DEBUG("xv#wrapper onTofStream type: %d, width = %d,height = %d", im.type, w, h);

        int s = w * h;
        //深度数据
        auto d = const_cast<unsigned char *>(tmp->data.get());

    }

    /**
    * @brief 关闭tof数据流获取
    */
    void stopTofStream() {
        if (!device || !device->tofCamera()) {
            return;
        }

        if (tofId != -1) {
            device->tofCamera()->unregisterCallback(tofId);
        }
        device->tofCamera()->stop();
    }

    /**
    * @brief 设置pmd tof
    */
    bool setPmdTofIRFunction() {
        bool ret = false;
        std::vector<unsigned char> result(63);
        bool bOK = device->hidWriteAndRead({0x02, 0x10, 0xf5, 0x02, 0x01}, result);
        if (bOK) {
            std::cout << "xv#wrapper Enable IR successfully" << std::endl;
            ret = true;
        } else {
            std::cout << "xv#wrapper Enable IR failed" << std::endl;
            ret = false;
        }
        return ret;
    }

    /**
      * @brief 开启tof数据流
      */
    void startTofStream() {
        LOG_DEBUG("startTofStream");
        if (!device || !device->tofCamera()) {
            return;
        }
        //   setPmdTofIRFunction();
        tofId = device->tofCamera()->registerCallback(onTofCallback);
        device->tofCamera()->setSonyTofSetting(xv::TofCamera::SonyTofLibMode::IQMIX_DF,
                                               xv::TofCamera::Resolution::VGA,
                                               xv::TofCamera::Framerate::FPS_10);
        device->tofCamera()->start();
    }

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
* @brief xv_get_tofir_image Get the last left image data in RGBA format
* @param data
* @param width
* @param height
* @return
*/
bool xv_get_tofir_image(unsigned char *data, int width, int height) {

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
        LOG_DEBUG("eddy xv_get_tofir_image srcWidth = %d,srcHeight = %d,%d,%d" ,
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
    /**
       * @brief 开启tof IR数据流
       */
    void startTofIRStream() {
        LOG_DEBUG("startTofStream");
        if (!device || !device->tofCamera()) {
            return;
        }
        device->tofCamera()->setStreamMode(xv::TofCamera::StreamMode::DepthAndCloud);
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

            } else if(tof.type == xv::DepthImage::Type::Cloud){
                std::shared_ptr<xv::PointCloud> pointCould = device->tofCamera()->formatPmdCloudToPointCloudStruct(tof);
                //测试第一个像素点的相机坐标系下 xyz
                float x =  pointCould->points[0][0];
                float y =  pointCould->points[0][1];
                float z =  pointCould->points[0][1];
#ifdef ANDROID
                __android_log_print(ANDROID_LOG_WARN, "xv#wrapper",
                                    "pointCould[0].x = %f,y = %f,z = %f",x,y,z);

#endif
            }
        });

        device->tofCamera()->start();
    }


    /**
     * @brief 开启手势获取并注册回调
     *
     * 该函数用于启动手势获取，并在手势数据更新时触发回调函数。回调函数将解析获取的手势数据，并将其转换为骨架信息（包括52个关键点数据），用于进一步处理。
     *
     * @return int 如果成功注册回调并启动手势获取，则返回注册的回调ID；否则返回 -1。
     *
     * @note
     * - 该函数仅在设备的 `gesture()` 模块可用时执行。
     * - 如果手势获取初始化失败，函数会尝试重新设置平台并再次启动手势获取。
     * - 回调函数会在每次获取到手势数据时被调用，并将手势的骨架数据存储到 `skelet` 结构体中。该结构体包括每个关键点的坐标和旋转数据。
     * - 骨架数据中的前26个点表示左手，后26个点表示右手。
     * - 手势的时间戳和其他状态数据也会被记录并输出。
     */
    int xv_start_skeleton_ex_with_cb() {

#ifdef ANDROID
        __android_log_print(ANDROID_LOG_WARN, "xv#wrapper",
                            "unity-wrapper gesture xslam_start_skeleton_ex_with_cb start");

#endif
        void *JVM;
        if (device && device->gesture()) {
            device->gesture()->setPlatform(s_gesturePlatform, s_gestureEgo);
            bool res = device->gesture()->start();
            if (res == false) {
#ifdef ANDROID
                __android_log_print(ANDROID_LOG_WARN, "xv#wrapper",
                                    "unity-wrapper gesture xslam_start_skeleton_ex_with_cb start with npu false");

#endif
                device->gesture()->setPlatform(1, s_gestureEgo);
                device->gesture()->start();
            }
            return device->gesture()->registerSlamKeypointsCallback(
                    [](std::shared_ptr<const xv::HandPose> handpose) {
                        XslamSkeleton skelet = {0};
                        int index = 0;
                        skelet.size = handpose->pose.size();
#ifdef ANDROID
                        __android_log_print(ANDROID_LOG_WARN, "xv#wrapper",
                                            "unity-wrapper gesture KeypointsCallback count:%d",
                                            skelet.size);

#endif
                        //52个关键点数据 0到25是左手 26到51是右手
                        for (auto pose: handpose->pose) {

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

                        skelet.fisheye_timestamp = handpose->fisheye_timestamp;
                        for (int i = 0; i < 2; ++i) {
                            skelet.scale[i] = handpose->scale[i];
                            skelet.timestamp[i] = handpose->timestamp[i];
                            //静态手势
                            skelet.status[i] = handpose->status[i];
                        }

                        __android_log_print(ANDROID_LOG_WARN, "xv#wrapper",
                                            "unity-wrapper gesture  fisheye_timestamp:%f,timestamp:%f,skelet.poseData[51].x = %f",
                                            skelet.fisheye_timestamp, skelet.timestamp[0],
                                            skelet.poseData[51].x);

                    });
        } else {
#ifdef ANDROID
            __android_log_print(ANDROID_LOG_WARN, "xv#wrapper",
                                "unity-wrapper gesture xslam_start_skeleton_ex_with_cb device->gesture() return false");

#endif
        }


        return -1;
    }

    /**
     * @brief 眼动回调 .
     * @param[in] eyedata  眼动数据 具体看结构体定义
     */
    void gazeCallback(xv::XV_ET_EYE_DATA_EX const &eyedata) {
#ifdef ANDROID
        __android_log_print(ANDROID_LOG_WARN, "xv#wrapper",
                            "eddy gazeCallback eyedata.timestamp = %lld", eyedata.timestamp);
        __android_log_print(ANDROID_LOG_WARN, "xv#wrapper", "eddy gazeCallback eyedata.Recomm gaze x= %f",
                            eyedata.recomGaze.gazePoint.x);
        __android_log_print(ANDROID_LOG_WARN, "xv#wrapper", "eddy gazeCallback eyedata.Recomm gaze x= %f",
                            eyedata.recomGaze.gazeOrigin.x);
        __android_log_print(ANDROID_LOG_WARN, "xv#wrapper", "eddy gazeCallback eyedata.Recomm gaze x= %f",
                            eyedata.recomGaze.gazeDirection.x);


#endif
    }

    /**
      * @brief 开启眼动数据获取
      */
    int start_et_gaze_callback() {
        if (xslam_ready()) {
            return gazeId = device->gaze()->registerCallback(gazeCallback);
        } else
            return -1;
    }

    /**
   * @brief 读取鱼眼标定参数
   */
    bool xvReadStereoFisheyesCalibration() {
        if (device) {
            std::vector<xv::CalibrationEx> m_FECalibration;

            xslam_readStereoFisheyesCalibration(m_FECalibration);
            if (m_FECalibration.size() > 0 && m_FECalibration[0].seucm.size() > 0) {

                for (int i = 0; i < m_FECalibration.size(); i++) {
#ifdef ANDROID
                    __android_log_print(ANDROID_LOG_WARN, "xv#wrapper",
                                        "eddy xvReadStereoFisheyesCalibration entry %f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                                        m_FECalibration[i].seucm[0].w,
                                        m_FECalibration[i].seucm[0].h,
                                        m_FECalibration[i].seucm[0].fx,
                                        m_FECalibration[i].seucm[0].fy,
                                        m_FECalibration[i].seucm[0].u0,
                                        m_FECalibration[i].seucm[0].v0,
                                        m_FECalibration[i].seucm[0].eu,
                                        m_FECalibration[i].seucm[0].ev,
                                        m_FECalibration[i].seucm[0].alpha,
                                        m_FECalibration[i].seucm[0].beta);

#endif
                }
                return true;
            } else {
                return false;
            }

        }
        return false;
    }
    /**
   * @brief 手柄数据回调 扫描连接在unity完成
   */
    void xv_controller_register() {
        __android_log_print(ANDROID_LOG_INFO, "xv#wrapper", "xv_wireless_register");
        if(device && device->wirelessController()) {
            device->wirelessController()->registerWirelessControllerDataCallback([](const xv::WirelessControllerData& data) {
                __android_log_print(ANDROID_LOG_INFO, "xv#wrapper", "xv_wireless_register %f", data.pose.confidence());
                ControllerPos pose;
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
                if(data.key == 16){
                    pose.keyA = 1;
                    pose.keyB = 0;
                } else if(data.key == 32){
                    pose.keyA = 0;
                    pose.keyB = 1;
                } else if(data.key == 48){
                    pose.keyA = 1;
                    pose.keyB = 1;
                }else {
                    pose.keyA = 0;
                    pose.keyB = 0;
                }
                __android_log_print(ANDROID_LOG_INFO, "xv#wrapper", " pose.position.x %f",  pose.position.x);
                __android_log_print(ANDROID_LOG_INFO, "xv#wrapper", " confidence %f",  pose.confidence);
                __android_log_print(ANDROID_LOG_INFO, "xv#wrapper", " keyA status %d",  pose.keyA);
                __android_log_print(ANDROID_LOG_INFO, "xv#wrapper", " keyB status %d",  pose.keyB);
                __android_log_print(ANDROID_LOG_INFO, "xv#wrapper", " rocker_x status %d",  pose.rocker_x);
                __android_log_print(ANDROID_LOG_INFO, "xv#wrapper", " rocker_y status %d",  pose.rocker_y);
            });
        } else {
            __android_log_print(ANDROID_LOG_INFO, "xv#wrapper", "xv_wireless_register FAIL");
        }
    }
    /**
      * @brief 开启光感检测
     */
    bool xv_start_light_preception(){
        return xslam_start_light_preception();
    }

    /**
    * @brief 关闭光感检测
   */
    bool xv_stop_light_preception(){
        return xslam_stop_light_preception();
    }
    /**
    * @brief 北斗/gps数据
    * @param mode 0北斗模式 1北斗&GPS混合模式
    */
    int xv_start_beidou_stream(int mode) {

        if (device && device->beiDouGPS()) {
            device->beiDouGPS()->setMode((xv::BeiDouGPSMode) mode);
            device->beiDouGPS()->start();
            return  device->beiDouGPS()->registerCallback([](xv::BeiDouGPSData const &data) {
#ifdef ANDROID
                __android_log_print(ANDROID_LOG_INFO, "xv#wrapper", "eddy BeiDouGPSData entry data_ready_flag = %d,lat = %f"
                ,(int)data.data_ready_flag,data.lat_data);
#endif
                /*    data结构体说明
                     data_ready_flag; //0 无效数据 1 有效数据
                     lat_data;//纬度数据
                     latdir;//1南纬 2 北纬
                     lon_data;//经度数据
                     londir;//1 东经 2 西经
                     satellite_num;//寻星数量
                     mode;//当前模式 0北斗模式 1北斗&GPS混合模式*/


            });
        } else {
            return -1;
        }
    }
    /**
     * @brief imu数据获取
     */
    void xv_start_imu() {
        if (device->imuSensor()) {
            device->imuSensor()->registerCallback([](xv::Imu const &imu) {
                s_imu = std::make_shared<xv::Imu>(imu);
#ifdef ANDROID
                __android_log_print(ANDROID_LOG_INFO, "xv#wrapper", "eddy xv_start_imu imu.accel[0]] = %f"
                        ,s_imu->accel[0]);
#endif

                Vector3 imu_t[3];
             /*    imu_t[0].x = s_imu->accel[0];
                imu_t[0].y = s_imu->accel[1];
                imu_t[0].z = s_imu->accel[2];
                imu_t[1].x = s_imu->gyro[0];
                imu_t[1].y = s_imu->gyro[1];
                imu_t[1].z = s_imu->gyro[2];
                imu_t[2].x = s_imu->magneto[0];
                imu_t[2].y = s_imu->magneto[1];
                imu_t[2].z = s_imu->magneto[2];*/
            });
            device->imuSensor()->start();
        }
    }

    /**
    * @brief 光感数据获取 眼镜佩戴状态获取 眼镜按键事件获取(详见文档)
    * @param mode 0北斗模式 1北斗&GPS混合模式
    */
    int xv_start_event_stream() {

        if (device && device->eventStream()) {
            int status =  device->eventStream()->registerCallback([](xv::Event const &event){
                int type = event.type;
                int key = event.state;
#ifdef ANDROID
                __android_log_print(ANDROID_LOG_INFO, "xv#wrapper", "eddy eventStream  entry type = %d,key = %d"
                        ,type,key);
#endif
                /*
                  * 佩戴检测：
                 type = 2 ,state = 0 眼镜摘掉状态
                 type = 2 ,state = 1 眼镜戴上状态

                 光感检测数据：
                 type = 6 ,state = 0 //state是数值单位Lux
                 */
            });
            device->eventStream()->start();
            return  status;
        } else {
            return -1;
        }
    }

    /**
    * @brief 显示源切换成微光和红外融合图像
    */
    bool xv_switch_rgb() {
        return xslam_switch_rgb();
    }
    /**
    * @brief 显示源切换成dp源数据
    */
    bool xv_switch_display() {
        return xslam_switch_display();
    }
    /**
   * @brief 获取当前音量级别
   *
   * 调用 Java 层的 `getVolumeCurr` 方法，获取当前 STREAM_MUSIC 类型的音量级别。
   * 该方法通过 JNI 连接到 Java 方法，返回当前音量的整数值。
   *
   * @return int 当前音量级别，如果方法或类未初始化则返回 -1
   */
    int xv_callGetVolumeCurr() {
        if (!s_getVolumeCurrMethod || !s_XvInterfaceClass) {
            printf("Method or class not initialized\n");
            return -1;
        }
        JNIEnv *jniEnv;
        jvm->AttachCurrentThread(&jniEnv, NULL);
        return jniEnv->CallStaticIntMethod(s_XvInterfaceClass, s_getVolumeCurrMethod);
    }
/**
 * @brief 获取最大音量级别
 *
 * 调用 Java 层的 `getVolumeMax` 方法，获取 STREAM_MUSIC 类型支持的最大音量级别。
 * 该方法通过 JNI 调用 Java 方法，返回最大音量的整数值。
 *
 * @return int 最大音量级别，如果方法或类未初始化则返回 -1
 */
    int xv_callGetVolumeMax() {
        if (!s_getVolumeMaxMethod || !s_XvInterfaceClass) {
            printf("Method or class not initialized\n");
            return -1;
        }
        JNIEnv *jniEnv;
        jvm->AttachCurrentThread(&jniEnv, NULL);
        return jniEnv->CallStaticIntMethod(s_XvInterfaceClass, s_getVolumeMaxMethod);
    }

    /**
   * @brief 调整音量
   *
   * 调用 Java 层的 `adjustVolume` 方法，根据指定的方向调整 STREAM_MUSIC 类型的音量。
   *
   * @param streamType 音量流类型（通常为 AudioManager.STREAM_MUSIC）
   * @param direction 调整方向（例如 AudioManager.ADJUST_RAISE 提高音量，AudioManager.ADJUST_LOWER 降低音量）
   * * ### 支持的音量流类型：
 * - `STREAM_VOICE_CALL = 0`：电话通话音量
 * - `STREAM_SYSTEM = 1`：系统声音音量
 * - `STREAM_RING = 2`：电话铃声和消息提醒音量
 * - `STREAM_MUSIC = 3`：音乐播放音量
 * - `STREAM_ALARM = 4`：闹钟音量
 * - `STREAM_NOTIFICATION = 5`：通知音量
 *
 * ### 调整方向常量：
 * - `ADJUST_RAISE = 1`：增加音量
 * - `ADJUST_LOWER = -1`：减小音量
 * - `ADJUST_SAME = 0`：保持当前音量
 * - `ADJUST_MUTE = -100`：静音
 * - `ADJUST_UNMUTE = 100`：取消静音
 *
   * @return int 调整后的音量级别，如果方法或类未初始化则返回 -1
   */
    int xv_callAdjustVolume(int direction, int streamType) {
        if (!s_adjustVolumeMethod || !s_XvInterfaceClass) {
            printf("Method or class not initialized\n");
            return -1;
        }
        JNIEnv *jniEnv;
        jvm->AttachCurrentThread(&jniEnv, NULL);
        return jniEnv->CallStaticIntMethod(s_XvInterfaceClass, s_adjustVolumeMethod, direction, streamType);
    }
    void xv_play_music( const std::string& filePath){
        JNIEnv *env;
        jvm->AttachCurrentThread(&env, NULL);
        // 转换 filePath 为 Java 字符串
        jstring jFilePath = env->NewStringUTF(filePath.c_str());

        // 调用静态方法 playAudio
        env->CallStaticVoidMethod(s_XvInterfaceClass, s_playAudioMethod, jFilePath);

        // 删除局部引用
        env->DeleteLocalRef(jFilePath);

    }
    // 调用 stopAudio 方法
    void xv_stopAudio() {
        JNIEnv *env;
        jvm->AttachCurrentThread(&env, NULL);
        // 调用静态方法 stopAudio
        env->CallStaticVoidMethod(s_XvInterfaceClass, s_stopAudioMethod);
    }
    /**
    * @brief 切换眼镜音频开关
    * @param status false 关闭 true 打开
    */
    bool xv_switch_audio(bool status) {
        return xslam_switch_audio(status);
    }
    /**
      * @brief 本地音频识别
      * @param name pcm或wav音频文件名 目前支持类似AudioRecord录制的 AudioFormat.CHANNEL_IN_MONO  AudioFormat.ENCODING_PCM_16BIT pcm或wav文件
      * 将文件放置到目录为sdcard/xv/
      */
    void xv_recognize_from_local(const char *name) {
        // 创建一个 Java 字符串作为参数
#ifdef ANDROID
        __android_log_print(ANDROID_LOG_INFO, "xv#wrapper", "eddy xv_recognize_from_local  name = %s"
                ,name);
#endif
        JNIEnv *jniEnv;
        jvm->AttachCurrentThread(&jniEnv, NULL);
        if (!jniEnv || !s_XvInterfaceClass || !s_startRecognizerFromPath) {
            return;
        }
        jniEnv->CallStaticVoidMethod(s_XvInterfaceClass, s_startRecognizerFromPath, jniEnv->NewStringUTF(name));
    }

    /**
  * @brief 本地音频识别
  * @param source 1: mic实时识别 2:本地文件或者byte[]识别
  */
    void xv_audio_recognize_switch_source(int source) {
        // 创建一个 Java 字符串作为参数
#ifdef ANDROID
        __android_log_print(ANDROID_LOG_INFO, "xv#wrapper", "eddy xv_audio_recognize_switch_source  source = %d"
                ,source);
#endif
        JNIEnv *jniEnv;
        jvm->AttachCurrentThread(&jniEnv, NULL);
        if (!jniEnv || !s_XvInterfaceClass || !s_switchRecognizeSource) {
            return;
        }
        jniEnv->CallStaticVoidMethod(s_XvInterfaceClass, s_switchRecognizeSource, source);
    }
    /**
      * @brief 本地音频识别
      * @param data 音频字节数组
      * @param length 音频字节数组长度
      */
    void xv_recognize_from_byte(const uint8_t* data, int length) {

        std::vector<uint8_t> vecData(data, data + length);
#ifdef ANDROID
        __android_log_print(ANDROID_LOG_INFO, "xv#wrapper", "eddy xv_recognize_from_local  data size = %d"
                ,vecData.size());
#endif
        JNIEnv *jniEnv;
        jvm->AttachCurrentThread(&jniEnv, NULL);
        if (!jniEnv || !s_XvInterfaceClass || !s_startRecognizerFromByte) {
            return;
        }
        // 3. 将 C++ 的 std::vector<uint8_t> 转换为 jbyteArray
        jbyteArray jData = jniEnv->NewByteArray(vecData.size());
        if (jData == nullptr) {
            // 处理错误，内存不足
            jniEnv->DeleteLocalRef(s_XvInterfaceClass);
            return;
        }
        jniEnv->SetByteArrayRegion(jData, 0, vecData.size(), reinterpret_cast<const jbyte*>(vecData.data()));

        // 4. 调用静态方法
        jniEnv->CallStaticVoidMethod(s_XvInterfaceClass, s_startRecognizerFromByte, jData);

        // 5. 清理局部引用
        jniEnv->DeleteLocalRef(jData);
        jniEnv->DeleteLocalRef(s_XvInterfaceClass);
    }
    /**
      * @brief 开启左rgb融合红外 数据流
      */
    int xv_start_RGB_L_ThermalFusionCamera() {
        auto deviceEX = std::dynamic_pointer_cast<xv::DeviceEx>(device);
        LOG_DEBUG("xv#wrapper xv_start_RGB_L_ThermalFusionCamera 1");
        if(deviceEX){
            deviceEX->RGB_L_ThermalFusionCamera()->start();
            LOG_DEBUG("xv#wrapper xv_start_RGB_L_ThermalFusionCamera 2");
            return deviceEX->RGB_L_ThermalFusionCamera()->registerCallback([](xv::ColorImage const &rgb) {
                int w = rgb.width;
                int h = rgb.height;
                auto d = rgb.data.get();
                LOG_DEBUG("xv#wrapper RGB_L_ThermalFusionCamera callback w = %d", w);
            });

        } else {
            LOG_DEBUG("xv#wrapper xv_start_RGB_L_ThermalFusionCamera deviceEx is null");
            return -1;
        }
    }
    /**
     * @brief 关闭左rgb融合红外 数据流
     */
    void xv_stop_RGB_L_thermalFusionCamera(int id) {
        auto deviceEX = std::dynamic_pointer_cast<xv::DeviceEx>(device);
        deviceEX->RGB_L_ThermalFusionCamera()->unregisterCallback(id);
        deviceEX->RGB_L_ThermalFusionCamera()->stop();
    }
    /**
     * @brief 开启右rgb融合红外 数据流
     */
    int xv_start_RGB_R_ThermalFusionCamera() {
        auto deviceEX = std::dynamic_pointer_cast<xv::DeviceEx>(device);
        deviceEX->RGB_R_ThermalFusionCamera()->start();
        return deviceEX->RGB_R_ThermalFusionCamera()->registerCallback([](xv::ColorImage const &rgb) {
            int w = rgb.width;
            int h = rgb.height;
            auto d = rgb.data.get();
            LOG_DEBUG("xv#wrapper RGB_R_ThermalFusionCamera callback w = %d", w);
        });
    }
    /**
     * @brief 关闭右rgb融合红外 数据流
     */
    void xv_stop_RGB_R_thermalFusionCamera(int id) {
        auto deviceEX = std::dynamic_pointer_cast<xv::DeviceEx>(device);
        deviceEX->RGB_R_ThermalFusionCamera()->unregisterCallback(id);
        deviceEX->RGB_R_ThermalFusionCamera()->stop();
    }

    /**
   * @brief 打开/关闭 rgb融合红外
   */
    bool xv_switch_fusionCameras_state(bool isOpen) {
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
    /**
      * @brief 切换rgb融合红外左右
      */
    bool xv_switch_fusionCamerasLR(int state) {
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
    /**
     * @brief 打开/关闭 光学模程使能
     * @param eye_type 目标眼类型，1：左眼光学，2：右眼光学，3：两眼光学
     * @param isOpen 是否打开光学模程
     * @return bool 操作是否成功
     */
    bool xv_switch_display_state(int eye_type, bool isOpen) {
        if (device) {
            std::vector<unsigned char> write;
            std::vector<unsigned char> vecRead;

            // 添加头部数据
            write.push_back(0x02);
            write.push_back(0xfe);
            write.push_back(0x20);
            write.push_back(0x27);

            // 根据 eye_type 和 isOpen 设置指令
            switch (eye_type) {
                case 1: // 左眼
                    write.push_back(isOpen ? 0x11 : 0x10);
                    break;
                case 2: // 右眼
                    write.push_back(isOpen ? 0x21 : 0x20);
                    break;
                case 3: // 两眼
                    write.push_back(isOpen ? 0x31 : 0x30);
                    break;
                default:
                    return false; // 无效的 eye_type
            }

            // 调用设备模块的 hidWriteAndRead 操作
            return device->hidWriteAndRead(write, vecRead);
        } else {
            return false; // 设备无效
        }
    }

/**
 * @brief 设置电致变色等级
 * @param level 电致变色等级，范围 0-20
 * @return bool 操作是否成功
 */
    bool xv_set_electrochromic_level(int level) {
        if (device) {
            if (level < 0 || level > 20) {
                return false; // 无效的等级
            }

            std::vector<unsigned char> write;
            std::vector<unsigned char> vecRead;

            // 添加头部数据
            write.push_back(0x02);
            write.push_back(0xcd);
            write.push_back(0x02);
            write.push_back(0x02);

            // 添加电致变色等级
            write.push_back(static_cast<unsigned char>(level));

            // 调用设备模块的 hidWriteAndRead 操作
            return device->hidWriteAndRead(write, vecRead);
        } else {
            return false; // 设备无效
        }
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
    bool xv_start_eyetracking() {
        if (!device || !device->eyetracking()) {
            return false;
        }

        bool ret = device->eyetracking()->start();
        device->eyetracking()->registerCallback([](xv::EyetrackingImage const &eyetracking_image) {
        //眼动仪相机图像
            std::shared_ptr<xv::EyetrackingImage> g_eyetracking_img = nullptr;
            g_eyetracking_img = std::make_shared<xv::EyetrackingImage>(eyetracking_image);
            int width = g_eyetracking_img->images[0].width;
            int height = g_eyetracking_img->images[0].height;
            LOG_DEBUG("xv-eyetracking", "eyetracking_image width:%d,height = %d",width,height);
            //左眼
             g_eyetracking_img->images[0].data.get();
            //右眼
            g_eyetracking_img->images[1].data.get();
        });
        LOG_DEBUG("xv-eyetracking", "start :%d", ret);
        return ret;
    }
    /**
     * @brief 读取眼动仪标定参数。
     *
     */
    void xv_read_eyetracking_calibrate(){
        if (!device) {
            LOG_ERROR("device is nullptr!");
            return;
        }

        auto eyetracking = device->eyetracking();
        if (!eyetracking) {
            LOG_ERROR("device->eyetracking() is nullptr!");
            return;
        }

        std::vector<xv::Calibration> calibrations = eyetracking->calibration();
        if (calibrations.size() == 2) {
            LOG_DEBUG("eyetracking m_Calibration[0].pose.x= %f, pose.y = %f",
                      (float )calibrations[0].pose.x()*1000,
                      (float ) calibrations[0].pose.y()*1000);
            LOG_DEBUG("eyetracking m_Calibration[1].pose.x= %f, pose.y = %f",
                      (float )calibrations[1].pose.x()*1000,
                      (float )calibrations[1].pose.y()*1000);
        } else {
            LOG_DEBUG("Calibration list is empty!");
        }
    }
    /**
 * @brief 读取红外标定参数。
 *
 */
    bool xv_read_thermal_calibration(){
        if (!device) {
            return false;
        }
        /**
         * @brief 红外相机标定扩展结构体
         *
         * 该结构体继承自 `Calibration`，用于表示热成像相机的标定数据扩展。它包含多个摄像机模型参数的集合，用于不同的相机分辨率。
         *
         * @note
         * - `seucm`：表示不同摄像头分辨率下的特殊统一相机模型参数，适用于热成像相机的标定。
         */
        std::vector<xv::CalibrationEx> m_Calibration;
        m_Calibration = reinterpret_cast<xv::ThermalCameraEX*>(device->thermalCamera().get())->calibrationEx();
        if (m_Calibration.size() > 0 ){
            LOG_DEBUG("IrTrackingCameraEX m_Calibration[0].pdcm.size() = %d,w = %d",
                      m_Calibration[0].pdcm.size(),
                      m_Calibration[0].pdcm[0].w);
            return true;
        } else {
            return false;
        }

    }

    /**
     * @brief IMU/导航相机标定读取接口
     *
     * @return bool 返回操作是否成功。如果设备无效或获取标定数据失败，返回 `false`；否则返回 `true`。

     */
    bool xv_read_irTracking_calibrationEx(bool isEx2){/*,std::vector<xv::CalibrationEx> &m_Calibration*/
        if (!device) {
            return false;
        }
        /**
         * @brief 热成像相机标定扩展结构体
         *
         * 该结构体继承自 `Calibration`，用于表示热成像相机的标定数据扩展。它包含多个摄像机模型参数的集合，用于不同的相机分辨率。
         *
         * @note
         * - `seucm`：表示不同摄像头分辨率下的特殊统一相机模型参数，适用于热成像相机的标定。
         */
        std::vector<xv::CalibrationEx> m_Calibration;
        if(isEx2){
            LOG_DEBUG("IrTrackingCameraEX isEx2 is true");
            m_Calibration = reinterpret_cast<xv::IrTrackingCameraEX*>(device->irTrackingCamera().get())->calibrationEx2();
        } else {
            m_Calibration = reinterpret_cast<xv::IrTrackingCameraEX*>(device->irTrackingCamera().get())->calibrationEx();
        }
        if (m_Calibration.size() > 0 ){
            LOG_DEBUG("IrTrackingCameraEX m_Calibration[0].pdcm.size() = %d,w = %d",
                      m_Calibration[0].pdcm.size(),
                      m_Calibration[0].pdcm[0].w);
            return true;
        } else {
            return false;
        }
    }
    /**
        * @brief 控制ir tracking camera的 LED 接口
        *
        * @param ldeIndex camera led index
        * @param enable
        *
        * @return bool 返回操作是否成功。如果设备无效或获取标定数据失败，返回 `false`；否则返回 `true`。

        */
    bool xv_enable_ir_tracking_camera_led(int ledIndex,bool enable){
        if (!device) {
            return false;
        }
       return device->irTrackingCamera()->enableLed(ledIndex, enable);

    }
    /**
       * @brief 控制ir tracking camera的 LED 接口
       *
       * @param ledIndex camera led index
       * @param time The unit is ms. The min value is 0, the max value is 1ms.
       *
       * @return bool 返回操作是否成功。如果设备无效或获取标定数据失败，返回 `false`；否则返回 `true`。

       */
    bool xv_set_ir_tracking_camera_led_time(int ledIndex,float time){
        if (!device) {
            return false;
        }
        return device->irTrackingCamera()->setLedTime(ledIndex, time);
    }
    /**
        * @brief Set ir tracking camera(1 and 2) exposure time.
        *
        * @param timeUs exposure time, the unit is microseconds，range is 16 μs to 22800 μs
        *
        * @return bool 返回操作是否成功。如果设备无效或获取标定数据失败，返回 `false`；否则返回 `true`。

        */
    bool xv_set_ir_tracking_exposureTime(int time){
        if (!device) {
            return false;
        }
        bool ret = device->irTrackingCamera()->setExposureTime(time);
        LOG_DEBUG("setExposureTime return = %d",ret);
        return ret;
    }


    /**
        * @brief 获取头盔温度 单位摄氏度
        *
        */
    int xv_get_glass_temperature(){
        if (device && device->deviceStatus()) {
            return xv_get_glass_tem();
        } else
            return -1;
    }
    void xv_register_device_status_callback() {

        if (device && device->deviceStatus()) {

#ifdef ANDROID
            LOG_DEBUG( "eddy xslam_register_device_status_callback entry");
#endif
            std::vector<unsigned char> command;
            command.push_back(0x02);
            command.push_back(0xfe);
            command.push_back(0x36);
            command.push_back(0x1);
            std::vector<unsigned char> result;
            bool res1 = device->hidWriteAndRead(command, result);
            device->deviceStatus()->registerCallback([](const std::vector<unsigned char>& deviceStatus){
                if(deviceStatus.size()>0){

                    int length =  deviceStatus.size();
                    for(int i = 11; i < 17; i ++){
                        std::cout << (int)deviceStatus[i] << " ";
                    }
                    LOG_DEBUG("eddy cpu_temp = %d", (int)deviceStatus[17]);
                }
                LOG_DEBUG("eddy deviceStatus size is = %d",deviceStatus.size());

            });
        }
    }

    /**
     * @brief 获取盒子cpu温度，需要系统权限应用
     *
     * @return 返回cpu温度 单位摄氏度

     */

    float xv_getCPUTemperature() {
        const std::string base_path = "/sys/class/thermal/";
        DIR* dir = opendir(base_path.c_str());
        if (!dir) return -1.0f;

        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            if (strncmp(entry->d_name, "thermal_zone", 12) != 0) continue;

            std::string zone_path = base_path + entry->d_name;
            std::ifstream type_file(zone_path + "/type");
            std::string type;
            if (type_file && std::getline(type_file, type)) {
                if (type.find("cpu") != std::string::npos) {
                    std::ifstream temp_file(zone_path + "/temp");
                    int temp_millicelsius = 0;
                    if (temp_file && (temp_file >> temp_millicelsius)) {
                        closedir(dir);
                        return temp_millicelsius / 1000.0f;
                    }
                }
            }
        }

        closedir(dir);
        return -1.0f;
    }
}
//被动调用 用于环境初始化
extern "C"
JNIEXPORT void JNICALL
Java_com_xv_aitalk_XvInterface_doInitEnv(JNIEnv *env, jclass clazz) {
    // TODO: implement doInitEnv()
    env->GetJavaVM(&jvm);
    s_XvInterfaceClass = reinterpret_cast<jclass>(env->NewGlobalRef(clazz));
    s_startRecognizerFromPath= env->GetStaticMethodID(s_XvInterfaceClass, "startRecognizerFromPath", "(Ljava/lang/String;)V" );

    // 2. 获取 startRecognizerFromPath 的方法 ID
    s_startRecognizerFromByte = env->GetStaticMethodID(s_XvInterfaceClass, "startRecognizerFromByte", "([B)V");
    s_switchRecognizeSource = env->GetStaticMethodID(s_XvInterfaceClass, "switchAudioSource", "(I)V");
    // 获取 getVolumeCurr 方法 ID
    s_getVolumeCurrMethod = env->GetStaticMethodID(s_XvInterfaceClass, "getVolumeCurr", "()I");
    // 获取 getVolumeMax 方法 ID
    s_getVolumeMaxMethod = env->GetStaticMethodID(s_XvInterfaceClass, "getVolumeMax", "()I");
    // 获取 adjustVolume 方法 ID
    s_adjustVolumeMethod = env->GetStaticMethodID(s_XvInterfaceClass, "adjustVolume", "(II)I");
    // 获取静态方法 playAudio
    s_playAudioMethod = env->GetStaticMethodID(s_XvInterfaceClass, "playAudioFromSdCard", "(Ljava/lang/String;)V");
    // 获取静态方法 stopAudio
    s_stopAudioMethod = env->GetStaticMethodID(s_XvInterfaceClass, "stopAudio", "()V");



#ifdef ANDROID
    __android_log_print(ANDROID_LOG_WARN, "xv#wrapper",
                        "eddy Java_com_xv_aitalk_XvInterface_doInitEnv");
#endif
}
/**
  * @brief 语音识别结果
  * @param result:识别结果
  * @param sc:置信度
  * @param id:定义的语义id
  */
extern "C"
JNIEXPORT void JNICALL
Java_com_xv_aitalk_XvInterface_doResult(JNIEnv *env, jclass clazz, jstring result, jint sc, jint id) {
    // TODO: implement doResult()
    const char *str = env->GetStringUTFChars(result, nullptr);

#ifdef ANDROID
    __android_log_print(ANDROID_LOG_WARN, "xv#wrapper",
                        "eddy Java_org_xv_aitalk_XvInterface_doResult = %s,sc = %d,id = %d", str,(int)sc,(int)id);

#endif
}

