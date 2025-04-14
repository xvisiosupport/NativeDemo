#ifndef XV_WRAPPER_H
#define XV_WRAPPER_H
#pragma once

#ifdef WIN32
#define EXPORT_API __declspec( dllexport )
#else
#define EXPORT_API __attribute__ ((visibility ("default")))
#endif

#include <string>
#include "unity-wrapper.h"



extern "C" {

    /**
     * UnityWrapper provide C++ functions to Unity
     */
    namespace XvWrapper {
        struct FishEyeSeucmCalibration {
            double K[2][10];
            double rotation[2][9];
            double translation[2][3];
        };
        struct ControllerPos
        {
            int type;
            Vector3 position;
            Vector4 quaternion;
            float confidence;
            int keyTrigger;
            int keySide;
            int rocker_x;
            int rocker_y;
            int keyA;
            int keyB;
        };
        EXPORT_API void initXvDevice();
        EXPORT_API void setXvDevice(std::shared_ptr<xv::Device> device);
        EXPORT_API bool xv_test_get_6dof(double *poseData, long long *timestamp, double predictionTime);
        EXPORT_API bool xv_test();
        EXPORT_API void startRgbStream();
        EXPORT_API void stopRgbStream();
        EXPORT_API void xv_rgb_set_exposure(int aecMode, int exposureGain, float exposureTimeMs);
        EXPORT_API void stopTofStream();
        EXPORT_API void startTofStream();
        EXPORT_API bool setPmdTofIRFunction();
        EXPORT_API int xv_start_skeleton_ex_with_cb();
        EXPORT_API bool xv_readDisplayCalibration(pdm_calibration *calib);
        EXPORT_API bool xvReadStereoFisheyesCalibration();
        EXPORT_API bool xvReadStereoFisheyesCalibration();
        EXPORT_API int start_et_gaze_callback();
        EXPORT_API void xv_controller_register();
        EXPORT_API bool stm_start();
        EXPORT_API bool stm_stop();
        EXPORT_API bool xv_start_light_preception();
        EXPORT_API bool xv_stop_light_preception();
        EXPORT_API int xv_start_beidou_stream(int mode);
        EXPORT_API int xv_start_event_stream();
        EXPORT_API bool xv_stop_infrared_stream(int id);
        EXPORT_API int xv_start_infrared_stream();
        EXPORT_API void startTofIRStream();
        EXPORT_API bool xv_get_tofir_image(unsigned char *data, int width, int height);
        EXPORT_API bool xv_switch_audio(bool status);
        EXPORT_API bool xv_switch_display();
        EXPORT_API bool xv_switch_rgb();
        EXPORT_API void xv_recognize_from_local(const char *name);
        EXPORT_API void xv_audio_recognize_switch_source(int source);
        EXPORT_API bool xv_switch_fusionCamerasLR(int state);
        EXPORT_API bool xv_switch_fusionCameras_state(bool isOpen);
        EXPORT_API int xv_start_RGB_L_ThermalFusionCamera();
        EXPORT_API void xv_stop_RGB_L_thermalFusionCamera(int id);
        EXPORT_API int xv_start_RGB_R_ThermalFusionCamera();
        EXPORT_API void xv_stop_RGB_R_thermalFusionCamera(int id);
        EXPORT_API int xv_start_colorCamera2();
        EXPORT_API int xv_start_thermalCamera();
        EXPORT_API bool xv_read_thermal_calibration();
        EXPORT_API void xv_stop_colorCamera2(int id);
        EXPORT_API void xv_stop_thermalCamera(int id);
        EXPORT_API bool xv_switch_display_state(int eye_type, bool isOpen);
        EXPORT_API bool xv_set_electrochromic_level(int level);
        EXPORT_API bool xv_start_eyetracking();
        EXPORT_API void xv_read_eyetracking_calibrate();
        EXPORT_API bool xv_enable_ir_tracking_camera_led(int ledIndex,bool enable);
        EXPORT_API bool xv_read_irTracking_calibrationEx(bool isEx2);
        EXPORT_API bool xv_set_ir_tracking_camera_led_time(int ledIndex,float time);
        EXPORT_API bool xv_set_ir_tracking_exposureTime(int time);
        EXPORT_API int xv_start_irTrackingCamera();
        EXPORT_API int xv_stop_irTrackingCamera();
        EXPORT_API bool xv_save_ir_tracking_image();
        EXPORT_API int xv_start_irTrackingCamera2();
        EXPORT_API int xv_stop_irTrackingCamera2();
        EXPORT_API void xv_get_irTrackingCamera_params();
        EXPORT_API float xv_getCPUTemperature();
        EXPORT_API void xv_register_device_status_callback();
    }
}

#endif