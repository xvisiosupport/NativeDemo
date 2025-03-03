#ifndef UNITY_WRAPPER_H
#define UNITY_WRAPPER_H
#pragma once

#ifdef WIN32
#define EXPORT_API __declspec(dllexport)
#else
#define EXPORT_API __attribute__((visibility("default")))
#endif

#include <string>
#include <xv-types.h>
#include "third-party/qvr/svrConfig.h"
//#include "XvXR_Client_Base.h"
#include <xv-sdk-ex.h>
#include <xv-sdk.h>
//#include "colors.h"
#ifdef ANDROID

#include <jni.h>

#endif
#define QRCODETAG
#define XV_GAZE
// #define XV_EYE_TRACK
#define ISMTK 0

#define XV_STM

#define XV_GPS
#define XV_JOYSTICK
// #define XV_MULTY_DEVICE_RGB
#define XV_GAZE_CALIB
#define XV_GAZE_CALIB_NEW
// #define XV_MULTY_DEVICE_RGB
#define XV_IRIS
#define ZHIYUAN_CAMERAS
#define FE_RECTIFICATION
// #define FE_RECTIFICATION_SEUCM_UPDATE
#ifdef FE_RECTIFICATION

#include <xv-sdk-private.h>

#endif

#ifdef XV_EYE_TRACK
struct XvPubData
{
    int etIndex;
    float gaze_x;
    float gaze_y;
    float pupl_x;
    float pupl_y;
    long long ts;
};

#endif

typedef void (*cb_data)(unsigned char *data, int len);

/** \cond */

struct Point
{
    double x;
    double y;
    double z;
};
struct Vector2
{
    float x;
    float y;
};

struct Vector3
{
    float x;
    float y;
    float z;
};
struct Vector3uint
{
    unsigned int x;
    unsigned int y;
    unsigned int z;
};

struct Vector4
{
    float x;
    float y;
    float z;
    float w;
};

struct RotationPoint
{
    float x;
    float y;
    float z;
    float w;
};

struct XslamSkeleton
{
    int size;

    //    Vector3 joints[2][21];
    Vector3 joints_ex[52];
    RotationPoint poseData[52];
    float scale[2];
    int status[2];
    double timestamp[2];
    double fisheye_timestamp;
    long long int dataFetchTimeMs;
    long long int dataTimeStampMs;

};
struct XslamSkeletonDebug
{
    int size;
    Vector3 joints_ex[52];
    RotationPoint poseData[52];
    long long int dataFetchTimeMs;
    long long int dataTimeStampMs;

    float hand_uv_leftcam[42 + 42];
    float hand_uv_rightcam[42 + 42];
    double handpose_data[37 + 37];

};
/* struct XslamGesture
{
    int index[2];
    xv::keypoint position[2];
    xv::keypoint slamPosition[2];
    long long int hostTimestamp;
    long long int edgeTimestampUs;
    float distance;
    float confidence;
}; */

struct XslamSurface
{
    unsigned int mapId;
    unsigned int version;
    unsigned int id;

    unsigned int verticesSize;
    Vector3 *vertices;
    Vector3 *vertexNormals;

    unsigned int trianglesSize;
    Vector3uint *triangles;

    Vector3 *textureCoordinates; //!< one per vertex
    unsigned int textureWidth;
    unsigned int textureHeight;
    //  const unsigned char *textureRgba; //!< row major
};
struct device_status
{
    int status[10];
    // char *id;
};
struct xplan_package
{
    unsigned int points_nb;
    char idStr[32];
    Vector3 normal; // X Y Z
    unsigned int verticesSize;
    Vector3 *vertices;
    unsigned int trianglesSize;
    Vector3uint *triangles;
    double distance; // distance

    // char *id;
};
struct GazePoint
{
    unsigned int gazeBitMask;   //!< gaze bit mask, identify the six data below are valid or invalid.
    Vector3 gazePoint;          //!< gaze point, x and y are valid, z default value is 0, x and y scope are related to the input calibration point, not fixed.
    Vector3 rawPoint;           //!< gaze point before smooth, x and y are valid, z default value is 0, x and y scope are as above.
    Vector3 smoothPoint;        //!< gaze point after smooth, x and y are valid, z default value is 0, x and y scope are as above.
    Vector3 gazeOrigin;         //!< origin gaze center coordinate.
    Vector3 gazeDirection;      //!< gaze direction.
    float re;                   //!< gaze re value, confidence level.
    unsigned int exDataBitMask; //!< reserved data.
    // float exData[32];           //!<reserved data.
};
struct PupilINFO
{
    unsigned int pupilBitMask; //!< pupil bit mask, identify the six data below are valid or invalid.
    Vector2 pupilCenter;       //!< pupil center(0-1), the coordinate value of pupil center in the image, normalization value, image height and width is 1.
    float pupilDistance;       //!< the distance between pupil and camera(mm)
    float pupilDiameter;       //!< pupil diameter, pupil long axis value(0-1), the ratio of the pixel value of the long axis size of the pupil ellipse to the image width, normalization value.
    float pupilDiameterMM;     //!< pupil diameter, pupil long axis value(mm).
    float pupilMinorAxis;      //!< pupil diameter, pupil minor axis value(0-1), the ratio of the pixel value of the minor axis size of the pupil ellipse to the image width, normalization value.
    float pupilMinorAxisMM;    //!< pupil diameter, pupil minor axis value(mm).
};
struct ObjectData
{

    Vector3 point;
    int blobIndex = -1;
    int typeID = -1;
    char *type;
    double x = 0;
    double y = 0;
    double width = 0;
    double height = 0;
    double confidence = 0.0f;
    int pointsSize = 0;
    Vector3 *keypoints;
};
#define IRIS_FEATURE_DATA_LEN (6532)
struct IrisFeature
{
    char data[IRIS_FEATURE_DATA_LEN];
    int size;
};
struct BeiDouGPSDataEx
{
    int data_ready_flag; // 0:invalid   1:valid
    double lat_data;     // Latitude data
    int latdir;          // latdir == 1(lat_data  is north latitude)  latdir == 2(lat_data  is south latitude)
    double lon_data;     // Longitude data
    int londir;          // londir == 1(lon_data is east longitude)  londir == 2(lon_data is west longitude)
    int satellite_num;   // Number of satellites
    int mode;            // mode == 1 (GPS + BDS)  mode == 2(BDS)
};

typedef void (*fn_iris_callback)(const char *name, IrisFeature feature);

typedef void (*fn_iris_identity_callback)(const char *name);

#ifdef XV_GAZE
struct XslamGazeData
{
    unsigned long long timestamp; //!< timestamp.
    int recommend;
    GazePoint gazePoints[2];
    PupilINFO pupilInfo[2];
    xv::XV_ET_EYE_DATA_EX metaData;
};

typedef void (*fn_gaze_callback)(xv::XV_ET_EYE_DATA_EX gazedata);

#endif

typedef void (*fn_surface_callback)(XslamSurface *surface, int size);

typedef void (*fn_tof_plan_callback)(xplan_package *planes, int size);

typedef void (*fn_skeleton_callback)(XslamSkeleton skeleton);

typedef void (*fn_skeleton_debug_callback)(XslamSkeletonDebug skeleton);

typedef void (*fn_gesture_callback)(xv::GestureData gesture);

typedef void (*cslam_switched_callback)(int map_quality);

typedef void (*cslam_localized_callback)(float percent);

typedef void (*cslam_saved_callback)(int status_of_saved_map, int map_quality);

typedef void (*device_stream_callback)(xv::Event event);

typedef void (*device_beidou_gps_callback)(BeiDouGPSDataEx data);

typedef void (*device_status_callback)(const unsigned char *deviceStatus, int length);

typedef void (*device_status_callback_ex)(device_status deviceStatus);

typedef void (*rknn_objectdetect_callback)(ObjectData *res, int size);

struct Matrix4x4
{
    float m[16];
};
/** \endcond */

/** \cond */
struct Quaternion
{
    double x, y, z, w;
};

struct Orientation
{
    long long hostTimestamp = 0;   //!< Timestamp in µs read on host
    long long deviceTimestamp = 0; //!< Timestamp in µs read on the device
    Quaternion quaternion;         //!< Absolute quaternion (3DoF)
    double roll = 0.0;             //!< Absolute roll euler angle (3DoF)
    double pitch = 0.0;            //!< Absolute pitch euler angle (3DoF)
    double yaw = 0.0;              //!< Absolute yaw euler angle (3DoF)
    double angularVelocity[3];     //!< Instantaneous angular velocity (radian/second)
};
/** \endcond */

/** \cond */

/**
 * @brief Rotation and translation structure
 */
struct transform
{
    double rotation[9];    //!< Rotation matrix (row major)
    double translation[3]; //!< Translation vector
};
struct slammap
{
    Vector3 vertice;
};
/**
 * @brief Polynomial Distortion Model
 */
struct pdm
{
    double K[11];
    /**
    Projection and raytrace formula can be found here:
    https://docs.opencv.org/3.4.0/d4/d94/tutorial_camera_calibration.html

    K[0] : fx
    K[1] : fy
    K[2] : u0
    K[3] : v0
    K[4] : k1
    K[5] : k2
    K[6] : p1
    K[7] : p2
    K[8] : k3
    K[9] : image width
    K[10] : image height
*/
};

/**
 * @brief Unified camera model
 */
struct unified
{
    double K[7];
    /**
  Projection and raytrace formula can be found here:
  1.  C. Geyer and K. Daniilidis, “A unifying theory for central panoramic systems and practical applications,” in Proc. 6th Eur. Conf. Comput. Vis.
II (ECCV’00), Jul. 26, 2000, pp. 445–461
  or
  2. "J.P. Barreto. General central projection systems, modeling, calibration and visual
servoing. Ph.D., University of Coimbra, 2003". Section 2.2.2.

  K[0] : fx
  K[1] : fy
  K[2] : u0
  K[3] : v0
  K[4] : xi
  K[5] : image width
  K[6] : image height

  More details,
  Projection:
    The simplest camera model is represented by projection relation:    p = 1/z K X
    where p=(u v)^T is an image point, X = (x y z)^T is a spatial point to be projected
    and K is a projection matrix: K = (fx 0 u0; 0 fy v0).

    The distortion model is added in the following manner.
    First we project all the points onto the unit sphere S
        Qs = X / ||X|| = 1/rho (x y z)   where rho = sqrt(X^2+Y^2+Z^2)
    and then we apply the perspective projection with center (0 0 -xi)^T of Qs onto plan image
        p = 1/(z/rho + xi) K (x/rho  y/rho).

  Back-projection/raytrace:
    The normalized coordinate of a pixel is (x y 1)^1.
    We know that a line joining this normalized point and the projection center intersects the unit sphere
    at a point Qs. This point is defined as
        Qs = (eta*x  eta*y  eta-xi)
    where scale factor    eta = (xi + sqrt(1 + (x^2+y^2)(1-xi^2))) / (x^2+y^2+1).
*/
};

struct unified_calibration
{
    transform extrinsic;
    unified intrinsic;
};

struct stereo_fisheyes
{
    unified_calibration calibrations[2];
};

struct pdm_calibration
{
    transform extrinsic;
    pdm intrinsic;
};

struct stereo_pdm_calibration
{
    pdm_calibration calibrations[2];
};

struct rgb_calibration
{
    transform extrinsic;
    pdm intrinsic1080; //!< 1920x1080
    pdm intrinsic720;  //!< 1280x720
    pdm intrinsic480;  //!< 640x480
};

struct imu_bias
{
    double gyro_offset[3];
    double accel_offset[3];
};

struct pointer_3dpose
{
    Vector2 rgbPixelPoint;
    Vector3 pointerPose;
    bool isValid;
};

struct hand_keypoints
{
    Vector3 point[21];
};
struct GestureData
{
    int index[2] = {-1,
                    -1};                                                       //!< Index array for hands gesture, max size is two, default is -1 means invalid.
    Vector3 position[2];                                                       //!< Position array for hand gesture,  max size is two, 2D points, z isn't used by default.
    Vector3 slamPosition[2];                                                   //!< Convert rgb points into slam points, Position array for hand gesture,  max size is two.
    double hostTimestamp = std::numeric_limits<double>::infinity();            //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`).
    std::int64_t edgeTimestampUs = (std::numeric_limits<std::int64_t>::min)(); //!< timestamp of the physical measurement (in microsecond based on edge clock).
    float distance;                                                            //!< reserved, dynamic gesture movement distance.
    float confidence;                                                          //!< reserved, gesture confidence.
};

struct TagData
{
    int tagID;
    Vector3 position;
    Vector3 orientation;
    Vector4 quaternion;
    long long edgeTimestamp;
    double hostTimestamp;
    double confidence;
};

struct DetectData
{
    int tagID;
    Vector3 position;
    Vector3 orientation;
    Vector4 quaternion;
    std::int64_t edgeTimestamp;
    double hostTimestamp;
    float confidence;
    char qrcode[512];
};

struct QrCodeData
{
    char qrcode[512];

};

struct TagArray
{
    DetectData detect[64];
};

struct GpsData
{
    unsigned char gpsData[80];
};

struct WirelessPos
{
    char type;
    Vector3 position;
    Vector4 quaternion;
    float confidence;
    char keyTrigger;
    char keySide;
    short rocker_x;
    short rocker_y;
    char key;
};
struct WirelessControllerDeviceInfo
{
    int battery;            // 1 bytes
    int temp;               // 1 bytes
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
struct GazeCalibStatus
{
    /** The status of API CalibrationEnter */
    int enter_status;

    /** The status of API CalibrationCollect */
    int collect_status;

    /** The status of API CalibrationSetup */
    int setup_status;

    /** The status of API CalibrationComputeApply */
    int compute_apply_status;

    /** The status of API CalibrationLeave */
    int leave_status;

    /** The status of API CalibrationReset */
    int reset_status;
};

typedef void (*wireless_pose_callback)(ControllerPos *pose);

typedef void (*wireless_scan_callback)(const char *name, const char *mac);

typedef void (*wireless_upload_callback)(bool ret);

typedef void (*wireless_state_callback)(const char *name, const char *mac, int state);

/** \endcond */

extern "C"
{

    /**
     * UnityWrapper provide C++ functions to Unity
     */
    namespace UnityWrapper
    {

        enum SlamType
        {
            Edge = 0,
            Mixed = 1,
            EdgeFusionOnHost
        };
        enum PLATFORM
        {
            LINUX_CPU = 0,
            ANDROID_GPU,
            ANDROID_DSP,
            ANDROID_NPU
        };
        enum RgbSource
        {
            UVC = 0,
            VSC = 1
        };

        // Should same to XSlam::VSC::RgbResolution
        enum RgbResolution
        {
            UNDEF = -1,        ///< Undefined
            RGB_1920x1080 = 0, ///< RGB 1080p
            RGB_1280x720 = 1,  ///< RGB 720p
            RGB_640x480 = 2,   ///< RGB 480p
            RGB_320x240 = 3,   ///< RGB QVGA
            RGB_2560x1920 = 4, ///< RGB 5m
            TOF = 5,           ///< TOF YUYV 224x172
        };

        // If not ALL, must specify channels and streams to use
        enum Component
        {
            COM_ALL = 0xFFFF,
            COM_IMU = 0x0001,
            COM_POSE = 0x0002,
            COM_STEREO = 0x0004,
            COM_RGB = 0x0008,
            COM_TOF = 0x0010,
            COM_EVENTS = 0x0040,
            COM_CNN = 0x0080,

            COM_HID = 0x0100,
            COM_UVC = 0x0200,
            COM_VSC = 0x0400,
            COM_SLAM = 0x0800,
            COM_EDGEP = 0x1000,
        };
        /*       XvxrClientBase* getXvXRClient();
               void destroy(XvxrClientBase* p);*/
        EXPORT_API bool xslam_ready();

        EXPORT_API void xslam_set_start_mode(int mode, bool isroot);

        EXPORT_API int xslam_get_start_mode();

        EXPORT_API bool xslam_init();

        EXPORT_API bool xslam_init_with_fd(int fd);

        EXPORT_API bool xslam_init_components(int components);

        EXPORT_API bool xslam_init_components_with_fd(int fd, int components);

        EXPORT_API bool xslam_uninit();

#ifdef ANDROID

        EXPORT_API std::shared_ptr<xv::Device> xslam_get_device();

#endif // ANDROID

        EXPORT_API void xslam_register_config_param(char *name, int defaultVaule);

        EXPORT_API void xslam_set_login_open(int mode, bool isroot);

        EXPORT_API void xslam_set_debug_log_open(int mode, bool isroot);

        EXPORT_API int xslam_get_debug_log_open();

        EXPORT_API int xslam_get_login_open();

        EXPORT_API void xslam_set_glass_ipd(int value, bool isroot);

        EXPORT_API int xslam_get_glass_ipd();

        EXPORT_API void xslam_set_glass_ipd2(int value, bool isroot);

        EXPORT_API int xslam_get_glass_ipd2();

        EXPORT_API void xslam_set_glass_Light(int value, bool isroot);

        EXPORT_API int xslam_get_glass_Light();

        EXPORT_API void xslam_set_box_channel(int value, bool isroot);

        EXPORT_API int xslam_get_box_channel();
        // SLAM
        EXPORT_API bool
        xslam_get_double_pose_old(xv::Pose &poseData1, xv::Pose &poseData2, double predictionTime);

        EXPORT_API bool
        xslam_get_double_pose(xv::Pose &poseData1, xv::Pose &poseData2, double predictionTime,
                              double syncDelTime);

        EXPORT_API bool xslam_set_sync(int Interval);

        EXPORT_API bool xslam_get_pose_angVal(double *poseData, double *angVel, double predictionTime);

        EXPORT_API bool
        xslam_SyncSet(int *frameDValue, int *Frequency, long long *staTime, char *isWriteST,
                      char *CurrentSync);

        EXPORT_API bool xslam_setStatus(bool *isUDCalibra);

        EXPORT_API bool xslam_GetDevicesCalibration(double devices[2][6], double translation[2][3],
                                                    double rotation[2][9]);

        EXPORT_API void xslam_slam_type(SlamType type);

        EXPORT_API bool
        xslam_get_pose_prediction(double *poseData, long long *timestamp, double predictionTime);

        EXPORT_API bool xslam_get_pose_at(double *poseData, double timestamp);

        EXPORT_API bool xslam_get_pose_confidence(double *confidence);

        EXPORT_API bool xslam_get_pose_prediction_with_sensor(double *poseData, double *angularVelocity,
                                                              double *angularAcceleration,
                                                              long long *timestamp,
                                                              double predictionTime);

        EXPORT_API bool xslam_get_6dof(Vector3 *position, Vector3 *orientation, long long *timestamp);

        EXPORT_API bool xslam_get_transform_matrix(float *matrix, long long *timestamp, int *status);

        EXPORT_API bool xslam_get_transform(Matrix4x4 *matrix, long long *timestamp, int *status);
        // 3DOF
        EXPORT_API bool xslam_get_3dof(Orientation *o);

        EXPORT_API bool xslam_reset_slam();

        EXPORT_API int xslam_get_config_variable(const char *name);
        // RGB
        EXPORT_API bool xslam_set_rgb_source(RgbSource source);

        EXPORT_API int xslam_get_rgb_width();

        EXPORT_API int xslam_get_rgb_height();

        EXPORT_API bool
        xslam_get_rgb_image_YUV(unsigned char *data, int width, int height, double *timestamp);

        EXPORT_API bool
        xslam_get_rgb_image_RGBA(unsigned char *data, int width, int height, double *timestamp);

        EXPORT_API bool
        xslam_get_rgb_image_RGBA_flip(unsigned char *data, int width, int height, double *timestamp);
        EXPORT_API bool
        xslam_get_rgb_image_RGB(unsigned char *data, int width, int height, double *timestamp);

        EXPORT_API bool xslam_set_rgb_resolution(RgbResolution res);
        // TOF
        EXPORT_API int xslam_get_tof_width();

        EXPORT_API int xslam_get_tof_height();

        EXPORT_API bool xslam_get_tof_image(unsigned char *data, int width, int height);

        EXPORT_API bool xslam_get_depth_data(float *data);

        EXPORT_API bool xslam_get_color_depth_data(float *data);

        EXPORT_API bool xslam_get_cloud_data(Vector3 *cloud);

        EXPORT_API bool xslam_get_cloud_data_ex(Vector3 *cloud);

        EXPORT_API int xslam_tof_set_steam_mode(int cmd);

        EXPORT_API void xslam_tof_set_brightness(int brightness);

        EXPORT_API void xslam_tof_set_framerate(float framerate);

        EXPORT_API void xslam_tof_set_resolution(int resolution);
        // Stereo
        EXPORT_API int xslam_get_stereo_width();

        EXPORT_API int xslam_get_stereo_height();
        EXPORT_API void xslam_stereo_set_framerate(float framerate);
        EXPORT_API void xslam_stereo_set_exposure(int aecMode, int exposureGain, float exposureTimeMs);

        EXPORT_API bool
        xslam_get_left_image(unsigned char *data, int width, int height, double *timestamp);

        EXPORT_API bool
        xslam_get_right_image(unsigned char *data, int width, int height, double *timestamp);

        EXPORT_API int xslam_get_stereo_max_points();

        EXPORT_API bool xslam_get_left_points(Vector2 *points, int *size);

        EXPORT_API bool xslam_get_right_points(Vector2 *points, int *size);

        // IMU
        EXPORT_API bool
        xslam_get_imu(Vector3 *accel, Vector3 *gyro, Vector3 *magn, long long *timestamp);

        EXPORT_API bool xslam_get_imu_array(Vector3 *imu, double *timestamp);

        // Event
        EXPORT_API void xslam_stop_event_stream();

        EXPORT_API int xslam_start_event_stream(device_stream_callback cb);

        EXPORT_API bool xslam_get_event(int *type, int *state, long long *timestamp);
        // Gps
        EXPORT_API int xslam_start_beidou_stream(device_beidou_gps_callback cb, int mode);
        EXPORT_API bool xslam_prime_lens();
        EXPORT_API bool xslam_zoom_lens();
        // light

        EXPORT_API bool xslam_start_light_preception();

        EXPORT_API bool xslam_stop_light_preception();
        // device status
        EXPORT_API int xslam_register_device_status_callback(device_status_callback cb);

        EXPORT_API void xslam_set_device_status_callback(device_status_callback_ex cb);

        // Configuration
        EXPORT_API bool xslam_set_aec(int p1, int p2, int p3, int p4, int p5, int p6);

        EXPORT_API bool
        xslam_set_imu_configuration(int mode, int stereoOffsetUs, int edgePredUs, bool edgeImuFusion,
                                    bool imuSyncedWithinEdgePacket);

        EXPORT_API bool xslam_set_flip(bool flip);

        EXPORT_API bool
        xslam_set_post_filter(bool enabled, float rotationParam, float translationParam);

        EXPORT_API bool
        xslam_set_imu_fusion(int imuFusionMode, bool synced, float delay, float prediction);

        // Switches
        EXPORT_API void xslam_start_imu();

        EXPORT_API void xslam_stop_imu();

        EXPORT_API void xslam_start_rgb_stream();

        EXPORT_API void xslam_stop_rgb_stream();

        EXPORT_API void xslam_start_tof_stream();

        EXPORT_API void xslam_start_tofir_stream();

        EXPORT_API bool xslam_get_tofir_image(unsigned char *data, int width, int height);

        EXPORT_API void xslam_start_sony_tof_stream(int libmode, int resulution, int fps);

        EXPORT_API void xslam_stop_tof_stream();

        EXPORT_API void xslam_start_stereo_stream();

        EXPORT_API void xslam_stop_stereo_stream();

        EXPORT_API void xslam_start_speaker_stream();

        EXPORT_API void xslam_stop_speaker_stream();

        // HID read and write z
        EXPORT_API bool xslam_write_hid(unsigned char *wdata, int wlen);

        EXPORT_API bool
        xslam_write_and_read_hid(unsigned char *wdata, int wlen, unsigned char *rdata, int rlen);

        EXPORT_API bool
        xslam_hidWriteAndRead(std::vector<unsigned char> &command, std::vector<unsigned char> &result);
        // read Calibrations
        EXPORT_API bool readIMUBias(imu_bias *bias);

        EXPORT_API bool
        xslam_readStereoFisheyesCalibration(std::vector<xv::CalibrationEx> &m_FECalibration);

        EXPORT_API bool
        readStereoFisheyesCalibration(stereo_fisheyes *calib, int *imu_fisheye_shift_us);

        EXPORT_API bool readDisplayCalibration(pdm_calibration *calib);

        EXPORT_API bool readToFCalibration(pdm_calibration *calib);

        EXPORT_API bool readRGBCalibration(rgb_calibration *calib);

        EXPORT_API bool readStereoFisheyesPDMCalibration(stereo_pdm_calibration *calib);

        EXPORT_API bool readStereoDisplayCalibration(stereo_pdm_calibration *calib);

        EXPORT_API bool updateCalibra(double distance);

        // CNN and hand
        EXPORT_API ObjectData *xslam_get_objects(int *num);

        EXPORT_API int xslam_start_skeleton();

        EXPORT_API bool xslam_stop_skeleton(int id);

        EXPORT_API int xslam_start_slam_skeleton();

        EXPORT_API bool xslam_stop_slam_skeleton(int id);

        EXPORT_API int xslam_start_gesture();

        EXPORT_API int xslam_start_gesture_ex(const std::string &path);

        EXPORT_API void xslam_stop_gesture();

        EXPORT_API int xslam_start_dynamic_gesture();

        EXPORT_API bool xslam_stop_dynamic_gesture(int id);

        EXPORT_API void xslam_set_object_ui_on(bool isshow);

        EXPORT_API bool xslam_set_hand_config_path_s(const char *path);

        EXPORT_API bool xslam_get_hand_keypoints(hand_keypoints *keypoints, int type);

        EXPORT_API bool xslam_get_gesture(GestureData *gesture);

        EXPORT_API bool xslam_get_dynamic_gesture(GestureData *gesture);

        EXPORT_API bool xslam_set_cnn_model(const char *path);

        EXPORT_API bool xslam_set_cnn_descriptor(const char *path);

        EXPORT_API bool xslam_set_cnn_model_s(const std::string &path);

        EXPORT_API bool xslam_set_cnn_descriptor_s(const std::string &path);

        EXPORT_API bool xslam_set_cnn_source(int source);

        EXPORT_API int xslam_start_cnn();

        EXPORT_API void xslam_stop_cnn();

        EXPORT_API int startObjectDetectRkNN(rknn_objectdetect_callback cb);

        EXPORT_API bool xslam_set_rknn_model(const char *path);

        EXPORT_API void stopObjectDetectRkNN(int id);

        EXPORT_API bool xslam_get_hand_landmark_xyz(Vector3 *result, int *type, Matrix4x4 *matrix4);

        EXPORT_API void xslam_read_version(unsigned char *version);

        EXPORT_API void xslam_read_device_version(unsigned char *version);

        // EXPORT_API bool start_skeleton(int type, fn_skeleton_callback cb);
        EXPORT_API int xslam_start_skeleton_with_cb(int type, fn_skeleton_callback cb);

        EXPORT_API int xslam_start_skeleton_debug_with_cb(fn_skeleton_debug_callback cb);

        EXPORT_API bool xslam_stop_slam_skeleton_with_cb(int type, int id);

        EXPORT_API int xslam_start_gesture_with_cb(int type, fn_gesture_callback cb);

        EXPORT_API bool start_gesture(int type, fn_gesture_callback cb);

        EXPORT_API int xslam_start_gesture_ex_with_cb(fn_gesture_callback cb);

        EXPORT_API void xslam_set_gesture_platform(int platform);

        EXPORT_API void xslam_set_gesture_ego(bool ego);

        EXPORT_API int xslam_start_skeleton_ex_with_cb(fn_skeleton_callback cb);
        // Audio
        EXPORT_API int xslam_transfer_speaker_buffer(const unsigned char *data, int len);

        EXPORT_API bool xslam_play_sound(const unsigned char *data, int len);

        EXPORT_API bool xslam_play_sound_file(const char *path);

        EXPORT_API bool xslam_is_playing();

        EXPORT_API void xslam_stop_play();

        EXPORT_API bool xslam_set_mic_callback(cb_data cb);

        EXPORT_API void xslam_unset_mic_callback();

        // planedetect_stereo
        EXPORT_API bool xslam_start_detect_plane_from_stereo();

        EXPORT_API bool xslam_get_plane_from_stereo(unsigned char *data, int *len);

        EXPORT_API bool xslam_stop_detect_plane_from_stereo();
        // planedetect_tof
        EXPORT_API bool xslam_start_detect_plane_from_tof(fn_tof_plan_callback cb);

        EXPORT_API bool xslam_start_detect_plane_from_tof_nosurface();

        EXPORT_API bool xslam_get_plane_from_tof(unsigned char *data, int *len);

        EXPORT_API bool xslam_stop_detect_plane_from_tof();
        // map
        EXPORT_API bool
        xslam_load_map_and_switch_to_cslam(const char *mapPath, cslam_switched_callback csc,
                                           cslam_localized_callback clc);

        EXPORT_API bool
        xslam_save_map_and_switch_to_cslam(const char *mapPath, cslam_saved_callback csc,
                                           cslam_localized_callback clc);

        EXPORT_API slammap *xslam_get_slam_map(int *len);

        EXPORT_API bool xslam_start_map();

        EXPORT_API bool xslam_stop_map();

        EXPORT_API void xslam_setLogLevel(int level);
        // mesh
        EXPORT_API int
        xslam_start_surface_callback(bool enableSuface, bool enableTexturing, fn_surface_callback cb);
        EXPORT_API void xslam_stop_surface_callback(int callbackId);
        EXPORT_API bool xslam_enable_surface_reconstruction(bool enable, int callbackId);


        EXPORT_API fn_surface_callback xslam_get_surface_cb();
        // april tag
        EXPORT_API int
        xslam_start_detect_tags(const char *tagFamily, double size, TagArray *tagsArray, int arraySize);

        EXPORT_API void xslam_stop_detect_tags();
        // rgb april tag
        EXPORT_API int
        xslam_start_rgb_detect_tags(const char *tagFamily, double size, TagArray *tagsArray,
                                    int arraySize);

        EXPORT_API void xslam_stop_rgb_detect_tags();
        // slam april tag

        EXPORT_API int
        xslam_detect_tags(const char *tagFamily, double size, TagArray *tagsArray, int arraySize);
        // slam qrcode tag
        EXPORT_API int
        xslam_start_detect_qrcode(const char *tagFamily, double size, QrCodeData *data, int arraySize);

        EXPORT_API int
        xslam_start_rgb_detect_qrcode(const char *tagFamily, double size, QrCodeData *data,
                                      int arraySize);
        // infrared stream
        EXPORT_API bool xslam_stop_infrared_stream(int id);

        EXPORT_API int xslam_start_infrared_stream();


        EXPORT_API bool xv_device_init();

        EXPORT_API bool xv_start_slam();

        EXPORT_API bool xv_stop_slam();

        EXPORT_API bool xv_get_6dof(Vector3 *position, Vector3 *orientation, Vector4 *quaternion,
                                    long long *edgeTimestamp, double *hostTimestamp,
                                    double *confidence);

        EXPORT_API bool
        xv_get_6dof_prediction(Vector3 *position, Vector3 *orientation, Vector4 *quaternion,
                               long long *edgeTimestamp, double *hostTimestamp, double *confidence,
                               double prediction);

        EXPORT_API bool xv_start_stereo();

        EXPORT_API bool
        xv_get_stereo_info(int *width, int *height, long long *edgeTimestamp, double *hostTimestamp,
                           unsigned int *dataSize);

        EXPORT_API bool xv_get_stereo_image(unsigned char *left, unsigned char *right);

        EXPORT_API bool xv_start_imu();

        EXPORT_API bool
        xv_get_imu(Vector3 *accel, Vector3 *gyro, long long *edgeTimestamp, double *hostTimestamp);

        EXPORT_API bool xv_start_rgb();

        EXPORT_API bool
        xv_get_rgb_info(int *width, int *height, long long *edgeTimestamp, double *hostTimestamp,
                        unsigned int *dataSize);

        EXPORT_API bool xv_get_rgb_image(unsigned char *data);

        EXPORT_API bool xv_start_tof();

        EXPORT_API bool
        xv_get_tof_info(int *width, int *height, long long *edgeTimestamp, double *hostTimestamp,
                        unsigned int *dataSize);

        EXPORT_API bool xv_get_tof_image(unsigned char *data);

        EXPORT_API bool xv_start_sgbm();

        EXPORT_API bool
        xv_get_sgbm_info(int *width, int *height, long long *edgeTimestamp, double *hostTimestamp,
                         unsigned int *dataSize);

        EXPORT_API bool xv_get_sgbm_image(unsigned char *data);

        EXPORT_API void xv_get_sn(char *sn, int bufferSize);
        //        EXPORT_API void xv_get_sn(std::string &sn);
        EXPORT_API void xv_get_fe_camera_intrinsics_param(int *trans_size, int *ucm_size);

        EXPORT_API void xv_get_fe_camera_intrinsics(transform *trans, xv::UnifiedCameraModel *ucm);

        EXPORT_API void xv_get_rgb_camera_intrinsics_param(int *trans_size, int *pdcm_size);

        EXPORT_API void
        xv_get_rgb_camera_intrinsics(transform *trans, xv::PolynomialDistortionCameraModel *pdcm);

        EXPORT_API void xv_get_tof_camera_intrinsics_param(int *trans_size, int *pdcm_size);

        EXPORT_API void
        xv_get_tof_camera_intrinsics(transform *trans, xv::PolynomialDistortionCameraModel *pdcm);

        EXPORT_API void xv_set_rgb_camera_resolution(xv::ColorCamera::Resolution resolution);

        EXPORT_API void xv_start_fe_tag_detector(std::string &tagDetectorId);

        EXPORT_API void xv_get_fe_tag_size(std::string &tagDetectorId, int *tagSize);

        EXPORT_API void xv_get_fe_tag_detection(TagData *tags);

        EXPORT_API void xslam_display_set_brightnesslevel(int level);

        EXPORT_API void xslam_rgb_set_brightness(int brightness);

        EXPORT_API void xslam_stereo_set_brightness(int brightness);

        EXPORT_API void xslam_tof_set_brightness(int brightness);

        EXPORT_API void xslam_tof_set_exposure(int aecMode, int exposureGain, float exposureTimeMs);

        void saveImages(const char *name, const unsigned char *data, int len);
        EXPORT_API int xslam_gaze_calibration_begin(int et_idx);
        EXPORT_API int xslam_gaze_set_conf_gaze(int et_idx, int s_idx, float *conf_gaze_o);
        EXPORT_API int xslam_gaze_calibration_end(int et_idx);
#ifdef XV_GAZE

        // gaze
        EXPORT_API bool xslam_get_gaze2rgb_xy(double *gaze, double *p2d);

        EXPORT_API bool xslam_start_gaze();

        EXPORT_API bool xslam_stop_gaze();

        EXPORT_API int xslam_set_gaze_callback(fn_gaze_callback cb);

        EXPORT_API bool xslam_unset_gaze_callback();

        EXPORT_API void xslam_gaze_set_config_path(const char *path);
        EXPORT_API bool xslam_get_gaze_status();
        EXPORT_API void xslam_gaze_enable_dump(bool enable);
        EXPORT_API void xslam_set_usr_eye_ready();
        EXPORT_API  void xslam_set_gaze_configs(int width,int height,float ipdDist,int srValue/*,int etWidth,int etHeight*/) ;
        //  #ifndef __linux__
        EXPORT_API int xslam_start_gaze_calibration(int points);

        EXPORT_API int
        xslam_start_calibration_point(int eye, int index, const xv::XV_ET_POINT_2D *point,
                                      xv::xv_ET_point_process_callback cb1, void *context1,
                                      xv::xv_ET_point_finish_callback cb2, void *context2);

        EXPORT_API int xslam_compute_calibration(int eye, xv::XV_ET_COEFFICIENT *out_coe);

        EXPORT_API int xslam_complete_calibration();

        EXPORT_API int xslam_cancel_gaze_calibration(int eye);

        EXPORT_API int
        xslam_set_default_calibration(int eye, float minX, float maxX, float minY, float maxY,
                                      const xv::XV_ET_COEFFICIENT *coe);
        //  #endif
#endif

        EXPORT_API bool
        xslam_set_exposure(int leftGain, float leftTimeMs, int rightGain, float rightTimeMs);

        EXPORT_API bool xslam_set_bright(int eye, int led, int brightness);

#ifdef XV_EYE_TRACK
        EXPORT_API void xslam_start_pub_eyetrack(int width, int height);
        EXPORT_API void xslam_stop_pub_eyetrack();
        EXPORT_API void xslam_get_pub_eyetrack(int eye, XvPubData *pub_data);
        EXPORT_API bool xslam_get_pub_dbg(int eye, unsigned char *data, int width, int height);
        EXPORT_API bool xslam_pub_calibration(int eye, int index, float x, float y);
#endif

        EXPORT_API void xv_start_rgb_pixel_pose();

        EXPORT_API void xv_stop_rgb_pixel_pose();

        EXPORT_API bool
        xslam_start_get_rgb_pixel_buff3d_pose(pointer_3dpose *pointerPose, Vector2 *rgbPixelPoint,
                                              int arraySize, double hostTimestamp, float radius);

        EXPORT_API bool
        xv_get_rgb_pixel_pose(Vector3 *pointerPose, Vector2 *rgbPixelPoint, double hostTimestamp,
                              float radius);

        EXPORT_API bool
        xv_get_rgb_pixel_3dpose(Vector3 *pointerPose, Vector2 *rgbPixelPoint, float radius);


#ifdef FE_RECTIFICATION

        EXPORT_API void xslam_start_fisheyes_rectification_thread();

        EXPORT_API void xslam_stop_fisheyes_rectification_thread();

        EXPORT_API bool xslam_get_fisheyes_rectification_thread();

        EXPORT_API bool
        xslam_get_fe_images_data(int *width, int *height, unsigned char *left, unsigned char *right,
                                 double *poseData);

        EXPORT_API bool
        xslam_get_fe_mesh_params(double *focal, double *baseline, int *camerasModelWidth,
                                 int *camerasModelHeight, double *leftPose, double *rightPose);

#endif

        EXPORT_API bool xvisio_start();

        EXPORT_API bool xvisio_stop();

#ifdef XV_GPS

        EXPORT_API bool set_rgb_source(RgbSource source);

        EXPORT_API bool set_rgb_resolution(RgbResolution res);

        EXPORT_API void start_rgb_stream();

        EXPORT_API void stop_rgb_stream();

        EXPORT_API int get_rgb_width();

        EXPORT_API int get_rgb_height();

        EXPORT_API bool
        get_rgb_image_RGBA_Byte(unsigned char *data, int width, int height, double *timestamp);

        EXPORT_API void xv_start_fe_tag_detector(std::string &tagDetectorId);

        EXPORT_API void xv_get_fe_tag_size(std::string &tagDetectorId, int *tagSize);

        EXPORT_API void xv_get_fe_tag_detection(TagData *tags);

        EXPORT_API void start_bdStream();

        EXPORT_API void stop_bdStream();

        EXPORT_API void get_gpsStream(GpsData *gps);

        EXPORT_API void start_laserDistanceStream();

        EXPORT_API void stop_laserDistanceStream();

        EXPORT_API void get_laserDistanceStream(xv::GPSDistanceData *gpsDistance);

#endif

#ifdef XV_JOYSTICK

        EXPORT_API void xv_wireless_start();

        EXPORT_API void xv_wireless_stop();

        EXPORT_API void xv_wireless_get_device_info(WirelessControllerDeviceInfo *device_info,int type);

        EXPORT_API void xv_wireless_register(wireless_pose_callback callback);

        EXPORT_API void xv_wireless_register_state(wireless_state_callback stateCallback);

        EXPORT_API void xv_wireless_scan(wireless_scan_callback callback);

        EXPORT_API void xv_wireless_connect(const char *name, const char *mac);

        EXPORT_API void xv_wireless_disconnect(const char *name, const char *mac);

        EXPORT_API void xv_wireless_upload_map(const char *path, wireless_upload_callback callback);

        EXPORT_API bool xv_wireless_pair(int type, const char *name, const char *mac);

        EXPORT_API bool xv_wireless_set_slam_type(int type, int mode);

        EXPORT_API int xv_wireless_get_slam_type(int type);

#endif
#ifdef ZHIYUAN_CAMERAS
        EXPORT_API bool xslam_switch_fusionCameras_state(bool isOpen);
        //state == 0 左 ; state == 1 右
        EXPORT_API bool xslam_switch_fusionCamerasLR(int state);
        EXPORT_API int xslam_start_RGB_L_ThermalFusionCamera();
        EXPORT_API void xslam_stop_RGB_L_thermalFusionCamera(int id);
        EXPORT_API int xslam_start_RGB_R_ThermalFusionCamera();
        EXPORT_API void xslam_stop_RGB_R_thermalFusionCamera(int id);
        EXPORT_API int xslam_start_colorCamera2();
        EXPORT_API int xslam_start_thermalCamera();
    EXPORT_API int xslam_start_irTrackingCamera();
        EXPORT_API void xslam_stop_colorCamera2(int id);
        EXPORT_API void xslam_stop_thermalCamera(int id);
    EXPORT_API void xslam_stop_irTrackingCamera(int id);
        EXPORT_API bool xslam_device_get_thermal(int width, int height, unsigned char *data);
        EXPORT_API bool xslam_device_get_irTracking(int width, int height, unsigned char *data1);
    //data1: rgb  data2: rgb2
        EXPORT_API bool xslam_device_get_rgba2(int width, int height, unsigned char *data2);
        EXPORT_API bool xslam_device_get_rgbfusitionCamera(int width, int height, unsigned char *rgbldata, unsigned char *rgbrdata);
#endif
#ifdef XV_MULTY_DEVICE_RGB
        EXPORT_API void xv_rgb_device_init(int fd);
        EXPORT_API bool xv_rgb_device_start_camera();
        EXPORT_API bool xv_rgb_device_stop_camera();
        EXPORT_API bool xv_rgb_device_get_rgba(int width, int height, unsigned char *data1, unsigned char *data2);
#endif

#ifdef XV_GAZE_CALIB

        EXPORT_API int xslam_gaze_calibration_enter();

        EXPORT_API int xslam_gaze_calibration_leave();

        EXPORT_API int xslam_gaze_calibration_collect(float x, float y, float z, int index);

        EXPORT_API int xslam_gaze_calibration_retrieve(const char *file);

        EXPORT_API int xslam_gaze_calibration_apply(const char *file);

        EXPORT_API int xslam_gaze_calibration_reset();

        EXPORT_API int xslam_gaze_calibration_compute_apply();

        EXPORT_API int xslam_gaze_calibration_setup();

        EXPORT_API int xslam_gaze_calibration_query_status(GazeCalibStatus *status);

#endif

#ifdef XV_IRIS

        EXPORT_API void xv_iris_init(JNIEnv *env, jobject context, jstring initLicence);

        EXPORT_API const char *xv_iris_active();

        EXPORT_API void xv_iris_init_licence(const char *licence);

        EXPORT_API void xv_iris_register(const char *name, fn_iris_callback callback);

        EXPORT_API void xv_iris_stop();

        EXPORT_API void
        xv_iris_start_identity(unsigned char *data, int size, fn_iris_identity_callback callback);

        EXPORT_API void xv_iris_stop_identity();

        EXPORT_API void xv_iris_setConfigPath(const char *path);

#endif

        EXPORT_API bool xv_eyetracking_support();

        EXPORT_API bool xv_eyetracking_start();

        EXPORT_API bool xv_eyetracking_stop();

        EXPORT_API bool
        xv_eyetracking_get_rgba(unsigned char *left, unsigned char *right, int *width_, int *height_);

        /*stm*/
#ifdef XV_STM

        EXPORT_API bool xv_stm_start();

        EXPORT_API bool xv_stm_stop();

        EXPORT_API bool xv_stm_get_stream(xv::TerrestrialMagnetismData *data);

#endif

        EXPORT_API bool
        xv_get_sync_rgb(unsigned char *data, int width, int height, double ts, double *timestamp);

        EXPORT_API bool
        xv_slam_set_tags(const char *tagFamily, double size, TagArray *tagsArray, int arraySize);

        EXPORT_API bool
        xv_slam_get_pose_in_tags(Vector3 *position, Vector4 *quaternion, double *confidence);

        EXPORT_API bool xslam_switch_audio(bool status);

        EXPORT_API bool xslam_switch_display();

        EXPORT_API bool xslam_switch_rgb();

    }
}
#endif