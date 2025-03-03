#pragma once
#include "xv-sdk.h"
#include "xv-types.h"
namespace xv_c_interface {
class Device_interface;

std::map<std::string,std::shared_ptr<Device_interface>> getDevicesUntilTimeout(double timeOut = 0., const std::string& desc = "", xv::SlamStartMode slamStartMode = xv::SlamStartMode::Normal, xv::DeviceSupport deviceSupport = xv::DeviceSupport ::ONLYUSB);


/**
 * @brief Class to get tracking results and raw outputs with a connected device.
 *
 * This class is the main entry point of the API, it gives access to the device and algorithms. See xv::getDevices() to have an instance corresponding to a device.
 *
 * A device can have multiple components, accessible with member functions :
 * - #xv::Device::slam() : 6dof tracker doing the SLAM algorithm on host based on informations coming from device (stereo camera, IMU sensor, ..)
 * - #xv::Device::imuSensor() : sensor with at least 3-axis accelerometer and 3-axis gyrometer
 * - #xv::Device::fisheyeCameras(): typically 2 fisheye cameras used for Visual SLAM
 * - #xv::Device::tofCamera(): a depth camera sensor
 * - #xv::Device::edge(): used to run some algorithm directly on embedded device (typically Visual SLAM) when it is possible to choose between host and edge processing
 * - #xv::Device::display(): used to handle informations about the display (if device can display like a HMD)
 * - #xv::Device::objectDetector(): used to run and get results of the CNN object detector
 *
 * If a device does not support a component or doesn't have the component (for example ToF), the accessor function will return `null_ptr`.
 * The data streams and processings under a component are activated only if at least one callback is registerd to the component. If all the callbacks are unregistered then steams can be deactivated.
 */
class Device_interface {

public:


    /**
     * @brief Get informations (Serial Number, version ...) about the device.
     * @return A map with key and values of the informations.
     */
    virtual std::map<std::string, std::string> info() = 0;

    /**
     * @brief Get the SLAM component.
     */
    virtual std::shared_ptr<Slam> slam() = 0;

    /**
     * @brief Get the IMU sensor of the device.
     */
    virtual std::shared_ptr<ImuSensor> imuSensor() = 0;

    /**
     * @brief Get the event component.
     */
    virtual std::shared_ptr<EventStream> eventStream() = 0;

    /**
     * @brief Get the 3dof component.
     */
    virtual std::shared_ptr<OrientationStream> orientationStream() = 0;

    /**
     * @brief Get the stereo cameras component of the device.
     */
    virtual std::shared_ptr<FisheyeCameras> fisheyeCameras() = 0;

    /**
     * @brief Get the color camera component of the device.
     */
    virtual std::shared_ptr<ColorCamera> colorCamera() = 0;

    /**
     * @brief Get the ToF component of the device.
     */
    virtual std::shared_ptr<TofCamera> tofCamera() = 0;

    /**
     * @brief Get the SGBM component of the device.
     */
    virtual std::shared_ptr<SgbmCamera> sgbmCamera() = 0;

    /**
     * @brief Get the thermal component of the device.
     */
    virtual std::shared_ptr<ThermalCamera> thermalCamera() = 0;

    /**
     * @brief Get the eyetracking component of the device.
     */
    virtual std::shared_ptr<EyetrackingCamera> eyetracking() = 0;

    /**
     * @brief Get the gaze data of the device.
     */
    virtual std::shared_ptr<GazeStream> gaze() = 0;

    /**
     * @brief Get the iris data of the device.
     */
    virtual std::shared_ptr<IrisStream> iris() = 0;

   /**
     * @brief Get the gesture component.
     */
    virtual std::shared_ptr<GestureStream> gesture() = 0;

    /**
     * @brief Get the GPS data of the device.
     */
    virtual std::shared_ptr<GPSStream> gpsModule() = 0;

    /**
     * @brief Get the GPS distance data of the device.
     */
    virtual std::shared_ptr<GPSDistanceStream> gpsDistanceModule() = 0;

    /**
     * @brief Get the terrestrial magnetism data of the device.
     */
    virtual std::shared_ptr<TerrestrialMagnetismStream> terrestrialMagnetismModule() = 0;

    /**
     * @brief Get the MIC component of the device.
     */
    virtual std::shared_ptr<MicStream> mic() = 0;

    /**
     * @brief Get the speaker component of the device.
     */
    virtual std::shared_ptr<Speaker> speaker() = 0;

    /**
     * @brief Get the display component.
     */
    virtual std::shared_ptr<Display> display() = 0;

    /**
     * @brief Get the object detection component.
     */
    virtual std::shared_ptr<ObjectDetector> objectDetector() = 0;

    /**
     * @brief Get the device status component.
     */
    virtual std::shared_ptr<DeviceStatusStream> deviceStatus() = 0;

    /**
     * @brief Let device sleep.
     */
    virtual bool sleep(int level = 0) = 0;
    /**
     * @brief Wake up device.
     */
    virtual bool wakeup() = 0;

    /**
     * @brief Control device.
     */
    virtual bool control(const DeviceSetting &setting) = 0;

    /**
     * @brief Write HID control command and read result. HID command list:
     * 
     * - Refresh rate Setting command: 
     * 
     *      Header: 0x02, 0xfe, 0x20, 0x03, 0x01, 0x09
     * 
     *      Data: 0x02-72hz, 0x03-90hz
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xfe, 0x20, 0x03, 0x01, 0x09, 0x03}, result);
     @endcode
     *
     * - Breathing lamp chip standby setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x01
     * 
     *      Data: 0x01
     * 
     *      Comment: If you want to wake up the sensor, send mode set command to the needed mode.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x01, 0x01}, result);
     @endcode
     * 
     * - Monochrome breathing lamp cycle setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x02,0x01
     * 
     *      Data: 0x01-FF0000 color cycle, 0x02-FFF300 color cycle, 0x03-36FF48 color cycle, 0x04-62F1FF color cycle, 0x05-000CFF color cycle, 0x06-8000FF color cycle.
     * 
     *      Comment: The default rise time is 1.04s, storage time is 0.004s, fall time is 1.04s, closing time is 0.04s, and other times are 0. To change time, refer to breathing speed setting command.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x02, 0x01, 0x01}, result);
     @endcode
     * 
     * - 4-color breathing lamp cycle setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x02, 0x02
     * 
     *      Data: 0x04
     * 
     *      Comment: The default rise time is 1.04s, storage time is 0.004s, fall time is 1.04s, closing time is 0.04s, and other times are 0. To change time, refer to breathing speed setting command. 4-color breathing cycle: 00F5A9->00CBF5->0C61F5->D700EF.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x02, 0x02, 0x04}, result);
     @endcode
     * 
     * - Breathing lamp speed setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x05, 0x30-rise time and storage time setting, 0x05, 0x31-fall time and closing time setting.
     * 
     *      Data: The upper four bits are rise time or fall time, the lower four bits are storage time or closing time.
     * 
     *      Comment: Rise time or fall time: 0000-0s, 0001-0.13s, 0010-0.26s, 0011-0.38s, 0100-0.51s, 0101-0.77s, 0110-1.04s, 0111-1.6s, 1000-2.1s, 1001-2.6s, 1010-3.1s, 1011-4.2s, 1100-5.2s, 1101-6.2s, 1110-7.3s, 1111-8.3s. Storage time or closing time: 0000-0.04s, 0001-0.13s, 0010-0.26s, 0011-0.38s, 0100-0.51s, 0101-0.77s, 0110-1.04s, 0111-1.6s, 1000-2.1s, 1001-2.6s, 1010-3.1s, 1011-4.2s, 1100-5.2s, 1101-6.2s, 1110-7.3s, 1111-8.3s.
     * @code 
     example:

        std::vector<unsigned char> result;

        //set rise time to 1.04s
        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x05, 0x30, 0x60}, result);
     @endcode
     * 
     * - Constant breathing lamp light cycle switch setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x02, 0x03
     * 
     *      Data: 0x08-8 color constant light cycle, 0x14-20 color constant light cycle, 0x60-96 color constant light cycle
     * 
     *      Comment: The default change time is 0.1s. To change time, refer to constant breathing lamp light cycle switch speed setting command. 8 colors cycle: FF18FF->FF1010->FF8000->EFFF00 ->00FF00->00FFFF ->1858FF ->8A00FF. 20 colors cycle: E000FF->E80093->FF000D->E82400->FF5300->E87500->FFA400->E8B200->FFE100->E8E800->97FF00->2DE800->00FF2B->00EB7C->00FFE5->00B0EB->0069FF->0012EB->4000FF->8500EB. 96 colors cycle: FF00FE->F000FF->E000FF->CF00FE->C001FF->B000FF->A000FF->8F00FF->7F00FF->700FF->6000FF->5000FF->3F00FF->2F00FE->2001FF->1000FF->0000FE->0110FF->0020FF->0030FF->0140FF->0050FF->0060FF->0071FE->0080FF->0090FF->00A0FE->00AFFE->00C0FF->00D0FF->01E0FF->00F0FF->01FFFF->00FFF1->00FFE1->00FFD0->01FFC1->00FFB1->00FFA1->01FE91->00FE81->00FF71->00FF61->01FF51->00FF41->00FF31->00FE20->01FF11->00FF01->10FF01->1FFF00->30FF00->40FF01->50FF00->5FFF00->6FFF00->80FF00->90FF00->A0FF01->AFFF00->C0FF00->D0FF00->E0FF01->EFFF00->FFFF01->FFF001->FFE001->FED000->FFC000->FFB001->FF9F00->FF9000->FF7F00->FF7000->FF6100->FF5001->FF4001->FE3000->FF2000->FF1001->FE0000->FF0010->FF0020->FF0030->FF0140->FF0050->FF0060->FE0070->FF0080->FF0090->FF01A1->FE00B0->FF00C0->FF00D0->FF00E0->FF00F0
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x02, 0x03, 0x08}, result);
     @endcode
     *
     * - Constant breathing lamp light cycle switch speed setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x07, 0x03
     * 
     *      Data: 0~255
     * 
     *      Comment: The unit is 100ms, set single color light time, only effect in constant light cycle switch.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x07, 0x03, 0x00}, result);
     @endcode
     * 
     * - Breathing lamp real color mode setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x02
     * 
     *      Data: 0x04
     * 
     *      Comment: Composed of red, green and blue lights. The real color can be formed if the on-off time of the three lights is inconsistent.The default red color rise time is 1.04s, red color storage time is 2.1s, red color fall time is 1.04s, red color closing time is 2.6s, red color delay time is 0s. The default green color rise time is 1.04s, green color storage time is 2.1s, green color fall time is 1.04s, green color closing time is 1.6s, green color delay time is 1.04s. The default blue color rise time is 1.04s, blue color storage time is 2.1s, blue color fall time is 1.04s, blue color closing time is 0.004s, blue color delay time is 3.1s. To change time, refer to breathing lamp real color mode speed setting command.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x02, 0x04}, result);
     @endcode
     * 
     * - Breathing lamp real color mode speed setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x05, 0x30-red color rise time and storage time setting, 0x05, 0x31-red color fall time and closing time setting, 0x05, 0x32-red color delay time. 0x05, 0x35-blue color rise time and storage time setting, 0x05, 0x36-blue color fall time and closing time setting, 0x05, 0x37-blue color delay time. 0x05, 0x3a-green color rise time and storage time setting, 0x05, 0x3b-green color fall time and closing time setting, 0x05, 0x3c-green color delay time.
     * 
     *      Data: The upper four bits are rise time or fall time, the lower four bits are storage time, closing time or delay time.
     * 
     *      Comment: Rise time or fall time: 0000-0s, 0001-0.13s, 0010-0.26s, 0011-0.38s, 0100-0.51s, 0101-0.77s, 0110-1.04s, 0111-1.6s, 1000-2.1s, 1001-2.6s, 1010-3.1s, 1011-4.2s, 1100-5.2s, 1101-6.2s, 1110-7.3s, 1111-8.3s. Storage time or closing time: 0000-0.04s, 0001-0.13s, 0010-0.26s, 0011-0.38s, 0100-0.51s, 0101-0.77s, 0110-1.04s, 0111-1.6s, 1000-2.1s, 1001-2.6s, 1010-3.1s, 1011-4.2s, 1100-5.2s, 1101-6.2s, 1110-7.3s, 1111-8.3s. Delay time: 0000-0s, 0001-0.13s, 0010-0.26s, 0011-0.38s, 0100-0.51s, 0101-0.77s, 0110-1.04s, 0111-1.6s, 1000-2.1s, 1001-2.6s, 1010-3.1s, 1011-4.2s, 1100-5.2s, 1101-6.2s, 1110-7.3s, 1111-8.3s.
     * @code 
     example:

        std::vector<unsigned char> result;

        //set red color delay time to 1.04s
        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x05, 0x30, 0x06}, result);
     @endcode
     * 
     * - Breathing lamp status display setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x03-green, 0x04-red
     * 
     *      Data: 0x01
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x03, 0x01}, result);
     @endcode
     * 
     * - Breathing lamp brightness setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x05, 0x03
     * 
     *      Data: 0x00-3.1875 Ma, 0x01-6.375 Ma, 0x10-12.75Ma, 0x11-25.5 Ma
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x05, 0x03, 0x00}, result);
     @endcode
     * 
     * - Breathing lamp register write command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x05
     * 
     *      Data: value0-register address, value1-register value.
     *      
     *      Comment: For detailed information, please refer to AW2026 chip register manual.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x05, 0x00, 0x00}, result);
     @endcode
     * 
     * - Breathing lamp register read command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x06
     * 
     *      Data: value0-register address, value1-register value.
     * 
     *      Comment: The data will be saved in result data. For detailed information, please refer to AW2026 chip register manual.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x06, 0x00, 0x00}, result);
     @endcode
     *
     * - Breathing lamp speed setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x08
     * 
     *      Data: 0x01-fast, 0x02-middle, 0x03-slow.
     * 
     *      Comment: Set breathing lamp speed.
     * 
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x08, 0x01}, result);
     @endcode
     * 
     * - Breathing lamp close command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x06
     * 
     *      Data: 0x06-close.
     * 
     *      Comment: Close breathing lamp.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x02, 0x06}, result);
     @endcode
     *
     * - Breathing lamp param reading command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x09
     * 
     *      Comment: The data will be returned in result value, value1-breathing lamp mode, value2-color index, value3-breathing lamp speed. For detailed information, please refer to AW2026 chip register manual.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x09}, result);
     @endcode
     *
     * - Save display panel 2/3D mode command: 
     * 
     *      Header: 0x02, 0xfe, 0x20
     * 
     *      Type: 0x16
     * 
     *      Data: 0x02-72hz, 0x03-90hz.
     * 
     *      Comment: The data will be saved in flash.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xfe, 0x20, 0x16, 0x02}, result);
     @endcode
     *
     * - Read display panel 2/3D mode command: 
     * 
     *      Header: 0x02, 0xfe, 0x20
     * 
     *      Type: 0x17
     * 
     *      Comment: The data will be returned in result value, value1-Hz in flash, value2-Hz of real display.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xfe, 0x20, 0x17}, result);
     @endcode
     *
     * - Set display panel brightness command: 
     * 
     *      Header: 0x02, 0xfe, 0x20
     * 
     *      Type: 0x02
     * 
     *      Data: Range: 0x01-0x20, 32 levels. 1 is darkest, 32 is brightest.
     * 
     *      Comment: The data will be saved in flash.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xfe, 0x20, 0x02, 0x01}, result);
     @endcode
     *
     * - Read display panel brightness command: 
     * 
     *      Header: 0x02, 0xfe, 0x20
     * 
     *      Type: 0x02
     * 
     *      Comment: The data will be returned in result value, value1-display brightness.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xfe, 0x20, 0x02}, result);
     @endcode
     *
     * - Set auto display pane brightness command: 
     * 
     *      Header: 0x02, 0xfe, 0x20
     * 
     *      Type: 0x04
     * 
     *      Data: 0x00-close, 0x01-open.
     * 
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xfe, 0x20, 0x04, 0x00}, result);
     @endcode
     * 
     * - Set display pane brightness level command: 
     * 
     *      Header: 0x02, 0xfe, 0x20
     * 
     *      Type: 0x02 or 0x07
     * 
     *      Data: Range: 0x01-0x07, 0x01-darkest, 0x07 brightest.
     * 
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xfe, 0x20, 0x02, 0x01}, result);
     @endcode
     * 
     * - Get display pane brightness setting command: 
     * 
     *      Header: 0x02, 0xfe, 0x20
     * 
     *      Type: 0x02 or 0x07
     * 
     *      Comment: The result except the header shows auto mode and brightness level. value1-auto mode, value2 brightness level.
     * 
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xfe, 0x20, 0x02}, result);
     @endcode
     * 
     * - Save display pane brightness mode command: 
     * 
     *      Header: 0x02, 0xfe, 0x20
     * 
     *      Type: 0x0f
     * 
     *      Data: 0x00-close, 0x01-open.
     * 
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xfe, 0x20, 0x0f}, result);
     @endcode
     * 
     * - Save display pane brightness level command: 
     * 
     *      Header: 0x02, 0xfe, 0x20
     * 
     *      Type: 0x10
     * 
     *      Data: Range: 0x01-0x07, 0x01-darkest, 0x07 brightest.
     * 
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xfe, 0x20, 0x10}, result);
     @endcode
     *
     */
    virtual bool hidWriteAndRead(const std::vector<unsigned char> &command, std::vector<unsigned char> &result) = 0;
    /**
     * @brief Write UVC control command and read result.
     */
    virtual bool uvcWriteAndRead(const std::vector<unsigned char> &command, std::vector<unsigned char> &result) = 0;
    /**
     * @brief Write VSC control command and read result.
     */
    virtual bool vscWriteAndRead(const std::vector<unsigned char> &command, std::vector<unsigned char> &result) = 0;

    /**
     * @brief set enable camera synchronize.
    */
    virtual bool enableSync(bool isEnable) = 0;

    /**
     * @brief Return the serial number of the device.
     */
    virtual std::string id() const = 0;

    virtual ~Device(){}

};

std::map<std::string,std::shared_ptr<Device_interface>> getDevicesUntilTimeout(double timeOut = 0., const std::string& desc = "", xv::SlamStartMode slamStartMode = xv::SlamStartMode::Normal, xv::DeviceSupport deviceSupport = xv::DeviceSupport ::ONLYUSB)
{

}

}