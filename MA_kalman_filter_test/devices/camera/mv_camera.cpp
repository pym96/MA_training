#include "mv_camera.hpp"
#include <fmt/color.h>
#include "MVSDK/CameraApi.h"
#include "MVSDK/CameraDefine.h"

namespace Devices
{
// Assert
#define MV_ASSERT_WARNING(expr, info, ...)                                                 \
    do {                                                                                   \
        if ((expr) == false) {                                                             \
            fmt::print(fg(fmt::color::orange), "[WARNING] " #expr info "\n"##__VA_ARGS__); \
        }                                                                                  \
    } while (0)

#define MV_ASSERT_ERROR(expr, info, ...)                                                \
    do {                                                                                \
        if ((expr) == false) {                                                          \
            fmt::print(fg(fmt::color::red), "[ERROR] " #expr info "\n", ##__VA_ARGS__); \
            return false;                                                               \
        }                                                                               \
    } while (0)

#define MV_ASSERT_THROW(expr, info, ...)                                         \
    do {                                                                         \
        if ((expr) == false) {                                                   \
            throw MindVision_FrameError(                                         \
                fmt::format("'" #expr "' = ({}) " info, status, ##__VA_ARGS__)); \
        }                                                                        \
    } while (0)

// check_api
#define MV_CHECK_API_WARNING(expr, info, ...)                                         \
    do {                                                                              \
        auto status = (expr);                                                         \
        if (status != CAMERA_STATUS_SUCCESS) {                                        \
            fmt::print(                                                               \
                fg(fmt::color::orange), "[WARNING] '" #expr "' = ({}) " info, status, \
                ##__VA_ARGS__);                                                       \
        }                                                                             \
    } while (0)

#define MV_CHECK_API_ERROR(expr, info, ...)                                           \
    do {                                                                              \
        auto status = (expr);                                                         \
        if (status != CAMERA_STATUS_SUCCESS) {                                        \
            fmt::print(                                                               \
                fg(fmt::color::red), "[ERROR] '" #expr "' = ({}) " info "\n", status, \
                ##__VA_ARGS__);                                                       \
            return false;                                                             \
        }                                                                             \
    } while (0)

#define MV_CHECK_API_THROW(expr, info, ...)                                      \
    do {                                                                         \
        auto status = (expr);                                                    \
        if (status != CAMERA_STATUS_SUCCESS) {                                   \
            throw MindVision_FrameError(                                         \
                fmt::format("'" #expr "' = ({}) " info, status, ##__VA_ARGS__)); \
        }                                                                        \
    } while (0)

MV_Camera::MV_Camera(const char * camera_cfg) : camera_cfg(camera_cfg)
{
    
    MV_CHECK_API_WARNING(CameraSdkInit(1), "");
    handle = -1;
}

MV_Camera::~MV_Camera()
{
    if (isOpen()) {
        close();
    }
    fmt::print(fg(fmt::color::red) | fmt::emphasis::bold, "相机已释放！");
}

bool MV_Camera::open()
{
    if (isOpen()) {
        if (!close()) return false;
    }

    tSdkCameraDevInfo infos[2];

    int dev_num = 2;
    //获得设备的信息, dev_num返回找到的相机个数
    MV_CHECK_API_ERROR(CameraEnumerateDevice(infos, &dev_num), "");

    MV_ASSERT_ERROR(dev_num > 0, "未发现设备！");
    //相机初始化，根据相机型号名从文件中加载参数，例如MV-U300
    MV_CHECK_API_ERROR(CameraInit(&infos[0], PARAM_MODE_BY_MODEL, -1, &handle), "");

    MV_ASSERT_ERROR(handle >= 0, "相机未找到！");

    tSdkCameraCapbility tCapability;  //设备描述信息
    MV_CHECK_API_ERROR(CameraGetCapability(handle, &tCapability), "");

    /*
    fmt::print(
        "是否可从文件读取参数: {}, 相机name: {}\n", tCapability.bParamInDevice,
        infos[0].acFriendlyName);

    if (tCapability.bParamInDevice == 0) {  //不支持从相机中读写参数组
        fmt::print("不支持从相机读写参数组\n");
        setConfig();
    } else {
        fmt::print("配置文件路径：{}\n", camera_cfg);
        MV_CHECK_API_WARNING(
            CameraReadParameterFromFile(handle, (char *)camera_cfg.data()),
            "从文件读取相机配置文件错误，路径为: {}", camera_cfg);
    }
    */
    // 保存相机配置
    // MV_CHECK_API_ERROR(CameraSaveParameterToFile(handle, "/home/dji/Codes/rm_auto_aim/Configs/camera/MV-SUA133GC-T-Manifold.config"), "");
    //相机配置文件的index要改成255, 否则设置的图片大小无效!!
    fmt::print("配置文件路径：{}\n", camera_cfg);
    MV_CHECK_API_WARNING(
        CameraReadParameterFromFile(handle, (char *)camera_cfg.data()),
        "从文件读取相机配置文件错误，路径为: {}", camera_cfg);

    // tSdkImageResolution * resolution;
    // MV_CHECK_API_ERROR(CameraGetImageResolution(handle, resolution), "");
    // fmt::print(fg(fmt::color::orange), "{} {}\n", resolution->iHeight, resolution->iWidth);

    // 设置相机触发模式, 用不了
    // MV_CHECK_API_ERROR(CameraSetTriggerMode(handle, 0), "");

    MV_CHECK_API_ERROR(CameraPlay(handle), "");
    //加载参数组
    //MV_CHECK_API_ERROR(CameraLoadParameter(handle, PARAMETER_TEAM_A), "");

    MV_CHECK_API_ERROR(CameraSetIspOutFormat(handle, CAMERA_MEDIA_TYPE_BGR8), "");


    return true;
}

/**
 * @brief 当从文件读取失效后，手动设置。
 * 
 */
bool MV_Camera::setConfig()
{
    /**
     * @brief 设置相机配置参数
     * 
     */
    CameraGetImageResolution(handle, &pImageResolution);
    /*
    pImageResolution.iIndex      = 0xFF;
    pImageResolution.iWidthFOV   = _CAMERA_RESOLUTION_COLS;
    pImageResolution.iHeightFOV  = _CAMERA_RESOLUTION_ROWS;
    pImageResolution.iWidth      = _CAMERA_RESOLUTION_COLS;
    pImageResolution.iHeight     = _CAMERA_RESOLUTION_ROWS;
    pImageResolution.iHOffsetFOV = static_cast<int>((1280 - _CAMERA_RESOLUTION_COLS) * 0.5);
    pImageResolution.iVOffsetFOV = static_cast<int>((1024 - _CAMERA_RESOLUTION_ROWS) * 0.5);
    */
    CameraSetImageResolution(handle, &pImageResolution);

    // 设置曝光时间
    CameraGetAeState(handle, &AEstate);
    CameraSetAeState(handle, FALSE);
    // CameraSetExposureTime(handle, _CAMERA_EXPOSURETIME);
    // 设置颜色增益
    CameraSetGain(handle, 145, 130, 105);
    return true;
}

bool MV_Camera::close()
{
    MV_ASSERT_WARNING(handle >= 0, "相机已经关闭！\n");  //
    if (handle < 0) return true;
    MV_CHECK_API_ERROR(CameraUnInit(handle), "");
    handle = -1;
    return true;
}

bool MV_Camera::isOpen() const
{
    return handle >= 0;
}

bool MV_Camera::read(cv::Mat & img) const
{
    // fmt::print(bg(fmt::color::green),"相机句柄为 {}",this->handle);
    MV_ASSERT_ERROR(isOpen(), "相机未打开！");
    tSdkFrameHead head;
    BYTE * buffer;
    // 100 为超时时间
    MV_CHECK_API_ERROR(CameraGetImageBuffer(handle, &head, &buffer, 1000), "");
    img = cv::Mat(head.iHeight, head.iWidth, CV_8UC3);
    MV_CHECK_API_ERROR(CameraImageProcess(handle, buffer, img.data, &head), "");
    MV_CHECK_API_ERROR(CameraReleaseImageBuffer(handle, buffer), "");
    return true;
}
/**
 * @brief 这个函数的double是可以获取当前帧的采集时间
 *
 */
bool MV_Camera::read(cv::Mat & img, double & timestamp_ms) const
{
    MV_ASSERT_ERROR(isOpen(), "相机未打开！");
    tSdkFrameHead head;
    BYTE * buffer;
    // 100 为超时时间
    MV_CHECK_API_ERROR(CameraGetImageBuffer(handle, &head, &buffer, 1000), "");
    // fmt::print(fg(fmt::color::green), "[{}, {}]", head.iHeight, head.iWidth);
    img = cv::Mat(head.iHeight, head.iWidth, CV_8UC3);
    MV_CHECK_API_ERROR(CameraImageProcess(handle, buffer, img.data, &head), "");
    // timestamp_ms = head.uiTimeStamp;  // uiTimeStamp 该帧的采集时间，单位0.1毫秒
    timestamp_ms = head.uiExpTime;  //当前图像的曝光值，单位为微秒us
    MV_CHECK_API_ERROR(CameraReleaseImageBuffer(handle, buffer), "");
    return true;
}

bool MV_Camera::get_exposure_us(double & us) const
{
    MV_ASSERT_ERROR(isOpen(), "相机未打开！");
    MV_CHECK_API_ERROR(CameraGetExposureTime(handle, &us), "");
    return true;
}

bool MV_Camera::set_exposure_us(double us) const
{
    MV_ASSERT_ERROR(isOpen(), "相机未打开！");
    MV_CHECK_API_ERROR(CameraSetExposureTime(handle, us), "");
    return true;
}

}  // namespace Devices
