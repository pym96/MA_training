#ifndef _MV_CAMERA_HPP_
#define _MV_CAMERA_HPP_


#include <opencv2/opencv.hpp>

#include <fmt/color.h>
#include <fmt/core.h>
#include <stdexcept>

#include "MVSDK/CameraApi.h"

namespace Devices
{

class mv_Frame_Error : std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

class MV_Camera
{
public:
    explicit MV_Camera(const char *camera_cfg = "/home/fuziming/MA/kalmen/MA-main（复件）/Configs/camera/MV-SUA133GC-T-Manifold.config");

    bool open();

    bool close();

    bool isOpen() const;

    bool read(cv::Mat &) const;

    bool read(cv::Mat &, double &) const;

    bool restart();

    bool get_exposure_us(double &us) const;

    bool set_exposure_us(double us) const;

    bool setConfig();

    ~MV_Camera();

    MV_Camera(MV_Camera const &) = delete;
    MV_Camera &operator=(MV_Camera const &) = delete;

private:
    CameraHandle handle; //相机句柄

    const std::string camera_cfg;
    // int          enemyColor; //敌方颜色
    // int          exposure;   //曝光时长，单位 ns, 1ms = 1000ns

    // unsigned char *   g_pRgbBuffer; //缓冲区

    // tSdkFrameHead       head;
    tSdkImageResolution pImageResolution;
    BYTE *              pbyBuffer;
    BOOL                AEstate = FALSE;

}; // class mv_camera

} // namespace Devices

#endif /*_MV_CAMERA_HPP_*/