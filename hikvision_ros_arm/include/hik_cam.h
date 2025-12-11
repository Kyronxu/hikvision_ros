#ifndef HIK_CAM_H
#define HIK_CAM_H

#include <iostream>
#include <string>
#include <cstring>

#include "hikvision_sdk/HCNetSDK.h"
#include "hikvision_sdk/PlayM4.h"
#include "hikvision_sdk/error.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_calibration_parsers/parse.h>

// 移除OpenCV和cv_bridge相关头文件
// #include <opencv2/opencv.hpp>
// #include <cv_bridge/cv_bridge.h>

class HikvisionCamera
{
private:
    /// frame delay detect
    bool is_first_frame;
    std_msgs::Header last_frame_header;
    std_msgs::Header this_frame_header;

    int time_difference;
    int frame_rate; //判断延时的阈值
    bool is_last_frame_delay,is_current_frame_delay;
    int frame_count;
    bool if_shutdown;
    

    /// ros parameters
    image_transport::CameraPublisher image_pub;
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_mgr;
    ros::ServiceServer SetCameraInfoSrv;


    /// camera parameters
    int image_width;
    int image_height;
    unsigned char* bgr_buffer;  // 用于存储转换后的BGR数据

    std::string ip_addr;
    std::string usr_name;
    std::string password;
    std::string frame_id;
    std::string camera_name;
    std::string camera_info_url;

    int port;
    int channel;
    int link_mode;

    LONG user_id;
    LONG data_play_handler;

    static void decodeCallback_(int nPort, char *pBuf, int nSize, FRAME_INFO *pFrameInfo, void *nUser, int nReserved2)
    {
        ((HikvisionCamera *) nUser)->decodeCallback(nPort, pBuf, nSize, pFrameInfo);
    }

    static void dataCallback(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void *pUser)
    {
        ((HikvisionCamera *) pUser)->dataCallback(lRealHandle, dwDataType, pBuffer, dwBufSize);
    }

    std::string expandUserPath(std::string path);
    void yv12ToBgr(const unsigned char* yv12_data, unsigned char* bgr_data, int width, int height);

    void decodeCallback(int nPort, char *pBuf, int nSize, FRAME_INFO *pFrameInfo);

    void dataCallback(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize);

    bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& rsp);

    bool initHikSDK();

    void initROSIO(ros::NodeHandle& priv_node);


public:

    void run();

    ~HikvisionCamera();

};

#endif //HIK_CAM_H
