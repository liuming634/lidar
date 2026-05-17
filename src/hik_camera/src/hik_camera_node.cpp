#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"

class HikCameraNode : public rclcpp::Node {
public:
    HikCameraNode() : Node("hik_camera_node") {
        // 初始化 SDK
        if (MV_CC_Initialize() != MV_OK) {
            RCLCPP_ERROR(get_logger(), "SDK 初始化失败");
            return;
        }

        // 枚举相机
        MV_CC_DEVICE_INFO_LIST stList;
        memset(&stList, 0, sizeof(stList));
        if (MV_CC_EnumDevices(MV_USB_DEVICE, &stList) != MV_OK || stList.nDeviceNum == 0) {
            RCLCPP_ERROR(get_logger(), "未找到相机");
            MV_CC_Finalize();
            return;
        }
        RCLCPP_INFO(get_logger(), "找到 %d 个相机", stList.nDeviceNum);

        // 创建句柄 + 打开
        if (MV_CC_CreateHandle(&handle_, stList.pDeviceInfo[0]) != MV_OK) {
            RCLCPP_ERROR(get_logger(), "创建句柄失败");
            MV_CC_Finalize();
            return;
        }
        if (MV_CC_OpenDevice(handle_) != MV_OK) {
            RCLCPP_ERROR(get_logger(), "打开设备失败");
            MV_CC_DestroyHandle(handle_);
            MV_CC_Finalize();
            return;
        }

        // 连续采集模式
        MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
        // 开始取流
        if (MV_CC_StartGrabbing(handle_) != MV_OK) {
            RCLCPP_ERROR(get_logger(), "开始取流失败");
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            MV_CC_Finalize();
            return;
        }

        pub_ = create_publisher<sensor_msgs::msg::Image>("camera_image", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(33),
            std::bind(&HikCameraNode::grab_and_publish, this));

        RCLCPP_INFO(get_logger(), "相机节点启动成功");
    }

    ~HikCameraNode() {
        if (handle_) {
            MV_CC_StopGrabbing(handle_);
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
        }
        MV_CC_Finalize();
    }

private:
    void grab_and_publish() {
        if (!handle_) return;

        MV_FRAME_OUT frame;
        memset(&frame, 0, sizeof(frame));
        if (MV_CC_GetImageBuffer(handle_, &frame, 1000) != MV_OK) {
            RCLCPP_WARN(get_logger(), "取图超时");
            return;
        }

        auto msg = sensor_msgs::msg::Image();
        msg.header.stamp = now();
        msg.header.frame_id = "camera";
        msg.height = frame.stFrameInfo.nHeight;
        msg.width = frame.stFrameInfo.nWidth;
        msg.step = frame.stFrameInfo.nWidth * 3;
        msg.is_bigendian = false;
        msg.encoding = "bgr8";

        // 判断是否需要格式转换
        if (frame.stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG8 ||
            frame.stFrameInfo.enPixelType == PixelType_Gvsp_BayerGB8 ||
            frame.stFrameInfo.enPixelType == PixelType_Gvsp_BayerGR8 ||
            frame.stFrameInfo.enPixelType == PixelType_Gvsp_BayerBG8) {
            // Bayer → BGR
            cv::Mat bayer(frame.stFrameInfo.nHeight, frame.stFrameInfo.nWidth, CV_8UC1,
                         (uint8_t*)frame.pBufAddr);
            cv::Mat bgr;
            cv::cvtColor(bayer, bgr, cv::COLOR_BayerRG2BGR);
            msg.data.assign(bgr.data, bgr.data + bgr.total() * 3);
        } else {
            msg.data.assign(
                (uint8_t*)frame.pBufAddr,
                (uint8_t*)frame.pBufAddr + frame.stFrameInfo.nFrameLen);
        }

        pub_->publish(msg);
        MV_CC_FreeImageBuffer(handle_, &frame);
    }

    void* handle_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HikCameraNode>());
    rclcpp::shutdown();
    return 0;
}
