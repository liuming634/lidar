// 【BaseInfer.hpp】推理基础数据结构和接口
// Image: 包装bgrptr+宽高的图像容器，不管理内存生命周期
// Norm:  归一化参数({MeanStd,AlphaBeta} × 可选通道交换)，提供工厂方法
// AffineMatrix: 图像→网络输入的等比例缩放仿射变换及其逆变换
// Infer<T>: 模板接口，定义forward(单张)和forwards(批量)纯虚函数
#ifndef __BASEINFER_HPP__
#define __BASEINFER_HPP__

#include <tuple>
#include <vector>

namespace tdt_radar {

#define GPU_BLOCK_THREADS 512

struct Image {
    const void* bgrptr = nullptr;
    int         width = 0, height = 0;

    Image() = default;
    Image(const void* bgrptr, int width, int height)
        : bgrptr(bgrptr), width(width), height(height)
    {
    }
};

enum class NormType : int { None = 0, MeanStd = 1, AlphaBeta = 2 };

enum class ChannelType : int { None = 0, SwapRB = 1 };

// 推理接口模板：支持单张forward和批量forwards
template <typename T>
class Infer {
public:
    virtual T forward(const Image& image, void* stream = nullptr) = 0;
    virtual std::vector<T> forwards(const std::vector<Image>& images,
                                    void* stream = nullptr) = 0;
};

// 图像归一化参数：支持MeanStd、AlphaBeta、None三种方式及RGB通道交换
struct Norm {
    float       mean[3];
    float       std[3];
    float       alpha, beta;
    NormType    type = NormType::None;
    ChannelType channel_type = ChannelType::None;

    static Norm mean_std(const float mean[3], const float std[3],
                         float       alpha = 1 / 255.0f,
                         ChannelType channel_type = ChannelType::None);

    static Norm alpha_beta(float alpha, float beta = 0,
                           ChannelType channel_type = ChannelType::None);

    static Norm None();
};

// 仿射变换矩阵：计算图像缩放和平移的变换与逆变换
struct AffineMatrix {
    float i2d[6];
    float d2i[6];

    void compute(const std::tuple<int, int>& from,
                 const std::tuple<int, int>& to);
};

}  // namespace tdt_radar

#endif  //__BASEINFER_HPP__
