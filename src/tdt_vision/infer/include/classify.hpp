// 【classify.hpp】分类器推理接口，当前支持DenseNet121
// 输入Image，输出int分类标签（对应装甲板编号），支持批量推理
#ifndef __CLASSIFY_HPP__
#define __CLASSIFY_HPP__

#include <future>
#include <memory>
#include <string>
#include <vector>
#include "BaseInfer.hpp"
#include "InferTool.hpp"

namespace classify {

using namespace tdt_radar;

enum class Type : int { densenet121 = 0 };

std::shared_ptr<Infer<int>> load(const std::string& engine_file, Type type);

const char* type_name(Type type);

}  // namespace classify

#endif  // __CLASSIFY_HPP__