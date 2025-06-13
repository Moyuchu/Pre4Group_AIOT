#ifndef NEURAL_NETWORK_H
#define NEURAL_NETWORK_H

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/micro_interpreter.h"

// NeuralNetwork 类声明：该类封装了 TensorFlow Lite Micro 模型加载和推理过程
class NeuralNetwork {
public:
    // 构造函数：加载模型、分配 arena、注册运算算子等
    NeuralNetwork();
    
    // 析构函数，负责释放内存
    ~NeuralNetwork();
    
    // 获取模型的输入缓冲区指针，便于向输入 tensor 填入预处理后的特征数据
    float* getInputBuffer();
    
    // 执行前向传播，返回模型的输出（例如预测的呼吸率）
    float predict();
    
private:
    TfLiteTensor* input;
    TfLiteTensor* output;
    tflite::MicroInterpreter* interpreter;
    tflite::ErrorReporter* error_reporter;
    tflite::MicroMutableOpResolver<10>* resolver;
    const tflite::Model* model;
    uint8_t* tensor_arena;
};

#endif // NEURAL_NETWORK_H
