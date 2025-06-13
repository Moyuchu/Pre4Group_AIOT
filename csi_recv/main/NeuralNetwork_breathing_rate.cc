#include "NeuralNetwork_breathing_rate.h"
#include "model_data.h"  // 此文件中包含： const unsigned char br_model_tflite[] 和模型长度等定义
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h"
#include <stdlib.h>
#include <stdio.h>

#define ARENA_SIZE 20000

NeuralNetwork::NeuralNetwork() {
    // 使用 MicroErrorReporter 输出日志
    error_reporter = new tflite::MicroErrorReporter();

    // 从 model_data.h 中加载 TFLite 模型数据
    model = tflite::GetModel(br_model_tflite);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        error_reporter->Report("Model schema version %d not equal to supported version %d.",
                                 model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }

    resolver = new tflite::MicroMutableOpResolver<10>();
    resolver->AddFullyConnected();
    resolver->AddMul();
    resolver->AddAdd();
    resolver->AddLogistic();
    resolver->AddReshape();
    resolver->AddQuantize();
    resolver->AddDequantize();
    resolver->AddSoftmax();
    resolver->AddRelu();

    tensor_arena = (uint8_t*)malloc(ARENA_SIZE);
    if (!tensor_arena) {
        error_reporter->Report("Failed to allocate tensor arena.");
        return;
    }
    error_reporter->Report("Allocated tensor arena of %d bytes.", ARENA_SIZE);

    interpreter = new tflite::MicroInterpreter(model, *resolver, tensor_arena, ARENA_SIZE, error_reporter);
    if (interpreter->AllocateTensors() != kTfLiteOk) {
        error_reporter->Report("AllocateTensors() failed");
        return;
    }

    size_t used_bytes = interpreter->arena_used_bytes();
    error_reporter->Report("Tensor arena used: %d bytes", used_bytes);

    input = interpreter->input(0);
    output = interpreter->output(0);
}

NeuralNetwork::~NeuralNetwork() {
    if (tensor_arena) {
        free(tensor_arena);
    }
    delete interpreter;
    delete resolver;
    delete error_reporter;
}

float* NeuralNetwork::getInputBuffer() {
    if (input == nullptr) {
        error_reporter->Report("Input tensor is null");
        return nullptr;
    }
    return input->data.f;
}

float NeuralNetwork::predict() {
    if (interpreter->Invoke() != kTfLiteOk) {
        error_reporter->Report("Invoke failed");
        return -1.0f;
    }
    return output->data.f[0];
}


int main(void) {
    // 创建神经网络实例，加载模型
    NeuralNetwork* nn = new NeuralNetwork();

    // 获取输入缓冲区（注意：这里假设模型输入维度为 FEATURE_SIZE，即 5 个特征）
    float* inputBuffer = nn->getInputBuffer();
    if (inputBuffer == nullptr) {
        printf("Error: Input buffer is null\n");
        return -1;
    }

    // 模拟预处理后的特征输入（请将此处替换为实际提取的特征值）
    // 示例数据：5 个特征，数值仅供测试使用
    const int FEATURE_SIZE = 5;
    float test_features[FEATURE_SIZE] = {0.5f, -0.2f, 1.3f, 0.0f, 0.7f};

    for (int i = 0; i < FEATURE_SIZE; i++) {
        inputBuffer[i] = test_features[i];
    }

    // 调用推理
    float predicted_bpm = nn->predict();

    // 输出预测结果
    printf("Predicted Breathing Rate: %.2f BPM\n", predicted_bpm);

    // 清理
    delete nn;
    return 0;
}
