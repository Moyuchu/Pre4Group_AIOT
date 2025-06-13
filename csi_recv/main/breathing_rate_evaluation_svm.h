#ifndef BREATHING_RATE_EVALUATION_SVM_H
#define BREATHING_RATE_EVALUATION_SVM_H

#define WINDOW_SIZE 300
#define STEP_SIZE 150
#define FEATURE_SIZE 5

// 特征提取函数
void extract_features(float* window, float* out_feat);

// 特征归一化函数
void normalize(float* feat);

// 预测函数
float predict(float* feat);

#endif // BREATHING_RATE_EVALUATION_SVM_H 