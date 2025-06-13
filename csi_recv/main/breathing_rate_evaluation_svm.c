#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_SAMPLES 16000
#define FEATURE_SIZE 5
#define WINDOW_SIZE 300
#define STEP_SIZE 150
#define MAX_GT 200

const char* csi_files[] = {
    "../../../benchmark/breathing_rate/evaluation/CSI20250227_193124.csv",
    "../../../benchmark/breathing_rate/evaluation/CSI20250227_191018.csv"
};
const char* gt_files[] = {
    "../../../benchmark/breathing_rate/evaluation/gt_20250227_193124.csv",
    "../../../benchmark/breathing_rate/evaluation/gt_20250227_191018.csv"
};

// === 模型参数（从 Python 导出） ===
float weights[FEATURE_SIZE] = {-0.15477075f, 0.29154388f, -0.26879227f, 0.14369498f, -0.03513335f};
float bias = 29.3084f;
float means[FEATURE_SIZE] = {1.6983f, 55.9275f, 107.5746f, -109.918f, 2425169.7f};
float scales[FEATURE_SIZE] = {20.6707f, 17.2686f, 33.3753f, 31.6961f, 1745971.1f};

// === 读取 CSI 数据 ===
int read_csi_data(const char* filename, float* buffer, int max_len) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        printf("Error: Cannot open file %s\n", filename);
        return -1;
    }

    char line[8192];
    int count = 0;
    fgets(line, sizeof(line), file); // skip header
    while (fgets(line, sizeof(line), file) && count < max_len) {
        char* start = strchr(line, '[');
        if (!start) continue;
        char* token = strtok(start + 1, ",]");
        while (token && count < max_len) {
            buffer[count++] = atof(token);
            token = strtok(NULL, ",]");
        }
    }

    fclose(file);
    return count;
}

// === 读取 GT 数据 ===
float read_gt_data(const char* filename, float* buffer, int max_len) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        printf("Error: Cannot open GT file %s\n", filename);
        return -1;
    }

    char line[128];
    int count = 0;
    fgets(line, sizeof(line), file); // skip header
    while (fgets(line, sizeof(line), file) && count < max_len) {
        buffer[count++] = atof(strtok(line, ","));
    }

    fclose(file);
    return count;
}


// === 特征提取 ===
void extract_features(float* window, float* out_feat) {
    float sum = 0, sqsum = 0, max = window[0], min = window[0], diff_energy = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum += window[i];
        sqsum += window[i] * window[i];
        if (window[i] > max) max = window[i];
        if (window[i] < min) min = window[i];
        if (i > 0) {
            float diff = window[i] - window[i - 1];
            diff_energy += diff * diff;
        }
    }
    float mean = sum / WINDOW_SIZE;
    float std = sqrtf(sqsum / WINDOW_SIZE - mean * mean);

    out_feat[0] = mean;
    out_feat[1] = std;
    out_feat[2] = max;
    out_feat[3] = min;
    out_feat[4] = diff_energy;
}

// === 特征归一化 ===
void normalize(float* feat) {
    for (int i = 0; i < FEATURE_SIZE; i++) {
        feat[i] = (feat[i] - means[i]) / scales[i];
    }
}

// === 预测函数 ===
float predict(float* feat) {
    normalize(feat);
    float sum = bias;
    for (int i = 0; i < FEATURE_SIZE; i++) {
        sum += feat[i] * weights[i];
    }
    return sum;
}

// === 每个文件评估 ===
float evaluate_file(const char* csi_file, const char* gt_file, int* global_index) {
    float csi[MAX_SAMPLES];
    float gt[MAX_GT];

    int csi_len = read_csi_data(csi_file, csi, MAX_SAMPLES);
    int gt_len = read_gt_data(gt_file, gt, MAX_GT);

    printf("CSI file: %s\n", csi_file);
    printf("Ground truth file: %s\n", gt_file);
    printf("Total CSI samples loaded: %d\n", csi_len);
    printf("Total GT samples loaded: %d\n", gt_len);

    int window_count = 0;
    float total_error = 0;
    for (int i = 0; i + WINDOW_SIZE <= csi_len && window_count < gt_len; i += STEP_SIZE) {
        float feat[FEATURE_SIZE];
        extract_features(&csi[i], feat);
        float pred = predict(feat);
        float error = fabs(pred - gt[window_count]);

        printf("[sample %03d] predicted: %.2f, ground truth: %.2f, error: %.2f\n",
       (*global_index), pred, gt[window_count], error);


        total_error += error;
        (*global_index)++;
        window_count++;
    }

    float mae = total_error / window_count;
    printf("MAE for file: %.2f\n\n", mae);
    return mae;
}

// === 主函数 ===
int main() {
    float total_mae = 0;
    int global_index = 0;
    int num_files = sizeof(csi_files) / sizeof(csi_files[0]);

    for (int i = 0; i < num_files; i++) {
        printf("\nProcessing file pair %d:\n", i + 1);
        total_mae += evaluate_file(csi_files[i], gt_files[i], &global_index);
    }

    float final_mae = total_mae / num_files;
    printf("Final MAE across all files: %.2f\n", final_mae);
    return 0;
}

