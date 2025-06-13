#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define FFT_SIZE 2000
#define MAX_SAMPLES 100
#define CSI_BUFFER_LENGTH 8000
#define PI 3.14159265358979323846
#define SAMPLING_RATE 20.0
#define MIN_BREATH_HZ 0.1
#define MAX_BREATH_HZ 0.6

// 复数结构体
typedef struct {
    float real;
    float imag;
} complex_t;

typedef struct {
    char* csi_file;
    char* gt_file;
} evaluation_pair_t;

static complex_t fft_buffer[FFT_SIZE];
static float fft_magnitude[FFT_SIZE / 2];
static int predicted_bpm[MAX_SAMPLES];
static int ground_truth_bpm[MAX_SAMPLES];
static int sample_count = 0;

static void fft_swap(complex_t *a, complex_t *b) {
    complex_t temp = *a;
    *a = *b;
    *b = temp;
}

static void bit_reverse(complex_t *data, int n) {
    int j = 0;
    for (int i = 0; i < n - 1; i++) {
        if (i < j) fft_swap(&data[i], &data[j]);
        int k = n >> 1;
        while (k <= j) {
            j -= k;
            k >>= 1;
        }
        j += k;
    }
}

static void fft(complex_t *data, int n) {
    bit_reverse(data, n);
    for (int size = 2; size <= n; size *= 2) {
        int halfsize = size / 2;
        float angle = -2 * PI / size;
        complex_t w = {cos(angle), sin(angle)};
        for (int k = 0; k < n; k += size) {
            complex_t w_k = {1.0, 0.0};
            for (int j = 0; j < halfsize; j++) {
                complex_t t = {
                    w_k.real * data[k + j + halfsize].real - w_k.imag * data[k + j + halfsize].imag,
                    w_k.real * data[k + j + halfsize].imag + w_k.imag * data[k + j + halfsize].real
                };
                complex_t u = data[k + j];
                data[k + j].real = u.real + t.real;
                data[k + j].imag = u.imag + t.imag;
                data[k + j + halfsize].real = u.real - t.real;
                data[k + j + halfsize].imag = u.imag - t.imag;
                float temp = w_k.real * w.real - w_k.imag * w.imag;
                w_k.imag = w_k.real * w.imag + w_k.imag * w.real;
                w_k.real = temp;
            }
        }
    }
}

static void compute_magnitude_spectrum(complex_t *fft_data, float *magnitude, int n) {
    for (int i = 0; i < n/2; i++) {
        magnitude[i] = sqrt(fft_data[i].real * fft_data[i].real + fft_data[i].imag * fft_data[i].imag);
    }
}

float bandpass_filter(float value, float alpha, float* prev_output) {
    float output = alpha * value + (1.0f - alpha) * (*prev_output);
    *prev_output = output;
    return output;
}

int improved_breathing_rate_estimation(float* csi_data) {
    float mean = 0;
    for (int i = 0; i < FFT_SIZE; i++) mean += csi_data[i];
    mean /= FFT_SIZE;

    static float prev_filtered = 0;

    for (int i = 0; i < FFT_SIZE; i++) {
        float filtered = bandpass_filter(csi_data[i] - mean, 0.1f, &prev_filtered);
        float hann = 0.5 * (1 - cos(2 * PI * i / (FFT_SIZE - 1)));
        fft_buffer[i].real = filtered * hann;
        fft_buffer[i].imag = 0;
    }

    fft(fft_buffer, FFT_SIZE);
    compute_magnitude_spectrum(fft_buffer, fft_magnitude, FFT_SIZE);

    int min_idx = (int)(MIN_BREATH_HZ * FFT_SIZE / SAMPLING_RATE);
    int max_idx = (int)(MAX_BREATH_HZ * FFT_SIZE / SAMPLING_RATE);

    float max_amp = 0;
    int peak_idx = 0;
    for (int i = min_idx; i <= max_idx; i++) {
        if (fft_magnitude[i] > max_amp) {
            max_amp = fft_magnitude[i];
            peak_idx = i;
        }
    }

    float refined_idx = (float)peak_idx;
    if (peak_idx > 0 && peak_idx < FFT_SIZE/2 - 1) {
        float alpha = fft_magnitude[peak_idx - 1];
        float beta = fft_magnitude[peak_idx];
        float gamma = fft_magnitude[peak_idx + 1];
        float denom = alpha - 2*beta + gamma;
        if (denom != 0) refined_idx = peak_idx + 0.5 * (alpha - gamma) / denom;
    }

    float freq = refined_idx * SAMPLING_RATE / FFT_SIZE;
    int bpm = (int)(freq * 60);

    if (bpm < 6) bpm = 6;
    if (bpm > 30) bpm = 30;
    return bpm;
}

float calculate_mae() {
    if (sample_count == 0) return 0.0f;
    float sum_abs_error = 0.0f;
    for (int i = 0; i < sample_count; i++) {
        sum_abs_error += abs(predicted_bpm[i] - ground_truth_bpm[i]);
    }
    return sum_abs_error / sample_count;
}

int read_csv_data(const char* filename, float* data_buffer, int max_samples) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        printf("Error: Cannot open file %s\n", filename);
        return -1;
    }

    char line[4096];
    int count = 0;

    fgets(line, sizeof(line), file);
    while (fgets(line, sizeof(line), file) && count + FFT_SIZE < max_samples) {
        char* data_start = strchr(line, '[');
        if (!data_start) continue;
        char* token = strtok(data_start + 1, ",]");
        while (token && count < max_samples) {
            data_buffer[count++] = atof(token);
            token = strtok(NULL, ",]");
        }
    }

    fclose(file);
    printf("Total CSI samples loaded from %s: %d\n", filename, count);
    return count;
}

int read_gt_data(const char* filename, float* data_buffer, int max_samples) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        printf("Error: Cannot open GT file %s\n", filename);
        return -1;
    }

    char line[256];
    int count = 0;

    fgets(line, sizeof(line), file); // skip header
    while (fgets(line, sizeof(line), file) && count < max_samples) {
        char* token = strtok(line, ",");
        if (token != NULL) {
            data_buffer[count++] = atof(token);
        }
    }

    fclose(file);
    printf("Total GT samples loaded from %s: %d\n", filename, count);
    return count;
}

int main() {
    evaluation_pair_t evaluation_files[] = {
        {"../../../benchmark/breathing_rate/evaluation/CSI20250227_193124.csv", "../../../benchmark/breathing_rate/evaluation/gt_20250227_193124.csv"},
        {"../../../benchmark/breathing_rate/evaluation/CSI20250227_191018.csv", "../../../benchmark/breathing_rate/evaluation/gt_20250227_191018.csv"}
    };

    int num_files = sizeof(evaluation_files) / sizeof(evaluation_files[0]);
    float* csi_data = (float*)malloc(CSI_BUFFER_LENGTH * sizeof(float));
    float* gt_data = (float*)malloc(MAX_SAMPLES * sizeof(float));
    if (!csi_data || !gt_data) {
        printf("Error: Memory allocation failed\n");
        return -1;
    }

    for (int i = 0; i < num_files; i++) {
        printf("\nProcessing file pair %d:\n", i + 1);
        printf("CSI file: %s\n", evaluation_files[i].csi_file);
        printf("Ground truth file: %s\n", evaluation_files[i].gt_file);

        int csi_samples = read_csv_data(evaluation_files[i].csi_file, csi_data, CSI_BUFFER_LENGTH);
        if (csi_samples <= 0) continue;
        int gt_samples = read_gt_data(evaluation_files[i].gt_file, gt_data, MAX_SAMPLES);
        if (gt_samples <= 0) continue;

        int sum_bpm = 0;
        int count = 0;
        for (int j = 0; j < csi_samples - FFT_SIZE; j += FFT_SIZE/2) {
            int predicted_rate = improved_breathing_rate_estimation(csi_data + j);
            sum_bpm += predicted_rate;
            count++;

            int gt_index = j / (FFT_SIZE/2);
            if (gt_index < gt_samples && sample_count < MAX_SAMPLES) {
                predicted_bpm[sample_count] = predicted_rate;
                ground_truth_bpm[sample_count] = (int)gt_data[gt_index];
                sample_count++;
            }
        }

        float current_mae = calculate_mae();
        printf("MAE for file %d: %.2f\n", i + 1, current_mae);
        printf("Average predicted BPM: %.2f\n", (float)sum_bpm / count);
    }

    float final_mae = calculate_mae();
    printf("\nFinal MAE across all files: %.2f\n", final_mae);
    free(csi_data);
    free(gt_data);
    return 0;
}
