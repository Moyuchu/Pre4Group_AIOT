/* Get Start Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * In this file, the following code blocks are marked for customization.
 * Each block starts with the comment: "// YOUR CODE HERE"
 * and ends with: "// END OF YOUR CODE".
 *
 * [1] Modify the CSI Buffer and FIFO Lengths:
 *     - Adjust the buffer configuration based on your system if necessary.
 *
 * [2] Implement Algorithms:
 *     - Develop algorithms for motion detection, breathing rate estimation, and MQTT message sending.
 *     - Implement them in their respective functions.
 *
 * [3] Modify Wi-Fi Configuration:
 *     - Modify the Wi-Fi settings–SSID and password to connect to your router.
 *
 * [4] Finish the function `csi_process()`:
 *     - Fill in the group information.
 *     - Process and analyze CSI data in the `csi_process` function.
 *     - Implement your algorithms in this function if on-board. (Task 2)
 *     - Return the results to the host or send the CSI data via MQTT. (Task 3)
 *
 * Feel free to modify these sections to suit your project requirements!
 *
 * Have fun building!
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include "nvs_flash.h"
#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "mqtt_client.h"
#include "breathing_rate_evaluation_svm.h"

// [1] YOUR CODE HERE
#define CSI_BUFFER_LENGTH 800
#define CSI_FIFO_LENGTH 100
#define VARIANCE_THRESHOLD 40.0f
static int16_t CSI_Q[CSI_BUFFER_LENGTH];
static int CSI_Q_INDEX = 0; // CSI Buffer Index
// Enable/Disable CSI Buffering. 1: Enable, using buffer, 0: Disable, using serial output
static bool CSI_Q_ENABLE = 1;
static void csi_process(const int8_t *csi_data, int length);
static const char *TAG = "csi_recv";
// MQTT
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;
// motion
extern float g_motion_amplitude;
extern int g_motion_intensity;
bool motion_detected = true;
int breathing_rate = 10;
static bool wifi_connected = false;
static int64_t last_send_time = -1;
// [1] END OF YOUR CODE

// [2] YOUR CODE HERE
// Modify the following functions to implement your algorithms.
// NOTE: Please do not change the function names and return types.
bool init_mqtt()
{
  ESP_LOGI(TAG, "Initializing MQTT client...");

  // Configure the MQTT client
  esp_mqtt_client_config_t mqtt_cfg = {
      .broker.address.uri = "mqtt://172.20.10.7", // Connect to the local MQTT server
      .broker.address.port = 1883,
      .credentials.username = NULL, // No authentication required
      .credentials.authentication.password = NULL,
      .session.keepalive = 20, // 20-second survival time
      .credentials.client_id = "esp32_c5_rx_csi_client",
  };

  // Create an MQTT client
  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  if (mqtt_client == NULL)
  {
    ESP_LOGE(TAG, "Failed to initialize MQTT client");
    return false;
  }

  // Start the MQTT client
  esp_err_t err = esp_mqtt_client_start(mqtt_client);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
    return false;
  }

  mqtt_connected = true;

  ESP_LOGI(TAG, "MQTT client initialized and started successfully");
  return true;
}

float g_motion_amplitude = 0.0f; // Global motion amplitude variable
int g_motion_intensity = 0;      // Exercise intensity levels: 0= none, 1= Slight, 2= moderate, 3= vigorous

bool motion_detection(bool verbose_logging)
{
  if (CSI_Q_INDEX < 50)
    return false; // The data is insufficient

  const int window_size = 30;
  const float alpha = 0.4;
  const float base_threshold = 50.0f;

  // Calculate the overall statistical data of the signal
  float signal_mean = 0;
  for (int i = 0; i < CSI_Q_INDEX; i++)
    signal_mean += CSI_Q[i];
  signal_mean /= CSI_Q_INDEX;

  float signal_variance = 0;
  for (int i = 0; i < CSI_Q_INDEX; i++)
    signal_variance += (CSI_Q[i] - signal_mean) * (CSI_Q[i] - signal_mean);
  float signal_std = sqrtf(signal_variance / CSI_Q_INDEX);

  float threshold = fmaxf(base_threshold, signal_std * 0.9f);

  // Apply moving average filtering
  int16_t smoothed[CSI_BUFFER_LENGTH];
  smoothed[0] = CSI_Q[0];
  for (int i = 1; i < CSI_Q_INDEX; i++)
  {
    smoothed[i] = alpha * CSI_Q[i] + (1 - alpha) * smoothed[i - 1];
  }

  // Calculate the short-term variance and the maximum variance
  float max_variance = 0, avg_variance = 0;
  int valid_windows = 0;
  for (int start = 0; start < CSI_Q_INDEX - window_size; start += window_size / 2)
  {
    float mean = 0, variance = 0;
    for (int i = 0; i < window_size && (start + i) < CSI_Q_INDEX; i++)
    {
      mean += smoothed[start + i];
    }
    mean /= window_size;

    for (int i = 0; i < window_size && (start + i) < CSI_Q_INDEX; i++)
    {
      variance += (smoothed[start + i] - mean) * (smoothed[start + i] - mean);
    }
    variance /= window_size;

    max_variance = fmaxf(max_variance, variance);
    avg_variance += variance;
    valid_windows++;
  }
  avg_variance /= fmaxf(valid_windows, 1);

  float diff_energy = 0;
  for (int i = 1; i < CSI_Q_INDEX; i++)
  {
    diff_energy += (smoothed[i] - smoothed[i - 1]) * (smoothed[i] - smoothed[i - 1]);
  }
  diff_energy /= (CSI_Q_INDEX - 1);

  float diff_threshold = fmaxf(60.0f, signal_std * 1.5f);

  bool motion_by_variance = (max_variance > threshold);
  bool motion_by_diff = (diff_energy > diff_threshold);
  bool motion_detected = (motion_by_variance && motion_by_diff) ||
                         (max_variance > threshold * 2.0f) ||
                         (diff_energy > diff_threshold * 1.5f);

  // Calculate the amplitude of motion
  float variance_score = fminf((max_variance / (threshold * 3.0f)) * 100.0f, 100.0f);
  float diff_score = fminf((diff_energy / (diff_threshold * 3.0f)) * 100.0f, 100.0f);
  g_motion_amplitude = (variance_score + diff_score) / 2.0f;

  // Determine the intensity of exercise
  if (g_motion_amplitude < 30.0f)
    g_motion_intensity = 0;
  else if (g_motion_amplitude < 50.0f)
    g_motion_intensity = 1;
  else if (g_motion_amplitude < 75.0f)
    g_motion_intensity = 2;
  else
    g_motion_intensity = 3;

  // Output the motion amplitude and the original value information
  ESP_LOGI(TAG, "Motion metrics: amplitude=%.1f, var_score=%.1f, diff_score=%.1f",
           g_motion_amplitude, variance_score, diff_score);
  ESP_LOGI(TAG, "Raw values: max_var=%.2f (threshold=%.2f), diff=%.2f (threshold=%.2f)",
           max_variance, threshold, diff_energy, diff_threshold);

  if (verbose_logging)
  {
    ESP_LOGI(TAG, "Motion detection stats: avg_var=%.2f, max_var=%.2f, diff_energy=%.2f",
             avg_variance, max_variance, diff_energy);
    ESP_LOGI(TAG, "Thresholds: variance=%.2f, diff=%.2f, signal_std=%.2f",
             threshold, diff_threshold, signal_std);
    ESP_LOGI(TAG, "Detection result: by_variance=%d, by_diff=%d, combined=%d",
             motion_by_variance, motion_by_diff, motion_detected);
  }

  static bool last_few_results[5] = {false, false, false, false, false};
  static int history_index = 0;
  last_few_results[history_index] = motion_detected;
  history_index = (history_index + 1) % 5;

  int motion_count = 0;
  for (int i = 0; i < 5; i++)
    if (last_few_results[i])
      motion_count++;

  bool history_vote = (motion_count >= 3);

  // State machine: It will be triggered only when sufficient motion evidence is accumulated
  static int continuous_motion_count = 0;
  if (motion_detected)
  {
    continuous_motion_count = (g_motion_amplitude > 60.0f) ? fminf(continuous_motion_count + 2, 10) : fminf(continuous_motion_count + 1, 10);
  }
  else
  {
    continuous_motion_count = fmaxf(continuous_motion_count - 1, 0);
  }

  bool state_machine_result = (g_motion_amplitude > 75.0f) || (continuous_motion_count >= 4);
  bool final_result = state_machine_result && history_vote;

  if (verbose_logging || final_result != motion_detected)
  {
    ESP_LOGI(TAG, "Motion status: history=%d, continuous=%d, intensity=%d, final=%d, final_result_output=%s",
             history_vote, continuous_motion_count, g_motion_intensity, final_result,
             final_result == 1 ? "True" : "False");
  }

  return final_result;
}

int breathing_rate_estimation()
{
  // Obtain the latest CSI data from CSI_Q and store it in the buffer
  // if (CSI_Q == NULL)
  // {
  //   printf("CSI queue is not initialized\n");
  //   return -1;
  // }
  if (CSI_Q_INDEX < 300)
  {
    ESP_LOGI(TAG, "Data insufficient, continuing to collect...CSI_Q_INDEX:%d", CSI_Q_INDEX);
    return 0; // 至少需要5秒数据(假设采样率60Hz)
  }
  int buffer_index = 0;
  float csi_buffer[WINDOW_SIZE];

  // Convert the data in CSI_Q to float and store it in csi_buffer
  ESP_LOGI(TAG, "Starting to collect CSI data into buffer...%d", buffer_index);
  while (buffer_index < WINDOW_SIZE && CSI_Q_INDEX > 0)
  {
    // Convert the CSI data of type int16_t to float
    csi_buffer[buffer_index] = (float)CSI_Q[CSI_Q_INDEX - 1];
    ESP_LOGD(TAG, "Collected CSI data[%d]: %.2f", buffer_index, csi_buffer[buffer_index]);
    buffer_index++;
    CSI_Q_INDEX--;
  }

  // When sufficient data is collected, the respiratory rate is estimated
  if (buffer_index >= WINDOW_SIZE)
  {
    ESP_LOGI(TAG, "Sufficient data collected, performing breathing rate estimation...");
    float features[FEATURE_SIZE];

    // The functions in breathing_rate_evaluation_svm.c are used for processing
    ESP_LOGI(TAG, "Extracting features from CSI data...");
    extract_features(csi_buffer, features);

    // Log extracted features
    for (int i = 0; i < FEATURE_SIZE; i++)
    {
      ESP_LOGD(TAG, "Feature[%d]: %.2f", i, features[i]);
    }

    ESP_LOGI(TAG, "Predicting breathing rate using features...");
    float breathing_rate = predict(features);

    ESP_LOGI(TAG, "Estimated breathing rate: %.2f BPM", breathing_rate);

    // Move the window (retain the second half of the data)
    ESP_LOGI(TAG, "Shifting the window...");
    for (int i = 0; i < WINDOW_SIZE - STEP_SIZE; i++)
    {
      csi_buffer[i] = csi_buffer[i + STEP_SIZE];
      ESP_LOGD(TAG, "Shifted CSI data[%d]: %.2f", i, csi_buffer[i]);
    }
    buffer_index = WINDOW_SIZE - STEP_SIZE;

    return (int)breathing_rate;
  }
  ESP_LOGI(TAG, "Data insufficient, continuing to collect...");
  return 0; // Insufficient data. Continue collecting
}

int64_t get_current_time()
{
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  int64_t time_ms = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
  return time_ms;
}

void mqtt_send(bool motion_detected, int breathing_rate)
{
  // TODO: Implement MQTT message sending using CSI data or Results
  // NOTE: If you implement the algorithm on-board, you can return the results to the host, else send the CSI data.
  if (!wifi_connected)
  {
    ESP_LOGE(TAG, "Can't send MQTT: WiFi not connected");
    return;
  }
  if (mqtt_client == NULL || !mqtt_connected)
  {
    ESP_LOGE(TAG, "Can't send MQTT: MQTT client not initialized or not connected");
    return;
  }

  // 节流，此处时间应该还能继续调整
  if (get_current_time() - last_send_time < 5000000)
  {
    return;
  }
  last_send_time = get_current_time();

  // Message creation
  char message[256];

  // 按需传参，csi_samples 也没必要但是先放着里了
  snprintf(message, sizeof(message),
           "{\"csi_samples\":%d,\"motion_detected\":%s,\"breathing_rate\":%d}",
           CSI_Q_INDEX,
           motion_detected ? "true" : "false",
           breathing_rate);

  ESP_LOGI(TAG, "MQTT message prepared: %s", message);

  // Publish a message to the MQTT topic
  int msg_id = esp_mqtt_client_publish(mqtt_client, "rx/data", message, 0, 1, 0);
  if (msg_id < 0)
  {
    ESP_LOGE(TAG, "Failed to publish MQTT message");
  }
  else
  {
    ESP_LOGI(TAG, "MQTT message published successfully, ID: %d", msg_id);
  }
  // subscribe
  // static bool subscribed = false;
  // if (!subscribed) {
  //   int sub_msg_id = esp_mqtt_client_subscribe(mqtt_client, "rx/cmd", 0);
  //   if (sub_msg_id < 0) {
  //     ESP_LOGE(TAG, "Failed to subscribe to MQTT topic");
  //   } else {
  //     ESP_LOGI(TAG, "MQTT topic subscribed successfully, ID: %d", sub_msg_id);
  //     subscribed = true;
  //   }
  // }
  // Regularly clean the buffer and retain the latest CSI_FIFO_LENGTH samples
  if (CSI_Q_INDEX > CSI_FIFO_LENGTH * 1.5)
  {
    memmove(CSI_Q, CSI_Q + (CSI_Q_INDEX - CSI_FIFO_LENGTH), CSI_FIFO_LENGTH * sizeof(int16_t));
    CSI_Q_INDEX = CSI_FIFO_LENGTH;
    ESP_LOGI(TAG, "CSI buffer trimmed to %d samples", CSI_Q_INDEX);
  }
}
// [2] END OF YOUR CODE

#define CONFIG_LESS_INTERFERENCE_CHANNEL 40
#define CONFIG_WIFI_BAND_MODE WIFI_BAND_MODE_5G_ONLY
#define CONFIG_WIFI_2G_BANDWIDTHS WIFI_BW_HT20
#define CONFIG_WIFI_5G_BANDWIDTHS WIFI_BW_HT20
#define CONFIG_WIFI_2G_PROTOCOL WIFI_PROTOCOL_11N
#define CONFIG_WIFI_5G_PROTOCOL WIFI_PROTOCOL_11N
#define CONFIG_ESP_NOW_PHYMODE WIFI_PHY_MODE_HT20
#define CONFIG_ESP_NOW_RATE WIFI_PHY_RATE_MCS0_LGI
#define CONFIG_FORCE_GAIN 1
#define CONFIG_GAIN_CONTROL CONFIG_FORCE_GAIN

// UPDATE: Define parameters for scan method
#if CONFIG_EXAMPLE_WIFI_ALL_CHANNEL_SCAN
#define DEFAULT_SCAN_METHOD WIFI_ALL_CHANNEL_SCAN
#elif CONFIG_EXAMPLE_WIFI_FAST_SCAN
#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
#else
#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
#endif /*CONFIG_EXAMPLE_SCAN_METHOD*/
//
// !Note: change to your current setting
static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x00, 0x03, 0x7f, 0x00, 0x00, 0x00};
// static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x64, 0x2c, 0xac, 0xa2, 0x3f, 0x93};
// Obtain the MAC address of the device itself
// static uint8_t local_mac[6];
// esp_wifi_get_mac(WIFI_IF_STA, local_mac);
// ESP_LOGI(TAG, "Device MAC Address: " MACSTR, MAC2STR(local_mac));

typedef struct
{
  unsigned : 32; /**< reserved */
  unsigned : 32; /**< reserved */
  unsigned : 32; /**< reserved */
  unsigned : 32; /**< reserved */
  unsigned : 32; /**< reserved */
  unsigned : 16; /**< reserved */
  unsigned fft_gain : 8;
  unsigned agc_gain : 8;
  unsigned : 32; /**< reserved */
  unsigned : 32; /**< reserved */
  unsigned : 32; /**< reserved */
  unsigned : 32; /**< reserved */
  unsigned : 32; /**< reserved */
  unsigned : 32; /**< reserved */
} wifi_pkt_rx_ctrl_phy_t;

#if CONFIG_FORCE_GAIN
/**
 * @brief Enable/disable automatic fft gain control and set its value
 * @param[in] force_en true to disable automatic fft gain control
 * @param[in] force_value forced fft gain value
 */
extern void phy_fft_scale_force(bool force_en, uint8_t force_value);

/**
 * @brief Enable/disable automatic gain control and set its value
 * @param[in] force_en true to disable automatic gain control
 * @param[in] force_value forced gain value
 */
extern void phy_force_rx_gain(int force_en, int force_value);
#endif

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);
// 这里原来的 wifi_connected 提升到开头了

//------------------------------------------------------WiFi Initialize------------------------------------------------------
static void wifi_init()
{
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(esp_netif_init());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      IP_EVENT_STA_GOT_IP,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      &instance_got_ip));

  // [3] YOUR CODE HERE
  // You need to modify the ssid and password to match your Wi-Fi network.
  // !Note: change to your current setting
  wifi_config_t wifi_config = {
      .sta = {
          .ssid = "温朝宗的iPhone",
          .password = "1234567890",
          .threshold.authmode = WIFI_AUTH_WPA2_PSK,
          // UPDATES: only use this scan method when you want to connect your mobile phone's hotpot
          .scan_method = DEFAULT_SCAN_METHOD,

          .pmf_cfg = {
              .capable = true,
              .required = false},
      },
  };
  // [3] END OF YOUR CODE

  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_LOGI(TAG, "wifi_init finished.");
}

//------------------------------------------------------WiFi Event Handler------------------------------------------------------
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    ESP_LOGI(TAG, "Trying to connect to AP...");
    esp_wifi_connect();
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    ESP_LOGI(TAG, "Connection failed! Retrying...");
    wifi_connected = false;
    esp_wifi_connect();
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
    wifi_connected = true;

    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
    {
      ESP_LOGI(TAG, "Connected to AP - SSID: %s, Channel: %d, RSSI: %d",
               ap_info.ssid, ap_info.primary, ap_info.rssi);
    }
  }
}

//------------------------------------------------------ESP-NOW Initialize------------------------------------------------------
static void wifi_esp_now_init(esp_now_peer_info_t peer)
{
  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));
  esp_now_rate_config_t rate_config = {
      .phymode = CONFIG_ESP_NOW_PHYMODE,
      .rate = CONFIG_ESP_NOW_RATE, //  WIFI_PHY_RATE_MCS0_LGI,
      .ersu = false,
      .dcm = false};
  ESP_ERROR_CHECK(esp_now_add_peer(&peer));
  ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer.peer_addr, &rate_config));
  ESP_LOGI(TAG, "================ ESP NOW Ready ================");
  ESP_LOGI(TAG, "esp_now_init finished.");
}

//------------------------------------------------------CSI Callback------------------------------------------------------
static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
  if (!info || !info->buf)
    return;

  ESP_LOGI(TAG, "CSI callback triggered");

  // Applying the CSI_Q_ENABLE flag to determine the output method
  // 1: Enable, using buffer, 0: Disable, using serial output
  if (!CSI_Q_ENABLE)
  {
    ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d\n",
               info->len, MAC2STR(info->mac), info->rx_ctrl.rssi,
               info->rx_ctrl.rate, info->rx_ctrl.noise_floor,
               info->rx_ctrl.channel);
  }
  else
  {
    csi_process(info->buf, info->len);
  }

  if (!info || !info->buf)
  {
    ESP_LOGW(TAG, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
    return;
  }

  ESP_LOGI(TAG, "Received MAC: " MACSTR ", Expected MAC: " MACSTR,
           MAC2STR(info->mac), MAC2STR(CONFIG_CSI_SEND_MAC));

  if (memcmp(info->mac, CONFIG_CSI_SEND_MAC, 6))
  {
    ESP_LOGI(TAG, "MAC address doesn't match, skipping packet");
    return;
  }
  // extern uint8_t local_mac[6]; // Declare external variables

  // ESP_LOGI(TAG, "Received MAC: " MACSTR ", Local MAC: " MACSTR,
  //          MAC2STR(info->mac), MAC2STR(local_mac));

  // if (memcmp(info->mac, local_mac, 6) == 0)
  // {
  //     ESP_LOGI(TAG, "Skipping packet from self (local MAC)");
  //     return;
  // }

  wifi_pkt_rx_ctrl_phy_t *phy_info = (wifi_pkt_rx_ctrl_phy_t *)info;
  static int s_count = 0;

#if CONFIG_GAIN_CONTROL
  static uint16_t agc_gain_sum = 0;
  static uint16_t fft_gain_sum = 0;
  static uint8_t agc_gain_force_value = 0;
  static uint8_t fft_gain_force_value = 0;
  if (s_count < 100)
  {
    agc_gain_sum += phy_info->agc_gain;
    fft_gain_sum += phy_info->fft_gain;
  }
  else if (s_count == 100)
  {
    agc_gain_force_value = agc_gain_sum / 100;
    fft_gain_force_value = fft_gain_sum / 100;
#if CONFIG_FORCE_GAIN
    phy_fft_scale_force(1, fft_gain_force_value);
    phy_force_rx_gain(1, agc_gain_force_value);
#endif
    ESP_LOGI(TAG, "fft_force %d, agc_force %d", fft_gain_force_value, agc_gain_force_value);
  }
#endif

  const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
  if (CSI_Q_ENABLE == 0)
  {
    ESP_LOGI(TAG, "================ CSI RECV via Serial Port ================");
    ets_printf("type, seq, mac, rssi, rate, noise_floor, fft_gain, agc_gain, channel, timestamp, sig_len, rx_state, len, first_word_invalid, data \n");
    ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d",
               s_count++, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate,
               rx_ctrl->noise_floor, phy_info->fft_gain, phy_info->agc_gain, rx_ctrl->channel,
               rx_ctrl->timestamp, rx_ctrl->sig_len, rx_ctrl->rx_state);
    ets_printf(",%d,%d,\"[%d", info->len, info->first_word_invalid, info->buf[0]);

    for (int i = 1; i < info->len; i++)
    {
      ets_printf(",%d", info->buf[i]);
    }
    ets_printf("]\"\n");
  }

  else
  {
    ESP_LOGI(TAG, "================ CSI RECV via Buffer ================");
    csi_process(info->buf, info->len);
  }
}

//------------------------------------------------------CSI Processing & Algorithms------------------------------------------------------
static void csi_process(const int8_t *csi_data, int length)
{
  ESP_LOGI(TAG, "CSI Processing...");
  if (CSI_Q_INDEX + length > CSI_BUFFER_LENGTH)
  {
    int shift_size = CSI_BUFFER_LENGTH - CSI_FIFO_LENGTH;
    memmove(CSI_Q, CSI_Q + CSI_FIFO_LENGTH, shift_size * sizeof(int16_t));
    CSI_Q_INDEX = shift_size;
  }
  ESP_LOGI(TAG, "CSI Buffer Status: %d samples stored", CSI_Q_INDEX);
  // Append new CSI data to the buffer
  for (int i = 0; i < length && CSI_Q_INDEX < CSI_BUFFER_LENGTH; i++)
  {
    CSI_Q[CSI_Q_INDEX++] = (int16_t)csi_data[i];
  }

  // [4] YOUR CODE HERE

  // 1. Fill the information of your group members
  ESP_LOGI(TAG, "================ GROUP INFO ================");
  const char *TEAM_MEMBER[] = {"Wang Zimo", "Yang Zhuang", "Liu Yuting", "Shen Yuhang"};
  const char *TEAM_UID[] = {"3036381151", "3036408961", "3036382313", "3036381474"};
  ESP_LOGI(TAG, "TEAM_MEMBER: %s, %s, %s, %s | TEAM_UID: %s, %s, %s, %s",
           TEAM_MEMBER[0], TEAM_MEMBER[1], TEAM_MEMBER[2], TEAM_MEMBER[3],
           TEAM_UID[0], TEAM_UID[1], TEAM_UID[2], TEAM_UID[3]);
  ESP_LOGI(TAG, "================ END OF GROUP INFO ================");

  // 2. Call your algorithm functions here, e.g.: motion_detection(), breathing_rate_estimation(), and mqtt_send()
  // If you implement the algorithm on-board, you can return the results to the host, else send the CSI data.
  ESP_LOGI(TAG, "================ START OF MOTION DETECTION ================");
  motion_detected = motion_detection(true);
  // motion_detected = true;
  ESP_LOGI(TAG, "Motion detected: %s (Amplitude: %.1f, Intensity: %d)",
           motion_detected ? "YES" : "NO", g_motion_amplitude, g_motion_intensity);
  ESP_LOGI(TAG, "================ END OF MOTION DETECTION ================");
  ESP_LOGI(TAG, "================ START OF BREATH DETECTION ================");
  breathing_rate = breathing_rate_estimation();
  // breathing_rate = 12;
  ESP_LOGI(TAG, "Breathing rate: %d breaths/minute", breathing_rate);
  ESP_LOGI(TAG, "================ END OF BREATH DETECTION ================");
  mqtt_send(motion_detected, breathing_rate);
  // [4] END YOUR CODE HERE
}

//------------------------------------------------------CSI Config Initialize------------------------------------------------------
static void wifi_csi_init()
{
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
  wifi_csi_config_t csi_config = {
      .enable = true,
      .acquire_csi_legacy = false,
      .acquire_csi_force_lltf = false,
      .acquire_csi_ht20 = true,
      .acquire_csi_ht40 = true,
      .acquire_csi_vht = false,
      .acquire_csi_su = false,
      .acquire_csi_mu = false,
      .acquire_csi_dcm = false,
      .acquire_csi_beamformed = false,
      .acquire_csi_he_stbc_mode = 2,
      .val_scale_cfg = 0,
      .dump_ack_en = false,
      .reserved = false};
  ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
  ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
  ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

//------------------------------------------------------Main Function------------------------------------------------------
void app_main()
{
  /**
   * @brief Initialize NVS
   */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  /**
   * @brief Initialize Wi-Fi
   */
  wifi_init();

  // Get Device MAC Address
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  ESP_LOGI(TAG, "Device MAC Address: " MACSTR, MAC2STR(mac));

  // Try to connect to WiFi
  ESP_LOGI(TAG, "Connecting to WiFi...");

  // Wait for Wi-Fi connection
  int retry_count = 0;
  bool wifi_connected = false;
  while (!wifi_connected && retry_count < 20)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    retry_count++;
    ESP_LOGI(TAG, "Waiting for Wi-Fi connection... (%d/20)", retry_count);

    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
    {
      ESP_LOGI(TAG, "Connected to SSID: %s, RSSI: %d, Channel: %d",
               ap_info.ssid, ap_info.rssi, ap_info.primary);
      wifi_connected = true;
    }
  }

  /**
   * @brief Initialize ESP-NOW
   */

  if (wifi_connected)
  {
    esp_now_peer_info_t peer = {
        .channel = CONFIG_LESS_INTERFERENCE_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
        .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    };
    // Initialize the MQTT client
    if (init_mqtt())
    {
      ESP_LOGI(TAG, "MQTT client initialized and started successfully!");
    }
    else
    {
      ESP_LOGE(TAG, "MQTT client initialization failed!");
    }
    wifi_esp_now_init(peer); // Initialize ESP-NOW Communication
    wifi_csi_init();         // Initialize CSI Collection
    // 用于测试 Mock 传参
    // for (int i = 0; i < 800; i++)
    // {
    //   mqtt_send(true, i + 10);
    //   usleep(300000);
    // }
  }
  else
  {
    ESP_LOGI(TAG, "WiFi connection failed");
    return;
  }
}
