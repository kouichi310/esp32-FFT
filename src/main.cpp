#include <WiFi.h>
#include <PubSubClient.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <Ticker.h>
#include <ESP32Servo.h>

// WiFi
const char *ssid = "test";
const char *password = "bbbbbbbb";
WiFiClient espClient;

// MQTTブローカー
const char *mqtt_broker = "192.168.137.1";
String topic = "mqtt-topics/esp32-" + String(WiFi.macAddress());
const char *mqtt_username = "emqx-user";
const char *mqtt_password = "emqx-user-password";
const int mqtt_port = 1883;
PubSubClient client(espClient);

// Ticker
Ticker ticker_heartbeat;
Ticker ticker_fft;

// FFT
#define SAMPLES 1024
#define SAMPLING_FREQUENCY 16000
arduinoFFT FFT = arduinoFFT();
double vReal[SAMPLES];
double vImag[SAMPLES];

// I2S
// esp32mini board
#define I2S_WS 4
#define I2S_SD 2
#define I2S_SCK 3

// esp32dev board
// #define I2S_WS 25
// #define I2S_SD 21
// #define I2S_SCK 22

#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE (16000)
#define I2S_SAMPLE_BITS (16)
#define I2S_READ_LEN (16 * 1024)

int i2s_read_len = I2S_READ_LEN;
int cnt = 0;
size_t bytes_read;
const int BLOCK_SIZE = 1024;
int32_t samples[BLOCK_SIZE];
uint8_t *flash_write_buff = (uint8_t *)calloc(i2s_read_len, sizeof(char));

// Servoモーター
Servo sg90;
const int SG90_PIN = 5;
#define SG90_MIN    500
#define SG90_MAX    2400

// esp32 dev board
//const int SV_PIN = 23;

// プロトタイプ宣言
void send_heartbeat();
void performFFT();
void init_wifi();
void init_mqtt();
void init_i2s();
void i2s_adc_data_scale(uint8_t *d_buff, uint8_t *s_buff, uint32_t len);

void setup()
{
  Serial.begin(115200);

  init_wifi();
  init_mqtt();
  init_i2s();

  sg90.attach(SG90_PIN, SG90_MIN, SG90_MAX);
  sg90.write(0);

  ticker_heartbeat.attach(10, send_heartbeat);
  ticker_fft.attach(1, performFFT);
}

void loop()
{
  // noting to do
}

void send_heartbeat()
{
  client.publish(topic.c_str(), "heartbeat ping=0");
}

void performFFT()
{
  size_t bytes_read=0;

  i2s_read(I2S_PORT, (char *)samples, BLOCK_SIZE, &bytes_read, portMAX_DELAY);
  i2s_adc_data_scale(flash_write_buff, (uint8_t *)samples, bytes_read);
  for (uint16_t i = 0; i < BLOCK_SIZE; i++)
  {
    vReal[i] = samples[i] << 4;
    vImag[i] = 0.0;
  }

  // FFTを実行
  FFT.Windowing(vReal, BLOCK_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, BLOCK_SIZE, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, BLOCK_SIZE);

  // 1000Hzの成分を探す
  int frequency = 0;
  int indexAt1000Hz = (1000 * SAMPLES) / SAMPLING_FREQUENCY;
  frequency = vReal[indexAt1000Hz] / 1000;

  // サーボモーターを動かす
  if (frequency > 40000) {
    if (cnt == 10) {
      sg90.write(180);
      cnt = 0;
    } else {
      cnt++;
    }
  } else {
    cnt = 0;
  }

  // 結果を表示
  Serial.println(frequency);

  client.publish(topic.c_str(), ("fft 1000hz=" + String(frequency)).c_str());
}

void init_wifi()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
}

void init_mqtt()
{
  client.setServer(mqtt_broker, mqtt_port);
  while (!client.connected())
  {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.println("Connecting to MQTT...");
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void init_i2s()
{
  esp_err_t err;
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = I2S_SAMPLE_RATE,
      .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S | I2S_COMM_FORMAT_STAND_MSB),
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = 1024,
      .use_apll = 1};

  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK)
  {
    Serial.println("Failed to install driver");
  }
  else
  {
    Serial.println("Installed driver");
  }

  const i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_out_num = -1,
      .data_in_num = I2S_SD};

  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK)
  {
    Serial.println("Failed to install pin");
  }
  else
  {
    Serial.println("Installed pin");
  }
}

void i2s_adc_data_scale(uint8_t *d_buff, uint8_t *s_buff, uint32_t len)
{
  uint32_t j = 0;
  uint32_t dac_value = 0;
  for (int i = 0; i < len; i += 2)
  {
    dac_value = ((((uint16_t)(s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
    d_buff[j++] = 0;
    d_buff[j++] = dac_value * 256 / 2048;
  }
}