#include <WiFi.h>
#include <PubSubClient.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <Ticker.h>

// WiFi
const char *ssid = "GL-AR750S-14e"; 
const char *password = "goodlife";

// MQTTブローカー  
const char *mqtt_broker = "192.168.8.228";
String topic = "mqtt-topics/esp32-" + String(WiFi.macAddress());
const char *mqtt_username = "emqx-user";
const char *mqtt_password = "emqx-user-password";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);
int tempratureValue;

Ticker ticker1;

#define SAMPLES 1024  // サンプル数（2の累乗である必要があります）
#define SAMPLING_FREQUENCY 16000  // サンプリング周波数（10kHz）

arduinoFFT FFT = arduinoFFT();
unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[SAMPLES]; // 実数部分
double vImag[SAMPLES]; // 虚数部分（FFTではゼロで初期化）

#define I2S_WS 4
#define I2S_SD 2
#define I2S_SCK 3
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE  (16000)
#define I2S_SAMPLE_BITS  (16)
#define I2S_READ_LEN   (16 * 1024)

int i2s_read_len = I2S_READ_LEN;
int flash_wr_size = 0;
size_t bytes_read;
const int BLOCK_SIZE = 512;
int32_t samples[BLOCK_SIZE];
char* i2s_read_buff = (char*) calloc(i2s_read_len, sizeof(char));
uint8_t* flash_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));

void init_wifi(){
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
}

void init_mqtt(){
  client.setServer(mqtt_broker, mqtt_port);
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.println("Connecting to MQTT...");
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void init_i2s(){
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
  .use_apll = 1
 };
 
 err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
 if (err!=ESP_OK) {
  Serial.println("Failed to install driver");
 } else {
  Serial.println("Installed driver");
 }
 
 const i2s_pin_config_t pin_config = {
  .bck_io_num = I2S_SCK,
  .ws_io_num = I2S_WS,
  .data_out_num = -1,
  .data_in_num = I2S_SD
 };
 
 err = i2s_set_pin(I2S_PORT, &pin_config);
 if (err!=ESP_OK) {
  Serial.println("Failed to install pin");
 } else {
  Serial.println("Installed pin");
 }
}
 
void i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len)
{
 uint32_t j = 0;
 uint32_t dac_value = 0;
 for (int i = 0; i < len; i += 2) {
  dac_value = ((((uint16_t) (s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
  d_buff[j++] = 0;
  d_buff[j++] = dac_value * 256 / 2048;
 }
}
 
void performFFT()
{
  size_t bytes_read;
  String result;
  
  //i2s_read(I2S_PORT, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
  i2s_read(I2S_PORT, (char *)samples, BLOCK_SIZE, &bytes_read, portMAX_DELAY);
  i2s_adc_data_scale(flash_write_buff, (uint8_t*)samples, bytes_read);
  for (uint16_t i = 0; i < BLOCK_SIZE; i++) {
    vReal[i] = samples[i] << 4;
    vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
  }


  // FFTを実行
  FFT.Windowing(vReal, BLOCK_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, BLOCK_SIZE, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, BLOCK_SIZE);

  // 400Hzの成分を探す
  int frequency = 0;
  int indexAt400Hz = (1000 * SAMPLES) / SAMPLING_FREQUENCY;  // 1000Hzに対応するインデックス
  frequency = vReal[indexAt400Hz];

  // 結果を表示
  Serial.println(frequency/1000);

  //client.publish(topic.c_str(), ("fft data=" + result).c_str());
  //Serial.println("fft data=" + result);
}

void setup(){
  Serial.begin(115200);
  init_wifi();
  init_mqtt();
  init_i2s();
  ticker1.attach(0.2, performFFT);
}

void loop(){
  tempratureValue = temperatureRead();
  //Serial.println("Temperature: " + String(tempratureValue) + "°C");
  client.publish(topic.c_str(), ("temp cpu=" + String(tempratureValue)).c_str());
  sleep(5);
}