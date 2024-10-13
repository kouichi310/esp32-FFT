#include "arduinoFFT.h"  // FFTライブラリのインクルード
#include <SPIFFS.h>
#include <Ticker.h> 

#define SAMPLES 512  // サンプル数（2の累乗である必要があります）
#define SAMPLING_FREQUENCY 10000  // サンプリング周波数（10kHz）

arduinoFFT FFT = arduinoFFT();
File spiffs_file;
unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[SAMPLES]; // 実数部分
double vImag[SAMPLES]; // 虚数部分（FFTではゼロで初期化）

const int micPin = 34; // マイクのアナログ入力ピン

const char fname[] = "/data1";
Ticker ticker;

void SPIFFSInit(){
  Serial.println("SPIFFS formatting...");
  SPIFFS.format();
 if(!SPIFFS.begin(true)){
  Serial.println("SPIFFS initialisation failed!");
  while(1) yield();
 }
 SPIFFS.remove(fname);
 spiffs_file = SPIFFS.open(fname, FILE_WRITE);
 if(!spiffs_file){
  Serial.println("File is not available!");
  while(1) yield();
 }
}

void performFFT(){
  // サンプリング
  for (int i = 0; i < SAMPLES; i++) {
    microseconds = micros(); // 現在の時間を取得
    vReal[i] = analogRead(micPin);  // マイクのアナログ値を取得
    vImag[i] = 0;  // 虚数部分はゼロに設定

    while (micros() < (microseconds + sampling_period_us)) {
      // 次のサンプルまで待機
    }
  }

  // FFTを実行
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  // 400Hzの成分を探す
  double frequency = 0;
  int indexAt400Hz = (400 * SAMPLES) / SAMPLING_FREQUENCY;  // 400Hzに対応するインデックス
  frequency = vReal[indexAt400Hz];

  // 結果を表示
  String output = String(millis()/100) + "," + String(frequency);
  Serial.println(output);

  //spiffs_file.println(output);
  //spiffs_file.flush();
}

void setup() {
  //SPIFFSInit();
  Serial.begin(115200);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  ticker.attach(0.2, performFFT);
}

void loop() {

}
