#ifndef ECG_RESP_ALGO_H  // 防止头文件重复包含
#define ECG_RESP_ALGO_H  // 定义宏，标识头文件开始

#include "Arduino.h"  // 包含Arduino核心库

#define TEMP_SENSOR 0  // 温度传感器定义（未使用）
#define FILTER_ORDER         161  // 滤波器阶数，定义滤波器长度
#define DC_REMOVE_COEFF (0.992)  // 直流去除系数，用于去除信号中的直流分量
#define WAVEFORM_SIZE  1  // 波形大小定义（未使用）

// ECG滤波器设置
#define MAX_PEAKS_TO_FIND         5  // 最大寻找的峰值数量
#define MAX_SEARCH_WINDOW      25  // 最大搜索窗口大小
#define MIN_SKIP_WINDOW       30  // 最小跳过窗口大小
#define SAMPLE_RATE             125  // 采样率，每秒采样次数
#define TWO_SEC_SAMPLES_COUNT   2 * SAMPLE_RATE  // 两秒内的采样数
#define QRS_THRESH_RATIO    0.4  // QRS波阈值比率
#define TRUE_VAL 1  // 真值定义
#define FALSE_VAL 0  // 假值定义

// 心电和呼吸信号处理类定义
class ECGRespirationProcessor
{
  public:
    // 公共成员函数
    void FilterECGSignal(int16_t * WorkBuffer, int16_t * CoeffBuffer, int16_t* FilterResult);  // ECG信号滤波函数
    void ProcessECGSample(int16_t *CurrentSample, int16_t *FilteredData);  // ECG信号处理函数
    void AnalyzeQRS(int16_t CurrentSample, volatile uint8_t *HeartRate);  // QRS波分析函数
    void FilterRespirationSignal(int16_t * RespWorkBuffer, int16_t * CoeffBuffer, int16_t* FilterResult);  // 呼吸信号滤波函数
    int16_t ProcessRespirationSample(int16_t CurrentSample);  // 呼吸信号处理函数
    void AnalyzeRespiration(int16_t CurrentSample, volatile uint8_t *BreathRate);  // 呼吸率分析函数
    
  private:
    // 私有成员函数
    void ProcessQRSBuffer(volatile uint8_t *HeartRate);  // QRS缓冲区处理函数
    void CheckQRSThreshold(uint16_t ScaledValue, volatile uint8_t *HeartRate);  // QRS阈值检查函数
    void DetectBreathRate(int16_t RespirationWave, volatile uint8_t *BreathRate);  // 呼吸率检测函数
};

#endif  // 头文件结束