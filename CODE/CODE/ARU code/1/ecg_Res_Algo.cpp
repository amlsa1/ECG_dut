#include "Arduino.h"              // Arduino核心库
#include "Ads1292r.h"             // ADS1292R芯片的驱动库
#include "ecg_Res_Algo.h"         // 当前算法头文件
#include <SPI.h>                  // SPI总线支持库

// 全局变量定义
uint8_t SampleStartFlag = 0;      // 开始采样标志位
uint8_t FirstPeakFound = FALSE_VAL;  // 是否检测到第一个QRS峰

uint16_t SampleCounter = 0;       // 样本计数器
uint16_t PeakIndices[MAX_PEAKS_TO_FIND + 2]; // 峰值索引记录

int PrevQRS2 = 0, PrevQRS1 = 0, CurrentQRS = 0, NextQRS1 = 0, NextQRS2 = 0;  // ECG QRS波处理变量
int PrevResp2 = 0, PrevResp1 = 0, CurrentResp = 0, NextResp1 = 0, NextResp2 = 0; // 呼吸信号变量

uint8_t BreathRateValue = 0;        // 呼吸率结果
volatile uint16_t HeartRateValue = 0;  // 心率结果

static uint16_t ThresholdBufferPtr = 0;    // 阈值缓存指针
int16_t RespWorkBuffer[2 * FILTER_ORDER];  // 呼吸滤波缓存
int16_t PrevDCSample = 0, PrevSample = 0;  // DC偏移处理变量
int16_t OldQRSThreshold = 0, NewQRSThreshold = 0; // QRS检测阈值
int16_t ECGWorkBuffer[2 * FILTER_ORDER];   // ECG滤波缓存

// FIR低通滤波器系数（ECG）
int16_t LowPassCoeffs[FILTER_ORDER] = {
  // 系数略（保留原始161项）
};

// FIR滤波器系数（呼吸）
int16_t RespCoeffs[FILTER_ORDER] = {
  // 系数略（保留原始161项）
};

// ECG滤波函数
void ECGRespirationProcessor::FilterECGSignal(int16_t *WorkBuffer, int16_t *CoeffBuffer, int16_t* FilterResult) {
  int32_t sum = 0;
  for (int k = 0; k < FILTER_ORDER; k++) {
    sum += (int32_t)(*CoeffBuffer++) * (int32_t)(*WorkBuffer--);
  }
  sum = constrain(sum, -0x40000000, 0x3fffffff); // 饱和控制
  *FilterResult = (int16_t)(sum >> 15);
}

// ECG预处理：去直流、滤波
void ECGRespirationProcessor::ProcessECGSample(int16_t *CurrentSample, int16_t *FilteredData) {
  static uint16_t BufferStart = 0, BufferCurrent = FILTER_ORDER - 1;
  static int16_t PreviousDCSample = 0, PreviousSample = 0;
  static uint8_t FirstRun = 1;

  if (FirstRun) {
    memset(ECGWorkBuffer, 0, sizeof(ECGWorkBuffer));
    FirstRun = 0;
  }

  int tempVal1 = DC_REMOVE_COEFF * PreviousDCSample;
  PreviousDCSample = (CurrentSample[0] - PreviousSample) + tempVal1;
  PreviousSample = CurrentSample[0];
  int16_t ECGData = PreviousDCSample >> 2;

  ECGWorkBuffer[BufferCurrent] = ECGData;
  FilterECGSignal(&ECGWorkBuffer[BufferCurrent], LowPassCoeffs, FilteredData);
  ECGWorkBuffer[BufferStart] = ECGData;

  if (++BufferStart >= FILTER_ORDER - 1) BufferStart = 0;
  if (++BufferCurrent >= 2 * FILTER_ORDER) BufferCurrent = FILTER_ORDER - 1;
}

// QRS检测入口
void ECGRespirationProcessor::AnalyzeQRS(int16_t CurrentSample, volatile uint8_t *HeartRate) {
  static int16_t previousValues[32] = {0};
  long movingAvg = 0;
  previousValues[0] = CurrentSample;
  for (int i = 31; i > 0; i--) {
    movingAvg += previousValues[i];
    previousValues[i] = previousValues[i - 1];
  }
  movingAvg += CurrentSample;
  CurrentSample = (int16_t)(movingAvg >> 2);

  // 平移QRS缓存
  PrevQRS2 = PrevQRS1;
  PrevQRS1 = CurrentQRS;
  CurrentQRS = NextQRS1;
  NextQRS1 = NextQRS2;
  NextQRS2 = CurrentSample;

  ProcessQRSBuffer(HeartRate);
}

// QRS检测核心处理逻辑（导出心率）
void ECGRespirationProcessor::ProcessQRSBuffer(volatile uint8_t *HeartRate) {
  static int16_t MaxValue = 0;
  int16_t derivative = abs(NextQRS1 - PrevQRS1);

  if (derivative > MaxValue) MaxValue = derivative;
  ThresholdBufferPtr++;

  if (ThresholdBufferPtr >= TWO_SEC_SAMPLES_COUNT) {
    OldQRSThreshold = NewQRSThreshold = (MaxValue * 7) / 10;
    MaxValue = 0;
    FirstPeakFound = TRUE_VAL;
    ThresholdBufferPtr = 0;
  }

  if (FirstPeakFound == TRUE_VAL) {
    CheckQRSThreshold(derivative, HeartRate);
  }
}

// 呼吸滤波处理
void ECGRespirationProcessor::FilterRespirationSignal(int16_t *RespWorkBuffer, int16_t *CoeffBuffer, int16_t* FilterResult) {
  int32_t sum = 0;
  for (int k = 0; k < FILTER_ORDER; k++) {
    sum += (int32_t)(*CoeffBuffer++) * (int32_t)(*RespWorkBuffer--);
  }
  sum = constrain(sum, -0x40000000, 0x3fffffff);
  *FilterResult = (int16_t)(sum >> 15);
}

// 呼吸采样处理：滤波+去直流
int16_t ECGRespirationProcessor::ProcessRespirationSample(int16_t CurrentSample) {
  static uint16_t bufStart = 0, bufCurrent = FILTER_ORDER - 1;
  int temp1 = DC_REMOVE_COEFF * PrevDCSample;
  PrevDCSample = (CurrentSample - PrevSample) + temp1;
  PrevSample = CurrentSample;
  int16_t RespData = PrevDCSample;

  RespWorkBuffer[bufCurrent] = RespData;
  int16_t FilterOutput;
  FilterRespirationSignal(&RespWorkBuffer[bufCurrent], RespCoeffs, &FilterOutput);
  RespWorkBuffer[bufStart] = RespData;

  if (++bufStart >= FILTER_ORDER - 1) bufStart = 0;
  if (++bufCurrent >= 2 * FILTER_ORDER) bufCurrent = FILTER_ORDER - 1;

  return FilterOutput;
}

// 呼吸波处理入口，滑动平均+送入呼吸率检测
void ECGRespirationProcessor::AnalyzeRespiration(int16_t CurrentSample, volatile uint8_t *BreathRate) {
  static int16_t prevData[64] = {0};
  long movingAvg = 0;
  prevData[0] = CurrentSample;
  for (int i = 63; i > 0; i--) {
    movingAvg += prevData[i];
    prevData[i] = prevData[i - 1];
  }
  movingAvg += CurrentSample;
  CurrentSample = (int16_t)(movingAvg >> 1);

  // 移动窗口更新
  PrevResp2 = PrevResp1;
  PrevResp1 = CurrentResp;
  CurrentResp = NextResp1;
  NextResp1 = NextResp2;
  NextResp2 = CurrentSample;

  DetectBreathRate(NextResp2, BreathRate);
}

// 呼吸率检测核心算法
void ECGRespirationProcessor::DetectBreathRate(int16_t RespWave, volatile uint8_t *BreathRate) {
  // 已经移植到python代码了
  *BreathRate = BreathRateValue;
}