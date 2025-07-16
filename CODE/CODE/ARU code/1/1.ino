#include "Ads1292r.h"  // 包含ADS1292R芯片控制库
#include "ecg_Res_Algo.h"  // 包含心电和呼吸信号处理算法库
#include <SPI.h>  // 包含SPI通信库

volatile uint8_t globalHR = 0;  // 全局变量，存储心率值（单位：次/分钟）
volatile uint8_t globalRR = 0;  // 全局变量，存储呼吸率值（单位：次/分钟）

const int ADS1292_DRDY_PIN = 6;  // ADS1292R数据就绪引脚
const int ADS1292_CS_PIN = 7;    // ADS1292R片选引脚
const int ADS1292_START_PIN = 5; // ADS1292R启动转换引脚
const int ADS1292_PWDN_PIN = 4;  // ADS1292R电源控制引脚

// 数据包协议定义
#define PKT_START_1   0x0A      // 数据包起始字节1
#define PKT_START_2   0xFA      // 数据包起始字节2
#define PKT_TYPE_DATA 0x02      // 数据类型标识
#define PKT_STOP      0x0B      // 数据包结束字节
#define DATA_SIZE     9         // 数据包大小
#define ZERO_VAL      0         // 填充零值

volatile char DataPacket[DATA_SIZE];  // 数据包缓冲区
const char PacketEnd[2] = {ZERO_VAL, PKT_STOP};  // 数据包结束标记
const char PacketStart[5] = {PKT_START_1, PKT_START_2, DATA_SIZE, ZERO_VAL, PKT_TYPE_DATA};  // 数据包起始标记

int16_t ecgRaw, ecgFiltered;  // 原始心电信号和滤波后心电信号
int16_t respRaw, respFiltered;  // 原始呼吸信号和滤波后呼吸信号

ads1292r ADS1292;  // ADS1292R芯片控制对象
ECGRespirationProcessor ECGProcessor;  // 心电和呼吸信号处理对象

// 发送UART数据包函数
void sendUARTData()
{
  // 填充数据包内容
  DataPacket[0] = ecgFiltered;       // 滤波后心电信号低字节
  DataPacket[1] = ecgFiltered >> 8;  // 滤波后心电信号高字节
  DataPacket[2] = respRaw;           // 原始呼吸信号低字节
  DataPacket[3] = respRaw >> 8;      // 原始呼吸信号高字节
  DataPacket[4] = globalRR;          // 呼吸率低字节
  DataPacket[5] = globalRR >> 8;     // 呼吸率高字节
  DataPacket[6] = globalHR;          // 心率低字节
  DataPacket[7] = globalHR >> 8;     // 心率高字节
  DataPacket[8] = 0;                 // 保留字节

  // 发送数据包起始标记
  for (int i = 0; i < 5; i++)
  {
    Serial.write(PacketStart[i]);
  }

  // 发送数据包内容
  for (int i = 0; i < DATA_SIZE; i++)
  {
    Serial.write(DataPacket[i]);
  }

  // 发送数据包结束标记
  for (int i = 0; i < 2; i++)
  {
    Serial.write(PacketEnd[i]);
  }
}

// 初始化函数
void setup()
{
  delay(2000);  // 延时2秒等待系统稳定

  // 初始化SPI通信
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

  // 设置ADS1292R引脚模式
  pinMode(ADS1292_DRDY_PIN, INPUT);   // 数据就绪引脚为输入
  pinMode(ADS1292_CS_PIN, OUTPUT);    // 片选引脚为输出
  pinMode(ADS1292_START_PIN, OUTPUT); // 启动引脚为输出
  pinMode(ADS1292_PWDN_PIN, OUTPUT);  // 电源控制引脚为输出

  Serial.begin(57600);  // 初始化串口通信，波特率57600
  ADS1292.ads1292Init(ADS1292_CS_PIN, ADS1292_PWDN_PIN, ADS1292_START_PIN);  // 初始化ADS1292R
  Serial.println("Initialization complete");  // 打印初始化完成信息
}

// 主循环函数
void loop()
{
  ads1292OutputValues sensorData;  // 存储传感器数据的结构体
  // 获取ADS1292R的心电和呼吸数据
  bool dataReady = ADS1292.getAds1292EcgAndRespirationSamples(ADS1292_DRDY_PIN, ADS1292_CS_PIN, &sensorData);

  if (dataReady)  // 如果数据就绪
  {
    // 提取原始心电信号（右移8位降低分辨率）
    ecgRaw = (int16_t)(sensorData.channelValues[1] >> 8);  
    // 提取原始呼吸信号（右移8位降低分辨率）
    respRaw = (int16_t)(sensorData.respValue >> 8);       

    if (sensorData.leadoffDetected == false)  // 如果电极接触良好
    {
      // 处理心电信号
      ECGProcessor.ProcessECGSample(&ecgRaw, &ecgFiltered);
      // 分析QRS波并计算心率
      ECGProcessor.AnalyzeQRS(ecgFiltered, &globalHR);
      // 呼吸信号处理（当前注释掉）
      // respFiltered = ECGProcessor.ProcessRespirationSample(respRaw);
      // ECGProcessor.AnalyzeRespiration(respFiltered, &globalRR);
    }
    else  // 如果电极脱落
    {
      ecgFiltered = 0;    // 心电信号清零
      respFiltered = 0;   // 呼吸信号清零
    }

    sendUARTData();  // 发送数据包
  }
}
