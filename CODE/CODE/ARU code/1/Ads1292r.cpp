#include "Arduino.h"  // 包含Arduino核心库
#include "Ads1292r.h"  // 包含ADS1292R芯片驱动头文件
#include <SPI.h>      // 包含SPI通信库

// 全局变量定义
int dataIndex, bufferIndex;  // 数据索引和缓冲区索引

volatile byte SPI_RX_Buffer[15];  // SPI接收缓冲区（15字节）
volatile static int SPI_RX_Count = 0;  // SPI接收计数器
volatile char *SPI_RX_Ptr;         // SPI接收数据指针
volatile bool dataReceived = false;  // 数据接收标志位

// 临时变量用于数据处理
unsigned long tempECG = 0;    // 临时存储ECG原始数据
unsigned long tempResult = 0; // 临时存储处理结果
signed long signedECG = 0;    // 有符号ECG数据
long statusData = 0;          // 状态数据
uint8_t leadStatus = 0;       // 导联状态

// 获取ADS1292R心电和呼吸数据函数
boolean ads1292r::getAds1292EcgAndRespirationSamples(const int drdyPin, const int csPin, ads1292OutputValues *outputData)
{
  if ((digitalRead(drdyPin)) == LOW)  // 检查数据就绪引脚是否为低电平
  {
    SPI_RX_Ptr = ads1292ReadData(csPin);  // 读取SPI数据

    // 将读取的数据存入缓冲区
    for (int i = 0; i < 9; i++)
    {
      SPI_RX_Buffer[SPI_RX_Count++] = *(SPI_RX_Ptr + i);
    }

    dataReceived = true;  // 设置数据接收标志
    dataIndex = 0;        // 重置数据索引

    // 处理3个通道的ECG数据（每通道3字节）
    for (bufferIndex = 3; bufferIndex < 9; bufferIndex += 3)
    {
      // 将3字节数据组合为24位整数
      tempECG = (unsigned long)(((unsigned long)SPI_RX_Buffer[bufferIndex + 0] << 16) | 
                               ((unsigned long)SPI_RX_Buffer[bufferIndex + 1] << 8) | 
                               (unsigned long)SPI_RX_Buffer[bufferIndex + 2]);
      tempECG = (unsigned long)(tempECG << 8);  // 左移8位
      signedECG = (signed long)(tempECG);       // 转换为有符号数
      signedECG = (signed long)(signedECG >> 8); // 右移8位恢复原始值

      (outputData->channelValues)[dataIndex++] = signedECG;  // 存储处理后的ECG数据
    }

    // 处理状态数据（3字节）
    statusData = (long)((long)SPI_RX_Buffer[2] | 
                       ((long)SPI_RX_Buffer[1]) << 8 | 
                       ((long)SPI_RX_Buffer[0]) << 16);
    statusData = (statusData & 0x0f8000) >> 15;  // 提取导联状态位
    leadStatus = (unsigned char)statusData;      // 转换为无符号字符

    // 处理呼吸数据（3字节）
    tempResult = (uint32_t)((0 << 24) | (SPI_RX_Buffer[3] << 16) | 
                           SPI_RX_Buffer[4] << 8 | SPI_RX_Buffer[5]);
    tempResult = (uint32_t)(tempResult << 8);    // 左移8位
    outputData->respValue = (long)(tempResult);  // 存储呼吸数据

    // 检查导联状态
    if (!((leadStatus & 0x1f) == 0))
    {
      outputData->leadoffDetected = true;  // 检测到导联脱落
    }
    else
    {
      outputData->leadoffDetected = false; // 导联连接正常
    }

    // 重置标志和计数器
    dataReceived = false;
    SPI_RX_Count = 0;
    return true;  // 返回数据获取成功
  }
  else
    return false; // 返回数据未就绪
}

// 从ADS1292R读取数据函数
char* ads1292r::ads1292ReadData(const int csPin)
{
  static char SPI_Dummy_Buffer[10];  // 静态缓冲区（10字节）
  digitalWrite(csPin, LOW);         // 使能片选（低电平有效）

  // 通过SPI读取9字节数据
  for (int i = 0; i < 9; ++i)
  {
    SPI_Dummy_Buffer[i] = SPI.transfer(SPI_DUMMY_BYTE);  // 发送哑字节并接收数据
  }

  digitalWrite(csPin, HIGH);  // 禁用片选
  return SPI_Dummy_Buffer;    // 返回数据缓冲区
}

// ADS1292R初始化函数
void ads1292r::ads1292Init(const int csPin, const int pwdnPin, const int startPin)
{
  ads1292Reset(pwdnPin);        // 硬件复位
  delay(100);                   // 延时100ms
  ads1292DisableStart(startPin); // 禁用启动
  ads1292EnableStart(startPin);  // 启用启动
  ads1292HardwareStop(startPin); // 硬件停止
  ads1292StartConversion(csPin); // 开始转换
  ads1292SoftwareStop(csPin);    // 软件停止
  delay(50);                    // 延时50ms
  ads1292StopContinuousRead(csPin); // 停止连续读取
  delay(300);                   // 延时300ms

  // 配置寄存器（按数据手册要求）
  ads1292RegWrite(REG_CONFIG1, 0x00, csPin);       // 配置寄存器1
  delay(10);
  ads1292RegWrite(REG_CONFIG2, 0b10100000, csPin); // 配置寄存器2
  delay(10);
  ads1292RegWrite(REG_LOFF, 0b00010000, csPin);    // 导联脱落检测配置
  delay(10);
  ads1292RegWrite(REG_CH1SET, 0b01000000, csPin);  // 通道1配置
  delay(10);
  ads1292RegWrite(REG_CH2SET, 0b01100000, csPin);  // 通道2配置
  delay(10);
  ads1292RegWrite(REG_RLDSENS, 0b00101100, csPin); // RLD感应配置
  delay(10);
  ads1292RegWrite(REG_LOFFSENS, 0x00, csPin);      // 导联脱落灵敏度
  delay(10);
  ads1292RegWrite(REG_RESP1, 0b11110010, csPin);   // 呼吸配置1
  delay(10);
  ads1292RegWrite(REG_RESP2, 0b00000011, csPin);   // 呼吸配置2
  delay(10);
  ads1292StartContinuousRead(csPin);  // 开始连续读取模式
  delay(10);
  ads1292EnableStart(startPin);       // 最后启用启动
}

// 硬件复位函数
void ads1292r::ads1292Reset(const int pwdnPin)
{
  digitalWrite(pwdnPin, HIGH);  // 拉高电源控制引脚
  delay(100);                   // 延时100ms
  digitalWrite(pwdnPin, LOW);   // 拉低电源控制引脚
  delay(100);                   // 延时100ms
  digitalWrite(pwdnPin, HIGH);  // 再次拉高电源控制引脚
  delay(100);                   // 延时100ms
}

// 禁用启动函数
void ads1292r::ads1292DisableStart(const int startPin)
{
  digitalWrite(startPin, LOW);  // 拉低启动引脚
  delay(20);                    // 延时20ms
}

// 启用启动函数
void ads1292r::ads1292EnableStart(const int startPin)
{
  digitalWrite(startPin, HIGH); // 拉高启动引脚
  delay(20);                    // 延时20ms
}

// 硬件停止函数
void ads1292r::ads1292HardwareStop(const int startPin)
{
  digitalWrite(startPin, LOW);  // 拉低启动引脚
  delay(100);                   // 延时100ms
}

// 开始转换函数
void ads1292r::ads1292StartConversion(const int csPin)
{
  ads1292SPISendCmd(START_CONV, csPin);  // 发送开始转换命令
}

// 软件停止函数
void ads1292r::ads1292SoftwareStop(const int csPin)
{
  ads1292SPISendCmd(STOP_CONV, csPin);   // 发送停止转换命令
}

// 开始连续读取函数
void ads1292r::ads1292StartContinuousRead(const int csPin)
{
  ads1292SPISendCmd(CONT_READ, csPin);   // 发送连续读取命令
}

// 停止连续读取函数
void ads1292r::ads1292StopContinuousRead(const int csPin)
{
  ads1292SPISendCmd(STOP_CONT_READ, csPin); // 发送停止连续读取命令
}

// SPI发送命令函数
void ads1292r::ads1292SPISendCmd(unsigned char cmd, const int csPin)
{
  digitalWrite(csPin, LOW);   // 使能片选
  delay(2);                   // 短暂延时
  digitalWrite(csPin, HIGH);  // 禁用片选
  delay(2);                   // 短暂延时
  digitalWrite(csPin, LOW);   // 再次使能片选
  delay(2);                   // 短暂延时
  SPI.transfer(cmd);          // 发送命令字节
  delay(2);                   // 短暂延时
  digitalWrite(csPin, HIGH);  // 禁用片选
}

// 寄存器写入函数
void ads1292r::ads1292RegWrite(unsigned char regAddr, unsigned char regData, const int csPin)
{
  // 根据寄存器地址进行特定处理
  switch (regAddr)
  {
    case 1:
      regData = regData & 0x87;  // 保留特定bit
      break;
    case 2:
      regData = regData & 0xFB;  // 保留特定bit
      regData |= 0x80;           // 设置特定bit
      break;
    case 3:
      regData = regData & 0xFD;  // 保留特定bit
      regData |= 0x10;           // 设置特定bit
      break;
    case 7:
      regData = regData & 0x3F;  // 保留特定bit
      break;
    case 8:
      regData = regData & 0x5F;  // 保留特定bit
      break;
    case 9:
      regData |= 0x02;           // 设置特定bit
      break;
    case 10:
      regData = regData & 0x87;  // 保留特定bit
      regData |= 0x01;           // 设置特定bit
      break;
    case 11:
      regData = regData & 0x0F;  // 保留特定bit
      break;
    default:
      break;  // 其他寄存器不处理
  }
  
  byte dataToSend = regAddr | WRITE_REG;  // 组合寄存器地址和写命令
  digitalWrite(csPin, LOW);    // 使能片选
  delay(2);                    // 短暂延时
  digitalWrite(csPin, HIGH);   // 禁用片选
  delay(2);                    // 短暂延时
  digitalWrite(csPin, LOW);    // 再次使能片选
  delay(2);                    // 短暂延时
  SPI.transfer(dataToSend);    // 发送寄存器地址
  SPI.transfer(0x00);         // 发送哑字节
  SPI.transfer(regData);      // 发送寄存器数据
  delay(2);                   // 短暂延时
  digitalWrite(csPin, HIGH);  // 禁用片选
}