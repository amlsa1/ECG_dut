#ifndef ADS1292R_H  // 防止头文件重复包含
#define ADS1292R_H  // 定义宏，标识头文件开始

#include "Arduino.h"  // 包含Arduino核心库

#define SPI_DUMMY_BYTE   0xFF  // 定义SPI通信中的哑字节值

// 定义SPI命令和寄存器地址
#define READ_REG  0x20;  // 读寄存器命令
#define WRITE_REG 0x40;  // 写寄存器命令
#define START_CONV 0x08  // 开始转换命令
#define STOP_CONV 0x0A   // 停止转换命令
#define CONT_READ 0x10   // 连续读取命令
#define STOP_CONT_READ 0x11  // 停止连续读取命令
#define SINGLE_READ 0x12  // 单次读取命令

// 定义ADS1292R寄存器地址
#define REG_ID 0x00       // 器件ID寄存器
#define REG_CONFIG1 0x01  // 配置寄存器1
#define REG_CONFIG2 0x02  // 配置寄存器2
#define REG_LOFF 0x03     // 导联脱落检测寄存器
#define REG_CH1SET 0x04   // 通道1设置寄存器
#define REG_CH2SET 0x05   // 通道2设置寄存器
#define REG_RLDSENS 0x06  // RLD感应寄存器
#define REG_LOFFSENS 0x07 // 导联脱落灵敏度寄存器
#define REG_LOFFSTAT 0x08 // 导联脱落状态寄存器
#define REG_RESP1 0x09    // 呼吸配置寄存器1
#define REG_RESP2 0x0A    // 呼吸配置寄存器2

// 定义存储传感器数据的结构体
typedef struct {
  volatile signed long channelValues[8];  // 存储8个通道的数据
  boolean leadoffDetected = true;        // 导联脱落检测标志
  signed long respValue;                 // 呼吸信号值
} ads1292OutputValues;

// ADS1292R控制类定义
class ads1292r
{
  public:
    // 公共成员函数
    boolean getAds1292EcgAndRespirationSamples(const int drdyPin, const int csPin, ads1292OutputValues *outputData);
    static void ads1292Init(const int csPin, const int pwdnPin, const int startPin);
    static void ads1292Reset(const int pwdnPin);

  private:
    // 私有成员函数
    static void ads1292RegWrite(unsigned char regAddr, unsigned char regData, const int csPin);
    static void ads1292SPISendCmd(unsigned char cmd, const int csPin);
    static void ads1292DisableStart(const int startPin);
    static void ads1292EnableStart(const int startPin);
    static void ads1292HardwareStop(const int startPin);
    static void ads1292StartConversion(const int csPin);
    static void ads1292SoftwareStop(const int csPin);
    static void ads1292StartContinuousRead(const int csPin);
    static void ads1292StopContinuousRead(const int csPin);
    static char* ads1292ReadData(const int csPin);
};

#endif  // 头文件结束