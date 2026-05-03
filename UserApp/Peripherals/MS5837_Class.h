#ifndef __MS5837_CLASS_H__
#define __MS5837_CLASS_H__

#include "main.h"
#include "i2c.h"
#include <math.h>

#define MS5837_ADDR            0x76 << 1
#define MS5837_RESET           0x1E
#define MS5837_ADC_READ        0x00
#define MS5837_PROM_READ       0xA0
#define MS5837_CONVERT_D1_8192 0x4A
#define MS5837_CONVERT_D2_8192 0x5A

// Models:
#define MS5837_30BA  0x00
#define MS5837_02BA  0x01

#define waterDensity 1029

// Values red from MS5837
typedef struct {
    int32_t TEMP;
    int32_t P;
    uint16_t C[8];
    uint32_t D1;
    uint32_t D2;
} MS5837_values;

class MS5837
{
private:
    I2C_HandleTypeDef* hi2c;
    uint8_t slave_address;
    // MS5837 model(Default MS5837_30BA)
    uint8_t model;
    // Fluid density (Default 1029)
    float fluidDensity;
    // Pressure unit (Default mBar)
    float temperture;
    float pressure;
    MS5837_values m_MS5837_values;
private:
    bool transmitByte(uint8_t *pData);
    bool receiveByte(uint8_t *pData);
    bool receive(uint8_t *pData, uint16_t Size);

    inline int8_t read8(uint8_t addr);
    inline int16_t read16(uint8_t addr);
    inline int32_t read32(uint8_t addr);
    uint8_t crc4(uint16_t n_prom[]);
    void calculate();
public:
    bool is_connected;
public:
    void Init(void);

    void Read();
    void Depth(float *p);
    
    inline void altitude(float *p);

    MS5837(I2C_HandleTypeDef* hi2c, uint8_t SLAVE_ADDRESS = MS5837_ADDR, uint16_t MemAddSize = 0);
    ~MS5837();
};

#endif

// // 1. 触发读取并进行内部计算 (calculate)
// depthSensor.Read();

// // 2. 获取压力和温度
// float currentP = depthSensor.pressure;    // 单位：mbar 或 Pa (取决于你的宏定义)
// float currentT = depthSensor.temperture;  // 单位：摄氏度

// // 3. 计算深度
// float currentDepth = 0;
// depthSensor.Depth(&currentDepth); // 结果存入 currentDepth，单位：米

// // 打印数据调试
// printf("Depth: %.2f m, Temp: %.2f C\r\n", currentDepth, currentT);

// Delay(100); // 控制采样频率