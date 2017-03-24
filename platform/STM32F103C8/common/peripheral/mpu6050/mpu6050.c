#include "stm32f10x.h"
#include "stdint.h"
#include "library.h"
#include "mpu6050.h"

#define acc_scale (9.80665f * 8.0f / 32768.0f)
#define gyro_scale (2000.0f / 32768.0f * 3.1415926536f / 180.0f)

mpu6050_struct_t mpu6050;
u8 mpu6050_ok;
u8 mpu6050_buffer[14];

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*��������:	  �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	dev  Ŀ���豸��ַ
reg	   �Ĵ�����ַ
bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1 
ʧ��Ϊ0
*******************************************************************************/ 
void IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
	u8 b;
	i2c_read_buffer(dev, reg, 1, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	mpu6050_ok = !( i2c_write_data(dev, reg, b) );
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
reg	   �Ĵ�����ַ
bitStart  Ŀ���ֽڵ���ʼλ
length   λ����
data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1 
ʧ��Ϊ0
*******************************************************************************/ 
void IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{
	
	u8 b,mask;
	i2c_read_buffer(dev, reg, 1, &b);
	mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
	data <<= (8 - length);
	data >>= (7 - bitStart);
	b &= mask;
	b |= data;
	i2c_write_data(dev, reg, b);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		
*��������:	    ���� ������
*******************************************************************************/
void MPU6050_set_SMPLRT_DIV(uint16_t hz)
{
	i2c_write_data(MPU6050_ADDR, MPU6050_RA_SMPLRT_DIV,1000/hz - 1);
		//I2C_Single_Write(MPU6050_ADDRESS,MPU_RA_SMPLRT_DIV, (1000/sample_rate - 1));
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setClockSource(uint8_t source)
*��������:	    ����  MPU6050 ��ʱ��Դ
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source)
{
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}
/** Set full-scale gyroscope range.
* @param range New full-scale gyroscope range value
* @see getFullScaleRange()
* @see MPU6050_GYRO_FS_250
* @see MPU6050_RA_GYRO_CONFIG
* @see MPU6050_GCONFIG_FS_SEL_BIT
* @see MPU6050_GCONFIG_FS_SEL_LENGTH
*/
void MPU6050_setFullScaleGyroRange(uint8_t range) {
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG,7, 3, 0x00);   //���Լ�
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*��������:	    ����  MPU6050 ���ٶȼƵ��������
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG,7, 3, 0x00);   //���Լ�
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setSleepEnabled(uint8_t enabled)
*��������:	    ����  MPU6050 �Ƿ����˯��ģʽ
enabled =1   ˯��
enabled =0   ����
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
	IICwriteBit(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
	IICwriteBit(MPU6050_ADDR, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
	IICwriteBit(MPU6050_ADDR, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		
*��������:	    ���õ�ͨ�˲���ֹƵ��
*******************************************************************************/
void MPU6050_setDLPF(uint8_t mode)
{
	IICwriteBits(MPU6050_ADDR, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

//void MPU6050_INT_Config()
//{
//	si2c_init();
//}

void mpu6050_init(void)
{
	u16 default_filter = 1;
	
	MPU6050_setSleepEnabled(0);																			  //���빤��״̬
	delay_ms(10);
	MPU6050_setClockSource(MPU6050_CLOCK_PLL_ZGYRO); 								  //����ʱ��  0x6b   0x03
	delay_ms(10);
	MPU6050_set_SMPLRT_DIV(1000);  																		//1000hz
	delay_ms(10);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);							//������������� +-2000��ÿ��
	delay_ms(10);
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_8);								//���ٶȶ�������� +-8G
	delay_ms(10);
	MPU6050_setDLPF(default_filter);																  //42hz
	delay_ms(10);
	MPU6050_setI2CMasterModeEnabled(0);	 															//����MPU6050 ����AUXI2C
	delay_ms(10);
	MPU6050_setI2CBypassEnabled(1);	 																	//����������I2C��	MPU6050��AUXI2C	ֱͨ������������ֱ�ӷ���HMC5883L
	delay_ms(10);
}

void mpu6050_read()
{
	int16_t acc_offset_x_temp = mpu6050.acc_offset.x;
	int16_t acc_offset_y_temp = mpu6050.acc_offset.y;
	int16_t acc_offset_z_temp = mpu6050.acc_offset.z;
	int16_t gyo_offset_x_temp = mpu6050.gyo_offset.x;
	int16_t gyo_offset_y_temp = mpu6050.gyo_offset.y;
	int16_t gyo_offset_z_temp = mpu6050.gyo_offset.z;
	
	I2C_FastMode = 1;
	i2c_read_buffer(MPU6050_ADDR,MPU6050_RA_ACCEL_XOUT_H,14,mpu6050_buffer);
	
	mpu6050.acc.x = ((((int16_t)mpu6050_buffer[ 0]) << 8) | mpu6050_buffer[ 1]) + acc_offset_x_temp;
	mpu6050.acc.y = ((((int16_t)mpu6050_buffer[ 2]) << 8) | mpu6050_buffer[ 3]) + acc_offset_y_temp;
	mpu6050.acc.z = ((((int16_t)mpu6050_buffer[ 4]) << 8) | mpu6050_buffer[ 5]) + acc_offset_z_temp;
	mpu6050.temp =  ((((int16_t)mpu6050_buffer[ 6]) << 8) | mpu6050_buffer[ 7]);
	mpu6050.gyo.x = ((((int16_t)mpu6050_buffer[ 8]) << 8) | mpu6050_buffer[ 9]) + gyo_offset_x_temp;
	mpu6050.gyo.y = ((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) + gyo_offset_y_temp;
	mpu6050.gyo.z = ((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) + gyo_offset_z_temp;
}

void mpu6050_get_data(float acc[], float gyro[])
{
	static uint32_t timestemp;
	static float pre_acc[3]={0};
	static float pre_gyro[3]={0};
	uint32_t current_time = sys_time();
	uint32_t dt = current_time - timestemp;
	timestemp = current_time;
	mpu6050_read();
	
	acc[0] = pre_acc[0] + (-mpu6050.acc.x * acc_scale - pre_acc[0]) * rc_filter(dt, 200);
	acc[1] = pre_acc[1] + (mpu6050.acc.y * acc_scale - pre_acc[1]) * rc_filter(dt, 200);
	acc[2] = pre_acc[2] + (-mpu6050.acc.z * acc_scale - pre_acc[2]) * rc_filter(dt, 200);
	
	gyro[0] = pre_gyro[0] + (-mpu6050.gyo.x * gyro_scale - pre_gyro[0]) * rc_filter(dt, 300);
	gyro[1] = pre_gyro[1] + (mpu6050.gyo.y * gyro_scale - pre_gyro[1]) * rc_filter(dt, 300);
	gyro[2] = pre_gyro[2] + (-mpu6050.gyo.z * gyro_scale - pre_gyro[2]) * rc_filter(dt, 300);
	
	pre_acc[0] = acc[0];
	pre_acc[1] = acc[1];
	pre_acc[2] = acc[2];
	pre_gyro[0] = gyro[0];
	pre_gyro[1] = gyro[1];
	pre_gyro[2] = gyro[2];
}

