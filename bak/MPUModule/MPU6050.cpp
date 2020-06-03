#include "MPU6050.h"
#define I2C_ADDRESS 0x68 // I2C address of MPU6050

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_PWR_MGMT_1 0x6B

/**
 * @brief Construct an %MPU6050 handler.
 */
MPU6050::MPU6050()
{
	accel_x = accel_y = accel_z = 0;
	gyro_x = gyro_y = gyro_z = 0;
	i2c = nullptr;
	inited = false;
}

/**
 * @brief Destory the class instance.
 */
MPU6050::~MPU6050()
{
	delete i2c;
}

/**
 * @brief Read the acceleration value from the device.
 *
 * Calling this method results in communication with the device to retrieve the
 * acceleration data that is then stored in the class instance ready for retrieval.
 */
void MPU6050::readAccel()
{
	assert(inited);
	i2c->beginTransaction();
	i2c->write(MPU6050_ACCEL_XOUT_H);
	i2c->endTransaction();

	uint8_t data[6];
	i2c->beginTransaction();
	i2c->read(data, 5, true);
	i2c->read(data + 5, false);
	i2c->endTransaction();

	accel_x = (data[0] << 8) | data[1];
	accel_y = (data[2] << 8) | data[3];
	accel_z = (data[4] << 8) | data[5];
} // readAccel

/**
 * @brief Read the gyroscopic values from the device.
 *
 * Calling this method results in communication with the device to retrieve the
 * gyroscopic data that is then stored in the class instance ready for retrieval.
 */
void MPU6050::readGyro()
{
	assert(inited);
	i2c->beginTransaction();
	i2c->write(MPU6050_GYRO_XOUT_H);
	i2c->endTransaction();

	uint8_t data[6];
	i2c->beginTransaction();
	i2c->read(data, 5, true);
	i2c->read(data + 5, false);
	i2c->endTransaction();

	gyro_x = (data[0] << 8) | data[1];
	gyro_y = (data[2] << 8) | data[3];
	gyro_z = (data[4] << 8) | data[5];
} // readGyro

/**
 * @brief Initialize the %MPU6050.
 * @param [in] sdaPin The %GPIO pin to use for %I2C SDA.
 * @param [in] clkPin The %GPIO pin to use for %I2C CLK.
 *
 * This method must be called before any other methods.
 */
void MPU6050::init(gpio_num_t sdaPin, gpio_num_t clkPin)
{
	i2c = new I2C();
	i2c->init(I2C_ADDRESS, sdaPin, clkPin);
	// Dummy call
	//i2c->setAddress(I2C_ADDRESS);
	try
	{
		

		setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
		i2c->beginTransaction();
		i2c->write(MPU6050_PWR_MGMT_1);
		i2c->write(0);
		i2c->endTransaction();

		i2c->beginTransaction();
		i2c->write(0x1B);
		i2c->write(0x08);
		i2c->endTransaction();

		i2c->beginTransaction();
		i2c->write(0x1C);
		i2c->write(0x10);
		i2c->endTransaction();
		
		i2c->beginTransaction();
		i2c->write(0x1A);
		i2c->write(0x03);
		i2c->endTransaction();


		inited = true;
	}
	catch (GeneralErrorCodes_te &error)
	{
		inited = false;
		throw error;
	}
}

void MPU6050::CalibrateGyroscope()
{
		double gxs=0,gys=0,gzs=0;
		for(int i=0 ; i<2000 ; i++)
		{
			readData();
			gxs += getGyroX();
			gys += getGyroY();
			gzs += getGyroZ();
			printf("Calibrating..\n");
			vTaskDelay(3/portTICK_PERIOD_MS);
			
		}
		errorX = gxs/2000;
		errorY = gys/2000;
		errorZ = gzs/2000;

}