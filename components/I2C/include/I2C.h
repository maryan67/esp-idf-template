#ifndef MAIN_I2C_H_
#define MAIN_I2C_H_
#include <stdint.h>
#include <sys/types.h>
#include <driver/i2c.h>
#include <driver/gpio.h>
#include "GeneralErrorCodes.h"

/**
 * @brief Interface to %I2C functions.
 */
class I2C {
private:
	uint8_t          m_address;
	i2c_cmd_handle_t m_cmd;
	bool             m_directionKnown;
	gpio_num_t       m_sdaPin;
	gpio_num_t       m_sclPin;
	i2c_port_t       m_portNum;

public:
	/**
	 * @brief The default SDA pin.
	 */
	static const gpio_num_t DEFAULT_SDA_PIN = GPIO_NUM_25;

	/**
	 * @brief The default Clock pin.
	 */
	static const gpio_num_t DEFAULT_CLK_PIN = GPIO_NUM_26;

	/**
	 * @brief The default Clock speed.
	 */
	static const uint32_t DEFAULT_CLK_SPEED = 100000;

	I2C();
	void beginTransaction() noexcept(false);
	void endTransaction() noexcept(false);
	uint8_t getAddress() const;
	void init(uint8_t address, gpio_num_t sdaPin = DEFAULT_SDA_PIN,
	gpio_num_t sclPin = DEFAULT_CLK_PIN, 
	uint32_t clkSpeed = DEFAULT_CLK_SPEED, i2c_port_t portNum = I2C_NUM_0) noexcept(false);
	void read(uint8_t* bytes, size_t length, bool ack=true)noexcept(false);
	void read(uint8_t* byte, bool ack=true)noexcept(false);
	void scan();
	void setAddress(uint8_t address) noexcept(false);
	void setDebug(bool enabled);
	bool slavePresent(uint8_t address);
	void start()noexcept(false);
	void stop()noexcept(false);
	void write(uint8_t byte, bool ack=true)noexcept(false);
	void write(uint8_t* bytes, size_t length, bool ack=true)noexcept(false);
};

#endif /* MAIN_I2C_H_ */