#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "easylogging++.h"
#include "math.h"
#include "protocol.h"
#include "i2c.h"
#include "power_controller.h"

PowerController::PowerController()
{
	int fd;
	if( (fd = i2c_open("/dev/i2c-0")) == -1 ) {

		perror("Open i2c bus error");
		return;
	}

	memset(&mcu, 0, sizeof(mcu));
	i2c_init_device(&mcu);

    mcu.bus = fd;
    mcu.addr = 0x2c;               // Nuvoton N76E003
    mcu.page_bytes = 32;
    mcu.iaddr_bytes = 1;
}

size_t PowerController::read(char inner_addr, void* buffer, size_t buffer_size)
{
	return	i2c_read(&mcu, inner_addr, buffer, buffer_size);
}

size_t PowerController::write(char inner_addr, void* buffer, size_t buffer_size)
{
	return	i2c_write(&mcu, inner_addr, buffer, buffer_size);
}

PowerController::~PowerController()
{
	i2c_close(mcu.bus);
}

bool PowerController::off_projector()
{
    unsigned char buffer[8] = {0};
    if (write(0xe0, buffer, PWR_CTRL_I2C_CMD_LEN) != PWR_CTRL_I2C_CMD_LEN) {
		return false;
	}
	return	true;
}

bool PowerController::off_board()
{
    unsigned char buffer[8] = {0};
    if (write(0xe1, buffer, PWR_CTRL_I2C_CMD_LEN) != PWR_CTRL_I2C_CMD_LEN) {
		return false;
	}
	return	true;
}