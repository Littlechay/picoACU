#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

#define I2C_PORT i2c0

// I2C address
// of the BNO
static const uint8_t addr = 0x28;

// magnetic declination Nelson NZ 07 2024 (decimal degrees, East positive)
const float MAG_DEC_DEG = 22.93;
const float MAG_DEC_RAD = MAG_DEC_DEG/(180/M_PI);

// Led driver stuff
#define NUM_MODULES 1
const uint8_t CMD_NOOP = 0;
const uint8_t CMD_DIGIT0 = 1; // Goes up to 8, for each line
const uint8_t CMD_DECODEMODE = 9;
const uint8_t CMD_BRIGHTNESS = 10;
const uint8_t CMD_SCANLIMIT = 11;
const uint8_t CMD_SHUTDOWN = 12;
const uint8_t CMD_DISPLAYTEST = 15;

#ifdef PICO_DEFAULT_SPI_CSN_PIN
static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
}
#endif

#if defined(spi_default) && defined(PICO_DEFAULT_SPI_CSN_PIN)
static void write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;
    cs_select();
    spi_write_blocking(spi_default, buf, 2);
    cs_deselect();
    sleep_ms(1);
}

static void write_register_all(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;
    cs_select();
    for (int i = 0; i< NUM_MODULES;i++) {
        spi_write_blocking(spi_default, buf, 2);
    }
    cs_deselect();
}
#endif

void display_num(unsigned long num)
{
    int digit = 0;
    while (num && digit < 8) {
       write_register_all(CMD_DIGIT0 + digit, num % 10);
        num /= 10;
        digit++;
    }
}

void clear()
{
    for (int i=0;i<8;i++) {
        write_register_all(CMD_DIGIT0 + i, 0);
    }
}

// Initialise Accelerometer Function
void accel_init(void){
    // Check to see if connection is correct
    sleep_ms(1000); // Add a short delay to help BNO005 boot up
    uint8_t reg = 0x00;
    uint8_t chipID[1];

    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, chipID, 1, false);

    if(chipID[0] != 0xA0){
        while(1){
            printf("Chip ID Not Correct - Check Connection!\n");
            sleep_ms(5000);
        }
    }
	// Select page 0
	uint8_t data[2];
	data[0] = 0x07;
	data[1] = 0x00;
	i2c_write_blocking(I2C_PORT, addr, data, 2, true);
	sleep_ms(50);

    // Use internal oscillator
    data[0] = 0x3F;
    data[1] = 0x40;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);
    sleep_ms(50);

    // Reset all interrupt status bits
    data[0] = 0x3F;
    data[1] = 0x01;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);
    sleep_ms(50);

    // Configure Power Mode
    data[0] = 0x3E;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);
    sleep_ms(50);

    // Default Axis Configuration
    data[0] = 0x41;
    data[1] = 0x24;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);
    sleep_ms(50);

    // Default Axis Signs
    data[0] = 0x42;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);
    sleep_ms(50);


    // Set operation to Fusion NDOF
    data[0] = 0x3D;
    data[1] = 0x0C;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);
    sleep_ms(50);
}

int main(void){
    stdio_init_all(); // Initialise STD I/O for printing over serial

    // Configure the I2C Communication
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);

    // Call BNO055 initialisation function
    accel_init();

    uint8_t quat[8]; // Store data from the 8 quaternion registers
    int16_t quatX, quatY, quatZ, quatW; // quaternion data
    double quatXn, quatYn, quatZn, quatWn; // scaled quaternion data
    uint8_t val = 0x20; // Start register address
    double roll, pitch, yaw, yawTrue;
	unsigned long cated;

    quatX = quatY = quatZ = quatW = 0;

	const double scale = (1.0 / (1<<14));
	memset(quat, 0, 8);

    /****************************************
    * 7 seg display
    *****************************************/
    #if !defined(spi_default) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
    #warning spi/max7219_8x7seg_spi example requires a board with SPI pins
    puts("Default SPI pins were not defined");
    #else

    printf("Hello, max7219! Drawing things on a 8 x 7 segment display since 2022...\n");

    // This example will use SPI0 at 10MHz.
    spi_init(spi_default, 10 * 1000 * 1000);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);

    // Make the SPI pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);

    // Make the CS pin available to picotool
    bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));

    // Send init sequence to device

    write_register_all(CMD_SHUTDOWN, 0);
    write_register_all(CMD_DISPLAYTEST, 0);
    write_register_all(CMD_SCANLIMIT, 7);
    write_register_all(CMD_DECODEMODE, 255);
    write_register_all(CMD_SHUTDOWN, 1);
    write_register_all(CMD_BRIGHTNESS, 5);

    clear();

    #endif

    /*// Infinite Loop*/
    while(1){
        i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
        i2c_read_blocking(I2C_PORT, addr, quat, 8, false);

        quatW = (((uint16_t)quat[1] << 8) | (uint16_t)quat[0]);
        quatX = (((uint16_t)quat[3] << 8) | (uint16_t)quat[2]);
        quatY = (((uint16_t)quat[5] << 8) | (uint16_t)quat[4]);
        quatZ = (((uint16_t)quat[7] << 8) | (uint16_t)quat[6]);

        // printf("W: %8.8d  X: %8.8d  Y: %8.8d   Z: %8.8d\n", quatW, quatX, quatY, quatZ);

        quatWn = scale*quatW;
        quatXn = scale*quatX;
        quatYn = scale*quatY;
        quatZn = scale*quatZ;

        // printf("Wn: %lf   Xn: %lf  Yn: %lf   Zn: %lf\n", quatWn, quatXn, quatYn, quatZn);

        // convert quaternions to roll, pitch and yaw
        roll=-atan2(2*((quatWn*quatXn)+(quatYn*quatZn)),(1-2*((quatXn*quatXn)+(quatYn*quatYn))));
        pitch=asin(2*((quatWn*quatYn)-(quatZn*quatXn)));
        yaw=-atan2(2*((quatWn*quatZn)+(quatXn*quatYn)),1-2*((quatYn*quatYn)+(quatZn*quatZn)));

		// convert to degrees and correct azimuth for magnetic declination
        roll = roll*(180/M_PI);
        pitch = pitch*(180/M_PI);
        yaw = 180+yaw*(180/M_PI);
		
		// convert magnetic azimuth to true
		yawTrue = yaw+MAG_DEC_DEG;
		if( yawTrue > 360 ){
			yawTrue = yawTrue-360;
		} 
		else {
			yawTrue = yawTrue;
			}
		
		cated = fabs((int)roll)*1000000 + (int)yawTrue;
        // Print to serial monitor
		printf("%.8lu\n", cated);
        // printf("%.2lf,%.2lf,%.2lf,%lu\n", roll, pitch, yaw, cated);
        clear();
        display_num(cated);
        sleep_ms(100);


    }
}
