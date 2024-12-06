/**************************************************************************
 *  @file     pn532_rpi.c
 *  @author   Blazej Floch
 *  @license  BSD
 *  
 *  This implements the peripheral interfaces.
 *  
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documnetation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to  whom the Software is
 * furished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 **************************************************************************/

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "pn532_pico.h"

#define _RESET_PIN                      (15)
#define _REQ_PIN                        (14)

#define _I2C_READY                      (0x01)
#define _I2C_ADDRESS                    (0x48 >> 1)
#define _I2C_CHANNEL                    (1)
#define _I2C_CLOCK                      (400)

#define HIGH                            1
#define LOW                             0

/**************************************************************************
 * Reset and Log implements
 **************************************************************************/
int PN532_Reset(void) {
    gpio_put(_RESET_PIN, HIGH);
    sleep_ms(100);
    gpio_put(_RESET_PIN, LOW);
    sleep_ms(500);
    gpio_put(_RESET_PIN, HIGH);
    sleep_ms(100);
    return PN532_STATUS_OK;
}

void PN532_Log(const char* log) {
    printf(log);
    printf("\r\n");
}
/**************************************************************************
 * End: Reset and Log implements
 **************************************************************************/
/**************************************************************************
 * I2C
 **************************************************************************/
int PN532_I2C_Wakeup(void) {
    gpio_put(_REQ_PIN, HIGH);
    sleep_ms(100);
    gpio_put(_REQ_PIN, LOW);
    sleep_ms(100);
    gpio_put(_REQ_PIN, HIGH);
    sleep_ms(500);
    return PN532_STATUS_OK;
}

void i2c_read(uint8_t* data, uint16_t count) {
    i2c_read_blocking(i2c_default, _I2C_ADDRESS, data, count, false);
}

void i2c_write(uint8_t* data, uint16_t count) {
    i2c_write_blocking(i2c_default, _I2C_ADDRESS, data, count, false);
}

int PN532_I2C_ReadData(uint8_t* data, uint16_t count) {
    uint8_t status[] = {0x00};
    uint8_t frame[count + 1];
    i2c_read(status, sizeof(status));
    if (status[0] != PN532_I2C_READY) {
        return PN532_STATUS_ERROR;
    }
    i2c_read(frame, count + 1);
    for (uint8_t i = 0; i < count; i++) {
        data[i] = frame[i + 1];
    }
    return PN532_STATUS_OK;
}

int PN532_I2C_WriteData(uint8_t* data, uint16_t count) {
    i2c_write(data, count);
    return PN532_STATUS_OK;
}


#define MILLIS() to_ms_since_boot(get_absolute_time())

bool PN532_I2C_WaitReady(uint32_t timeout) {
  // FIXME: Better use of a timer?

    uint8_t status[] = {0x00};
    uint32_t tickstart = MILLIS();
    while (MILLIS() - tickstart < timeout) {
        i2c_read(status, sizeof(status));
        if (status[0] == PN532_I2C_READY) {
            return true;
        } else {
            sleep_ms(5);
        }
    }
    return false;
}

void PN532_I2C_Init(PN532* pn532) {


    #if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else

    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
    bi_decl(bi_program_description("PN532 NFC driver via I2C for the Raspberry Pi Pico"));

    printf("Initializing PN532 via I2C ...\n");

    i2c_init(i2c_default, _I2C_CLOCK * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    gpio_set_dir(_REQ_PIN, GPIO_OUT);
    gpio_set_dir(_RESET_PIN, GPIO_OUT);

    gpio_pull_down(_RESET_PIN);
    gpio_pull_up(_REQ_PIN);

    // init the pn532 functions
    pn532->reset = PN532_Reset;
    pn532->read_data = PN532_I2C_ReadData;
    pn532->write_data = PN532_I2C_WriteData;
    pn532->wait_ready = PN532_I2C_WaitReady;
    pn532->wakeup = PN532_I2C_Wakeup;
    pn532->log = PN532_Log;

    printf("Resetting PN532...\r\n");

    // hardware reset
    pn532->reset();

    printf("Wakeup PN532...\r\n");
    // hardware wakeup
    pn532->wakeup();
    printf("... Done\r\n");
#endif
}
/**************************************************************************
 * End: I2C
 **************************************************************************/
