/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <chprintf.h>

#define I2C_DRIVER	I2CD1
#define HMC5883L_DEFAULT_ADDRESS 0x1E 	// Адрес 0х1E без восьмого бита чтения/записи адрес на запись 0x3C
										// на чтение 0x3D (0x1E << 1)
#define I2C_SEND i2cMasterTransmitTimeout
#define TIMEOUT MS2ST(1000)

#define PI 3.14159265

/* buffers depth */
#define ACCEL_RX_DEPTH 6
#define ACCEL_TX_DEPTH 6

static uint8_t rxbuf[ACCEL_RX_DEPTH];
static uint8_t txbuf[ACCEL_TX_DEPTH];
static i2cflags_t errors = 0;
msg_t status = RDY_OK;		// Флаг статуса передачи по I2C

double angle, x, y, z;
uint16_t deg;

// Структура адресов микросхемы HMC5883L
/* Сделать описание регистров побитно
 *
 */
typedef enum {
    HMC5883L_CONF_REGA = 0,
    HMC5883L_CONF_REGB,
    HMC5883L_MODE_REG,
    HMC5883L_DATA_OUT_X_MSB_REG,
    HMC5883L_DATA_OUT_X_LSB_REG,
    HMC5883L_DATA_OUT_Z_MSB_REG,
    HMC5883L_DATA_OUT_Z_LSB_REG,
    HMC5883L_DATA_OUT_Y_MSB_REG,
    HMC5883L_DATA_OUT_Y_LSB_REG,
    HMC5883L_STATUS_REG,
    HMC5883L_IDENT_REGA,
    HMC5883L_IDENT_REGB,
    HMC5883L_IDENT_REGC
} HMC5883L_Registers;

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 * Создаем поток, который будет мигать светодиодом
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {
  (void)arg;
  chRegSetThreadName("blinker");
  while (TRUE) {
    palSetPad(GPIOD, GPIOD_LED4);       /* Orange.  */
    chThdSleepMilliseconds(500);
    palClearPad(GPIOD, GPIOD_LED4);     /* Orange.  */
    chThdSleepMilliseconds(500);
  }
}

// Инициализируем I2C1
void I2C1_Init() {
	static const I2CConfig i2c1_config = {	//Заполняем структуру параметров I2C
			OPMODE_I2C,		//Параметр управляет режимом, биты 0-1 и 3 регистра I2C_CR1
			100000,			//Установка скорости передачи по I2C
			STD_DUTY_CYCLE	//Параметр управления рабочим циклом, регистр I2C_CCR (Clock Control Register)
	};
	//Явно определяем альтернативную функцию для выводов, которые используют I2C. Настройка PAL driver
	palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);	//SCL
	palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);	//SDA
	chThdSleepMilliseconds(100);	//Немного ждем

	i2cStart(&I2C_DRIVER, &i2c1_config);	//Запускаем I2C
}


// Инициализируем HMC5883L
void HMC5883_Init() {
	txbuf[0] = HMC5883L_CONF_REGB;
	txbuf[1] = 0x00;
	HMC5883L_I2C_BytesWrite(HMC5883L_DEFAULT_ADDRESS, &txbuf);	// Записываем биты конфигурации HMC5883L
	txbuf[0] = HMC5883L_MODE_REG;
	txbuf[1] = 0x00;
	HMC5883L_I2C_BytesWrite(HMC5883L_DEFAULT_ADDRESS, &txbuf);	// Записываем биты конфигурации HMC5883L
}

/* Запись байтов из буффера txbuf в HMC5883L
 * slaveAddr - Адрес HMC5883L HMC5883L_DEFAULT_ADDRESS
 * buffer    - буффер передачи
 */
void HMC5883L_I2C_BytesWrite(uint8_t slaveAddr, uint8_t* txBuffer) {
	i2cAcquireBus(&I2C_DRIVER);	// Получение эксклюзивного доступа к шине I2C
								// In order to use this function the option I2C_USE_MUTUAL_EXCLUSION must be enabled.

	/* Функция #define I2C_SEND i2cMasterTransmitTimeout возвращает в status следующие значения
	 * RDY_OK		если функция завершилась успешно
	 * RDY_RESET	если функция выдала одну или несколько ошибок, ошибки можно получить вызовом i2cGetErrors()
	 * RDY_TIMEOUT	истек таймаут до окончания операции передачи, скорее всего проблема в физическом подключении
	 */
	status = I2C_SEND 	( &I2C_DRIVER,	// Указатель на объект I2CDriver
					      slaveAddr,	// Slave адресс устройства (7 бит) без бита чтения/записи R/W
						  txBuffer,		// Указатель на буффер передачи
						  2,			// Количество байт для передачи
						  rxbuf,		// Указатель на приемный буффер
						  0,			// Количество байт для приема, 0 если делаем только передачу
						  TIMEOUT		// Количество тиков перед операцией таймаута, доступно TIME_INFINITE без таймаута
						);

	i2cReleaseBus(&I2C_DRIVER);	// Освобождает доступ к шине I2C
								// In order to use this function the option I2C_USE_MUTUAL_EXCLUSION must be enabled.

	if (status != RDY_OK) { // если функция завершилась с ошибками, получаем ошибки
	    errors = i2cGetErrors(&I2C_DRIVER);
	}
}

uint8_t HMC5883L_I2C_BytesRead(uint8_t slaveAddr, uint8_t* txBuffer) {
	uint8_t rxBuffer = 0;
	i2cAcquireBus(&I2C_DRIVER);	// Получение эксклюзивного доступа к шине I2C
								// In order to use this function the option I2C_USE_MUTUAL_EXCLUSION must be enabled.

	/* Функция #define I2C_SEND i2cMasterTransmitTimeout возвращает в status следующие значения
	 * RDY_OK		если функция завершилась успешно
	 * RDY_RESET	если функция выдала одну или несколько ошибок, ошибки можно получить вызовом i2cGetErrors()
	 * RDY_TIMEOUT	истек таймаут до окончания операции передачи, скорее всего проблема в физическом подключении
	 */
	status = I2C_SEND 	( &I2C_DRIVER,	// Указатель на объект I2CDriver
					      slaveAddr,	// Slave адресс устройства (7 бит) без бита чтения/записи R/W
						  txBuffer,		// Указатель на буффер передачи
						  1,			// Количество байт для передачи
						  rxBuffer,		// Указатель на приемный буффер
						  1,			// Количество байт для приема, 0 если делаем только передачу
						  TIMEOUT		// Количество тиков перед операцией таймаута, доступно TIME_INFINITE без таймаута
						);

	i2cReleaseBus(&I2C_DRIVER);	// Освобождает доступ к шине I2C
								// In order to use this function the option I2C_USE_MUTUAL_EXCLUSION must be enabled.

	if (status != RDY_OK) { // если функция завершилась с ошибками, получаем ошибки
	    errors = i2cGetErrors(&I2C_DRIVER);
	    chprintf((BaseSequentialStream *) &SD2, " %x", errors);
	}

	return rxBuffer;
}

int16_t complement2signed(uint8_t msb, uint8_t lsb){
  uint16_t word = 0;
  word = (msb << 8) + lsb;
  if (msb > 0x7F){
    return -1 * ((int16_t)((~word) + 1));
  }
  return (int16_t)word;
}

/* Проверка на ошибки
 *  Функция i2cMasterTransmitTimeout может возвращать следующие значения:
 *   RDY_OK = 0
 *   RDY_TIMEOUT = -1
 *   RDY_RESET = -2
 *  В случае получения RDY_RESET следует получить расширенный код ошибки:
 *   i2cflags_t errors = i2cGetErrors(&I2C_DRIVER);
 *  Расширенный код ошибки определён так:
 *   #define I2CD_NO_ERROR               0x00   // Нет ошибки
 *   #define I2CD_BUS_ERROR              0x01   // Ошибка шины
 *	 #define I2CD_ARBITRATION_LOST       0x02   // Потеря арбитража
 *   #define I2CD_ACK_FAILURE            0x04   // Сбой подтверждения
 *   #define I2CD_OVERRUN                0x08   // Пере- или недополнение
 *   #define I2CD_PEC_ERROR              0x10   // PEC ошибка при приёме
 *   #define I2CD_TIMEOUT                0x20   // Таймаут в железе
 *   #define I2CD_SMB_ALERT              0x40   // Тревога SMBus
 */

// Application entry point.
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  I2C1_Init();		//Вызываем инициализацию I2C1
  HMC5883_Init();	//Вызываем инициализацию HMC5883L магнитометра
  chThdSleepMilliseconds(10);

  // Запускаем Serial2 отладочную печать
  sdStart(&SD2, NULL);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  int16_t dataX = 0, dataY = 0, dataZ = 0;

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL); // Запускаем поток для мигания светодиодом

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop.
   */
  i2cAcquireBus(&I2C_DRIVER);
  while (TRUE) {

	  status = i2cMasterReceiveTimeout(&I2C_DRIVER, HMC5883L_DEFAULT_ADDRESS, rxbuf, 6, TIMEOUT);

	  dataX = complement2signed(rxbuf[0], rxbuf[1]);
	  dataY = complement2signed(rxbuf[2], rxbuf[3]);
	  dataZ = complement2signed(rxbuf[4], rxbuf[5]);

	  chprintf((BaseSequentialStream *) &SD2, "%d ", dataX);
	  chprintf((BaseSequentialStream *) &SD2, "%d ", dataY);
	  chprintf((BaseSequentialStream *) &SD2, "%d\r", dataZ);
	  /*x = dataX * 0.92;
	  y = dataY * 0.92;
	  z = dataZ * 0.92;*/

	  x = dataX; y = dataY; z = dataZ;

	  // считаем угол
	  angle = atan2(y, x);
	  if (angle < 0)
		  angle += 2*PI;

	  if (angle > 2*PI)
	      angle -= 2*PI;

	  deg = (uint16_t)(angle * (180 / PI));

	  //chprintf((BaseSequentialStream *) &SD2, " %d\r", deg);

	  txbuf[0] = HMC5883L_DATA_OUT_X_MSB_REG;
	  status = i2cMasterTransmitTimeout(&I2C_DRIVER, HMC5883L_DEFAULT_ADDRESS, txbuf, 1, rxbuf, 0, TIMEOUT);


	  chThdSleepMilliseconds(80);


  }
}
