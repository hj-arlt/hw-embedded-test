/*
 * main.h
 *
 * Created on: 25.09.2019
 *     Author: hans-jürgen arlt <hj@arlt2net.de>
 */

#ifndef TEST_HW_MAIN_H_
#define TEST_HW_MAIN_H_

#include <stddef.h>
#include <stdio.h>              /* printf */
#include <sys/types.h>
#include <getopt.h>             /* getopt_long() */
#include <errno.h>              /* errno */
#include <time.h>               /* time_t, time, ctime */
#include <pthread.h>
#include <semaphore.h>

#include <mtd/mtd-user.h>       // mtd flasher
#include <linux/spi/spidev.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>


/*
 * Board defines
 */
#define BUFFER_MAX 16

struct etherControl {
	int fd0;
	int fd1;
	int fd2;
	uint32_t ip0;
	uint32_t ip1;
	uint32_t ip2;
	char ip0String[24];
	char ip1String[24];
	char ip2String[24];
};

struct boardValue {
	FILE *logFile;
	int log;

	pthread_t tid_main;
	pthread_mutex_t main_mutex;
	int mainisrun;

	/* ethernet */
	struct etherControl eth;

	/* uart */
	int fdUart0;
	int fdUart1;
	int fdUart2;
	int baudrate0;
	int baudrate1;
	int baudrate2;
	unsigned char txBuffer[BUFFER_MAX];
	unsigned char rxBuffer[BUFFER_MAX];
	unsigned char trcBuffer[BUFFER_MAX];

	/* eMMC/HDD */
	int fdMmc;
	struct mmcData mmcStruct;
	char           mmcSector;

	/* can */
	pthread_t tid_canrx;
	pthread_t tid_cantx;
	sem_t can_sem;
	int canfd;
	int canrxisrun;
	int cantxisrun;

	/*
	 * SPI
	 */

	/*
	 * I2C
	 */

	/* gnss */
	enum GNSS_DEVICE device;
	const char *serdevice;
	struct gnssstr *gnss;
	/* ntrip client */
	int rtcminit;
	struct Args ntrip;
	sem_t nmeasem;

	/* common */
	int errorCnt;
	/* temp */
	int cpuTemp;
	int cpuTempMin;
	int cpuTempMax;
	int oneWireTemp;

};

/*
 * GPIO defines
 */
#define GPIO_PORT0 	0
#define GPIO_PORT1 	32
#define GPIO_PORT2 	64
#define GPIO_PORT3 	96
#define GPIO_PORT4 	128
#define GPIO_PORT5 	160
#define GPIO_PORT6 	182
#define GPIO_PORT7 	214

#if BOARD_RPI3
const char gnssClassUsb[] = "/sys/class/tty/ttyUSB1/device";
const char gnssDeviceUsb[] = "/dev/ttyUSB1";
#else
const char gnssClassUsb[] = "/sys/class/tty/ttyUSB0/device";
const char gnssDeviceUsb[] = "/dev/ttyUSB0";
#endif
const char emmcFileName[] = "./emmc.info";
const char nmeaFileName[] = "./nmea.log";

#endif /* TEST_HW_MAIN_H_ */
