/*
 *  interfaces.h
 *
 *  Created on: 19.05.2017
 *      Author: hans-juergen.arlt <hj@arlt2net.de>
 */

#ifndef INTERFACES_H
#define INTERFACES_H

#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>

#include <dirent.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip_icmp.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <resolv.h>
#include <netdb.h>

#define UNUSED(x) (void)(x)

enum tempSet {
	TEMP_ACTUAL = 0,
    TEMP_MIN,
	TEMP_MAX,
};

#define PACKETSIZE  64

struct packet
{
    struct icmphdr hdr;
    char msg[PACKETSIZE-sizeof(struct icmphdr)];
};


int gpioExport (int pin, const char *mode);
void gpioOut(int pin, int val);
uint8_t gpioIn (int pin);
uint8_t gpioRead(int pin);
int gpioWrite(int pin, int value, const char *entry);

int spiInit   (char *dev, uint8_t mode, uint32_t speed, uint8_t bits);
int spiDataRW (int fd, uint8_t *dataout, int lenout
                     , uint8_t *datain, int lenin);

int setBaudrate(int fd, int baudrate, int bytemode);
int initTTyDriver(int *fd, const char* drv, int baudrate);
int writeUart(int fd, uint8_t *buf, int len);
int readUart(int fd, uint8_t *buf, int len);
int syncUart(int fdtx, int fdrx);
int serialRead(int fd, char *buffer, size_t size);
int serialWrite(int fd, const char *buffer, size_t size);

int setPwmChn(int chip, int chn, int periode, int duty);

int i2cInit (const char *device, int devId);
int i2cWrRd(int fd, uint8_t addr, uint8_t*txbuf, int tlen, uint8_t*rxbuf, int rlen);

int getAdcChannel(int device, int chn);

int changeFanDuty(int duty);

int getTemperature(int id, enum tempSet set);

int get1WireTemp(int id, int nbr);

uint32_t getEthernetIp(char *interface, char *ipString);

int ping(const char *adress);

int getkey(void);

int setLed(char *led, int on);

void SystemCommand(const char *command);

#endif

