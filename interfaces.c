/*
 *  interfaces.c
 *
 *  Created on: 19.05.2017
 *      Author: hans-juergen.arlt <hj@arlt2net.de>
 */

#include "interfaces.h"

/* ***************************************************************
 * GPIO
 * ***************************************************************/

int gpioExport (int pin, const char *mode)
{
  int fd, err = 0;
  char fName [128] ;

  	if (pin == 0)
    	return -1;

	/* can be allocated */
  	if ((fd = open ("/sys/class/gpio/export", O_WRONLY)) < 0) {
      	printf("ERR: gpio export\n");
    	return -1;
	} else {
    	sprintf(fName, "%d", pin);
		err = write(fd, fName, strlen(fName));
	}
	close (fd) ;

	printf("exported: pin %d mode %s\n", pin, mode);

  	sprintf (fName, "/sys/class/gpio/gpio%d/direction", pin) ;
  	if ((fd = open (fName, O_WRONLY)) < 0) {
      	printf("ERR: pin %d dir\n", pin);
    	return -1;
	}

  	if ((strcasecmp (mode, "in")   == 0) 
   	|| (strcasecmp (mode, "input")  == 0))
  		err = write (fd, "in", 2) ;
  	else if ((strcasecmp (mode, "out")  == 0) 
        || (strcasecmp (mode, "output") == 0))
  		err = write (fd, "out", 3) ;
  	else if ((strcasecmp (mode, "high") == 0) 
        || (strcasecmp (mode, "up")     == 0))
  		err = write (fd, "high", 4) ;
  	else if ((strcasecmp (mode, "low")  == 0) 
        || (strcasecmp (mode, "down")   == 0))
  		err = write (fd, "low", 3) ;

  	close (fd) ;

	UNUSED(err);

  	return 0;
}

int gpioWrite(int pin, int value, const char *entry)
{
    int fd, err = 0;
    char fName [128] ;

    sprintf (fName, "/sys/class/gpio/gpio%d/%s", pin, entry) ;
    if ((fd = open (fName, O_WRONLY)) < 0)
    {
      //printf("ERR: pin %s wr in use\n", fName);
      return -1;
    }
    if (value == 0)
    	err = write (fd, "0", 1) ;
    else
    	err = write (fd, "1", 1) ;

    close (fd);

	UNUSED(err);

    return 0;
}

void gpioOut(int pin, int val)
{
    int err = gpioExport (pin, "out");
    if (err < 0) {
    	printf("WARN: gpioExport pin %d\n", pin);
		return;
    }
    gpioWrite(pin, val, (char*)"value");
}

uint8_t gpioRead(int pin)
{
	int fd;
	uint8_t val = -1;
    char fName [128] ;

    sprintf (fName, "/sys/class/gpio/gpio%d/value", pin) ;
    if ((fd = open (fName, O_RDWR)) < 0)
    {
      printf("ERR: pin %d rd in use\n", pin);
      return -1;
    }
	if (read(fd, &val, 1) != 1)
    {
      	printf("ERR: pin %d rd\n", pin);
    }
    close (fd);

    val &= 1;  // 1=0x31, 0=0x30

    //printf("Pin %d rd 0x%X\n", pin, val);
    return val;
}

uint8_t gpioIn(int pin)
{
    if (gpioExport (pin, "in") < 0)
		return -1;

    return (gpioRead(pin));
}


/* ***************************************************************
 * UART
 * ***************************************************************/
int setBaudrate(int fd, int baudrate, int mode)
{
    struct termios termios_p;

#if 1
    tcgetattr(fd, &termios_p); /* read in the current settings */
	memset(&termios_p, 0, sizeof(struct termios));
	termios_p.c_cflag = baudrate | 0 /*SPASTOPBITS_1*/
			                     | 0 /*SPAPARITY_NONE*/
								 | CS8 /*SPADATABITS_8*/
	                             | CLOCAL | CREAD;
	//termios_p.c_cflag |= CRTSCTS;
	termios_p.c_cflag |= 0;      //SPAPROTOCOL_NONE;
	termios_p.c_cc[VMIN] = 1;    /* block until n bytes are received */
	tcflush(fd, TCIOFLUSH);
	tcsetattr(fd, TCSANOW, &termios_p);
	tcflush(fd, TCIOFLUSH);
	fcntl(fd, F_SETFL, O_NONBLOCK);

#else

    tcgetattr(fd, &termios_p); /* read in the current settings */
    termios_p.c_cflag = (CREAD | CS8 | CLOCAL); // enable reading + 8N1, 1 stopp bit , no parity
    termios_p.c_iflag = IGNPAR;  // disable flow control, ignore parity and framing error
    termios_p.c_oflag = 0;       /* output mode */
    termios_p.c_lflag = 0;       /* local terminal */

    /* binary mode */
    if (bytemode)
    {
		termios_p.c_cc[VMIN]  = 0;  /* block until n bytes are received */
		termios_p.c_cc[VTIME] = 1; /* block until a timer expires (n * 100 mSec.) */
    }
    else /* string mode */
    {
		termios_p.c_cc[VMIN]  = 1; /* block until n bytes are received */
		termios_p.c_cc[VTIME] = 0; /* block until a char arrived */
	    termios_p.c_cc[VEOL]  = 0x0D; /* end off line character 0D/0A */
	    termios_p.c_cc[VEOL2] = 0x0A; /* end off line character 0D/0A */
    }

    cfsetispeed(&termios_p, baudrate); //set input baud
    cfsetospeed(&termios_p, baudrate); //set output baud
    tcsetattr(fd, TCSANOW, &termios_p);
    tcflush(fd, TCIFLUSH);
#endif

   return 0;
}

int initTTyDriver(int *fd, const char* drv, int baudrate)
{
   if (!drv) {
      printf("TTY: Empty driver name!\n");
      return -1;
   }

   *fd = open(drv, O_RDWR | O_NOCTTY | O_NONBLOCK); // Open in non blocking read/write mode

   if (*fd<0) {
      printf("TTY: Failed opening device %s!\n", drv);
      return -1;
   }
   setBaudrate(*fd, baudrate, 0); // byte timeout mode
   return 0;
}

int writeUart(int fd, uint8_t *buf, int len)
{
    int wrByte = write(fd, buf, len);
    if (wrByte != len) {
    	printf("ERR: Uart wr %d (%d)\n", wrByte, len);
    	return -1;
    }
    return 0;
}

int readUart(int fd, uint8_t *buf, int len)
{
	int rd, cnt, to = len;
	cnt = len/2; // timeout len * 100ms
	while (to) {
		rd = read(fd, buf, to);
		if (rd <= 0) {
			cnt--;
			if (cnt <= 0)
				return -1;
		}
		to -= rd;
	}
    return 0;
}

/*
 * not successful
 */
int syncUart(int fdtx, int fdrx)
{
	int rd, cnt;
	uint8_t val;
	val = 0xA5; // not used char
	writeUart(fdtx, &val, 1);
	cnt = 3; // 300ms timeout
	rd = 0;
	while ((rd <= 0) && (cnt--)) {
		rd = read(fdrx, &val, 1);
	}
	if ((rd == 1) && (val == 0xA5)) {
		printf("SYNC: %02X\n", val);
		/* clear receiver */
		cnt = 3; // 300ms timeout
		rd = 1;
		while ((rd > 0) && (cnt--)) {
			rd = read(fdrx, &val, 1);
		}
		return 0;
	}
	return -1;
}

int serialRead(int fd, char *buffer, size_t size)
{
  int j = read(fd, buffer, size);
  if(j < 0)
  {
    if(errno == EAGAIN)
      return 0;
    else
      return j;
  }
  return j;
}

int serialWrite(int fd, const char *buffer, size_t size)
{
  int j = write(fd, buffer, size);
  if(j < 0)
  {
    if(errno == EAGAIN)
      return 0;
    else
      return j;
  }
  return j;
}

/* ***************************************************************
 * SPI
 * ***************************************************************/

uint8_t spimode;
uint32_t spispeed;
uint8_t  spibits;

int spiInit(char *dev, uint8_t mode, uint32_t speed, uint8_t bits)
{
        int fd;
        if ((fd = open (dev, O_RDWR)) < 0) {
                printf("ERR: dev %s open", dev);
                return -1;
        }
        mode    &= 3 ;    // Mode is 0, 1, 2 or 3
        spimode = mode;
        spispeed = speed;
        spibits = bits;
        if (ioctl (fd, SPI_IOC_WR_MODE, &mode)  < 0) {
                return -2;
        }

        if (ioctl (fd, SPI_IOC_RD_MODE, &mode)  < 0) {
                return -2;
        }

        if (ioctl (fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
                return -3;
        }

        if (ioctl (fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
                return -3;
        }

        if (ioctl (fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
                return -4;
        }

        if (ioctl (fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
                return -4;
        }
        return fd;
}

int spiDataRW (int fd, uint8_t *dataout, int lenout, uint8_t *datain, int lenin)
{
        int ret;
        struct spi_ioc_transfer spi ;

        memset (&spi, 0, sizeof (spi)) ;

        spi.tx_buf        = (unsigned long)dataout ;
        spi.rx_buf        = (unsigned long)datain ;
        spi.len           = lenout + lenin ;
        spi.delay_usecs   = 0;
        spi.speed_hz      = spispeed ;
        spi.bits_per_word = spibits ;
        spi.cs_change     = 0; // CS high ok.
        ret = ioctl (fd, SPI_IOC_MESSAGE(1), &spi) ;
        if (ret < 0) {
                return -1;
        }
        return 0;
}

/* ***************************************************************
 * linux system command
 * ***************************************************************/

void SystemCommand(const char *command)
{
    int err = system(command);
	UNUSED(err);

}

/* ***************************************************************
 * LED
 * ***************************************************************/

int setLed(char *led, int on)
{
    int fd, err = 0;
	char path[128];
	strcpy(path, "/sys/class/leds/");
	strcat(path, led);
	strcat(path, "/brightness");

	if ((fd = open (path, O_WRONLY)) < 0) {
		printf("ERR: %s open\n", path);
		return -1;
	}
    if (on == 0)
    	err = write (fd, "0", 1) ;
    else err = write (fd, "1", 1) ;
	close(fd);

	UNUSED(err);

	return 0;
}

/* ***************************************************************
 * I2C
 *
 *  strcpy(txt, "/dev/i2c-5");
 *  addr = 0x60;
 *  fd = i2cInit (txt, addr);
 *
 * ***************************************************************/

int i2cInit (const char *device, int devId)
{
	int fd;
	if ((fd = open (device, O_RDWR)) < 0)    return -1;
	if (ioctl (fd, I2C_SLAVE, devId) < 0) {
		close(fd);
		//printf("ERR: I2C %s at 0x%02X\n", device, devId);
		return -1;
	}
	return fd ;
}

static struct i2c_msg msg[2];

int i2cWrRd(int fd, uint8_t addr, uint8_t*txbuf, int tlen, uint8_t*rxbuf, int rlen)
{
    int err;
    struct i2c_rdwr_ioctl_data i2cdata;

    i2cdata.nmsgs = 1;
    if (tlen) {
        msg[0].addr = addr;
        msg[0].flags = 0;
        msg[0].len = tlen;
        msg[0].buf = txbuf;
        if (rlen) {
            msg[1].addr = addr;
            msg[1].flags = I2C_M_RD;
            msg[1].len = rlen;
            msg[1].buf = rxbuf;
            i2cdata.nmsgs = 2;
        }
    } else {
        msg[0].addr = addr;
        msg[0].flags = I2C_M_RD;
        msg[0].len = rlen;
        msg[0].buf = rxbuf;
    }
    i2cdata.msgs = msg;

    err = ioctl(fd, I2C_RDWR, &i2cdata);
    if (err < 0) {
        printf("ERR: i2c on 0x%x\n", msg[0].addr);
    }
    return err;
}

/* ***************************************************************
 * PWM
 * /sys/class/pwm/pwmchip0..7/export -> pwm0..7
 *
 * ***************************************************************/

int setPwmChn(int chip, int chn, int periode, int duty)
{
    int fd, err = 0;
	char path[128];
  	sprintf (path, "/sys/class/pwm/pwmchip%d/export", chip) ;
	if ((fd = open (path, O_WRONLY)) < 0) {
		printf("ERR: %s open\n", path);
		return -1;
	}
	sprintf(path, "%d", chn);
	err = write(fd, path, strlen(path));
	close (fd);

  	sprintf (path, "/sys/class/pwm/pwmchip%d/pwm%d/period", chip, chn) ;
  	if ((fd = open (path, O_WRONLY)) < 0) {
      	printf("ERR: pwm %s\n", path);
    	return -1;
	}
	sprintf(path, "%d", periode);
	err = write(fd, path, strlen(path));
	close (fd);

  	sprintf (path, "/sys/class/pwm/pwmchip%d/pwm%d/duty_cycle", chip, chn) ;
  	if ((fd = open (path, O_WRONLY)) < 0) {
      	printf("ERR: pwm %s\n", path);
    	return -1;
	}
	sprintf(path, "%d", duty);
	err = write(fd, path, strlen(path));
	close (fd);

  	sprintf (path, "/sys/class/pwm/pwmchip%d/pwm%d/enable", chip, chn) ;
  	if ((fd = open (path, O_WRONLY)) < 0) {
      	printf("ERR: pwm %s\n", path);
    	return -1;
	}
  	err = write(fd, "1", 1);
	close (fd);

	UNUSED(err);
	return 0;
}

/* ***************************************************************
 * ADC input
 * /sys/bus/iio/devices/iio:device0/in_voltage0..7_raw
 * out: Volt in mV
 *
 * ***************************************************************/

int getAdcChannel(int device, int chn)
{
    int fd, len;
    char *ptr;
	char path[128];
  	sprintf (path, "/sys/bus/iio/devices/iio:device%d/in_voltage%d_raw", device, chn) ;
  	if ((fd = open (path, O_RDONLY)) < 0) {
      	printf("ERR: adc %s\n", path);
    	return -1;
	}
  	len = read(fd, &path, 8);
	close (fd);

	if (len < 1)
  		return -1;

	fd = strtol(path, &ptr, 10);
  	//printf("adc: %d rd strtol %d\n", chn, fd);
	return fd;
}

/* ***************************************************************
 * HWmonitor
 * Fan:         /sys/class/hwmon/hwmon0/fan1_input  0..5000 rpm
 *              /sys/class/hwmon/hwmon0/pwm1        0..255  duty cycle
 * Temperature: /sys/class/hwmon/hwmon1/temp1_input 22000 m°C
 *
 * ***************************************************************/

int changeFanDuty(int duty)
{
    int fd, len, err = 0;
    char *ptr;
	char path[128];
	/* set pwm duty */
  	sprintf (path, "/sys/class/hwmon/hwmon0/pwm1") ;
  	if ((fd = open (path, O_WRONLY)) < 0) {
  	  	sprintf (path, "/sys/class/hwmon/hwmon1/pwm1") ;
  	  	if ((fd = open (path, O_WRONLY)) < 0) {
			printf("ERR: fan not found %s\n", path);
			return -1;
  	  	}
	}
	sprintf(path, "%d", duty);
	err = write(fd, path, strlen(path));
	close (fd);

	usleep(200000);
	/* get rpm */
  	sprintf (path, "/sys/class/hwmon/hwmon0/fan1_input") ;
  	if ((fd = open (path, O_RDONLY)) < 0) {
  	  	sprintf (path, "/sys/class/hwmon/hwmon1/fan1_input") ;
  	  	if ((fd = open (path, O_RDONLY)) < 0) {
			printf("ERR: fan not found %s\n", path);
			return -1;
  	  	}
	}
  	len = read(fd, &path, 8);
	close (fd);

	if (len < 1)
  		return -1;

	fd = strtol(path, &ptr, 10);
  	//printf("fan: %d rpm\n", fd);

	UNUSED(err);

	return fd;
}

/* id not used */
int getTemperature(int id, enum tempSet set)
{
    int fd, len;
    char *ptr;
	char path[128];

	/* get actual temp */
	switch (set) {
	case TEMP_ACTUAL:

	  	sprintf (path, "/sys/class/hwmon/hwmon0/temp1_input") ;
	  	if ((fd = open (path, O_RDONLY)) < 0) {
	  	  	sprintf (path, "/sys/class/hwmon/hwmon1/temp1_input") ;
	  	  	if ((fd = open (path, O_RDONLY)) < 0) {
				printf("ERR: temp not found %s\n", path);
				return -1;
	  	  	}
		}
	  	len = read(fd, &path, 8);
		close (fd);

		if (len < 1)
	  		return -1;

		fd = strtol(path, &ptr, 10);
	  	//printf("temp actual: %d m°C\n", fd);
		return fd;

	case TEMP_MAX:

	  	sprintf (path, "/sys/class/hwmon/hwmon0/temp1_crit") ;
	  	if ((fd = open (path, O_RDONLY)) < 0) {
	  	  	sprintf (path, "/sys/class/hwmon/hwmon1/temp1_crit") ;
	  	  	if ((fd = open (path, O_RDONLY)) < 0) {
				printf("ERR: temp not found %s\n", path);
				return -1;
	  	  	}
		}
	  	len = read(fd, &path, 8);
		close (fd);

		if (len < 1)
	  		return -1;

		fd = strtol(path, &ptr, 10);
	  	//printf("temp max: %d m°C\n", fd);
		return fd;

	default:
		break;
	}
	return -1;
}

/* ***************************************************************
 * 1Wire: HWmonitor
 * ID: 10- DS1820 + DS18S20,  28- DS18B20
 * Nbr: sensor number with ID
 * Temperature: /sys/bus/w1/devices/28-041684b656ff/hwmon/hwmon2/temp1_input 22000 m°C
 *
 * ***************************************************************/

int get1WireTemp(int id, int nbr)
{
    int fd, cnt = 0;
    char *ptr;
	char path[310];
	DIR *pDir;
	struct dirent *dent;

  	sprintf (path, "/sys/bus/w1/devices/");
	pDir = opendir(path);   //this part
	if (pDir != NULL) {
	  	sprintf (path, "%d-", id); // find 28-xxx
		while((dent=readdir(pDir)) != NULL) {
            //printf("> %s\n", dent->d_name);
			if (strncmp(dent->d_name, path, 3) == 0) {
				/* count of nbr */
				if (cnt == nbr) {
    			  	sprintf (path, "/sys/bus/w1/devices/%s/hwmon/hwmon2/temp1_input", dent->d_name);
    				if ((fd = open(path, O_RDONLY )) < 0) {
    					printf("ERR: 1wire open %s\n", path);
    					break;
    				}
					if (read(fd, path, 8) > 0) {
						//printf("temp: %s\n", path);
						close (fd);

						fd = strtol(path, &ptr, 10);
						//printf("1Wire: temp %d m°C\n", fd);
						return fd;
					}
					close(fd);
    			}
	            cnt++;
            }
		}
		closedir(pDir);
	}
	return -1;
}

/* ***************************************************************
 * Ethernet IP
 * ***************************************************************/

uint32_t getEthernetIp(char *interface, char *ipString)
{
	int fd;
	uint32_t addr;
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(struct ifreq));

    fd = socket(AF_INET, SOCK_DGRAM, 0);

    /* I want to get an IPv4 IP address */
    ifr.ifr_addr.sa_family = AF_INET;

    /* I want IP address attached to "eth0" */
    strncpy(ifr.ifr_name, interface, IFNAMSIZ-1);

    ioctl(fd, SIOCGIFADDR, &ifr);

    if (ipString)
    	strcpy(ipString, inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));

    addr = ((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr;

    /* display result */
    //printf("%s,\t%08X\n", inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr), addr);

    close (fd);
    /* address in network byte order, low digit first */
    return addr;
}

/*--------------------------------------------------------------------*/
/*--- checksum - standard 1s complement checksum                   ---*/
/*--------------------------------------------------------------------*/
unsigned short checksum(void *b, int len)
{
    unsigned short *buf = (unsigned short *)b;
    unsigned int sum=0;
    unsigned short result;

    for ( sum = 0; len > 1; len -= 2 )
        sum += *buf++;
    if ( len == 1 )
        sum += *(unsigned char*)buf;
    sum = (sum >> 16) + (sum & 0xFFFF);
    sum += (sum >> 16);
    result = ~sum;
    return result;
}


/*--------------------------------------------------------------------*/
/*--- ping - Create message and send it.                           ---*/
/*    return 0 is ping Ok, return 1 is ping not OK.                ---*/
/*--------------------------------------------------------------------*/
int ping(const char *adress)
{
#if 1
	int err;
	char msg[64];

	sprintf(msg, "%s -c2", adress);
	err = system(msg);
	return err;

#else
    const int val=255;
    int i, sd, cnt, pid;
    struct packet pckt;
    struct sockaddr_in r_addr;
    int loop;
    struct hostent *hname;
    struct sockaddr_in addr_ping, *addr;
	struct protoent *proto;

    pid = getpid();
    if (pid < 0)
    	return -1;
    proto = getprotobyname("ICMP");
    if (proto == NULL)
    	return -1;
    hname = gethostbyname(adress);
    bzero(&addr_ping, sizeof(addr_ping));
    addr_ping.sin_family = hname->h_addrtype;
    addr_ping.sin_port = 0;
    addr_ping.sin_addr.s_addr = *(long*)hname->h_addr;

    addr = &addr_ping;

    sd = socket(PF_INET, SOCK_RAW, proto->p_proto);
    if ( sd < 0 )
    {
    	printf("ERR: ping socket");
        return 1;
    }
    if ( setsockopt(sd, SOL_IP, IP_TTL, &val, sizeof(val)) != 0)
    {
        perror("Set TTL option");
        return 1;
    }
    if ( fcntl(sd, F_SETFL, O_NONBLOCK) != 0 )
    {
    	printf("ERR: ping Request nonblocking I/O");
        return 1;
    }

    cnt=1;
    for (loop=0;loop < 10; loop++)
    {

        unsigned int len = sizeof(r_addr);

        if ( recvfrom(sd, &pckt, sizeof(pckt), 0, (struct sockaddr*)&r_addr, &len) > 0 )
        {
            return 0;
        }

        bzero(&pckt, sizeof(pckt));
        pckt.hdr.type = ICMP_ECHO;
        pckt.hdr.un.echo.id = pid;
        for ( i = 0; i < sizeof(pckt.msg)-1; i++ )
            pckt.msg[i] = i+'0';
        pckt.msg[i] = 0;
        pckt.hdr.un.echo.sequence = cnt++;
        pckt.hdr.checksum = checksum(&pckt, sizeof(pckt));
        if ( sendto(sd, &pckt, sizeof(pckt), 0, (struct sockaddr*)addr, sizeof(*addr)) <= 0 ) {
        	printf("ERR: ping sendto");
            return 1;
        }

        usleep(300000);
    }

    return 1;
#endif
}

/* ***************************************************************
 * Keyboard IO
 * ***************************************************************/

int getkey(void)
{
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}
