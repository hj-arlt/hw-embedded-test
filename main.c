/*
 * main.c
 *
 * Created on: 25.09.2019
 *     Author: Hans-Jürgen Arlt <hj@arlt2net.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License v2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#include "interfaces.h"
#include "memory.h"
#include "mmc.h"
#include "can.h"
#include "gnss.h"
#include "ntripclient.h"
#include "video.h"
#include "lin.h"

#include "main.h"

#define CONSOLE_PRINT
//#define LOGGING_PRINT

// ------------------------------------------
#ifdef CONSOLE_PRINT
#define CPRINT(x) printf x
#else
#define CPRINT(x)
#endif

#ifdef LOGGING_PRINT
#define LOG_PRINT(...) do{                                    \
	    time_t rawtime;	struct tm *now; char m_str[128];	  \
	    time(&rawtime);	now = gmtime(&rawtime);               \
	    strftime(m_str, sizeof(m_str), "[%H:%M:%S] ", now);   \
	    fprintf( board.logFile, m_str );                           \
		fprintf( board.logFile, __VA_ARGS__ );                     \
} while( 0 )
#else
#define LOG_PRINT(...) do{ } while ( 0 )
#endif

/*
 * variable
 */
struct boardValue  board;
static char fileName[128];

/*
 * Log and Results
 */
void logPrint(const char *txt)
{
	time_t rawtime;
	struct tm *now;
	char m_str[128];

	time(&rawtime);
	now = gmtime(&rawtime);
	strftime(m_str, sizeof(m_str), "[%H:%M:%S] %s", now);
	strcat(m_str, txt);
	fwrite(txt, strlen(m_str), 1, board.logFile);

	printf("%s\n", txt);
}

// ######################################################

void *main_thread(void *arg)
{
	struct boardValue *board = (struct boardValue *)arg;

	//CPRINT(("Thread: main is started..\n"));

	board->mainisrun = 1;
	while (board->mainisrun) {
		/* get dgps info */
		if (board->rtcminit) {
			CPRINT(("GNSS: rtcm is started..\n"));
			get_rtcm_msg(board->fdUart0, &board->ntrip);
		}
		usleep(20000);
	}
	//CPRINT(("Thread: main .. end\n"));
	return NULL;
}

/*
 * can receive thread
 *
 * Handle ICAN communication to car
 */
void *doCanRx(void *arg)
{
	struct boardValue *board = (struct boardValue *)arg;
	int i, size;
    struct can_frame cf;

    if (board->canfd <= 0)
		return NULL;

    CPRINT(("Thread: canrx is started..\n"));

    i = fcntl(board->canfd ,F_GETFL, 0);
    fcntl(board->canfd, F_SETFL, i | O_NONBLOCK);

    sem_post(&board->can_sem);

    board->canrxisrun = 1;
    while (board->canrxisrun)
    {
    	sem_wait(&board->can_sem);         /* start of critical section */
        size = read(board->canfd, &cf, sizeof(struct can_frame));
        if (size < 0) {
           usleep(50000);  // 100ms=4,4%, 50ms=4,4%, 10ms=4,7%
        }
        else if (size == sizeof(struct can_frame))
        {

        }
        sem_post(&board->can_sem);
    } // while
	CPRINT(("Thread: canrx .. end\n"));
	return NULL;
}

/*
 * can send thread
 *
 * Handle ATB communication On /Off / Hid
 */
void *doCanTx(void *arg)
{
	struct boardValue *board = (struct boardValue *)arg;
	//struct can_frame cf;

	if (board->canfd <= 0)
		return NULL;

	CPRINT(("Thread: cantx is started..\n"));

	board->cantxisrun = 1;
	while (board->cantxisrun)
	{
	  /* send cycle min. 1ms */
	  usleep(1000);
	}
	CPRINT(("Thread: cantx .. end\n"));
	return NULL;
}

int gnssInit(void)
{
	int fd;

	strcpy(fileName, gnssClassUsb);
    if ((fd = open(fileName, O_RDONLY )) < 0)
	{
		perror("GPS: Failed device!");
		return -1;
	}
    close(fd);

    board.serdevice = gnssDeviceUsb;
    board.baudrate0 = B4800;

    CPRINT(("GNSS: on %s (Sirf)\n", board.serdevice));

	if (initTTyDriver(&board.fdUart0, board.serdevice, board.baudrate0) < 0)
	{
		perror("GPS: Failed connecting to device!");
		return -1;
	}
	board.device = GNSS_SIRF; //GNSS_UBX_SERIAL; GNSS_SIRF;

    /* gnss module */
	board.gnss = initGnss(board.fdUart0);
	if (board.gnss == NULL)
	{
		perror("GPS: Failed gnss init!");
		return -1;
	}

	if (sem_init(&board.nmeasem, 0, 0) != 0)
	{
		perror("GNSS Sema init failed\n");
		return -1;
	}
	sem_post(&board.nmeasem);

	return 0;
}

int ntripInit(struct Args *args, const char *dev, double lat, double lon)
{
	args->server = "www.igs-ip.net";
	args->port = "2101";
	args->user = "myname";
	args->password = "mypass";
	args->nmea = 0;                // string to server $GPGGA
	args->data = "LEIJ00DEU0";     // Leipzig
	args->data = nextNtripStream(IGS_SYSTEM, lat, lon);
	args->bitrate = 1;             // display output rate
	args->proxyhost = 0;
	args->proxyport = "2101";
	args->mode = AUTO;
	args->initudp = 0;
	args->udpport = 0;
	args->serlogfile = nmeaFileName;
    args->serdevice = dev;

	return 0;
}

int initLogFile(void)
{
	int cnt, n;
	char path[128];
    char *ptr;
	DIR *pDir;
	/*
	 * get last file name number
	 */
	cnt = 0;
  	sprintf (path, "/tmp/");
	pDir = opendir(path);   //this part
	if (pDir != NULL) {
		struct dirent *dent;
	  	sprintf (path, "trace_"); // find trace_ files
		while((dent=readdir(pDir)) != NULL) {
            //printf("> %s\n", dent->d_name);
			if (strncmp(dent->d_name, path, 6) == 0) {
				n = strtol(&dent->d_name[6], &ptr, 10);
				if (cnt < n) {
					cnt = n;
    			}
            }
		}
		closedir(pDir);
	}

	/*
	 * no file name input
	 * parameter -l filename
	 */
	if (! board.log) {
		time_t rawtime;
		time(&rawtime);
		sprintf(fileName, "/tmp/trace_%d", cnt+1);
	}
	board.logFile = fopen(fileName, "w+");
	if (board.logFile == NULL) {
		perror("TraceFile: not opened!");
		return -1;
	}
	CPRINT(("LOGfile: %s\n\n", fileName));
	return 0;
}

// ######################################################

static void main_usage(FILE *fp, int argc, char **argv)
{
	fprintf(fp,
			 "Usage: %s [options]\n"
			 "Version 1.0\n"
			 "-h | --help          Print this message\n"
			 "-b | --baud          Baudrate UART/USB\n"
			 "-l | --log           Logfile on\n"
			 "",
			 argv[0]);
}

static const char main_short_options[] = "b:hl:";

static const struct option
main_long_options[] = {
	{ "baud", required_argument, NULL, 'b' },
	{ "help", no_argument,       NULL, 'h' },
	{ "log",  required_argument, NULL, 'l' },
	{ 0, 0, 0, 0 }
};

// ######################################################

int main(int argc, char *argv[])
{
	int n, ok;

	CPRINT(("\n######### HW Test Ver.0.1 ##########\n\t%s %s\n\n", __DATE__, __TIME__));

	memset(&board, 0, sizeof(struct boardValue));

	for (;;) {
		int c;

		c = getopt_long(argc, argv,
						main_short_options, main_long_options, &n);

		if (-1 == c)
				break;

		switch (c) {
		case 'h':
				main_usage(stdout, argc, argv);
				exit(EXIT_SUCCESS);

		case 'b': /* dummy with parameters */
				errno = 0;
				board.baudrate0 = strtol(optarg, NULL, 0);
				if (errno)
					exit(EXIT_FAILURE);
				break;
		case 'l':
				strcpy(fileName, optarg);
				board.log = 1;
				break;
		case 'i':
				break;
		case  0: /* getopt_long() flag */
				break;
		default:
				main_usage(stderr, argc, argv);
				exit(EXIT_FAILURE);
		}
	}

	initLogFile();

	/*
	 * create a main thread
	 */
	board.mainisrun = 0;
	n = pthread_create(&board.tid_main, NULL, main_thread, &board);
	if (n != 0)
	  printf("can't create main thread :[%s]", strerror(n));

	/*
	 * eMMC block device
	 */
	board.mmcStruct.infofile = emmcFileName;
	if (initCardInfo(&board.mmcStruct) == 0) {
		board.fdMmc = open("/dev/mmcblk0", O_RDWR /*| O_DIRECT*/);
		if (board.fdMmc >= 0) {
			strcpy(board.mmcStruct.device, "/dev/mmcblk0");

			CPRINT(("eMMC: %s\n",board.mmcStruct.device));
		} else {
			board.fdMmc = open("/dev/sda", O_RDWR);
			if (board.fdMmc >= 0) {
				strcpy(board.mmcStruct.device, "/dev/sdc");

				CPRINT(("HDD: %s\n",board.mmcStruct.device));
			}
		}
		if (board.fdMmc >= 0) {
			checkEmmc(board.fdMmc, &board.mmcStruct);
		} else {
			CPRINT(("eMMC: no card device found!\n"));
		}
	}

	/*
	 * MTD
	 */
	ok = 0;
    for (n = 0; n < 8; n++)
    {
        mtd_info_t mtd_info;           // the MTD structure
		sprintf(fileName, "/dev/mtd%d", n);
	    int fd = open(fileName, O_RDWR); // open the mtd device for reading and
	                                        // writing. Note you want mtd0 not mtdblock0
	                                        // also you probably need to open permissions
	                                        // to the dev (sudo chmod 777 /dev/mtd0)
		if (fd >= 0)
		{
		    ioctl(fd, MEMGETINFO, &mtd_info);   // get the device info

		    // dump it for a sanity check, should match what's in /proc/mtd
		    CPRINT(("MTD:  %s %x\nMTD total size: 0x%x bytes\nMTD erase size: 0x%x bytes\n",
		    		fileName, mtd_info.type, mtd_info.size, mtd_info.erasesize));

		    close(fd);
		    ok = 1;
		}
    }
    if (! ok) {
	    CPRINT(("MTD:  no device found!\n"));
    }

	/*
	 * ethernet
	 */
	board.eth.ip0 = getEthernetIp((char*)"eth0", board.eth.ip0String);
	if (board.eth.ip0) {
		LOG_PRINT("LAN:  eth0: %s\n",board.eth.ip0String);
		CPRINT(("LAN:  eth0: %s\n",board.eth.ip0String));
	}
	board.eth.ip1 = getEthernetIp((char*)"eth1", board.eth.ip1String);
	if (board.eth.ip1) {
		LOG_PRINT("LAN:  eth1: %s\n",board.eth.ip1String);
		CPRINT(("LAN:  eth1: %s\n",board.eth.ip1String));
	}
	board.eth.ip2 = getEthernetIp((char*)"eth2", board.eth.ip2String);
	if (board.eth.ip2) {
		LOG_PRINT("LAN:  eth2: %s\n",board.eth.ip2String);
		CPRINT(("LAN:  eth2: %s\n",board.eth.ip2String));
	}

	/*
	 * can interface
	 */
	n = 0;
	sprintf(fileName, "can%d",n);
	if (initCan(fileName, &board.canfd) == 0)
	{
		/*
		 * create a can threads
		 */
		if (board.canfd >= 0) {
			n = pthread_create(&board.tid_canrx, NULL, doCanRx, &board);
			if (n != 0)
			  printf("can't create canrx thread :[%s]", strerror(n));
			n = pthread_create(&board.tid_cantx, NULL, doCanTx, &board);
			if (n != 0)
			  printf("can't create cantx thread :[%s]", strerror(n));
			n = sem_init(&board.can_sem, 0, 0);
			if (n != 0)
				printf("Sema can init failed\n");
		}
	} else {
		CPRINT(("CAN:  no interface found!\n"));
	}

	/*
	 * UART
	 */
	ok = 0;
    for (n = 0; n < 8; n++)
    {
#if BOARD_IMX
		sprintf(fileName, "/dev/ttymxc%d", n);
#endif
		sprintf(fileName, "/dev/ttyS%d", n);
		int fd = open(fileName, O_RDWR | O_NOCTTY | O_NDELAY); // Open in non blocking read/write mode
		if (fd >= 0)
		{
			CPRINT(("UART: check %s connected\n", fileName));
			close(fd);
			ok = 1;
		}
    }
    for (n = 0; n < 8; n++)
    {
		sprintf(fileName, "/dev/ttyUSB%d", n);
		int fd = open(fileName, O_RDWR | O_NOCTTY | O_NDELAY); // Open in non blocking read/write mode
		if (fd >= 0)
		{
			CPRINT(("UART: check %s connected\n", fileName));
			close(fd);
			ok = 1;
		}
    }
    if (! ok) {
	    CPRINT(("UART: no device found!\n"));
    }

	/*
	 * SPI
	 */
	ok = 0;
    for (n = 0; n < 8; n++)
    {
		for (int j = 0; j < 3; j++)
		{
			sprintf(fileName, "/dev/spidev%d.%d", n, j);
			int fd = open(fileName, O_RDWR);
			if (fd >= 0)
			{
				CPRINT(("SPI:  check %s connected\n", fileName));
				close(fd);
				ok = 1;
			}
		}
    }
    if (! ok) {
	    CPRINT(("SPI:  no device found!\n"));
    } else {
    	/* example */
        int frq = 20000000;
        strcpy(fileName, "/dev/spidev0.0");
        int fd = spiInit(fileName, 1, frq, 32);
        if (fd >= 0)
        {
        	uint32_t buf[3];
			buf[0] = 0x8000;
			buf[1] = 0;
			buf[2] = 0;
			if (spiDataRW (fd, (uint8_t*)buf, 1, (uint8_t*)&buf[1], 2) >= 0)
			{
				CPRINT(("SPI:  0:0 data 0x%08X, 0x%08X, 0x%08X\n", buf[0], buf[1], buf[2]));
			}
			close(fd);
        }
    }

	/*
	 * I2C
	 */
	ok = 0;
    for (n = 0; n < 10; n++)
    {
		sprintf(fileName, "/dev/i2c-%d", n);
		int fd = open(fileName, O_RDWR);
		if (fd >= 0)
		{
			CPRINT(("I2C:  check %s connected\n", fileName));
			close(fd);

			/* example eeprom */
			uint8_t addr = 0xA0;
			fd = i2cInit (fileName, addr);
			if (fd >= 0)
			{
				uint8_t reg = 0x20;
				uint8_t value;
				i2cWrRd(fd, addr, &reg, 1, &value, 1);
				CPRINT(("I2C:  eep at addr %02X - %02X\n", addr, value));
				close(fd);
			}
			ok = 1;
		}
    }
    if (! ok) {
	    CPRINT(("I2C:  no device found!\n"));
    }

	/*
	 * gnss
	 */
	if (gnssInit() == 0) {
		CPRINT(("GNSS: read..\n"));
		while (! board.gnss->geo.lat || ! board.gnss->geo.lon)
		{
		    getGnssMsg(board.fdUart0, board.gnss);
		}
		CPRINT(("\nGNSS: location lat %4.2f lon %4.2f\n", board.gnss->geo.lat,board.gnss->geo.lon));

		/* ntrip client */
		if (ntripInit(&board.ntrip, board.serdevice, board.gnss->geo.lat, board.gnss->geo.lon) == 0) {
			/* get dgps info */
			board.rtcminit = 1;
			sleep(10);
		}
	}

	/*
	 * Frame buffer
	 */



	/*
	 * end of gnss
	 */
	sem_close(&board.nmeasem);

	/* end of can */
	if (board.canfd >= 0) {
		board.canrxisrun = 0;
		pthread_join(board.tid_canrx, (void **) &n);
		board.cantxisrun = 0;
		pthread_join(board.tid_cantx, (void **) &n);
		sem_destroy(&board.can_sem);
	}

	/* end of main thread */
	board.rtcminit = 0;
	board.mainisrun = 0;
	pthread_join(board.tid_main, NULL);

	if (board.fdUart0)	close(board.fdUart0);
	if (board.fdUart1)	close(board.fdUart1);
	if (board.fdUart2)	close(board.fdUart2);
	if (board.logFile)	fclose(board.logFile);

	CPRINT(("\nEnd Of Test\n"));

	return 0;
}

