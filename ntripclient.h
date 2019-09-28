/*
 * ntripclient.h
 *
 *  Created on: 27.09.2019
 *      Author: user
 */

#ifndef TEST_HW_NTRIPCLIENT_H_
#define TEST_HW_NTRIPCLIENT_H_

#include <ctype.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>

  typedef int sockettype;
  #include <signal.h>
  #include <fcntl.h>
  #include <unistd.h>
  #include <arpa/inet.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netdb.h>

#define EMBEDDED_MODE

#include <termios.h>
#include "interfaces.h"

// #####################################################

#ifdef DEBUG
#define NTRIP_DEBUG
#endif

#ifdef NTRIP_DEBUG
# define NTRIP_PRINT(x) printf x
#else
# define NTRIP_PRINT(x)
#endif

#define SERVER_DEBUG
//#define RTCM_DEBUG
//#define NMEA_DEBUG

#define BITRATE_CYCLE  10

#ifdef EMBEDDED_MODE
enum MODE { HTTP = 1, RTSP = 2, NTRIP1 = 3, AUTO = 4, UDP = 5, END };

struct Args
{
  const char *server;
  const char *port;
  const char *user;
  const char *proxyhost;
  const char *proxyport;
  const char *password;
  const char *nmea;
  const char *data;
  int         bitrate;
  int         mode;

  int         udpport;
  int         initudp;
  const char *serdevice;
  const char *serlogfile;
};

struct serial
{
  struct termios Termios;
  int            Stream;
};

int get_rtcm_msg(int fd, struct Args *args);

#endif

#endif /* TEST_HW_NTRIPCLIENT_H_ */
