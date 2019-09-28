/*
 * gnss.h
 *
 *  Created on: 19.05.2017
 *      Author: hans-juergen.arlt <hj@arlt2net.de>
 */

#ifndef GNSS_H_
#define GNSS_H_

#include <termios.h>
#include <math.h>
#include <stdbool.h>

#ifdef DEBUG
#define GNSS_DEBUG
#endif

#ifdef GNSS_DEBUG
# define GNSS_PRINT(x) printf x
#else
# define GNSS_PRINT(x)
#endif


#define TXT_SUPPORT
#define GBS_SUPPORT
#define PUBX_SUPPORT
#define RMC_SUPPORT
#define GGA_SUPPORT
#define VTG_SUPPORT
#define GSV_SUPPORT
#define GLL_SUPPORT
#define GSA_SUPPORT
#define ZDA_SUPPORT
#define IISMD_SUPPORT

enum GNSS_DEVICE
{
   GNSS_UNKNOWN = 0,
   GNSS_SIRF,
   /* usb */
   GNSS_UBX_USB,
   GNSS_UBX_M8030,
   GNSS_UBX_NEO_M8P,
   GNSS_UBX_NEO_M8L,
   /* intern serial */
   GNSS_UBX_SERIAL,
   GNSS_UBX_NEO_M8N_INT,
   GNSS_UBX_NEO_M8P_INT,

   GNSS_MAX
};

//! A WGS84 coordinate
struct coord_geo {
   double lon; /*!< Longitude */
   double lat; /*!< Latitude */
   double alt;
};

struct gps_sat {
    int prn;
    int elevation;
    int azimuth;
    int snr;
};

struct gnssstr {

    struct coord_geo geo;
    double speed;
    double direction;
    double height;
    double pdop;
    double hdop;
    double vdop;
    double errLat;
    double errLon;
    double errAlt;
    int magnetic_direction;
    int mode;
    char fixtime[20];
    int fixyear;
    int fixmonth;
    int fixday;
    int status;
    int sats_used[5];
    int sats_visible[5];
    int sats_signal;
    int baseline;
};

struct gnsspriv {
    int fd;
    char hw[16];
    char modem[16];
    int timesync;
    int time;
    int current_count;
    struct gps_sat current[24];
    int next_count;
    struct gps_sat next[24];
    int valid;
    char *statefile;
    int pwr;
};

enum ntripsrv
{
   IGS_SYSTEM = 0,
   SAPOS_SYSTEM,
   MAX_SYSTEM
};

struct ntrip
{
   char stream[16];
   char location[20];
   double lat;         // y Pol
   double lon;         // x Äquator
};

struct gnssstr *initGnss(int fd);
int getGnssMsg(int fd, struct gnssstr *gnss);

int gnss_parse(char *buffer);

char *nextNtripStream(int id, double lat, double lon);

#endif /* CONNECTIVITY_BOX_CAN_GNSS_H_ */
