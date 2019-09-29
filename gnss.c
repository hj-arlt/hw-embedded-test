/*
 * gnss.c
 *
 *  Created on: 19.05.2017
 *      Author: hans-juergen.arlt <hj@arlt2net.de>
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <errno.h>
#include <time.h>

#include <sys/stat.h>
#include <dirent.h>

#include "gnss.h"

#define BUFFER_MAX 128

static unsigned char rxBuffer[BUFFER_MAX];
static struct gnsspriv priv;
static struct gnssstr gnss;

/* ***************************************************************
 * GPS
 * ***************************************************************/
/* 4924.2811 */
double g_ascii_strtod(const char *nptr)
{
    return (strtod(nptr, NULL));
}
/* N, */
int g_ascii_strcasecmp(char *s1, const char *s2)
{
    char *ptr;
    ptr = strchr(s1, *s2);
    if (ptr == NULL)
        return 1;
    return 0;
}

void g_time(char *nptr, struct gnssstr *gnss)
{
    strncpy(gnss->fixtime, nptr, 2);
    strcat(gnss->fixtime + 2, ":");
    strncpy(gnss->fixtime + 3, nptr + 2, 2);
    strcat(gnss->fixtime + 5, ":");
    strncpy(gnss->fixtime + 6, nptr + 4, 2);
    gnss->fixtime[8] = 0;
}

void g_print(struct gnssstr *gnss)
{
    int i;
    printf("UTC %s / %d.%d.%d FIX %dD\n", gnss->fixtime, gnss->fixday, gnss->fixmonth, gnss->fixyear, gnss->mode);
    printf("LAT %2.4f LON %2.4f HGT %4.2lf m\n", gnss->geo.lat, gnss->geo.lon, gnss->height);
    printf("DIR %3.2lf, Speed %3.1lf km/h\n", gnss->direction, gnss->speed);

    printf("HDP %2.2f PDP %2.2f VDP %2.2f\n", gnss->hdop, gnss->pdop, gnss->vdop);

    for (i = 0; i < 4; i++)
    {
        if (gnss->sats_visible[i] || gnss->sats_used[i])
        {
            if (i == 2)
            {
                printf("SATs %d / %d used/visible\n", gnss->sats_used[i], (gnss->sats_visible[0] + gnss->sats_visible[1]));
            }
            else if (gnss->sats_used[i])
            {
                printf("SATs of %i %d / %d used/visible\n", i, gnss->sats_used[i], gnss->sats_visible[i]);
            }
        }
    }
    printf("\n");
}

/* ############################################################ */

char bdctrl[] = "$PUBX,41,1,0007,0003,115200,0*18\r\n";
char tmpoll[] = "$GLGLQ,TXT*25\r\n$GPGPQ,TXT*25\r\n";
char timepoll[] = "$PUBX,04*37\r\n";

/* disable / enable protocol
 *
 * fix data, range error, date & time, position data lat,log,alt, sat activ & DOP,
 * course & speed, date & time, lon, lat, speed, sat in view */
#define MAX_NMEA_INIT 14

char protocolctrl[MAX_NMEA_INIT][28] =
    {
        /* off */
        "$PUBX,40,GLL,0,0,0,0,0,0*XX", // Latitude and longitude, with time of position fix and status: RMC
        "$PUBX,40,GNS,0,0,0,0,0,0*XX", // GNSS fix data: RMC
        "$PUBX,40,ZDA,0,0,0,0,0,0*XX", // Time and Date: RMC
        "$PUBX,40,GST,0,0,0,0,0,0*XX", // GNSS Pseudo Range Error Statistics: GBS
        "$PUBX,40,GRS,0,0,0,0,0,0*XX", // GNSS Range Residuals: like GGA, GSA
        "$PUBX,40,DTM,0,0,0,0,0,0*XX", // Datum code: W84 only
        "$PUBX,40,THS,0,0,0,0,0,0*XX", // True Heading and Status: PUBX,00
        "$PUBX,40,GSA,0,0,0,0,0,0*XX", // GNSS DOP and Active Satellites: P/H/VDOP
        "$PUBX,40,VLW,0,0,0,0,0,0*XX", // GNSS DOP and Active Satellites: P/H/VDOP

        /* on        DDC,Uart1,Uart2,USB, SPI */
        //"$PUBX,40,TXT,0,1,0,0,0,0*XX",  // GNSS Satellites in View: satin view = 0 on differential
        "$PUBX,40,GSV,0,1,0,0,0,0*XX", // GNSS Satellites in View: satin view = 0 on differential
        "$PUBX,40,GBS,0,1,0,0,0,0*XX", // GNSS Satellite Fault Detection: better then GST
        "$PUBX,40,GGA,0,1,0,0,0,0*XX", // Global positioning system fix data: HDOP, Differential
        "$PUBX,40,VTG,0,1,0,0,0,0*XX", // Course over ground: Differential, speed km/h
        "$PUBX,40,RMC,0,1,0,0,0,0*XX", // Recommended Minimum data: Date, Time, Pos, Diff.
};

/* CFG-MSG                              len 3 */
uint8_t cfg_msg[9] = {
    0xB5, 0x62, 0x06, 0x01, 3, 0, 0xF1 // msg class
    ,
    0x00 // msg id
    ,
    1 // msg rate
};

/* CFG-NMEA                              len 4 */
uint8_t cfg_nmea[10] = {
    0xB5, 0x62, 0x06, 0x17, 4, 0, 0 // no filter
    ,
    0x23 // nmea 2.3
    ,
    12 // 12 sat/msg
    ,
    1 // flag compat mode
};

/* cfg rate                               len 6 */
uint8_t cfg_rate[12] = {
    0xB5, 0x62, 0x06, 0x08, 6, 0, 0xE8 // low
    ,
    0x03 // 1000ms meas
    ,
    1 // low
    ,
    0 // high rate 1 meas/nav
    ,
    0, 0 // UTC time
};

/* CFG-PRT                                len 20 */
uint8_t cfg_port[26] = {0xB5, 0x62, 0x06, 0x00, 20, 0, 1 // port usb=3, Uart=1
                        ,
                        0 // n.u.
                        ,
                        0x00, 0x00 // txpin, n.u.
                        ,
                        0x00, 0x00, 0xC0 // low mode 8/N/1
                        ,
                        0x08, (115200 & 0x000000FF) // baud
                        ,
                        (115200 & 0x0000FF00) >> 8, (115200 & 0x00FF0000) >> 16, (115200 & 0xFF000000) >> 24, 0x23 // low inmask
                        ,
                        0x00 //         UBX,NMEA,RTCM3
                        ,
                        0x00 // low outmask
                        ,
                        0x00 //        2 = NMEA
                        ,
                        0x00 // flags eout
                        ,
                        0x00, 0 // n.u.
                        ,
                        0};

unsigned char getCrc(char *buffer, int len)
{
    int i;
    unsigned char csum = 0;
    for (i = 1; i < len - 3; i++)
    {
        csum ^= (unsigned char)(buffer[i]);
    }
    return csum;
}

void ubx_calc_crc(const char *msg, char *buf, int len)
{
    unsigned char crc;
    char txt[12];

    memcpy(buf, msg, len);
    crc = getCrc((char *)msg, len);
    sprintf(txt, "%02X", crc);
    memcpy((char *)&buf[len - 2], txt, 2);
    buf[len] = 0;
}

void writeCmd(int fd, char *cmd, char *buf)
{
    int len = strlen(cmd);
    ubx_calc_crc(cmd, buf, len);
    buf[len++] = '\r';
    buf[len++] = '\n';
    if (write(fd, buf, len) != len)
    {
        printf("ERR: write cmd!\n");
    }
}

int ubx_get_ack(int fd, uint8_t *buf)
{
    int i, len, timeout = 1000;
    while (1)
    {
        i = read(fd, &buf[0], 1);
        if (buf[0] == 0xB5) /// Binary u-blox
        {
            i = read(fd, &buf[1], 1);
            if (buf[1] == 0x62)
            {
                i = read(fd, &buf[2], 4); // clsID, msgID, lenL, lenH
                len = buf[4] + (buf[5] << 8);
                i = read(fd, &buf[6], len + 2); // len + 2*crc
                                                // parse_ack
#ifdef DEBUG
                for (i = 0; i < len + 8; i++)
                {
                    GNSS_PRINT(("%02X,", buf[i]));
                }
                GNSS_PRINT(("\n"));
#endif
                return 0;
            }
        }
        else
        {
            if (!timeout--)
                break;
        }
    }
    return -1;
}

void ubx_crc32(uint8_t *buf, int len)
{
    int i;
    int CK_A = 0, CK_B = 0;
    for (i = 0; i < len; i++)
    {
        CK_A = CK_A + buf[i];
        CK_B = CK_B + CK_A;
    }
    CK_A &= 0xFF;
    CK_B &= 0xFF;
    buf[len] = CK_A;
    buf[len + 1] = CK_B;
}

int ubx_send_bin(int fd, uint8_t *msg, uint8_t *buf, int len)
{
    memcpy(buf, msg, len);
    ubx_crc32(&buf[2], len - 2);
    if (write(fd, buf, len + 2) != len + 2)
    {
        printf("ERR: ublox write cmd!\n");
        return -1;
    }
    ubx_get_ack(fd, buf);
    return 0;
}

//***************************************************************************
/** @fn global int initGnss(int fd, struct gnssstr *my_gnss)
*
***************************************************************************
**/
struct gnssstr *initGnss(int fd)
{
    int i;
    GNSS_PRINT(("GNSS: init..\n"));

    memset(&priv, 0, sizeof(struct gnsspriv));
    memset(&gnss, 0, sizeof(struct gnssstr));
    /*
	 * baudrate out, not supported to Sirf NL-442U
	 */
    if (fd)
    {
        strcpy((char *)rxBuffer, "$PSRF100,1,115200,8,1,0,*FF\n\r");
        strcpy((char *)rxBuffer, "$PSRF100,1,4800,8,1,0*0C\n\r");

        i = strlen((char *)rxBuffer);
        unsigned char csum = getCrc((char *)rxBuffer, i - 2);
        sprintf((char *)&rxBuffer[i - 4], "%02X", csum);

        i = write(fd, rxBuffer, i);
    }
    return &gnss;
}

//***************************************************************************
/** @fn global gnss_parse(char *buffer)
*****************************************************************************
* @b Description: Parse the buffer
*****************************************************************************
* @param      buffer : data buffer (null terminated)
*****************************************************************************
* @return     1 if The GPRMC Sentence is found
*             0 if not found
*****************************************************************************
**/
int gnss_parse(char *buffer)
{
    char *p, *item[32];
    int i, j, bcsum;
    int len = strlen(buffer);
    unsigned char csum = 0;
    int valid = 0;
    int ret = 0;
    int psys;

    for (;;)
    {
        if (len < 4)
        {
            printf("%s too short\n", buffer);
            return ret;
        }
        if (buffer[len - 1] == '\r' || buffer[len - 1] == '\n')
        {
            buffer[--len] = '\0';
            if (buffer[len - 1] == '\r')
                buffer[--len] = '\0';
        }
        else
            break;
    }
    if (buffer[0] != '$')
    {
        printf("no leading $ in '%s'\n", buffer);
        return ret;
    }
    if (buffer[len - 3] != '*')
    {
        printf("no *XX in '%s' size %d\n", buffer, len);
        return ret;
    }
    csum = getCrc(buffer, len);

    if (!sscanf(buffer + len - 2, "%x", &bcsum))
    {
        printf("no checksum in '%s'\n", buffer);
        return ret;
    }
    if (bcsum != csum)
    {
        printf("wrong checksum in '%s was %x should be %x'\n", buffer, bcsum, csum);
        return ret;
    }

    i = 0;
    p = buffer;
    while (i < 31)
    {
        item[i++] = p;
        while (*p && *p != ',')
            p++;
        if (!*p)
            break;
        *p++ = '\0';
    }

    if (!strncmp(&buffer[1], "GP", 2))
    {
        psys = 0;
    }
    else if (!strncmp(&buffer[1], "GL", 2))
    {
        psys = 1;
    }
    else if (!strncmp(&buffer[1], "GN", 2))
    {
        psys = 2;
    }
    else if (!strncmp(&buffer[1], "BD", 2))
    {
        psys = 3;
    }
    else
        psys = 4;

    /*
     *
     */
    if (0)
    {
    }
#ifdef TXT_SUPPORT
    /*
     *    0    1  2  3  4
     * $GNTXT,01,01,01,DGNSS baseline big: 93 km*5C
     * $GNTXT,01,01,02,HW UBX-M80xx 00080000 *43'   USB mouse
     * $GNTXT,01,01,02,HW UBX-M8030 00080000*60     KIT
     * $GNTXT,01,01,02,MOD=NEO-M8P-2*7B
     *
     */
    else if (!strncmp(&buffer[3], "TXT", 3))
    {

        p = strstr(item[4], "HW ");
        if (p)
        {
            memset(priv.hw, 0, 16);
            strncpy(priv.hw, p + 3, 16);

            GNSS_PRINT(("HW: %s\n", priv.hw));
        }
        p = strstr(item[4], "MOD=");
        if (p)
        {
            memset(priv.modem, 0, 16);
            strncpy(priv.modem, p + 4, 16);

            GNSS_PRINT(("Modem: %s\n", priv.modem));
        }
        p = strstr(item[4], "baseline big:");
        if (p)
        {
            char valstr[5];
            memset(valstr, 0, 5);
            strncpy(valstr, p + 13, 4);
            gnss.baseline = atol(valstr);

            GNSS_PRINT(("Distance to Base %d km!\n", gnss.baseline));
        }
    }
#endif
#ifdef GBS_SUPPORT
    /*
     * 0           1      2    3    4  5  6  7  8
     * $GNGBS,135641.000,9.6,16.7,22.2,,,,*66
     */
    else if (!strncmp(&buffer[3], "GBS", 3))
    {

        gnss.errLat = 0;
        gnss.errLon = 0;
        gnss.errAlt = 0;
        if (*item[2])
        {
            sscanf(item[2], "%lf", &gnss.errLat);
        }
        if (*item[3])
        {
            sscanf(item[3], "%lf", &gnss.errLon);
        }
        if (*item[4])
        {
            sscanf(item[4], "%lf", &gnss.errAlt);
        }
        GNSS_PRINT(("dlat %2.2fm ,dlon %2.2fm ,dalt %2.2fm\n", gnss.errLat, gnss.errLon, gnss.errAlt));
    }
#endif
#ifdef PUBX_SUPPORT
    /*
     *     0  1      2       3       4     5       6   7     8   9 10   11   12      13    14  15   16   17 18 19 20
     * $PUBX,00,065614.00,5104.00113,N,01338.24372,E,325.059,D3,19,21,3.651,300.84,-1.117,1.0,1.33,1.79,1.35,0,0,0*69
     *
     */
    else if (!strncmp(buffer, "$PUBX", 5))
    {

        /*
        * Lat/Long Position Data
        * in Grad/Minuten
        */
        gnss.geo.lat = 0.0;
        gnss.geo.lon = 0.0;
        gnss.geo.alt = 0.0;

        if (!strncmp(&buffer[6], "00", 2))
        {
            if (*item[2])
            {
                g_time(item[2], &gnss);
            }
            if (*item[3])
            {
                sscanf(item[3], "%lf", &gnss.geo.lat);
                gnss.geo.lat /= 100;
                if (*item[4] == 'S')
                    gnss.geo.lat = -gnss.geo.lat;
            }
            if (*item[5])
            {
                sscanf(item[5], "%lf", &gnss.geo.lon);
                gnss.geo.lon /= 100;
                if (*item[6] == 'W')
                    gnss.geo.lon = -gnss.geo.lon;
            }
            if (*item[7])
            {
                sscanf(item[7], "%lf", &gnss.geo.alt);
            }
            if (*item[11])
            {
                gnss.speed = g_ascii_strtod(item[11]);
            }
            GNSS_PRINT(("lat %4.7f°m ,lon %4.7f°m ,alt %4.3fm ,speed %3.3fkm/h\n", gnss.geo.lat, gnss.geo.lon, gnss.geo.alt, gnss.speed));
        }
        /*
        * Satellite Status
        *     0  1  2 3 4   5  6 7 8 9 10 11 12 13 14
        * $PUBX,03,20,2,-,121,25,,000,3,-,009,01,,000,6,U,073,28,12,000,12,U,034,81,18,000,14,U,316,23,19,000,15,-,184,-2,,
        *       000,17,-,042,08,,000,19,U,049,29,13,000,22,-,349,01,,000,24,U,148,52,12,000,25,U,270,48,27,064,29,-,207,13,,
        *       000,31,-,307,02,13,000,32,U,289,35,23,000,65,-,340,16,,000,71,e,245,08,20,000,72,U,293,23,34,064,73,U,357,
        *       75,12,000,74,U,303,28,26,048,80,-,103,34,,000*74
        */
        else if (!strncmp(&buffer[6], "03", 2))
        {
        }
        /*
        * Time of Day and Clock Information
        *     0  1    2        3      4        5    6   7     8      9
        * $PUBX,04,065614.00,110718,284173.99,2009,18,236119,227.087,21*1B
        */
        else if (!strncmp(&buffer[6], "04", 2))
        {
            sscanf(item[3], "%02d%02d%02d",
                   &gnss.fixday,
                   &gnss.fixmonth,
                   &gnss.fixyear);
            gnss.fixyear += 2000;
        }
    }
#endif
#ifdef RMC_SUPPORT
    else if (!strncmp(&buffer[3], "RMC", 3))
    {
        /*                                                          11 1
           0      1      2 3        4 5         6 7     8     9     01 2
           $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,,,A*6A
           Time[1],Active/Void[2],lat[3],N/S[4],long[5],W/E[6],speed in knots[7],track angle[8],date[9],
           magnetic variation[10],magnetic variation direction[11]
         */

        if (*item[1])
        {
            g_time(item[1], &gnss);

            GNSS_PRINT(("UTC %s\n", gnss.fixtime));
        }
        if (*item[9])
        {
            sscanf(item[9], "%02d%02d%02d",
                   &gnss.fixday,
                   &gnss.fixmonth,
                   &gnss.fixyear);
            gnss.fixyear += 2000;

            /* date -s "20180717 08:58:43" */
            GNSS_PRINT(("DATE %d.%d.%d\n", gnss.fixday, gnss.fixmonth, gnss.fixyear));
        }
        if (*item[1] && *item[9])
        {
            char d[30];
            sprintf(d, "date -s \"%04d%02d%02d %s\"", gnss.fixyear, gnss.fixmonth, gnss.fixday, gnss.fixtime);
            //GNSS_PRINT(("GNSS: date %s\n", d));
        }

        if (*item[2] == 'A')
            valid = 1;
        if ((*item[12] == 'A') || (*item[12] == 'D'))
        {

            gnss.direction = g_ascii_strtod(item[8]);
            gnss.speed = g_ascii_strtod(item[7]);
            gnss.speed *= 1.852;

            if (*item[12] == 'D')
            {
                GNSS_PRINT(("DGPS\n"));
            }
            else if (*item[12] == 'A')
            {
                GNSS_PRINT(("GPS 3D\n"));
            }
        }
    }
#endif
#ifdef GGA_SUPPORT
    else if (!strncmp(&buffer[3], "GGA", 3))
    {
        /*                                                           1 1111
           0      1          2         3 4          5 6 7  8   9     0 1234
           $GPGGA,184424.505,4924.2811,N,01107.8846,E,1,05,2.5,408.6,M,,,,0000*0C
           UTC of Fix[1],Latitude[2],N/S[3],Longitude[4],E/W[5],Quality(0=inv,1=gps,2=dgps)[6],Satelites used[7],
           HDOP[8],Altitude[9],"M"[10],height of geoid[11], "M"[12], time since dgps update[13], dgps ref station [14]
         */
        if (*item[1])
        {
            g_time(item[1], &gnss);
            //           GNSS_PRINT(("UTC %s\n", gnss.fixtime));
        }
        if (*item[2] && *item[3] && *item[4] && *item[5])
        {

            sscanf(item[2], "%lf", &gnss.geo.lat);
            gnss.geo.lat /= 100;
            if (*item[3] == 'S')
                gnss.geo.lat = -gnss.geo.lat;

            sscanf(item[4], "%lf", &gnss.geo.lon);
            gnss.geo.lon /= 100;
            if (*item[5] == 'W')
                gnss.geo.lon = -gnss.geo.lon;
        }

        if (*item[6])
            sscanf(item[6], "%d", &gnss.status);
        if (*item[7])
        {
            sscanf(item[7], "%d", &gnss.sats_used[psys]);
            //           GNSS_PRINT(("SAT %d used: %d\n", psys, gnss.sats_used[psys]));
        }
        if (*item[8])
        {
            sscanf(item[8], "%lf", &gnss.hdop);
        }
        if (*item[9])
        {
            sscanf(item[9], "%lf", &gnss.geo.alt);
        }
        GNSS_PRINT(("lat %4.7f° ,lon %4.7f° ,alt %4.3f ,speed %3.3fkm/h\n", gnss.geo.lat, gnss.geo.lon, gnss.geo.alt, gnss.speed));
    }
#endif
#ifdef VTG_SUPPORT
    else if (!strncmp(&buffer[3], "VTG", 3))
    {
        /* 0      1      2 34 5    6 7   8
           $GPVTG,143.58,T,,M,0.26,N,0.5,K*6A
           Course Over Ground Degrees True[1],"T"[2],Course Over Ground Degrees Magnetic[3],"M"[4],
           Speed in Knots[5],"N"[6],"Speed in KM/H"[7],"K"[8]
         */
        if (item[1] && item[7])
            valid = 1;
        if (i >= 10 && (*item[9] == 'A' || *item[9] == 'D'))
            valid = 1;
        if (valid)
        {
            gnss.direction = g_ascii_strtod(item[1]);
            gnss.speed = g_ascii_strtod(item[7]);

            //          GNSS_PRINT(("VTG: dir %lf, speed %2.1lf km/h\n", gnss.direction, gnss.speed));
        }
    }
#endif
#ifdef GSV_SUPPORT
    else if (!strncmp(&buffer[3], "GSV", 3) && i >= 4)
    {
        /*
        0 GSV      Satellites in view
        1 2        Number of sentences for full data
        2 1        sentence 1 of 2
        3 08       Number of satellites in view

        4 01       Satellite PRN number
        5 40       Elevation, degrees
        6 083      Azimuth, degrees
        7 46       SNR - higher is better
               for up to 4 satellites per sentence
        *75    the checksum data, always begins with *
    */
        if (item[3])
        {
            sscanf(item[3], "%d", &gnss.sats_visible[psys]);

            GNSS_PRINT(("SAT %d visible: %d\n", psys, gnss.sats_visible[psys]));
        }
        j = 4;
        while (j + 4 <= i && priv.current_count < 24)
        {
            struct gps_sat *sat = &priv.next[priv.next_count++];
            sat->prn = atoi(item[j]);
            sat->elevation = atoi(item[j + 1]);
            sat->azimuth = atoi(item[j + 2]);
            sat->snr = atoi(item[j + 3]);
            j += 4;
        }
        if (!strcmp(item[1], item[2]))
        {
            gnss.sats_signal = 0;
            for (i = 0; i < priv.next_count; i++)
            {
                priv.current[i] = priv.next[i];
                if (priv.current[i].snr)
                    gnss.sats_signal++;
            }
            priv.current_count = priv.next_count;
            priv.next_count = 0;
        }
    }
#endif
#ifdef GLL_SUPPORT
    else if (!strncmp(&buffer[3], "GLL", 3))
    {
        /*
     * 0      1          2 3           4 5         6
     * $GNGLL,5103.96593,N,01338.26468,E,115249.00,A,A*7D
     */
        if (*item[5])
        {
            g_time(item[5], &gnss);

            GNSS_PRINT(("GLL: UTC %s\n", gnss.fixtime));
        }
    }
#endif
#ifdef GSA_SUPPORT
    else if (!strncmp(&buffer[3], "GSA", 3))
    {
        /*
     * 0      1 2 3  4  5  6  7  8  9  10 ....15   16   17
     * $GNGSA,A,3,29,31,25,12,02,21,05,26,,,,,1.63,0.95,1.32*19
     */
        if (*item[2])
        {
            gnss.mode = atoi(item[2]);
        }
        if (*item[15])
        {
            sscanf(item[15], "%lf", &gnss.pdop);
        }
        if (*item[16])
        {
            sscanf(item[16], "%lf", &gnss.hdop);
        }
        if (*item[17])
        {
            sscanf(item[17], "%lf", &gnss.vdop);
        }
        GNSS_PRINT(("Hdop %2.4f Pdop %2.4f Vdop %2.4f\n", gnss.hdop, gnss.pdop, gnss.vdop));
    }
#endif
#ifdef ZDA_SUPPORT
    else if (!strncmp(&buffer[3], "ZDA", 3))
    {
        /*
        0        1        2  3  4    5  6
        $GPZDA,hhmmss.ss,dd,mm,yyyy,xx,yy*CC
            hhmmss    HrMinSec(UTC)
            dd,mm,yyy Day,Month,Year
            xx        local zone hours -13..13
            yy        local zone minutes 0..59
    */
        if (item[1] && item[2] && item[3] && item[4])
        {
            strncpy(gnss.fixtime, item[1], strlen(gnss.fixtime));
            gnss.fixday = atoi(item[2]);
            gnss.fixmonth = atoi(item[3]);
            gnss.fixyear = atoi(item[4]);
        }
    }
#endif
#ifdef IISMD_SUPPORT
    else if (!strncmp(buffer, "$IISMD", 6))
    {
        /*
        0      1   2     3      4
        $IISMD,dir,press,height,temp*CC"
            dir       Direction (0-359)
            press     Pressure (hpa, i.e. 1032)
            height    Barometric height above ground (meter)
            temp      Temperature (Degree Celsius)
    */
        if (item[1])
        {
            gnss.magnetic_direction = g_ascii_strtod(item[1]);

            GNSS_PRINT(("IISMD: magnetic %d\n", gnss.magnetic_direction));
        }
    }
#endif
    /* sirf */
    else if (!strncmp(buffer, "$PSRF150", 6))
    {
        /*
        0        1
        $PSRF150,status*3F"
    */
        if (item[1])
        {
            priv.pwr = g_ascii_strtod(item[1]);

            GNSS_PRINT(("PSRF150: power %s\n", priv.pwr ? "on" : "off"));
        }
    }
    return ret;
}

/* ***************************************************************
 * getGnssMsg string
 * ***************************************************************/
int getGnssMsg(int fd, struct gnssstr *gnss)
{
    int i, ret;

    while (1)
    {
        ret = read(fd, rxBuffer, 1);
        if (ret > 0)
        {
            /* readln */
            if (rxBuffer[0] == '$')
            {
                i = 1;
                while (1)
                {
                    ret = read(fd, &rxBuffer[i], 1);
                    if (ret > 0)
                    {
                        if ((rxBuffer[i] == '\r') || (rxBuffer[i] == '\n'))
                        {
                            rxBuffer[i] = 0;
                            break;
                        }
                        i++;
                        if (i > BUFFER_MAX - 2)
                            break;
                    }
                }
                printf("%s\n", rxBuffer);

                gnss_parse((char *)rxBuffer);

                return 0;
            }
        }
        else
        {
            usleep(100);
        }
    }
    return -1;
}

/* ***************************************************************
 * differential NTRIP
 * ***************************************************************/
struct ntrip igsIpNet[] =
    {
        {"HUEG00DEU0", "Huegelheim", 47.83, 7.60},
        {"OBE400DEU0", "Oberpfaffenhofen", 48.08, 11.28},
        {"WTZR00DEU0", "Wettzell", 49.14, 12.88},
        {"GOPE00CZE0", "Praha", 49.91, 14.79},
        {"FFMJ00DEU0", "Frankfurt", 50.09, 8.66},
        {"FFMJ00DEU1", "Frankfurt-A", 50.09, 8.66},
        {"TIT200DEU0", "Titz", 51.04, 6.43},
        {"LEIJ00DEU0", "Leipzig", 51.35, 12.37}, // Zoo
        {"POTS00DEU0", "Potsdam", 52.38, 13.07}, // Helmholz Institut
        {"WARN00DEU0", "Warnemuende", 54.17, 12.10},
        {"SASS00DEU0", "Sassnitz", 54.51, 13.64},
        {"", "", 0, 0},
};

struct ntrip ntripSaposDe[] =
    {
        {"0129GG", "Loebau", 51.11, 14.69},
        {"0132GG", "Chemnitz", 50.83, 12.92},
        {"0133GG", "Leipzig", 51.35, 12.37},
        {"0136GG", "Torgau", 51.56, 13.00},
        {"0139GG", "Rothenburg", 51.36, 14.95},
        {"0140GG", "Waldheim", 51.07, 13.00},
        {"0142GG", "Thiendorf", 51.29, 13.73},
        {"0143GG", "Zwickau", 50.72, 12.49},
        {"0144GG", "Rabenberg", 50.45, 12.74},
        {"0145GG", "Bad Elster", 50.28, 12.24},
        {"0147GG", "Zittau", 50.90, 14.83},
        {"0148GG", "Plauen 2", 50.50, 12.15},
        {"0149GG", "Altendorf", 50.93, 14.18},
        {"0151GG", "Oschatz 2", 51.29, 13.11},
        {"0153GG", "Hoyerswerda", 51.44, 14.25},
        {"0154GG", "Elstra 2", 51.22, 14.13},
        {"0156GG", "Sayda", 50.71, 13.43},
        {"0157GG", "Dresden 4", 51.03, 13.73},
        {"", "", 0, 0},
};

/*
 * X = lon (äquator linie) in grad
 * Y = lat (pol linie)     in grad
 * 1° = π/180 rad ≈ 0.01745
 */
int distance(double lon1, double lat1, double lon2, double lat2)
{
    double lat = (lat1 + lat2) / 2 * 0.01745;
    double dx = 111.13 * cos(lat) * (lon1 - lon2);
    //double dx = 71.5 * (lon1 - lon2);
    double dy = 111.13 * (lat1 - lat2);
    double dst = (sqrt((dx * dx) + (dy * dy))) * 1000; // m
    return ((int)dst);
}

char *nextNtripStream(int id, double lat, double lon)
{
    int i, d1, dmin, idxmin;

    struct ntrip *net = igsIpNet;
    if (id == IGS_SYSTEM)
        net = igsIpNet;
    else if (id == SAPOS_SYSTEM)
        net = ntripSaposDe;
    else
        return (char *)"";

    dmin = 300000;
    idxmin = 0;

    for (i = 0;; i++)
    {
        if (net[i].lon == 0)
            break;
        d1 = distance(lon, lat, net[i].lon, net[i].lat) / 1000; // leipzig
        if (d1 < dmin)
        {
            dmin = d1;
            idxmin = i;
        }
    }
    //NTRIP_PRINT((" GNSS: next dStation => %s %d km stream: %s\n",net[idxmin].location, dmin, net[idxmin].stream));

    return net[idxmin].stream;
}
