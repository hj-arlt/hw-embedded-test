/*
 * can.c
 *
 *  Created on: 28.04.2017
 *      Author: hans-juergen.arlt <hj@arlt2net.de>
 */
#include "can.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/can.h>
#include <linux/can/raw.h>


const struct canmb_msg msgtbl[] =
{

{
#ifndef CANMSG_REDUCED
      "Klemmen_Status",
#endif
      0x3C0, 4, 100, 1000 }, /* Klemmen_Status_01_BZ    2  0  4
							 RSt_Fahrerhinweise         2  4  4
							 ZAS_Kl_S                   3  0  1
							 ZAS_Kl_15                  3  1  1
							 ZAS_Kl_X                   3  2  1
							 ZAS_Kl_50_Startanforderung 3  3  1
							 BCM_Remotestart_Betrieb    3  4  1
							 ZAS_Kl_Infotainment        3  5  1
							 BCM_Remotestart_KL15_Anf   3  6  1
							 BCM_Remotestart_MO_Start   3  7  1
							 KST_Warn_P1_ZST_def        4  0  1
							 KST_Warn_P2_ZST_def        4  1  1
							 KST_Fahrerhinweis_1        4  2  1
							 KST_Fahrerhinweis_2        4  3  1
							 BCM_Ausparken_Betrieb      4  4  1
							 KST_Fahrerhinweis_4        4  5  1
							 KST_Fahrerhinweis_5        4  6  1
							 KST_Fahrerhinweis_6        4  7  1 */
{
#ifndef CANMSG_REDUCED
            "",
#endif
            0, 0, 0, 0 },
};

/* ****************************************************
 *                 helper
 ****************************************************** */

#ifdef DEBUG_DATA
/* DEBUG only */
void ddata(int id, uint8_t *pd)
{
   printf("%08X - %02X %02X %02X %02X %02X %02X %02X %02X\n"
         ,id,pd[0],pd[1],pd[2],pd[3],pd[4],pd[5],pd[6],pd[7]);
}
#endif

int getcanmb(uint32_t id)
{
   int i = 0;
   if (id < 0)
      return -1;
   while (msgtbl[i].id)
   {
      if (id == msgtbl[i].id)
         return i;
      i++;
   }
   return -1;
}

int getcyclefast(uint32_t id)
{
   uint32_t i = getcanmb(id);
   if (i < 0)
      return 1000; // default 1 sec in ms
   return msgtbl[i].cycle_fast ? msgtbl[i].cycle_fast : 1000;
}

/* ****************************************************
 *                  receive msg
 ****************************************************** */

/* Klemmen_Status           2  0  4
 RSt_Fahrerhinweise         2  4  4
 ZAS_Kl_S                   3  0  1
 ZAS_Kl_15                  3  1  1
 ZAS_Kl_X                   3  2  1
 ZAS_Kl_50_Startanforderung 3  3  1
 BCM_Remotestart_Betrieb    3  4  1
 ZAS_Kl_Infotainment        3  5  1
 BCM_Remotestart_KL15_Anf   3  6  1
 BCM_Remotestart_MO_Start   3  7  1
 KST_Warn_P1_ZST_def        4  0  1
 KST_Warn_P2_ZST_def        4  1  1
 KST_Fahrerhinweis_1        4  2  1
 KST_Fahrerhinweis_2        4  3  1
 BCM_Ausparken_Betrieb      4  4  1
 KST_Fahrerhinweis_4        4  5  1
 KST_Fahrerhinweis_5        4  6  1
 KST_Fahrerhinweis_6        4  7  1
 */
int display_klemme(struct can_frame *cf, struct canstr *can)
{
   int ret = -1;

   if (cf->can_id != 0x3C0)
      return ret;

   can->kl_status = cf->data[2];
   /* KL15 off */
   if ((can->kl_status & 2) && ((cf->data[2] & 2) == 0))
   {
      ret = 1;
   }
   else if (((can->kl_status & 2) == 0) && (cf->data[2] & 2))
   {
      ret = 2;
   }

#ifdef DEBUG
   if (cf->data[2] & 1) printf("KL S on, ");
   if (cf->data[2] & 2) printf("KL 15 on, ");
   if (cf->data[2] & 4) printf("KL X on, ");
   if (cf->data[2] & 8) printf("KL 50 start, ");
   if (cf->data[2] & 0x10) printf("Rem on, ");
   if (cf->data[2] & 0x20) printf("ZAS on, ");
   if (cf->data[2] & 0x40) printf("RemKL 15 on, ");
   if (cf->data[2] & 0x80) printf("RemMO on, ");
   if (cf->data[2]) printf("\n");
#endif
   return ret;
}

/*
 * 000003C0 - CA 0A 43 00 CF 00 00 00
 */
void init_Klemme_msg(struct can_frame *cf, int klemme)
{
   if (!cf)
      return;
   cf->can_id = 0x3C0;
   cf->can_dlc = 8;
   memset(cf->data, 0, 8);
   cf->data[0] = 0xCA;
   cf->data[1] = 0x0A;
   cf->data[2] = 0x43;
   cf->data[4] = 0xCF;
}

/* ***************************************************************
 * CAN                                   MSG_DONTWAIT, MSG_WAITALL
 * ***************************************************************/
int initCan(char *device, int *canfd)
{
   struct sockaddr_can saddr;
   struct ifreq ifr;

   if ((*canfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
   {
      perror("Error while opening socket");
      return -1;
   }
   if (device != NULL)
      strcpy(ifr.ifr_name, device);
   else
      strcpy(ifr.ifr_name, CAN_DEFAULT_DEVICE);
   if (ioctl(*canfd, SIOCGIFINDEX, &ifr) == -1)
   {
	   close(*canfd);
	   *canfd = -1;
	   //perror("Error in can ioctrl");
	   return -2;
   }

   saddr.can_family  = AF_CAN;
   saddr.can_ifindex = ifr.ifr_ifindex;

   fcntl(*canfd, F_SETFL, O_NONBLOCK);

   if (bind(*canfd, (struct sockaddr *) &saddr, sizeof(saddr)) < 0)
   {
	   close(*canfd);
	   *canfd = -1;
       perror("Error in socket bind");
       return -3;
   }
   printf("CAN: %s init at %d ok.\n", ifr.ifr_name, ifr.ifr_ifindex);

   return 0;
}

