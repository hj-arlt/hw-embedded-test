/*
 * can.h
 *
 *  Created on: 28.04.2017
 *      Author: hans-juergen.arlt <hj@arlt2net.de>
 */

#ifndef CAN_H_
#define CAN_H_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>
#include <stdbool.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>

//#define DEBUG
//#define DEBUG_DATA
//#define CANMSG_REDUCED

#define CAN_DEFAULT_DEVICE   "can0"

struct canmb_msg
{
#ifndef CANMSG_REDUCED
   char description[32];
#endif
   uint32_t id;
   unsigned char len;
   int cycle_fast;
   int cycle_slow;
};


struct canstr
{
   uint8_t kl_status;
   uint32_t kmstand;
   uint8_t dimm;
   uint8_t tanklow;
   uint16_t licht;
};

extern const struct canmb_msg msgtbl[];
void ddata(int id, uint8_t *pd);

/* functions*/
int getcanmb(uint32_t id);
int getcyclefast(uint32_t id);

/* can rx */
int  display_klemme(struct can_frame *cf, struct canstr *can);
/* can tx */
void init_Klemme_msg(struct can_frame *cf, int klemme);


int initCan(char *device, int *canfd);


#endif /* CONNECTIVITY_BOX_CAN_CAN_SRC_CANMSG_H_ */
