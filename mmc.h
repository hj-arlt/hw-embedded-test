/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License v2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 021110-1307, USA.
 */

#ifndef MMC_H
#define MMC_H

#include <asm-generic/int-ll64.h>
#include <linux/mmc/ioctl.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>               /* time_t, time, ctime */


// TN-FC-14/27 eMMC Power loss data integrity
//#define RELIABILITY_WRITE_SUPPORT

// TN-FC-32 eMMC Health commands
#define HEALTH_SUPPORT

// TN-FC-60 eMMC bkops refresh support
#define BKOPS_SUPPORT

// TN-FC-16 eMMC BUFFER BLOCKS REFRESH
//#define CMD56_BUFFER_BLOCKS_REFRESH

//#define CONTINUE_READ_SUPPORT



#define CHECK(expr, msg, err_stmt) { if (expr) { fprintf(stderr, msg); err_stmt; } }

/* From kernel linux/major.h */
#define MMC_BLOCK_MAJOR			179


/*
  MMC status in R1, for native mode (SPI bits are different)
  Type
	e : error bit
	s : status bit
	r : detected and set for the actual command response
	x : detected and set during command execution. the host must poll
            the card by sending status command in order to read these bits.
  Clear condition
	a : according to the card state
	b : always related to the previous command. Reception of
            a valid command will clear it (with a delay of one command)
	c : clear by read
 */

#define R1_OUT_OF_RANGE			(1 << 31)	/* er, c */
#define R1_ADDRESS_ERROR		(1 << 30)	/* erx, c */
#define R1_BLOCK_LEN_ERROR		(1 << 29)	/* er, c */
#define R1_ERASE_SEQ_ERROR      (1 << 28)	/* er, c */
#define R1_ERASE_PARAM			(1 << 27)	/* ex, c */
#define R1_WP_VIOLATION			(1 << 26)	/* erx, c */
#define R1_CARD_IS_LOCKED		(1 << 25)	/* sx, a */
#define R1_LOCK_UNLOCK_FAILED	(1 << 24)	/* erx, c */
#define R1_COM_CRC_ERROR		(1 << 23)	/* er, b */
#define R1_ILLEGAL_COMMAND		(1 << 22)	/* er, b */
#define R1_CARD_ECC_FAILED		(1 << 21)	/* ex, c */
#define R1_CC_ERROR				(1 << 20)	/* erx, c */
#define R1_ERROR				(1 << 19)	/* erx, c */
#define R1_UNDERRUN				(1 << 18)	/* ex, c */
#define R1_OVERRUN				(1 << 17)	/* ex, c */
#define R1_CID_CSD_OVERWRITE	(1 << 16)	/* erx, c, CID/CSD overwrite */
#define R1_WP_ERASE_SKIP		(1 << 15)	/* sx, c */
#define R1_CARD_ECC_DISABLED	(1 << 14)	/* sx, a */
#define R1_ERASE_RESET			(1 << 13)	/* sr, c */
#define R1_STATUS(x)            (x & 0xFFFFE000)
#define R1_CURRENT_STATE(x)		((x & 0x00001E00) >> 9)	/* sx, b (4 bits) */
#define R1_READY_FOR_DATA		(1 << 8)	/* sx, a */
#define R1_SWITCH_ERROR			(1 << 7)	/* sx, c */
#define R1_EXCEPTION_EVENT		(1 << 6)	/* sr, a */
#define R1_APP_CMD				(1 << 5)	/* sr, c */

/*
 * BKOPS status level
 */
#define EXT_CSD_BKOPS_LEVEL_2		0x2

/*
 * BKOPS modes
 */
#define EXT_CSD_MANUAL_BKOPS_MASK	0x01
#define EXT_CSD_AUTO_BKOPS_MASK		0x02

/*
 * Command Queue
 */
#define EXT_CSD_CMDQ_MODE_ENABLED	BIT(0)
#define EXT_CSD_CMDQ_DEPTH_MASK		GENMASK(4, 0)
#define EXT_CSD_CMDQ_SUPPORTED		BIT(0)

/*
 * MMC_SWITCH access modes
 */
#define MMC_SWITCH_MODE_CMD_SET		0x00	/* Change the command set */
#define MMC_SWITCH_MODE_SET_BITS	0x01	/* Set bits which are 1 in value */
#define MMC_SWITCH_MODE_CLEAR_BITS	0x02	/* Clear bits which are 1 in value */
#define MMC_SWITCH_MODE_WRITE_BYTE	0x03	/* Set target to value */

/* Standard MMC commands (4.1)           type  argument     response */
   /* class 1 */
#define MMC_GO_IDLE_STATE         0   /* bc                          */
#define MMC_SEND_OP_COND          1   /* bcr  [31:0] OCR         R3  */
#define MMC_ALL_SEND_CID          2   /* bcr                     R2  */
#define MMC_SET_RELATIVE_ADDR     3   /* ac   [31:16] RCA        R1  */
#define MMC_SET_DSR               4   /* bc   [31:16] RCA            */
#define MMC_SLEEP_AWAKE		      5   /* ac   [31:16] RCA 15:flg R1b */
#define MMC_SWITCH                6   /* ac   [31:0] See below   R1b */
#define MMC_SELECT_CARD           7   /* ac   [31:16] RCA        R1  */
#define MMC_SEND_EXT_CSD          8   /* adtc                    R1  */
#define MMC_SEND_CSD              9   /* ac   [31:16] RCA        R2  */
#define MMC_SEND_CID             10   /* ac   [31:16] RCA        R2  */
#define MMC_READ_DAT_UNTIL_STOP  11   /* adtc [31:0] dadr        R1  */
#define MMC_STOP_TRANSMISSION    12   /* ac                      R1b */
#define MMC_SEND_STATUS          13   /* ac   [31:16] RCA        R1  */
#define MMC_BUS_TEST_R           14   /* adtc                    R1  */
#define MMC_GO_INACTIVE_STATE    15   /* ac   [31:16] RCA            */
#define MMC_BUS_TEST_W           19   /* adtc                    R1  */
#define MMC_SPI_READ_OCR         58   /* spi                  spi_R3 */
#define MMC_SPI_CRC_ON_OFF       59   /* spi  [0:0] flag      spi_R1 */

  /* class 2 */
#define MMC_SET_BLOCKLEN         16   /* ac   [31:0] block len   R1  */
#define MMC_READ_SINGLE_BLOCK    17   /* adtc [31:0] data addr   R1  */
#define MMC_READ_MULTIPLE_BLOCK  18   /* adtc [31:0] data addr   R1  */
#define MMC_SEND_TUNING_BLOCK    19   /* adtc                    R1  */
#define MMC_SEND_TUNING_BLOCK_HS200	21	/* adtc R1  */

  /* class 3 */
#define MMC_WRITE_DAT_UNTIL_STOP 20   /* adtc [31:0] data addr   R1  */

  /* class 4 */
#define MMC_SET_BLOCK_COUNT      23   /* adtc [31:0] data addr   R1  */
#define MMC_WRITE_BLOCK          24   /* adtc [31:0] data addr   R1  */
#define MMC_WRITE_MULTIPLE_BLOCK 25   /* adtc                    R1  */
#define MMC_PROGRAM_CID          26   /* adtc                    R1  */
#define MMC_PROGRAM_CSD          27   /* adtc                    R1  */

  /* class 6 */
#define MMC_SET_WRITE_PROT       28   /* ac   [31:0] data addr   R1b */
#define MMC_CLR_WRITE_PROT       29   /* ac   [31:0] data addr   R1b */
#define MMC_SEND_WRITE_PROT      30   /* adtc [31:0] wpdata addr R1  */

  /* class 5 */
#define MMC_ERASE_GROUP_START    35   /* ac   [31:0] data addr   R1  */
#define MMC_ERASE_GROUP_END      36   /* ac   [31:0] data addr   R1  */
#define MMC_ERASE                38   /* ac                      R1b */

  /* class 9 */
#define MMC_FAST_IO              39   /* ac   <Complex>          R4  */
#define MMC_GO_IRQ_STATE         40   /* bcr                     R5  */

  /* class 7 */
#define MMC_LOCK_UNLOCK          42   /* adtc                    R1b */

  /* class 8 */
#define MMC_APP_CMD              55   /* ac   [31:16] RCA        R1  */
#define MMC_GEN_CMD              56   /* adtc [0] RD/WR          R1  */

  /* class 11 */
#define MMC_QUE_TASK_PARAMS      44   /* ac   [20:16] task id    R1  */
#define MMC_QUE_TASK_ADDR        45   /* ac   [31:0] data addr   R1  */
#define MMC_EXECUTE_READ_TASK    46   /* adtc [20:16] task id    R1  */
#define MMC_EXECUTE_WRITE_TASK   47   /* adtc [20:16] task id    R1  */
#define MMC_CMDQ_TASK_MGMT       48   /* ac   [20:16] task id    R1b */


/*
 * EXT_CSD fields
 */
#define EXT_CSD_S_CMD_SET			504
#define EXT_CSD_HPI_FEATURE			503
#define EXT_CSD_BKOPS_SUPPORT		502	/* RO */


#define EXT_CSD_VENDOR_PROP_HEALTH	301

#define EXT_CSD_DEV_LIFETIME_B		269
#define EXT_CSD_DEV_LIFETIME_A		268
#define EXT_CSD_PRE_EOL_INFO		267

#define EXT_CSD_CACHE_SIZE_3		252
#define EXT_CSD_CACHE_SIZE_2		251
#define EXT_CSD_CACHE_SIZE_1		250
#define EXT_CSD_CACHE_SIZE_0		249

#define EXT_CSD_BKOPS_STATUS		246	/* RO */
#define EXT_CSD_SEC_CORRECT_3		245
#define EXT_CSD_SEC_CORRECT_2		244
#define EXT_CSD_SEC_CORRECT_1		243
#define EXT_CSD_SEC_CORRECT_0		242

#define EXT_CSD_BOOT_INFO			228	/* R/W */
#define EXT_CSD_HC_ERASE_GRP_SIZE	224
#define EXT_CSD_REL_WR_SEC_C		222
#define EXT_CSD_SEC_COUNT_3			215
#define EXT_CSD_SEC_COUNT_2			214
#define EXT_CSD_SEC_COUNT_1			213
#define EXT_CSD_SEC_COUNT_0			212

#define EXT_CSD_MIN_PERF_W_8_52		210
#define EXT_CSD_MIN_PERF_R_8_52		209
#define EXT_CSD_MIN_PERF_W_8_26_4_52		208
#define EXT_CSD_MIN_PERF_R_8_26_4_52		207
#define EXT_CSD_MIN_PERF_W_4_26		206
#define EXT_CSD_MIN_PERF_R_4_26		205

#define EXT_CSD_PART_SWITCH_TIME	199
#define EXT_CSD_REV					192
#define EXT_CSD_BOOT_CFG			179
#define EXT_CSD_PART_CONFIG			179
#define EXT_CSD_ERASE_GROUP_DEF		175
#define EXT_CSD_BOOT_WP				173

#define EXT_CSD_WR_REL_SET			167
#define EXT_CSD_WR_REL_PARAM		166
#define EXT_CSD_SANITIZE_START		165

#define EXT_CSD_BKOPS_START			164	/* R/W */
#define EXT_CSD_BKOPS_EN			163	/* R/W */
#define EXT_CSD_RST_N_FUNCTION		162	/* R/W */
#define EXT_CSD_PARTITIONING_SUPPORT		160	/* RO */
#define EXT_CSD_PARTITIONS_ATTRIBUTE		156	/* R/W */
#define EXT_CSD_PARTITION_SETTING_COMPLETED	155	/* R/W */
#define EXT_CSD_ENH_SIZE_MULT_2		142
#define EXT_CSD_ENH_SIZE_MULT_1		141
#define EXT_CSD_ENH_SIZE_MULT_0		140
#define EXT_CSD_ENH_START_ADDR_3	139
#define EXT_CSD_ENH_START_ADDR_2	138
#define EXT_CSD_ENH_START_ADDR_1	137
#define EXT_CSD_ENH_START_ADDR_0	136
#define EXT_CSD_SEC_BAD_BLK_MGMNT	134

#define EXT_CSD_SELF_REF_DATE_X		95
#define EXT_CSD_SELF_REF_DATE_0		88

#define EXT_CSD_SELF_REF_PERCENT_1	81 /* R */
#define EXT_CSD_SELF_REF_PERCENT_0	80 /* R */
#define EXT_CSD_SELF_REF_STATUS		74 /* R */
#define EXT_CSD_SELF_REF_DELAY2		73
#define EXT_CSD_SELF_REF_DELAY1		72
#define EXT_CSD_SELF_REF_ENABLE		71

#define EXT_CSD_NATIVE_SECTOR_SIZE	63 /* R */
#define EXT_CSD_USE_NATIVE_SECTOR	62 /* R/W */
#define EXT_CSD_DATA_SECTOR_SIZE	61 /* R */

#define EXT_CSD_EXCEPTION_EVENT_CTRL_1		57
#define EXT_CSD_EXCEPTION_EVENT_CTRL_0		56
#define EXT_CSD_EXCEPTION_EVENT_STATUS_1	55
#define EXT_CSD_EXCEPTION_EVENT_STATUS_0	54

#define EXT_CSD_PACKED_CMD_STATUS	36
#define EXT_CSD_PACKED_FAILURE_IDX	35

#define EXT_CSD_CACHE_CONTROL		33
#define EXT_CSD_CACHE_FLUSH 		32

#define EXT_CSD_PRELOAD_DATA_3		25
#define EXT_CSD_PRELOAD_DATA_2		24
#define EXT_CSD_PRELOAD_DATA_1		23
#define EXT_CSD_PRELOAD_DATA_0		22

#define EXT_CSD_MAX_PRELOAD_DATA_3	21
#define EXT_CSD_MAX_PRELOAD_DATA_2	20
#define EXT_CSD_MAX_PRELOAD_DATA_1	19
#define EXT_CSD_MAX_PRELOAD_DATA_0	18

/*
 * WR_REL_SET field definitions
 */
#define WR_DATA_REL_USER			(1<<0)
#define WR_DATA_REL_1				(1<<1)
#define WR_DATA_REL_2				(1<<2)
#define WR_DATA_REL_3				(1<<3)
#define WR_DATA_REL_4				(1<<4)
#define WR_DATA_REL_ALL				0x1F
/*
 * WR_REL_PARAM field definitions
 */
#define HS_CTRL_REL					(1<<0)
#define EN_REL_WR					(1<<2)

/*
 * BKOPS_EN field definition
 */
#define BKOPS_ENABLE				(1<<0)
#define BKOPS_AUTO_ENABLE			(1<<1)

/*
 * CMD56 args
 */
#define CMD56_ARG_FIRMWARE					0x00000001
#define CMD56_ARG_BAD_BLK_COUTERS 			0x00000011   // response 512
#define CMD56_ARG_ERASE_COUNTERS			0x00000021   // response 512
#define CMD56_ARG_ERASE_COUNTERS_MLC		0x00000023   // response 512
#define CMD56_ARG_ERASE_COUNTERS_SLC		0x00000025   // response 512
//                             tbl[23:16], die[15:8]
#define CMD56_ARG_BAD_BLOCK_INFO			0x00000081   // response 512
#define CMD56_ARG_BLOCK_ERASE_TOTAL			0x00000083   // response 512
//										   tbl[15:8]
#define CMD56_ARG_BLOCK_ERASE_RETRIEVE		0x00000085   // response 512
//										   tbl[15:8]
#define CMD56_ARG_BLOCK_ADDRESS_RETRIEVE	0x00000087   // response 512
#define CMD56_ARG_BUFFER_BLOCK_REFRESH		0x000000A7
#define CMD56_ARG_

#define CMD56_ARG_PPEU						0x110005FB

/*
 * EXT_CSD field definitions
 */
#define EXT_CSD_HPI_SUPP					(1<<0)
#define EXT_CSD_HPI_IMPL					(1<<1)
#define EXT_CSD_CMD_SET_NORMAL				(1<<0)
#define EXT_CSD_BOOT_WP_B_PWR_WP_DIS		(0x40)
#define EXT_CSD_BOOT_WP_B_PERM_WP_DIS		(0x10)
#define EXT_CSD_BOOT_WP_B_PERM_WP_EN		(0x04)
#define EXT_CSD_BOOT_WP_B_PWR_WP_EN			(0x01)
#define EXT_CSD_BOOT_INFO_HS_MODE			(1<<2)
#define EXT_CSD_BOOT_INFO_DDR_DDR			(1<<1)
#define EXT_CSD_BOOT_INFO_ALT				(1<<0)
#define EXT_CSD_BOOT_CFG_ACK				(1<<6)
#define EXT_CSD_BOOT_CFG_EN					(0x38)
#define EXT_CSD_BOOT_CFG_ACC				(0x07)
#define EXT_CSD_RST_N_EN_MASK				(0x03)
#define EXT_CSD_HW_RESET_EN					(0x01)
#define EXT_CSD_HW_RESET_DIS				(0x02)
#define EXT_CSD_PART_CONFIG_ACC_MASK	  	(0x7)
#define EXT_CSD_PART_CONFIG_ACC_BOOT0	  	(0x1)
#define EXT_CSD_PART_CONFIG_ACC_BOOT1	  	(0x2)
#define EXT_CSD_PART_CONFIG_ACC_USER_AREA 	(0x7)
#define EXT_CSD_PART_CONFIG_ACC_ACK	  		(0x40)
#define EXT_CSD_PARTITIONING_EN				(1<<0)
#define EXT_CSD_ENH_ATTRIBUTE_EN			(1<<1)
#define EXT_CSD_ENH_USR						(1<<0)
// EXT_CSD_EXCEPTION_EVENT_STATUS
#define EXT_CSD_EXCEPTION_PACKED_FAILURE	(1<<3)
#define EXT_CSD_EXCEPTION_SYSPOOL_EXHAUSTED	(1<<2)
#define EXT_CSD_EXCEPTION_DYNCAP_NEEDED		(1<<1)
#define EXT_CSD_EXCEPTION_URGENT_BKOPS		(1<<0)
// EXT_CSD_EXCEPTION_EVENT_CTRL
#define EXT_CSD_EXCEPTION_PACKED_EVENT_EN	(1<<3)
#define EXT_CSD_EXCEPTION_SYSPOOL_EVENT_EN	(1<<2)
#define EXT_CSD_EXCEPTION_DYNCAP_EVENT_EN	(1<<1)
// EXT_CSD_PACKED_CMD_STATUS
#define EXT_CSD_PACKED_STATUS_ERROR			(1<<0)
#define EXT_CSD_PACKED_STATUS_IDX_ERROR		(1<<1)
// EXT_CSD_SELF_REF_ENABLE
#define EXT_CSD_SELF_REF_EN					(1<<0)
#define EXT_CSD_SELF_ECC_THRESHOLD			(2<<1)
#define EXT_CSD_SELF_RTC_EN					(1<<3)
// EXT_CSD_SELF_REF_STATUS
#define EXT_CSD_SELF_REF_DONE				(1<<0)


/* From kernel linux/mmc/core.h */
#define MMC_RSP_PRESENT		(1 << 0)
#define MMC_RSP_136			(1 << 1)		/* 136 bit response */
#define MMC_RSP_CRC			(1 << 2)		/* expect valid crc */
#define MMC_RSP_BUSY		(1 << 3)		/* card may send busy */
#define MMC_RSP_OPCODE		(1 << 4)		/* response contains opcode */

#define MMC_CMD_AC			(0 << 5)
#define MMC_CMD_ADTC		(1 << 5)
#define MMC_CMD_BC			(2 << 5)
#define MMC_CMD_BCR			(3 << 5)

#define MMC_RSP_SPI_S1		(1 << 7)		/* one status byte */
#define MMC_RSP_SPI_S2		(1 << 8)		/* new */
#define MMC_RSP_SPI_B4		(1 << 9)		/* new */
#define MMC_RSP_SPI_BUSY 	(1 << 10)		/* card may send busy */

#define MMC_RSP_SPI_R1		(MMC_RSP_SPI_S1)
#define MMC_RSP_SPI_R2		(MMC_RSP_SPI_S1|MMC_RSP_SPI_S2)
#define MMC_RSP_SPI_R1B		(MMC_RSP_SPI_S1|MMC_RSP_SPI_BUSY)

#define MMC_RSP_R1			(MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R1B			(MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE|MMC_RSP_BUSY)

#define MMC_RSP_R2 			(MMC_RSP_PRESENT|MMC_RSP_136|MMC_RSP_CRC)
#define MMC_RSP_R3 			(MMC_RSP_PRESENT)
#define MMC_RSP_R6 			(MMC_RSP_PRESENT|MMC_RSP_CRC)


#define MFG_MICRON_1		0x13
#define MFG_MICRON_2		0x1F

#define BLOCK_SIZE  		512

struct mmcData
{
	char device[64];

	FILE *finfo;
	const char *infofile;
	char infoline[256];

	/* card info */
	uint32_t status;
	uint32_t currState;
	uint32_t r1Status;

	uint32_t csd[4];
	uint8_t csd_version;

	uint32_t cid[4];
	uint8_t cid_manuid;
	uint8_t cid_oemid;
	uint8_t cid_rev;
	char cid_name[7];

	uint8_t ext_csd[BLOCK_SIZE];
	uint8_t ext_csd_rev;
	unsigned int sectors;
	unsigned int blksize;
	unsigned int capacity;
	unsigned int corrSectors;
	unsigned int cachsize;
	uint8_t cachectrl;
	uint8_t bootinfo;
	uint8_t perf_w8_52;
	uint8_t perf_r8_52;
	uint8_t perf_w4_26;
	uint8_t perf_r4_26;
	uint8_t perf_w8_26_w4_52;
	uint8_t perf_r8_26_r4_52;
	uint8_t hcEraGrpSize;

	/* service info */
	uint8_t badBlkMngm;
    uint16_t exception_ctrl;
    uint16_t exception_status;
    uint8_t packed_cmd_status;
    uint8_t packed_failure_idx;
	/* blops */
	uint8_t bkopsSupport;
	uint8_t bkopsStatus;
	uint8_t bkopsEnable;
	uint8_t bkopsStart;
	/* refresh */
	uint8_t selfRefStatus;
	uint8_t selfRefEnable;
	uint16_t selfRefPercent;
	uint8_t selfRefDelay1;
	uint8_t selfRefDelay2;
	/* write rel */
	uint8_t wrrelset;
	uint8_t wrrelparam;
	long lastSector;
	/* health */
	uint8_t health_data[BLOCK_SIZE];
	uint8_t healt_percent_step;
	uint8_t healt_tlc_utilization;
	uint8_t healt_slc_utilization;
	uint8_t lifetime_a;
	uint8_t lifetime_b;
	uint8_t lifetime_eol;

	/* read parameter */
	int offset;
	int length;
};


/* mmc.c */
unsigned int get_sector_count(uint8_t *ext_csd);
unsigned int get_correct_sectors(uint8_t *ext_csd);

int select_card(int fd, int select);
int read_csd(int fd, uint32_t *csd);
int read_cid(int fd, uint32_t *cid);
int read_extcsd(int fd, uint8_t *ext_csd);
int read_status(int fd, uint32_t *response);
int read_health(int fd, int arg, uint8_t *lba_block_data);
int readSector(int fd, char *sector, int offset, int length);
int set_block_length(int fd, int len);
int write_extcsd_value(int fd, uint8_t index, uint8_t value);
int write_CMD56(int fd, int arg);

void print_extcsd(uint8_t *ext_csd);
void print_writeprotect_status(uint8_t *ext_csd);
void print_block(char *msg, uint8_t *lba_block_data, int len);

int set_writeprotect_tmp(int fd, uint8_t *ext_csd);
int start_sanitize(int fd);
int hwreset_en(int fd, uint8_t *ext_csd);
int hwreset_dis(int fd, uint8_t *ext_csd);
int disable_512B_emulation(int fd, uint8_t *ext_csd);
int write_boot_en(int fd, uint8_t *ext_csd, int boot_area);
int write_bkops_en(int fd, uint8_t *ext_csd);
int enh_area_set(int fd, uint8_t *ext_csd, unsigned int start_kib, unsigned int length_kib, int dry_run);

int refresh_ext_cid(int fd, struct mmcData *pdata);
void print_card_info(struct mmcData *pdata);

int checkEmmc(int fd, struct mmcData *pdata);
int initCardInfo(struct mmcData *pdata);

#endif
