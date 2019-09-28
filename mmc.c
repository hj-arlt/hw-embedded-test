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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <libgen.h>
#include <limits.h>
#include <ctype.h>

#include "mmc.h"

//#define  SECTOR_DEBUG

/*
 * ------------------------------------------------------
 */
unsigned int get_sector_count(uint8_t *ext_csd)
{
	return (ext_csd[EXT_CSD_SEC_COUNT_3] << 24) |
		   (ext_csd[EXT_CSD_SEC_COUNT_2] << 16) |
		   (ext_csd[EXT_CSD_SEC_COUNT_1] << 8)  |
		    ext_csd[EXT_CSD_SEC_COUNT_0];
}

unsigned int get_correct_sectors(uint8_t *ext_csd)
{
	return (ext_csd[EXT_CSD_SEC_CORRECT_3] << 24) |
		   (ext_csd[EXT_CSD_SEC_CORRECT_2] << 16) |
	       (ext_csd[EXT_CSD_SEC_CORRECT_1] << 8)  |
			ext_csd[EXT_CSD_SEC_CORRECT_0];
}

int is_blockaddresed(uint8_t *ext_csd)
{
	unsigned int sectors = get_sector_count(ext_csd);

	return (sectors > (2u * 1024 * 1024 * 1024) / 512);
}

unsigned int get_hc_wp_grp_size(uint8_t *ext_csd)
{
	return ext_csd[221];
}

unsigned int get_hc_erase_grp_size(uint8_t *ext_csd)
{
	return ext_csd[224];
}

/*
 * ------------------------------------------------------
 */

/*
 * (CMD7)
 */
int select_card(int fd, int select)
{
	int ret = 0;
	struct mmc_ioc_cmd idata;
	memset(&idata, 0, sizeof(idata));

	idata.opcode = MMC_SELECT_CARD;
	if (select) {
		idata.arg = (1 << 16);
		idata.flags = 0x49D; //MMC_RSP_R1B | MMC_RSP_SPI_R1 | MMC_RSP_SPI_B4 | MMC_CMD_AC;  // 0x49D
	}
	else { // no response, else error
		idata.arg = 0;
		idata.flags = 0; //0x15; //MMC_RSP_R1 | MMC_CMD_AC;  // 0x15
	}

	ret = ioctl(fd, MMC_IOC_CMD, &idata);
	if (ret)
		return -1;

	return ret;
}

/*
 * (CMD9) only from standby state !
 * CSD - 16byte response
 */
int read_csd(int fd, uint32_t *csd)
{
	int ret = 0;
	struct mmc_ioc_cmd idata;
	memset(&idata, 0, sizeof(idata));
	memset(csd, 0, sizeof(uint8_t) * 16);

	//idata.opcode = MMC_SEND_CID;
	idata.opcode = MMC_SEND_CSD;
	idata.arg = (1 << 16);
	idata.flags = MMC_RSP_R2 | MMC_CMD_AC;  // 0x07

	ret = ioctl(fd, MMC_IOC_CMD, &idata);
	if (ret)
		return -1;;

	if (csd) {
		csd[0] = idata.response[0];
		csd[1] = idata.response[1];
		csd[2] = idata.response[2];
		csd[3] = idata.response[3];
	}
	return ret;
}

/*
 * (CMD10) only from standby state !
 * CID - 16byte response
 */
int read_cid(int fd, uint32_t *cid)
{
	int ret = 0;
	struct mmc_ioc_cmd idata;
	memset(&idata, 0, sizeof(idata));
	memset(cid, 0, sizeof(uint8_t) * 16);

	//idata.opcode = MMC_SEND_CID;
	idata.opcode = MMC_SEND_CID;
	idata.arg = (1 << 16);
	idata.flags = MMC_RSP_R2 | MMC_CMD_AC;  // 0x07

	ret = ioctl(fd, MMC_IOC_CMD, &idata);
	if (ret)
		return -1;;

	if (cid) {
		cid[0] = idata.response[0];
		cid[1] = idata.response[1];
		cid[2] = idata.response[2];
		cid[3] = idata.response[3];
	}
	return ret;
}

/*
 * STATUS_SEND (CMD13)
 * response, call back from standby
 */
int read_status(int fd, uint32_t *response)
{
	int ret = 0;
	struct mmc_ioc_cmd idata;

	memset(&idata, 0, sizeof(idata));
	idata.opcode = MMC_SEND_STATUS;
	idata.arg = (1 << 16);                 // 0x10000
	idata.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC; // 0x195

	ret = ioctl(fd, MMC_IOC_CMD, &idata);
	if (ret)
		perror("ioctl");

	if (response)
		*response = idata.response[0];

	return ret;
}

/*
 * ext csd (CMD8)
 */
int read_extcsd(int fd, uint8_t *ext_csd)
{
	int ret = 0;
	struct mmc_ioc_cmd idata;
	memset(&idata, 0, sizeof(idata));
	memset(ext_csd, 0, sizeof(uint8_t) * 512);
	idata.write_flag = 0;
	idata.opcode = MMC_SEND_EXT_CSD;
	idata.arg = 0;
	idata.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;
	idata.blksz = BLOCK_SIZE;
	idata.blocks = 1;
	mmc_ioc_cmd_set_data(idata, ext_csd);

	ret = ioctl(fd, MMC_IOC_CMD, &idata);
	if (ret)
		perror("ioctl");

	return ret;
}

/*
 * Set Block lenght (CMD16)
 * ignored, if EN_REL_WR=1, write reliability protection
 */
int set_block_length(int fd, int len)
{
	int ret = 0;
	struct mmc_ioc_cmd idata;

	memset(&idata, 0, sizeof(idata));
	idata.opcode = MMC_SET_BLOCKLEN;
	idata.arg = len;
	idata.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC; // 0x195

	ret = ioctl(fd, MMC_IOC_CMD, &idata);
	if (ret)
		perror("ioctl");

	return ret;
}

/*
 * (CMD6) MMC_SWITCH
 */
int write_extcsd_value(int fd, uint8_t index, uint8_t value)
{
	int ret = 0;
	struct mmc_ioc_cmd idata;

	memset(&idata, 0, sizeof(idata));
	idata.write_flag = 1;
	idata.opcode = MMC_SWITCH;
	/*           03-index-value-0x10 */
	idata.arg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
			     (index << 16) |
			     (value << 8) |
			     EXT_CSD_CMD_SET_NORMAL;
	idata.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;

	ret = ioctl(fd, MMC_IOC_CMD, &idata);
	if (ret)
		perror("ioctl");

	return ret;
}

/*
 * CMD56 implementation
 */
int read_health(int fd, int arg, uint8_t *lba_block_data)
{
	int ret = 0;
	struct mmc_ioc_cmd idata;

	memset(&idata, 0, sizeof(idata));
	memset(lba_block_data, 0, BLOCK_SIZE);
	idata.write_flag = 0;
	idata.opcode = MMC_GEN_CMD;
	idata.arg = arg;
	idata.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;
	idata.blksz = BLOCK_SIZE;
	idata.blocks = 1;

	mmc_ioc_cmd_set_data(idata, lba_block_data);
	ret = ioctl(fd, MMC_IOC_CMD, &idata);
	if (ret)
		perror("ioctl");
	return ret;
}

/*
 * CMD56 write like BUFFER_BLOCK_REFRESH
 */
int write_CMD56(int fd, int arg)
{
	int ret = 0;
	struct mmc_ioc_cmd idata;

	memset(&idata, 0, sizeof(idata));
	idata.write_flag = 1;
	idata.opcode = MMC_GEN_CMD;
	idata.arg = arg;
	idata.flags = 0; // no response

	ret = ioctl(fd, MMC_IOC_CMD, &idata);
	if (ret) {
		printf("ERR: cmd56 arg %X\n", arg);
		return -1;
	}
	return ret;
}

/*
 * read physical sectors of eMMC
 */
int readSector(int fd, char *sector, int offset, int count)
{
	int i, rc;
	lseek(fd, offset, SEEK_SET);
	for (i = 0; i < count; i++) {
		rc = read(fd, sector, BLOCK_SIZE);
		if (rc < 0) {
			printf("sector read error at offset = %d + %d\n", offset, i);
			return -1;
		}
#ifdef SECTOR_DEBUG
		printf("Sector: %d\n", i+offset);
		for (int j = 0; j < BLOCK_SIZE; j++) {
			printf("%02X", sector[i]);
			if ((j + 1) % 16 == 0)
				printf("\n");
		}
#endif
	}
	return 0;
}

int set_writeprotect_tmp(int fd, uint8_t *ext_csd)
{
	uint8_t value = ext_csd[EXT_CSD_BOOT_WP] | EXT_CSD_BOOT_WP_B_PWR_WP_EN;
	int ret = write_extcsd_value(fd, EXT_CSD_BOOT_WP, value);
	if (ret) {
		fprintf(stderr, "Could not write 0x%02x to "
			"EXT_CSD[%d]\n", value, EXT_CSD_BOOT_WP);
	}

	return ret;
}

/*
 * This will delete the unmapped memory region of the device.
 */
int start_sanitize(int fd)
{
	int ret = write_extcsd_value(fd, EXT_CSD_SANITIZE_START, 1);
	if (ret) {
		fprintf(stderr, "Could not write 0x%02x to EXT_CSD[%d]\n",
			1, EXT_CSD_SANITIZE_START);
	}

	return ret;
}

int hwreset(int fd, uint8_t *ext_csd, int value)
{
	int ret;

	if ((ext_csd[EXT_CSD_RST_N_FUNCTION] & EXT_CSD_RST_N_EN_MASK) ==
	    EXT_CSD_HW_RESET_EN) {
		fprintf(stderr,
			"H/W Reset is already permanently enabled\n");
		return -1;
	}
	if ((ext_csd[EXT_CSD_RST_N_FUNCTION] & EXT_CSD_RST_N_EN_MASK) ==
	    EXT_CSD_HW_RESET_DIS) {
		fprintf(stderr,
			"H/W Reset is already permanently disabled\n");
		return -1;
	}

	ret = write_extcsd_value(fd, EXT_CSD_RST_N_FUNCTION, value);
	if (ret) {
		fprintf(stderr,
			"Could not write 0x%02x to EXT_CSD[%d]\n",
			value, EXT_CSD_RST_N_FUNCTION);
		exit(1);
	}

	return ret;
}

/*
 * Permanently enable the eMMC H/W Reset feature on <device>.
 * NOTE!  This is a one-time programmable (unreversible) change.
 */
int hwreset_en(int fd, uint8_t *ext_csd)
{
	return hwreset(fd, ext_csd, EXT_CSD_HW_RESET_EN);
}

/*
 * Permanently disable the eMMC H/W Reset feature on <device>.
 * NOTE!  This is a one-time programmable (unreversible) change.
 */
int hwreset_dis(int fd, uint8_t *ext_csd)
{
	return hwreset(fd, ext_csd, EXT_CSD_HW_RESET_DIS);
}

/*
 * disable 512B emulation"
   Set the eMMC data sector size to 4KB by disabling emulation.
 */
int disable_512B_emulation(int fd, uint8_t *ext_csd)
{
	uint8_t native_sector_size, data_sector_size, wr_rel_param;
	int ret;

	wr_rel_param = ext_csd[EXT_CSD_WR_REL_PARAM];
	native_sector_size = ext_csd[EXT_CSD_NATIVE_SECTOR_SIZE];
	data_sector_size = ext_csd[EXT_CSD_DATA_SECTOR_SIZE];

	if (native_sector_size && !data_sector_size &&
	   (wr_rel_param & EN_REL_WR)) {
		ret = write_extcsd_value(fd, EXT_CSD_USE_NATIVE_SECTOR, 1);

		if (ret) {
			fprintf(stderr, "Could not write 0x%02x to EXT_CSD[%d]\n",
					1, EXT_CSD_BOOT_WP);
			exit(1);
		}
		printf("MMC disable 512B emulation successful.  Now reset the device to switch to 4KB native sector mode.\n");
	} else if (native_sector_size && data_sector_size) {
		printf("MMC 512B emulation mode is already disabled; doing nothing.\n");
	} else {
		printf("MMC does not support disabling 512B emulation mode.\n");
	}

	return ret;
}

/*
 * bootpart enable", "<boot_partition> " "<send_ack> "
   "Enable the boot partition for the <device>.
   To receive acknowledgment of boot from the card set <send_ack> to 1, else set it to 0.",
 */
int write_boot_en(int fd, uint8_t *ext_csd, int boot_area)
{
	uint8_t value = 0;
	int ret, send_ack;

	/*
	 * If <send_ack> is 1, the device will send acknowledgment
	 * pattern "010" to the host when boot operation begins.
	 * If <send_ack> is 0, it won't.
	 */
	send_ack = 1;

	value = ext_csd[EXT_CSD_PART_CONFIG];

	switch (boot_area) {
	case EXT_CSD_PART_CONFIG_ACC_BOOT0: // 1
		value |= (1 << 3);
		value &= ~(3 << 4);
		break;
	case EXT_CSD_PART_CONFIG_ACC_BOOT1: // 2
		value |= (1 << 4);
		value &= ~(1 << 3);
		value &= ~(1 << 5);
		break;
	case EXT_CSD_PART_CONFIG_ACC_USER_AREA: // 7
		value |= (boot_area << 3);
		break;
	default:
		fprintf(stderr, "Cannot enable the boot area %d\n", boot_area);
		return -1;
	}
	if (send_ack)
		value |= EXT_CSD_PART_CONFIG_ACC_ACK;
	else
		value &= ~EXT_CSD_PART_CONFIG_ACC_ACK;

	ret = write_extcsd_value(fd, EXT_CSD_PART_CONFIG, value);
	if (ret) {
		fprintf(stderr, "Could not write 0x%02x to EXT_CSD[%d]\n",
			value, EXT_CSD_PART_CONFIG);
	}
	return ret;
}

/*
 * Enable the eMMC BKOPS feature on <device>.
 * NOTE!  This is a one-time programmable (unreversible) change.
 */
int write_bkops_en(int fd, uint8_t *ext_csd)
{
	uint8_t value = 0;
	int ret;

	if (!(ext_csd[EXT_CSD_BKOPS_SUPPORT] & 0x1)) {
		fprintf(stderr, "doesn't support BKOPS\n");
		return -1;
	}

	ret = write_extcsd_value(fd, EXT_CSD_BKOPS_EN, BKOPS_ENABLE);
	if (ret) {
		fprintf(stderr, "Could not write 0x%02x to EXT_CSD[%d]\n",
			value, EXT_CSD_BKOPS_EN);
	}

	return ret;
}

/*
 * enh_area set", "<-y|-n> " "<start KiB> " "<length KiB> " "<device>
   Enable the enhanced user area for the <device>.
   Dry-run 1=y 0=n.
   NOTE!  This is a one-time programmable (unreversible) change.
 */
int enh_area_set(int fd, uint8_t *ext_csd, unsigned int start_kib, unsigned int length_kib, int dry_run)
{
	uint8_t value;
	int ret;
	unsigned int enh_start_addr, enh_size_mult;
	unsigned long align;

	/* assert ENH_ATTRIBUTE_EN */
	if (!(ext_csd[EXT_CSD_PARTITIONING_SUPPORT] & EXT_CSD_ENH_ATTRIBUTE_EN))
	{
		printf(" Device cannot have enhanced tech.\n");
		return -1;
	}

	/* assert not PARTITION_SETTING_COMPLETED */
	if (ext_csd[EXT_CSD_PARTITION_SETTING_COMPLETED])
	{
		printf(" Device is already partitioned\n");
		return -1;
	}

	align = 512l * get_hc_wp_grp_size(ext_csd) * get_hc_erase_grp_size(ext_csd);

	enh_size_mult = (length_kib + align/2l) / align;

	enh_start_addr = start_kib * 1024 / (is_blockaddresed(ext_csd) ? 512 : 1);
	enh_start_addr /= align;
	enh_start_addr *= align;

	/* set EXT_CSD_ERASE_GROUP_DEF bit 0 */
	ret = write_extcsd_value(fd, EXT_CSD_ERASE_GROUP_DEF, 0x1);
	if (ret) {
		fprintf(stderr, "Could not write 0x1 to EXT_CSD[%d]\n",
			EXT_CSD_ERASE_GROUP_DEF);
		return -1;
	}

	/* write to ENH_START_ADDR and ENH_SIZE_MULT and PARTITIONS_ATTRIBUTE's ENH_USR bit */
	value = (enh_start_addr >> 24) & 0xff;
	ret = write_extcsd_value(fd, EXT_CSD_ENH_START_ADDR_3, value);
	if (ret) {
		fprintf(stderr, "Could not write 0x%02x to EXT_CSD[%d]\n", value,
			EXT_CSD_ENH_START_ADDR_3);
		return -1;
	}
	value = (enh_start_addr >> 16) & 0xff;
	ret = write_extcsd_value(fd, EXT_CSD_ENH_START_ADDR_2, value);
	if (ret) {
		fprintf(stderr, "Could not write 0x%02x to EXT_CSD[%d]\n", value,
			EXT_CSD_ENH_START_ADDR_2);
		return -1;
	}
	value = (enh_start_addr >> 8) & 0xff;
	ret = write_extcsd_value(fd, EXT_CSD_ENH_START_ADDR_1, value);
	if (ret) {
		fprintf(stderr, "Could not write 0x%02x to EXT_CSD[%d]\n", value,
			EXT_CSD_ENH_START_ADDR_1);
		return -1;
	}
	value = enh_start_addr & 0xff;
	ret = write_extcsd_value(fd, EXT_CSD_ENH_START_ADDR_0, value);
	if (ret) {
		fprintf(stderr, "Could not write 0x%02x to EXT_CSD[%d]\n", value,
			EXT_CSD_ENH_START_ADDR_0);
		return -1;
	}

	value = (enh_size_mult >> 16) & 0xff;
	ret = write_extcsd_value(fd, EXT_CSD_ENH_SIZE_MULT_2, value);
	if (ret) {
		fprintf(stderr, "Could not write 0x%02x to EXT_CSD[%d]\n", value,
			EXT_CSD_ENH_SIZE_MULT_2);
		return -1;
	}
	value = (enh_size_mult >> 8) & 0xff;
	ret = write_extcsd_value(fd, EXT_CSD_ENH_SIZE_MULT_1, value);
	if (ret) {
		fprintf(stderr, "Could not write 0x%02x to EXT_CSD[%d]\n", value,
			EXT_CSD_ENH_SIZE_MULT_1);
		return -1;
	}
	value = enh_size_mult & 0xff;
	ret = write_extcsd_value(fd, EXT_CSD_ENH_SIZE_MULT_0, value);
	if (ret) {
		fprintf(stderr, "Could not write 0x%02x to EXT_CSD[%d]\n", value,
			EXT_CSD_ENH_SIZE_MULT_0);
		return -1;
	}

	ret = write_extcsd_value(fd, EXT_CSD_PARTITIONS_ATTRIBUTE, EXT_CSD_ENH_USR);
	if (ret) {
		fprintf(stderr, "Could not write EXT_CSD_ENH_USR to EXT_CSD[%d]\n",
			EXT_CSD_PARTITIONS_ATTRIBUTE);
		return -1;
	}

	if (dry_run)
	{
		fprintf(stderr, "NOT setting PARTITION_SETTING_COMPLETED\n");
		return -1;
	}

	fprintf(stderr, "setting OTP PARTITION_SETTING_COMPLETED!\n");
	ret = write_extcsd_value(fd, EXT_CSD_PARTITION_SETTING_COMPLETED, 0x1);
	if (ret) {
		fprintf(stderr, "Could not write 0x1 to EXT_CSD[%d]\n",
			EXT_CSD_PARTITION_SETTING_COMPLETED);
		return -1;
	}

	uint32_t response;
	ret = read_status(fd, &response);
	if (ret) {
		fprintf(stderr, "Could not get response to SEND_STATUS\n");
		return -1;
	}

	if (response & R1_SWITCH_ERROR)
	{
		fprintf(stderr, "Setting ENH_USR area failed\n");
		return -1;
	}

	fprintf(stderr, "Setting ENH_USR area SUCCESS\n");
	fprintf(stderr, "Device power cycle needed for settings to take effect.\n"
		"Confirm that PARTITION_SETTING_COMPLETED bit is set using 'extcsd read'"
		"after power cycle\n");

	return 0;
}

/*
 * ######################################################
 */
void print_writeprotect_status(uint8_t *ext_csd)
{
	uint8_t reg;
	uint8_t ext_csd_rev = ext_csd[EXT_CSD_REV];

	/* A43: reserved [174:0] */
	if (ext_csd_rev >= 5) {
		printf("Boot write protection status registers"
			" [BOOT_WP_STATUS]: 0x%02x\n", ext_csd[174]);

		reg = ext_csd[EXT_CSD_BOOT_WP];
		printf("Boot Area Write protection [BOOT_WP]: 0x%02x\n", reg);
		printf(" Power ro locking: ");
		if (reg & EXT_CSD_BOOT_WP_B_PWR_WP_DIS)
			printf("not possible\n");
		else
			printf("possible\n");

		printf(" Permanent ro locking: ");
		if (reg & EXT_CSD_BOOT_WP_B_PERM_WP_DIS)
			printf("not possible\n");
		else
			printf("possible\n");

		printf(" ro lock status: ");
		if (reg & EXT_CSD_BOOT_WP_B_PWR_WP_EN)
			printf("locked until next power on\n");
		else if (reg & EXT_CSD_BOOT_WP_B_PERM_WP_EN)
			printf("locked permanently\n");
		else
			printf("not locked\n");
	}
}

void print_extcsd(uint8_t *ext_csd)
{
	uint8_t ext_csd_rev, reg;
	const char *str;

	ext_csd_rev = ext_csd[EXT_CSD_REV];

	printf("csd rev: %d\n", ext_csd_rev);

	switch (ext_csd_rev) {
	case 8:
		str = "5.1";
		break;
	case 7:
		str = "5.0";
		break;
	case 6:
		str = "4.5";
		break;
	case 5:
		str = "4.41";
		break;
	case 3:
		str = "4.3";
		break;
	case 2:
		str = "4.2";
		break;
	case 1:
		str = "4.1";
		break;
	case 0:
		str = "4.0";
		break;
	default:
		return;
	}
	printf("=============================================\n");
	printf("  Extended CSD rev 1.%d (MMC %s)\n", ext_csd_rev, str);
	printf("=============================================\n\n");

	if (ext_csd_rev < 3)
		return; /* No ext_csd */

	/* Parse the Extended CSD registers.
	 * Reserved bit should be read as "0" in case of spec older
	 * than A441.
	 */
	reg = ext_csd[EXT_CSD_S_CMD_SET];
	printf("Card Supported Command sets [S_CMD_SET: 0x%02x]\n", reg);
	if (!reg)
		printf(" - Standard MMC command sets\n");

	reg = ext_csd[EXT_CSD_HPI_FEATURE];
	printf("HPI Features [HPI_FEATURE: 0x%02x]: ", reg);
	if (reg & EXT_CSD_HPI_SUPP) {
		if (reg & EXT_CSD_HPI_IMPL)
			printf("implementation based on CMD12\n");
		else
			printf("implementation based on CMD13\n");
	}

	printf("Background operations support [BKOPS_SUPPORT: 0x%02x]\n",
		ext_csd[502]);

	if (ext_csd_rev >= 6) {
		printf("Max Packet Read Cmd [MAX_PACKED_READS: 0x%02x]\n",
			ext_csd[501]);
		printf("Max Packet Write Cmd [MAX_PACKED_WRITES: 0x%02x]\n",
			ext_csd[500]);
		printf("Data TAG support [DATA_TAG_SUPPORT: 0x%02x]\n",
			ext_csd[499]);

		printf("Data TAG Unit Size [TAG_UNIT_SIZE: 0x%02x]\n",
			ext_csd[498]);
		printf("Tag Resources Size [TAG_RES_SIZE: 0x%02x]\n",
			ext_csd[497]);
		printf("Context Management Capabilities"
			" [CONTEXT_CAPABILITIES: 0x%02x]\n", ext_csd[496]);
		printf("Large Unit Size [LARGE_UNIT_SIZE_M1: 0x%02x]\n",
			ext_csd[495]);
		printf("Extended partition attribute support"
			" [EXT_SUPPORT: 0x%02x]\n", ext_csd[494]);
		printf("Generic CMD6 Timer [GENERIC_CMD6_TIME: 0x%02x]\n",
			ext_csd[248]);
		printf("Power off notification [POWER_OFF_LONG_TIME: 0x%02x]\n",
			ext_csd[247]);
		printf("Cache Size [CACHE_SIZE] is %d KiB\n",
			ext_csd[249] << 0 | (ext_csd[250] << 8) |
			(ext_csd[251] << 16) | (ext_csd[252] << 24));
	}

	/* A441: Reserved [501:247]
	    A43: reserved [246:229] */
	if (ext_csd_rev >= 5) {
		printf("Background operations status"
			" [BKOPS_STATUS: 0x%02x]\n", ext_csd[246]);

		/* CORRECTLY_PRG_SECTORS_NUM [245:242] TODO */
		//unsigned int corrSectors = get_correct_sectors(ext_csd);

		printf("1st Initialisation Time after programmed sector"
			" [INI_TIMEOUT_AP: 0x%02x]\n", ext_csd[241]);

		/* A441: reserved [240] */
		printf("Power class for 52MHz, DDR at 3.6V"
			" [PWR_CL_DDR_52_360: 0x%02x]\n", ext_csd[239]);
		printf("Power class for 52MHz, DDR at 1.95V"
			" [PWR_CL_DDR_52_195: 0x%02x]\n", ext_csd[238]);

		/* A441: reserved [237-236] */

		if (ext_csd_rev >= 6) {
			printf("Power class for 200MHz at 3.6V"
				" [PWR_CL_200_360: 0x%02x]\n", ext_csd[237]);
			printf("Power class for 200MHz, at 1.95V"
				" [PWR_CL_200_195: 0x%02x]\n", ext_csd[236]);
		}
		printf("Minimum Performance for 8bit at 52MHz in DDR mode:\n");
		printf(" [MIN_PERF_DDR_W_8_52: 0x%02x]\n", ext_csd[235]);
		printf(" [MIN_PERF_DDR_R_8_52: 0x%02x]\n", ext_csd[234]);
		/* A441: reserved [233] */
		printf("TRIM Multiplier [TRIM_MULT: 0x%02x]\n", ext_csd[232]);
		printf("Secure Feature support [SEC_FEATURE_SUPPORT: 0x%02x]\n",
			ext_csd[231]);
	}
	if (ext_csd_rev == 5) { /* Obsolete in 4.5 */
		printf("Secure Erase Multiplier [SEC_ERASE_MULT: 0x%02x]\n",
			ext_csd[230]);
		printf("Secure TRIM Multiplier [SEC_TRIM_MULT: 0x%02x]\n",
			ext_csd[229]);
	}
	reg = ext_csd[EXT_CSD_BOOT_INFO];
	printf("Boot Information [BOOT_INFO: 0x%02x]\n", reg);
	if (reg & EXT_CSD_BOOT_INFO_ALT)
		printf(" Device supports alternative boot method\n");
	if (reg & EXT_CSD_BOOT_INFO_DDR_DDR)
		printf(" Device supports dual data rate during boot\n");
	if (reg & EXT_CSD_BOOT_INFO_HS_MODE)
		printf(" Device supports high speed timing during boot\n");

	/* A441/A43: reserved [227] */
	printf("Boot partition size [BOOT_SIZE_MULTI: 0x%02x]\n", ext_csd[226]);
	printf("Access size [ACC_SIZE: 0x%02x]\n", ext_csd[225]);

	reg = get_hc_erase_grp_size(ext_csd);
	printf("High-capacity erase unit size [HC_ERASE_GRP_SIZE: 0x%02x]\n",
		reg);
	printf(" i.e. %u KiB\n", 512 * reg);

	printf("High-capacity erase timeout [ERASE_TIMEOUT_MULT: 0x%02x]\n",
		ext_csd[223]);
	printf("Reliable write sector count [REL_WR_SEC_C: 0x%02x]\n",
		ext_csd[222]);

	reg = get_hc_wp_grp_size(ext_csd);
	printf("High-capacity W protect group size [HC_WP_GRP_SIZE: 0x%02x]\n",
		reg);
	printf(" i.e. %lu KiB\n", 512l * get_hc_erase_grp_size(ext_csd) * reg);

	printf("Sleep current (VCC) [S_C_VCC: 0x%02x]\n", ext_csd[220]);
	printf("Sleep current (VCCQ) [S_C_VCCQ: 0x%02x]\n", ext_csd[219]);
	/* A441/A43: reserved [218] */
	printf("Sleep/awake timeout [S_A_TIMEOUT: 0x%02x]\n", ext_csd[217]);
	/* A441/A43: reserved [216] */

	unsigned int sectors =	get_sector_count(ext_csd);
	printf("Sector Count [SEC_COUNT: 0x%08x] %d\n", sectors, sectors);
	if (is_blockaddresed(ext_csd))
		printf(" Device is block-addressed\n");
	else
		printf(" Device is NOT block-addressed\n");

	/* A441/A43: reserved [211] */
	printf("Minimum Write Performance for 8bit:\n");
	printf(" [MIN_PERF_W_8_52: 0x%02x]\n", ext_csd[210]);
	printf(" [MIN_PERF_R_8_52: 0x%02x]\n", ext_csd[209]);
	printf(" [MIN_PERF_W_8_26_4_52: 0x%02x]\n", ext_csd[208]);
	printf(" [MIN_PERF_R_8_26_4_52: 0x%02x]\n", ext_csd[207]);
	printf("Minimum Write Performance for 4bit:\n");
	printf(" [MIN_PERF_W_4_26: 0x%02x]\n", ext_csd[206]);
	printf(" [MIN_PERF_R_4_26: 0x%02x]\n", ext_csd[205]);
	/* A441/A43: reserved [204] */
	printf("Power classes registers:\n");
	printf(" [PWR_CL_26_360: 0x%02x]\n", ext_csd[203]);
	printf(" [PWR_CL_52_360: 0x%02x]\n", ext_csd[202]);
	printf(" [PWR_CL_26_195: 0x%02x]\n", ext_csd[201]);
	printf(" [PWR_CL_52_195: 0x%02x]\n", ext_csd[200]);

	/* A43: reserved [199:198] */
	if (ext_csd_rev >= 5) {
		printf("Partition switching timing "
			"[PARTITION_SWITCH_TIME: 0x%02x]\n", ext_csd[199]);
		printf("Out-of-interrupt busy timing"
			" [OUT_OF_INTERRUPT_TIME: 0x%02x]\n", ext_csd[198]);
	}

	/* A441/A43: reserved	[197] [195] [193] [190] [188]
	 * [186] [184] [182] [180] [176] */

	if (ext_csd_rev >= 6)
		printf("I/O Driver Strength [DRIVER_STRENGTH: 0x%02x]\n",
			ext_csd[197]);

	/* DEVICE_TYPE in A45, CARD_TYPE in A441 */
	reg = ext_csd[196];
	printf("Card Type [CARD_TYPE: 0x%02x]\n", reg);
	if (reg & 0x20) printf(" HS200 Single Data Rate eMMC @200MHz 1.2VI/O\n");
	if (reg & 0x10) printf(" HS200 Single Data Rate eMMC @200MHz 1.8VI/O\n");
	if (reg & 0x08) printf(" HS Dual Data Rate eMMC @52MHz 1.2VI/O\n");
	if (reg & 0x04)	printf(" HS Dual Data Rate eMMC @52MHz 1.8V or 3VI/O\n");
	if (reg & 0x02)	printf(" HS eMMC @52MHz - at rated device voltage(s)\n");
	if (reg & 0x01) printf(" HS eMMC @26MHz - at rated device voltage(s)\n");

	printf("CSD structure version [CSD_STRUCTURE: 0x%02x]\n", ext_csd[194]);
	/* ext_csd_rev = ext_csd[EXT_CSD_REV] (already done!!!) */
	printf("Command set [CMD_SET: 0x%02x]\n", ext_csd[191]);
	printf("Command set revision [CMD_SET_REV: 0x%02x]\n", ext_csd[189]);
	printf("Power class [POWER_CLASS: 0x%02x]\n", ext_csd[187]);
	printf("High-speed interface timing [HS_TIMING: 0x%02x]\n",
		ext_csd[185]);
	/* bus_width: ext_csd[183] not readable */
	printf("Erased memory content [ERASED_MEM_CONT: 0x%02x]\n",
		ext_csd[181]);
	reg = ext_csd[EXT_CSD_BOOT_CFG];
	printf("Boot configuration bytes [PARTITION_CONFIG: 0x%02x]\n", reg);
	switch ((reg & EXT_CSD_BOOT_CFG_EN)>>3) {
	case 0x0:
		printf(" Not boot enable\n");
		break;
	case 0x1:
		printf(" Boot Partition 1 enabled\n");
		break;
	case 0x2:
		printf(" Boot Partition 2 enabled\n");
		break;
	case 0x7:
		printf(" User Area Enabled for boot\n");
		break;
	}
	switch (reg & EXT_CSD_BOOT_CFG_ACC) {
	case 0x0:
		printf(" No access to boot partition\n");
		break;
	case 0x1:
		printf(" R/W Boot Partition 1\n");
		break;
	case 0x2:
		printf(" R/W Boot Partition 2\n");
		break;
	case 0x3:
		printf(" R/W Replay Protected Memory Block (RPMB)\n");
		break;
	default:
		printf(" Access to General Purpose partition %d\n",
			(reg & EXT_CSD_BOOT_CFG_ACC) - 3);
		break;
	}

	printf("Boot config protection [BOOT_CONFIG_PROT: 0x%02x]\n",
		ext_csd[178]);
	printf("Boot bus Conditions [BOOT_BUS_CONDITIONS: 0x%02x]\n",
		ext_csd[177]);
	printf("High-density erase group definition"
		" [ERASE_GROUP_DEF: 0x%02x]\n", ext_csd[EXT_CSD_ERASE_GROUP_DEF]);

	print_writeprotect_status(ext_csd);

	if (ext_csd_rev >= 5) {
		/* A441]: reserved [172] */
		printf("User area write protection register"
			" [USER_WP]: 0x%02x\n", ext_csd[171]);
		/* A441]: reserved [170] */
		printf("FW configuration [FW_CONFIG]: 0x%02x\n", ext_csd[169]);
		printf("RPMB Size [RPMB_SIZE_MULT]: 0x%02x\n", ext_csd[168]);
		printf("Write reliability setting register"
			" [WR_REL_SET]: 0x%02x\n", ext_csd[167]);
		printf("Write reliability parameter register"
			" [WR_REL_PARAM]: 0x%02x\n", ext_csd[166]);
		/* sanitize_start ext_csd[165]]: not readable
		 * bkops_start ext_csd[164]]: only writable */
		printf("Enable background operations handshake"
			" [BKOPS_EN]: 0x%02x\n", ext_csd[163]);
		printf("H/W reset function"
			" [RST_N_FUNCTION]: 0x%02x\n", ext_csd[162]);
		printf("HPI management [HPI_MGMT]: 0x%02x\n", ext_csd[161]);
		reg = ext_csd[EXT_CSD_PARTITIONING_SUPPORT];
		printf("Partitioning Support [PARTITIONING_SUPPORT]: 0x%02x\n",
			reg);
		if (reg & EXT_CSD_PARTITIONING_EN)
			printf(" Device support partitioning feature\n");
		else
			printf(" Device NOT support partitioning feature\n");
		if (reg & EXT_CSD_ENH_ATTRIBUTE_EN)
			printf(" Device can have enhanced tech.\n");
		else
			printf(" Device cannot have enhanced tech.\n");

		reg = (ext_csd[159] << 16) | (ext_csd[158] << 8) |
			ext_csd[157];
		printf("Max Enhanced Area Size [MAX_ENH_SIZE_MULT]: 0x%06x\n",
			   reg);
		unsigned int wp_sz = get_hc_wp_grp_size(ext_csd);
		unsigned int erase_sz = get_hc_erase_grp_size(ext_csd);
		printf(" i.e. %lu KiB\n", 512l * reg * wp_sz * erase_sz);

		printf("Partitions attribute [PARTITIONS_ATTRIBUTE]: 0x%02x\n",
			ext_csd[EXT_CSD_PARTITIONS_ATTRIBUTE]);
		reg = ext_csd[EXT_CSD_PARTITION_SETTING_COMPLETED];
		printf("Partitioning Setting"
			" [PARTITION_SETTING_COMPLETED]: 0x%02x\n",
			reg);
		if (reg)
			printf(" Device partition setting complete\n");
		else
			printf(" Device partition setting NOT complete\n");

		printf("General Purpose Partition Size\n"
			" [GP_SIZE_MULT_4]: 0x%06x\n", (ext_csd[154] << 16) |
			(ext_csd[153] << 8) | ext_csd[152]);
		printf(" [GP_SIZE_MULT_3]: 0x%06x\n", (ext_csd[151] << 16) |
			   (ext_csd[150] << 8) | ext_csd[149]);
		printf(" [GP_SIZE_MULT_2]: 0x%06x\n", (ext_csd[148] << 16) |
			   (ext_csd[147] << 8) | ext_csd[146]);
		printf(" [GP_SIZE_MULT_1]: 0x%06x\n", (ext_csd[145] << 16) |
			   (ext_csd[144] << 8) | ext_csd[143]);

		reg =	(ext_csd[EXT_CSD_ENH_SIZE_MULT_2] << 16) |
			(ext_csd[EXT_CSD_ENH_SIZE_MULT_1] << 8) |
			ext_csd[EXT_CSD_ENH_SIZE_MULT_0];
		printf("Enhanced User Data Area Size"
			" [ENH_SIZE_MULT]: 0x%06x\n", reg);
		printf(" i.e. %lu KiB\n", 512l * reg *
		       get_hc_erase_grp_size(ext_csd) *
		       get_hc_wp_grp_size(ext_csd));

		reg =	(ext_csd[EXT_CSD_ENH_START_ADDR_3] << 24) |
			(ext_csd[EXT_CSD_ENH_START_ADDR_2] << 16) |
			(ext_csd[EXT_CSD_ENH_START_ADDR_1] << 8) |
			ext_csd[EXT_CSD_ENH_START_ADDR_0];
		printf("Enhanced User Data Start Address"
			" [ENH_START_ADDR]: 0x%06x\n", reg);
		printf(" i.e. %lu bytes offset\n", (is_blockaddresed(ext_csd) ?
				1l : 512l) * reg);

		/* A441]: reserved [135] */
		printf("Bad Block Management mode"
			" [SEC_BAD_BLK_MGMNT]: 0x%02x\n", ext_csd[134]);
		/* A441: reserved [133:0] */
	}
	/* B45 */
	if (ext_csd_rev >= 6) {
		int j;
		/* tcase_support ext_csd[132] not readable */
		printf("Periodic Wake-up [PERIODIC_WAKEUP]: 0x%02x\n",
			ext_csd[131]);
		printf("Program CID/CSD in DDR mode support"
			" [PROGRAM_CID_CSD_DDR_SUPPORT]: 0x%02x\n",
			   ext_csd[130]);

		for (j = 127; j >= 64; j--)
			printf("Vendor Specific Fields"
				" [VENDOR_SPECIFIC_FIELD[%d]]: 0x%02x\n",
				j, ext_csd[j]);

		printf("Native sector size [NATIVE_SECTOR_SIZE]: 0x%02x\n",
			ext_csd[63]);
		printf("Sector size emulation [USE_NATIVE_SECTOR]: 0x%02x\n",
			ext_csd[62]);
		printf("Sector size [DATA_SECTOR_SIZE]: 0x%02x\n", ext_csd[61]);
		printf("1st initialization after disabling sector"
			" size emulation [INI_TIMEOUT_EMU]: 0x%02x\n",
			ext_csd[60]);
		printf("Class 6 commands control [CLASS_6_CTRL]: 0x%02x\n",
			ext_csd[59]);
		printf("Number of addressed group to be Released"
			"[DYNCAP_NEEDED]: 0x%02x\n", ext_csd[58]);
		printf("Exception events control"
			" [EXCEPTION_EVENTS_CTRL]: 0x%04x\n",
			(ext_csd[57] << 8) | ext_csd[56]);
		printf("Exception events status"
			"[EXCEPTION_EVENTS_STATUS]: 0x%04x\n",
			(ext_csd[55] << 8) | ext_csd[54]);
		printf("Extended Partitions Attribute"
			" [EXT_PARTITIONS_ATTRIBUTE]: 0x%04x\n",
			(ext_csd[53] << 8) | ext_csd[52]);

		for (j = 51; j >= 37; j--)
			printf("Context configuration"
				" [CONTEXT_CONF[%d]]: 0x%02x\n", j, ext_csd[j]);

		printf("Packed command status"
			" [PACKED_COMMAND_STATUS]: 0x%02x\n", ext_csd[36]);
		printf("Packed command failure index"
			" [PACKED_FAILURE_INDEX]: 0x%02x\n", ext_csd[35]);
		printf("Power Off Notification"
			" [POWER_OFF_NOTIFICATION]: 0x%02x\n", ext_csd[34]);
		printf("Control to turn the Cache ON/OFF"
			" [CACHE_CTRL]: 0x%02x\n", ext_csd[33]);
		/* flush_cache ext_csd[32] not readable */
		/*Reserved [31:0] */
	}
}

void print_block(char *msg, uint8_t *lba_block_data, int len)
{
	int count=0;
	printf("%s", msg);
	while( count < len) {
		if(count % 16 == 0)
			printf("\n%03d: ", count);
		printf("%02x ", lba_block_data[count]);
		count++;
	}
	printf("\n");
}

int refresh_ext_cid(int fd, struct mmcData *pdata)
{
	int err = read_extcsd(fd, pdata->ext_csd);
	if (err) {
		printf("eMMC: ERR read ext_csd!\n");
		return -1;
	}
	pdata->ext_csd_rev = pdata->ext_csd[EXT_CSD_REV];
	//DPRINT(("eMMC: EXT_CSD rev. %s\n",	(pdata->ext_csd_rev==8)?"5.1":
	//									(pdata->ext_csd_rev==7)?"5.0":
	//									(pdata->ext_csd_rev==6)?"4.5":
	//									(pdata->ext_csd_rev==5)?"4.41":"older 4.3"));

	//print_block("EXT_CSD data block dumping:", pdata->ext_csd, BLOCK_SIZE);

	/*
	 * card info
	 */
	pdata->blksize          = BLOCK_SIZE;
	pdata->sectors          = get_sector_count(pdata->ext_csd);
	pdata->capacity         = pdata->sectors * (uint32_t)pdata->blksize;
	pdata->corrSectors      = get_correct_sectors(pdata->ext_csd);
	pdata->cachectrl        = pdata->ext_csd[EXT_CSD_CACHE_CONTROL];
	pdata->cachsize         = pdata->ext_csd[EXT_CSD_CACHE_SIZE_0] << 0   | (pdata->ext_csd[EXT_CSD_CACHE_SIZE_1] << 8)
			                  | (pdata->ext_csd[EXT_CSD_CACHE_SIZE_2] << 16) | (pdata->ext_csd[EXT_CSD_CACHE_SIZE_3] << 24);
	printf("eMMC: CAP sectors %du, cap %du Byte, corrSectors %d cache %s size %d kB\n"
			, pdata->sectors, pdata->capacity, pdata->corrSectors
			, (pdata->cachectrl==0)?"off":"on", pdata->cachsize);

	pdata->bootinfo         = pdata->ext_csd[EXT_CSD_BOOT_INFO];
	pdata->perf_w8_52       = pdata->ext_csd[EXT_CSD_MIN_PERF_W_8_52];
	pdata->perf_r8_52       = pdata->ext_csd[EXT_CSD_MIN_PERF_R_8_52];
	pdata->perf_w4_26       = pdata->ext_csd[EXT_CSD_MIN_PERF_W_4_26];
	pdata->perf_r4_26       = pdata->ext_csd[EXT_CSD_MIN_PERF_R_4_26];
	pdata->perf_w8_26_w4_52 = pdata->ext_csd[EXT_CSD_MIN_PERF_W_8_26_4_52];
	pdata->perf_r8_26_r4_52 = pdata->ext_csd[EXT_CSD_MIN_PERF_R_8_26_4_52];

	//DPRINT(("eMMC: BOOT %X, PERF 8_52 (%d %d) 4_26 (%d %d) 8-4_26-52 (%d %d)\n", pdata->bootinfo
	//		, pdata->perf_w8_52, pdata->perf_r8_52, pdata->perf_w4_26
	//		, pdata->perf_r4_26, pdata->perf_w8_26_w4_52, pdata->perf_r8_26_r4_52));

	/*
	 * life time
	 */
	pdata->lifetime_a   = pdata->ext_csd[EXT_CSD_DEV_LIFETIME_A];
	pdata->lifetime_b   = pdata->ext_csd[EXT_CSD_DEV_LIFETIME_B];
	pdata->lifetime_eol = pdata->ext_csd[EXT_CSD_PRE_EOL_INFO];

	//DPRINT(("eMMC: LIFETIME slc %d mlc %d eol %s\n"
	//		, pdata->lifetime_a*10, pdata->lifetime_b*10, (pdata->lifetime_eol==1)?"normal":
	//													(pdata->lifetime_eol==2)?"Warning":"Urgent!"));

	pdata->hcEraGrpSize = pdata->ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE];
	pdata->wrrelset     = pdata->ext_csd[EXT_CSD_WR_REL_SET];
	pdata->wrrelparam   = pdata->ext_csd[EXT_CSD_WR_REL_PARAM];

	/* bkops actual */
	pdata->bkopsSupport = pdata->ext_csd[EXT_CSD_BKOPS_SUPPORT];
	pdata->bkopsStatus  = pdata->ext_csd[EXT_CSD_BKOPS_STATUS];
	pdata->bkopsEnable  = pdata->ext_csd[EXT_CSD_BKOPS_EN];
	pdata->bkopsStart   = pdata->ext_csd[EXT_CSD_BKOPS_START];
	pdata->badBlkMngm   = pdata->ext_csd[EXT_CSD_SEC_BAD_BLK_MGMNT];

	/* refresh, actual */
	pdata->selfRefStatus = pdata->ext_csd[EXT_CSD_SELF_REF_STATUS];
	pdata->selfRefEnable = pdata->ext_csd[EXT_CSD_SELF_REF_ENABLE];
	pdata->selfRefDelay1 = pdata->ext_csd[EXT_CSD_SELF_REF_DELAY1];
	pdata->selfRefDelay2 = pdata->ext_csd[EXT_CSD_SELF_REF_DELAY2];
	pdata->selfRefPercent = (pdata->ext_csd[EXT_CSD_SELF_REF_PERCENT_1] << 8)
							  | pdata->ext_csd[EXT_CSD_SELF_REF_PERCENT_0];

	pdata->exception_ctrl     = pdata->ext_csd[EXT_CSD_EXCEPTION_EVENT_CTRL_0] << 0
                                | (pdata->ext_csd[EXT_CSD_EXCEPTION_EVENT_CTRL_1] << 8);
	pdata->exception_status   = pdata->ext_csd[EXT_CSD_EXCEPTION_EVENT_STATUS_0] << 0
			                    | (pdata->ext_csd[EXT_CSD_EXCEPTION_EVENT_STATUS_1] << 8);
	pdata->packed_cmd_status  = pdata->ext_csd[EXT_CSD_PACKED_CMD_STATUS];
	pdata->packed_failure_idx = pdata->ext_csd[EXT_CSD_PACKED_FAILURE_IDX];

	return 0;
}

int initCardInfo(struct mmcData *pdata)
{
	/*
	 * emmc info file, backup
	 */
	if (pdata->infofile[0]) {
		/* create, read, write */
		pdata->finfo = fopen(pdata->infofile, "w+");
		if (pdata->finfo == NULL) {
			printf("ERROR: infoFile: %s open..\n", pdata->infofile);
			return -1;
		}
	}
	return 0;
}

void writeCardInfo(struct mmcData *pdata)
{
	printf("%s", pdata->infoline);
	if (pdata->infofile[0]) {
		fwrite(pdata->infoline, strlen(pdata->infoline), 1, pdata->finfo);
		fflush(pdata->finfo);
	}
}

void print_card_info(struct mmcData *pdata)
{
	time_t rawtime;
	struct tm *now;
	time(&rawtime);
	now = gmtime(&rawtime);

	sprintf(pdata->infoline, "--------------------------------------\n");
	writeCardInfo(pdata);
	sprintf(pdata->infoline, "eMMC refresh V0.1 %s %s\n\n", __DATE__, __TIME__);
	writeCardInfo(pdata);

	strftime(pdata->infoline, sizeof(pdata->infoline), "time     : %Y-%m-%d_%H:%M:%S\n", now);
	writeCardInfo(pdata);

	sprintf(pdata->infoline, "device   : %s\n", pdata->device);
	writeCardInfo(pdata);

	sprintf(pdata->infoline, "refresh  : sector %lu %lu%%\n", pdata->lastSector
			, (pdata->lastSector * 100 / pdata->sectors));
	writeCardInfo(pdata);
	sprintf(pdata->infoline, "cid      : Ver. %d Rev. %s\n", pdata->csd_version
			, (pdata->ext_csd_rev==8)?"5.1":
	 		  (pdata->ext_csd_rev==7)?"5.0":
			  (pdata->ext_csd_rev==6)?"4.5":
			  (pdata->ext_csd_rev==5)?"4.41":"older 4.3");
	writeCardInfo(pdata);
	sprintf(pdata->infoline, "type     : %s OID %X Rev %d PNM >%s<\n"
			, (pdata->cid_manuid==0x13)?"Micron":
			  (pdata->cid_manuid==0xFE)?"Micron":
		      (pdata->cid_manuid==0x11)?"Toshiba":"other"
			, pdata->cid_oemid, pdata->cid_rev, pdata->cid_name);
	writeCardInfo(pdata);
	sprintf(pdata->infoline, "volume   : %du sectors - %du kB, cache %d kB\n"
			, pdata->sectors, pdata->capacity / 1024
			, (pdata->cachectrl==0)?0:pdata->cachsize);
	writeCardInfo(pdata);

	sprintf(pdata->infoline, "lifetime : slc %d%% mlc %d%% eol %s\n"
			, pdata->lifetime_a*10, pdata->lifetime_b*10,
			 (pdata->lifetime_eol==1)?"normal":
			 (pdata->lifetime_eol==2)?"Warning":"Urgent!");
	writeCardInfo(pdata);
	sprintf(pdata->infoline, "Feature 1: Cache         %s\n", (pdata->cachectrl==0)?"off":"on");
	writeCardInfo(pdata);
	if ((pdata->wrrelparam & EN_REL_WR) == 0) {
		sprintf(pdata->infoline, "Feature 2: WrReliability legacy only\n");
	}
	else {
		if ((pdata->wrrelset & WR_DATA_REL_ALL) == WR_DATA_REL_ALL) {
			sprintf(pdata->infoline, "Feature 2: WrReliability enhanced  / enabled");
		}
		else {
			sprintf(pdata->infoline, "Feature 2: WrReliability enhanced  / disabled");
		}
#ifndef RELIABILITY_WRITE_SUPPORT
		strcat(pdata->infoline, " by kernel\n");
#else
		strcat(pdata->infoline, " by us\n");
#endif
	}
	writeCardInfo(pdata);
	sprintf(pdata->infoline, "Feature 3: Refresh       %s %s (OTP)\n", pdata->bkopsSupport?"supported /":"not /"
			                           , pdata->bkopsEnable?"enabled":"disabled");
	writeCardInfo(pdata);
#ifdef CMD56_BUFFER_BLOCKS_REFRESH
	sprintf(pdata->infoline, "Feature 4: Buffer Blocks Refresh   / enabled\n");
#else
	sprintf(pdata->infoline, "Feature 4: Buffer Blocks Refresh   / disabled\n");
#endif
	writeCardInfo(pdata);
	sprintf(pdata->infoline, "Feature 5: ExceptionCtrl %s\n", pdata->exception_status?"events pending":"no event");
	writeCardInfo(pdata);
	if (pdata->exception_status) {
		sprintf(pdata->infoline, "           Event 0x%X\n", pdata->exception_status);
		writeCardInfo(pdata);
	}
	sprintf(pdata->infoline, "Feature 6: HEALTH not supported\n");
	writeCardInfo(pdata);
#ifdef CONTINUE_READ_SUPPORT
	sprintf(pdata->infoline, "Feature 7: manual sector read / enabled\n");
#else
	sprintf(pdata->infoline, "Feature 7: manual sector read / disabled\n");
#endif
	writeCardInfo(pdata);
	sprintf(pdata->infoline, "--------------------------------------\n");
	writeCardInfo(pdata);
}

int checkEmmc(int fd, struct mmcData *pdata)
{
	/*
	 * card status
	 */
	read_status(fd, &pdata->status);	// 0x100 ready for data
	pdata->currState = R1_CURRENT_STATE(pdata->status); // 4=transition, 3=standby
	pdata->r1Status  = R1_STATUS(pdata->status);

	printf("eMMC: STATUS 0x%02X = currState %X, r1Status %X\n"
			               , pdata->status, pdata->currState, pdata->r1Status);
	/*
	 * read ext CSD
	 */
	refresh_ext_cid(fd, pdata);

	print_card_info(pdata);

	return 0;
}
