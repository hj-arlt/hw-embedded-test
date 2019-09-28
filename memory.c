/*
 * memory.c
 *
 * Created on: 25.09.2019
 *     Author: hans-jürgen arlt <hj@arlt2net.de>
 */

#include "memory.h"

unsigned long g_paddr = 0;

void read_mem(void * addr, uint32_t count, uint32_t size)
{
	int i;
	uint8_t * addr8 = (uint8_t *)addr;
	uint16_t * addr16 = (uint16_t *)addr;
	uint32_t * addr32 = (uint32_t *)addr;

	switch (size)
	{
		case 1:
			for (i = 0; i < (int)count; i++) {
				if ( (i % 16) == 0 )
					printf("\n0x%08lX: ", g_paddr);
				printf(" %02X", addr8[i]);
				g_paddr++;
			}
			break;
		case 2:
			for (i = 0; i < (int)count; i++) {
				if ( (i % 8) == 0 )
					printf("\n0x%08lX: ", g_paddr);
				printf(" %04X", addr16[i]);
				g_paddr += 2;
			}
			break;
		case 4:
			for (i = 0; i < (int)count; i++) {
				if ( (i % 4) == 0 )
					printf("\n0x%08lX: ", g_paddr);
				printf(" %08X", addr32[i]);
				g_paddr += 4;
			}
			break;
	}
	printf("\n\n");

}

void write_mem(void * addr, uint32_t value, uint32_t size)
{
	uint8_t * addr8 = (uint8_t *)addr;
	uint16_t * addr16 = (uint16_t *)addr;
	uint32_t * addr32 = (uint32_t *)addr;

	switch (size)
	{
		case 1:
			*addr8 = value;
			break;
		case 2:
			*addr16 = value;
			break;
		case 4:
			*addr32 = value;
			break;
	}
}

int mem_fuction(unsigned long paddr, uint32_t *value, uint32_t g_count, int size, int g_is_write)
{
	int fd;
	void * mem;
	void * aligned_vaddr;
	unsigned long aligned_paddr;
	uint32_t aligned_size;
	int g_size = 4;
	g_paddr = paddr;
	uint32_t g_value = *value;

	if (size <= 8) g_size = 1;
	else if (size <= 16) g_size = 2;

	/* Align address to access size */
	g_paddr &= ~(g_size - 1);

	aligned_paddr = g_paddr & ~(4096 - 1);
	aligned_size = g_paddr - aligned_paddr + (g_count * g_size);
	aligned_size = (aligned_size + 4096 - 1) & ~(4096 - 1);

	if (g_is_write)
		printf("Writing %d-bit value 0x%uX to address 0x%08lX\n"
                        , g_size*8, g_value, g_paddr);
	else
		printf("Reading 0x%uX count starting at address 0x%08lX\n"
                        , g_count, g_paddr);

	if ((fd = open("/dev/mem", O_RDWR, 0)) < 0)
		return 1;

	aligned_vaddr = mmap(NULL, aligned_size, PROT_READ | PROT_WRITE
                                           , MAP_SHARED
                                 , fd, aligned_paddr);
	if (aligned_vaddr == NULL) {
		printf("Error mapping address\n");
		close(fd);
		return 1;
	}

	mem = (void *)(aligned_vaddr + (g_paddr - aligned_paddr));

	if (g_is_write) {
		write_mem(mem, g_value, g_size);
	}
	else {
		read_mem(mem, g_count, g_size);
	}

	munmap(aligned_vaddr, aligned_size);
	close(fd);
	return 0;
}


