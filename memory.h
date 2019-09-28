/*
 *  memory.h.h
 *
 *  Created on: 19.05.2017
 *      Author: hans-juergen.arlt <hj@arlt2net.de>
 */
#ifndef MEMORY_H
#define MEMORY_H

#include <stddef.h>
#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

int mem_fuction(unsigned long g_paddr, uint32_t *g_value
              , uint32_t g_count, int size, int g_is_write);

#endif

