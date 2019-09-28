#########################################################################
# Makefile
#
# hans-jürgen arlt @ hj@arlt2net.de
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation version 2.
#
# This program is distributed "as is" WITHOUT ANY WARRANTY of any
# kind, whether express or implied; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
#########################################################################

DEBUG = y

FILE_NAME = hwtest

COBJS := main.o interfaces.o can.o memory.o mmc.o ntripclient.o gnss.o lin.o video.o

SRCS	 := $(SOBJS:.o=.S) $(COBJS:.o=.c) $(COBJS:.o=.cpp)
OBJS	 := $(addprefix $(obj),$(COBJS))
INCLUDE  :=
CFLAGS   := -c -Wall -DBOARD_$(shell echo $(BOARD) | tr a-z A-Z)
CXXFLAGS := 
LDLIBS   := -lc -lrt -lm -lpthread

ifeq ($(DEBUG),y)
  # "-O" is needed to expand inlines
  CFLAGS += -O -g -DDEBUG
else
  CFLAGS += -O2 -s
endif

BOARD ?= X86
CFLAGS += -DBOARD_$(shell echo $(BOARD) | tr a-z A-Z)

CROSS_COMPILE ?=
ifneq (,$(findstring $(BOARD),imx28))
CROSS_COMPILE = arm-linux-gnueabi-
endif
ifneq (,$(findstring $(BOARD),rpi3))
CROSS_COMPILE = arm-linux-gnueabihf-
endif

CC  = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)gcc
RM  = rm


# tool invocation
all: $(OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C Linker'
	$(CC) $(OBJS) $(LDLIBS) $(INCLUDE) -o $(FILE_NAME) -Wl,-Map,$(FILE_NAME).map
	@echo 'Finished building target: $@'
	@echo ' ' 

# other targets
clean:
	rm -f $(OBJS)

distclean:	clean
	rm -f $(LIB) core *.bak $(obj).depend

# Pattern rules #######################################################

.cpp.o:
	$(CXX) $(CFLAGS) -o $@ -c $<

.c.o:
	$(CXX) $(CFLAGS) -o $@ -c $<

.s.o:
	$(CXX) $(CFLAGS) -o $@ -c $<


