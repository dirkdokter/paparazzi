# Hey Emacs, this is a -*- makefile -*-
#
# $Id$
# Copyright (C) 2012 The Paparazzi Team
#
# This file is part of Paparazzi.
#
# Paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# Paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#
#




################################################################################
#
#
#  Test program for the ADNS3080 OFS sensor
#
################################################################################

ARCH=stm32
SRC_ARCH=arch/$(ARCH)
SRC_LISA=lisa
SRC_LISA_ARCH=$(SRC_LISA)/arch/$(ARCH)
#SRC_ROTORCRAFT=rotorcraft
SRC_BOARD=boards/$(BOARD)

SRC_FIRMWARE=firmwares/rotorcraft
SRC_SUBSYSTEMS=subsystems
SRC_AIRBORNE=.


#
# common test
#
# configuration
#   SYS_TIME_LED
#   MODEM_PORT
#   MODEM_BAUD
#
PERIODIC_FREQUENCY = 512

COMMON_TEST_CFLAGS  = -I$(SRC_FIRMWARE) -I$(ARCH) -DPERIPHERALS_AUTO_INIT
COMMON_TEST_CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
COMMON_TEST_SRCS    = $(SRC_AIRBORNE)/mcu.c            \
                      $(SRC_ARCH)/mcu_arch.c           \
                      $(SRC_ARCH)/stm32_exceptions.c   \
                      $(SRC_ARCH)/stm32_vector_table.c
COMMON_TEST_CFLAGS += -DUSE_SYS_TIME
ifneq ($(SYS_TIME_LED),none)
  COMMON_TEST_CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
endif
COMMON_TEST_CFLAGS += -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)
COMMON_TEST_SRCS   += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c

COMMON_TEST_CFLAGS += -DUSE_LED
COMMON_TEST_SRCS   += $(SRC_ARCH)/led_hw.c

COMMON_TELEMETRY_CFLAGS  = -DUSE_$(MODEM_PORT) -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
COMMON_TELEMETRY_CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=PprzTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
COMMON_TELEMETRY_SRCS    = mcu_periph/uart.c
COMMON_TELEMETRY_SRCS   += $(SRC_ARCH)/mcu_periph/uart_arch.c
COMMON_TELEMETRY_SRCS   += subsystems/datalink/downlink.c subsystems/datalink/pprz_transport.c

#COMMON_TEST_SRCS   += math/pprz_trig_int.c


#
# test_adns3080_hispeed : Sends ALIVE telemetry messages
#
# configuration
#   MODEM_PORT :
#   MODEM_BAUD :
#   DOWNLINK_TRANSPORT :
#   DATALINK  :
#
test_adns3080_hispeed.ARCHDIR = $(ARCH)

test_adns3080_hispeed.CFLAGS  = $(COMMON_TEST_CFLAGS)
test_adns3080_hispeed.srcs    = $(COMMON_TEST_SRCS)
test_adns3080_hispeed.CFLAGS += $(COMMON_TELEMETRY_CFLAGS)
test_adns3080_hispeed.srcs   += $(COMMON_TELEMETRY_SRCS)

test_adns3080_hispeed.CFLAGS += -I$(SRC_LISA) -I$(SRC_ARCH) -DPERIPHERALS_AUTO_INIT -DUSE_OPTFLOW_ADNS3080
test_adns3080_hispeed.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_adns3080_hispeed.srcs = $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 test/test_adns3080_hispeed.c            \
                 $(SRC_ARCH)/stm32_exceptions.c   \
                 $(SRC_ARCH)/stm32_vector_table.c \
                 $(SRC_AIRBORNE)/modules/opticflow/opticflow_ADNS3080.c \
                 $(SRC_SUBSYSTEMS)/datalink/xbee.c \
                 $(SRC_SUBSYSTEMS)/datalink/downlink.c 
test_adns3080_hispeed.CFLAGS += -DUSE_LED
test_adns3080_hispeed.srcs += $(SRC_ARCH)/led_hw.c
test_adns3080_hispeed.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_adns3080_hispeed.srcs += $(SRC_AIRBORNE)/mcu_periph/uart.c
test_adns3080_hispeed.CFLAGS += -DUSE_SYS_TIME
test_adns3080_hispeed.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./2048.)'
test_adns3080_hispeed.CFLAGS += -DPERIODIC_FREQUENCY='512.'
test_adns3080_hispeed.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
test_adns3080_hispeed.srcs += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c
test_adns3080_hispeed.CFLAGS += -DUSE_$(MODEM_PORT)
test_adns3080_hispeed.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_adns3080_hispeed.CFLAGS += -DXBEE_UART=$(MODEM_PORT)
test_adns3080_hispeed.CFLAGS += -DDATALINK=XBEE
test_adns3080_hispeed.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=XBeeTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)

#
# test_adns3080_capture : Sends ALIVE telemetry messages
#
# configuration
#   MODEM_PORT :
#   MODEM_BAUD :
#   DOWNLINK_TRANSPORT :
#   DATALINK  :
#
test_adns3080_capture.ARCHDIR = $(ARCH)
test_adns3080_capture.CFLAGS += -I$(SRC_LISA) -I$(SRC_ARCH) -DPERIPHERALS_AUTO_INIT -DUSE_OPTFLOW_ADNS3080
test_adns3080_capture.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)
test_adns3080_capture.srcs = $(SRC_AIRBORNE)/mcu.c \
                 $(SRC_ARCH)/mcu_arch.c \
                 test/test_adns3080_capture.c            \
                 $(SRC_ARCH)/stm32_exceptions.c   \
                 $(SRC_ARCH)/stm32_vector_table.c \
                 $(SRC_AIRBORNE)/modules/opticflow/opticflow_ADNS3080.c \
                 $(SRC_AIRBORNE)/xbee.c \
                 $(SRC_AIRBORNE)/downlink.c 
test_adns3080_capture.CFLAGS += -DUSE_LED
test_adns3080_capture.srcs += $(SRC_ARCH)/led_hw.c
test_adns3080_capture.srcs   += $(SRC_ARCH)/mcu_periph/uart_arch.c
test_adns3080_capture.srcs += $(SRC_AIRBORNE)/mcu_periph/uart.c
test_adns3080_capture.CFLAGS += -DUSE_SYS_TIME
test_adns3080_capture.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC(1./2048.)'
test_adns3080_capture.CFLAGS += -DSYS_TIME_LED=$(SYS_TIME_LED)
test_adns3080_capture.srcs   += sys_time.c $(SRC_ARCH)/sys_time_hw.c
test_adns3080_capture.CFLAGS += -DUSE_$(MODEM_PORT)
test_adns3080_capture.CFLAGS += -D$(MODEM_PORT)_BAUD=$(MODEM_BAUD)
test_adns3080_capture.CFLAGS += -DXBEE_UART=$(MODEM_PORT)
test_adns3080_capture.CFLAGS += -DDATALINK=XBEE
test_adns3080_capture.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=XBeeTransport -DDOWNLINK_DEVICE=$(MODEM_PORT)
