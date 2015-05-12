# put your *.o targets here, make should handle the rest!

NP2_PATH=$(NP2_ROOT)/src/boards/netduinoplus2
VCP_PATH=$(NP2_ROOT)/src/boards/netduinoplus2/usb/vcp
USBCORE_PATH=$(NP2_ROOT)/src/Libraries/STM32_USB_Device_Library/Core
USBCDC_PATH=$(NP2_ROOT)/src/Libraries/STM32_USB_Device_Library/Class/CDC
STDPERPH_PATH=$(NP2_ROOT)/src/Libraries/stm32f4xx_periph
COMMON_PATH=$(NP2_ROOT)/src/Libraries/common
CMSIS_PATH=$(NP2_ROOT)/src/Libraries/cmsis
FREERTOS_PATH=$(NP2_ROOT)/src/FreeRTOS

PROJ_NAME=main

SRCS = $(PROJ_NAME).c 
SRCS += $(NP2_PATH)/src/*.c
SRCS += $(STDPERPH_PATH)/src/*.c $(COMMON_PATH)/*.c
SRCS += $(USBCORE_PATH)/Src/*.c $(USBCDC_PATH)/Src/*.c $(VCP_PATH)/src/*.c
#SRCS += $(FREERTOS_PATH)/*.c $(FREERTOS_PATH)/portable/GCC/ARM_CM4F/*.c $(FREERTOS_PATH)/portable/MemMang/heap_1.c
# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)

###################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS  = -g -O1 -T$(NP2_PATH)/STM32F405.ld -Wmaybe-uninitialized
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

###################################################

vpath %.c src
vpath %.a lib

ROOT=$(shell pwd)

CFLAGS += -I$(USBCORE_PATH)/Inc -I$(USBCDC_PATH)/Inc -I$(VCP_PATH)/inc -I. -I..
CFLAGS += -I$(CMSIS_PATH) -I$(NP2_PATH)/inc -I$(STDPERPH_PATH)/inc -I$(COMMON_PATH) -DSTM32F405xx
#CFLAGS += -I$(FREERTOS_PATH)/include -I$(FREERTOS_PATH)/portable/GCC/ARM_CM4F
CFLAGS += -DENABLE_VCP #Enable USB VCP for debug_printf
#CFLAGS += -DENABLE_FRVCP #Enable USB VCP for debug_printf with FreeRTOS
CFLAGS += -DENABLE_DEBUG_UART	#Enable UART4 for debug printf

SRCS += $(NP2_PATH)/startup_STM32F40x.s # add startup file to build

OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: lib proj

all: proj

lib:
	$(MAKE) -C lib

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ -Llib -lm -lc -lgcc -lnosys #-lstm32f4
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

prog:
	sudo dfu-util -d 0483:df11 -c 1 -i 0 -a 0 -s 0x08000000 -D $(PROJ_NAME).bin

clean:
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
