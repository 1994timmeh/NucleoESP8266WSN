PROJ_NAME=main
SRCS = $(PROJ_NAME).c
# Add additional project sources like this:
# SRCS += X.c
#
# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)

###################################################
ROOT=$(shell pwd)

OBJS=$(SRCS:.c=.o)

###################################################

.PHONY: lib proj all depend
all: proj

include $(NP2_ROOT)/src/boards/netduinoplus2/Makefile.np2

lib:
	$(MAKE) -C lib

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf:	$(NP2OBJ) $(OBJS)
	$(CC) $(LDFLAGS) -o $@ -Llib $(OBJS) $(NP2OBJ) $(LD_SYS_LIBS)
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

%.o:
	$(CC) $(CFLAGS) -c $< -o $@

depend: .depend

.depend: $(SRCS)
	rm -f ./.depend
	mkdir -p obj
	$(CC) $(CFLAGS) -MM $^>>./.depend;

prog:
	sudo dfu-util -d 0483:df11 -c 1 -i 0 -a 0 -s 0x08000000 -D $(PROJ_NAME).bin		#Program NP2

pron:
	sudo ~/gitdev/stlink/st-flash write $(PROJ_NAME).bin 0x8000000	#Program Nucleo

qemu:
	np2-qemu $(PROJ_NAME).elf

gdb:
	np2-gdb-qemu $(PROJ_NAME).elf

clean:
	rm -f *.o
	rm -f $(OBJDIR)/*.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
	rm -f $(LIBNP2)


include .depend
