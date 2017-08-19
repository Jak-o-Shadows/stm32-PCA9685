#this makefile is helped in part by https://github.com/libopencm3/libopencm3-examples/blob/master/examples/Makefile.rules

#My includes/own library location
ROOT = 
LOWLEVEL= $(ROOT)lowlevel
MIDLEVEL = $(ROOT)midlevel
HIGHLEVEL= $(ROOT)highlevel
SUBHIGH = 
SUBMID = 
#Use a separate file to keep track of own includes
HL_LIBS = 
include Makefile.config

#LibopenCM3 location
libopenCM3Path =./libopencm3
SRCLIBDIR = $(libopenCM3Path)

INCLUDE_DIR = $(libopenCM3Path)/include
LIB_DIR = $(libopenCM3Path)/lib
SCRIPT_DIR = $(libopenCM3)/scripts

######Device specific things
DEVICE = stm32f103c8t6
ARCH_FLAGS = 


#######  LD Script #################
# libopencm3 makefile that does the ldscript
#uses libopencm3 to make a .ld file for the specified device
LDFLAGS		?= 
LDFLAGS		+= --static -nostartfiles
LDFLAGS		+= -L$(LIB_DIR)
LDFLAGS		+= -T$(LDSCRIPT)
LDFLAGS		+= -Wl,-Map=$(*).map
LDFLAGS		+= -Wl,--gc-sections

LIBNAME = opencm3_stm32f1
#LIBNAME = opencm3_stm32f4




#what is the main file 
BINARY = main
OBJS += $(BINARY).o $(HL_LIBS:.c=.o)



toolchainPath ?= ~/arm-none-eabi/bin/

PREFIX		?= $(toolchainPath)arm-none-eabi

CC		:= $(PREFIX)-gcc
LD		:= $(PREFIX)-gcc
AR		:= $(PREFIX)-ar
AS		:= $(PREFIX)-as
OBJCOPY		:= $(PREFIX)-objcopy
OBJDUMP		:= $(PREFIX)-objdump
GDB		:= $(PREFIX)-gdb



CFLAGS = 
#with thanks to libopencm3 makefile
CFLAGS += -g
CFLAGS += -Wextra -Wshadow -Wimplicit-function-declaration
CFLAGS += -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
CFLAGS += -fno-common -ffunction-sections -fdata-sections

CFLAGS += -MD
CFLAGS += -Wall -Wundef
CFLAGS += -mthumb


CFLAGS += -T $(LDSCRIPT)

#because libopencm3 is slightly broken
CFLAGS += -DSTM32F1


#Includes
CFLAGS += -I $(INCLUDE_DIR)
CFLAGS += -I $(HIGHLEVEL)
CFLAGS += -I $(MIDLEVEL)
CFLAGS += -I $(LOWLEVEL)
CFLAGS += -I $(SUBHIGH)
CFLAGS += -I $(SUBMID)


LDLIBS		+= -l$(LIBNAME)
LDLIBS		+= -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group 
#include <math.h>
LDLIBS      += -lm

	
.SUFFIXES: .elf .bin .hex .srec .list .map .images
.SECONDEXPANSION:
.SECONDARY:

all: elf bin

include $(libopenCM3Path)/ld/Makefile.linker

elf: $(BINARY).elf
bin: $(BINARY).bin
hex: $(BINARY).hex
srec: $(BINARY).srec
list: $(BINARY).list

images: $(BINARY).images
flash: $(BINARY).flash

%.images: %.bin %.hex %.srec %.list %.map
	@#printf "*** $* images generated ***\n"

%.bin: %.elf
	@#printf "  OBJCOPY $(*).bin\n"
	$(Q)$(OBJCOPY) -Obinary $(*).elf $(*).bin

%.hex: %.elf
	@#printf "  OBJCOPY $(*).hex\n"
	$(Q)$(OBJCOPY) -Oihex $(*).elf $(*).hex

%.srec: %.elf
	@#printf "  OBJCOPY $(*).srec\n"
	$(OBJCOPY) -Osrec $(*).elf $(*).srec

%.list: %.elf
	@#printf "  OBJDUMP $(*).list\n"
	$(OBJDUMP) -S $(*).elf > $(*).list

%.elf %.map: $(OBJS) $(LDSCRIPT)
	@#printf "  LD      $(*).elf\n"
	$(LD) $(LDFLAGS) $(ARCH_FLAGS) $(OBJS) $(LDLIBS) -o $(*).elf

%.o: %.c
	@#printf "  CC      $(*).c\n"
	$(Q) $(CC) $(CFLAGS) $(CPPFLAGS) $(ARCH_FLAGS) -o $(*).o -c $(*).c

clean:
	@#printf "  CLEAN\n"
	$(Q)$(RM) $(OBJS)
	$(Q)$(RM) *.o *.d *.elf *.bin *.hex *.srec *.list *.map





openocd:;
	openocd -s "software\OpenOCD-0.9.0\share\openocd\scripts" -f "openocd.cfg" -c "init" -c "halt" -c "reset halt"

debug: main.elf main.bin;
	$(GDB) --eval-command="target ext:3335"  main.elf

telnet:;
	telnet localhost 4446


    
    
    
    
testecho:;
	echo $(libopenCM3Path)
	echo $(LD_PARAMS)
