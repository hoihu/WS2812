# Makefile for the WS2812 demo

CPU = msp430f5529
#~ CPU = msp430f5508
NAME = simple_led

# enter here the path to the msp430 library
LD_LIBRARY_PATH=/home/martin/msp430lib

export LD_LIBRARY_PATH

BIN_NAME = $(NAME)
# Switch the compiler (for the internal make rules)
CC              = msp430-gcc
AS              = msp430-gcc
AR              = msp430-ar

SOURCE_ROOT = .
VPATH += $(SOURCE_ROOT)    

# Application C files
ALL_CSOURCES    = $(notdir $(wildcard $(SOURCE_ROOT)/*.c))

EXTRA_INCLUDES = -I $(SOURCE_ROOT) 
# compiler flags        -Wcast-qual
CFLAGS          = -mmcu=${CPU} -Os -Wall  -Wundef --std=gnu99 -g $(EXTRA_INCLUDES) 

# here are all object files
OBJECTS         = $(notdir $(ALL_CSOURCES:.c=.o)) 

.PHONY: all FORCE clean

# main target: build application
all: clean ${NAME}.elf ${NAME}.a43 ${NAME}.lst

#additional rules for files

${BIN_NAME}.elf: ${OBJECTS}
	${CC} -mmcu=${CPU} -o $@ $^ -Wl,--gc-sections
	msp430-size $@

${BIN_NAME}.a43: ${NAME}.elf
	msp430-objcopy -O ihex $^ $@

# Write a listing file, also add some object size information.
${BIN_NAME}.lst: ${NAME}.elf
	msp430-objdump -dSt $^ >$@
	echo >>$@ "----- RAM/Flash Usage Application -----"
	msp430-size $(OBJECTS) $^ >>$@
    
.PHONY: clean
clean: 
	$(RM) ${NAME}.elf ${NAME}.a43 ${NAME}.lst ${OBJECTS} 

#~ download: download-hid_0
download: download-hid

download-hid_0: all
	python -m msp430.bsl5.hid ${NAME}.a43 -r

download-hid: all
	sudo mspdebug load-bsl "prog simple_led.a43"

download-jtag: ${BIN_NAME}.a43
	msp430-jtag -lTIUSB -e $^
    
