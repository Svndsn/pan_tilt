.PHONY: all flash clean

CC := arm-none-eabi-gcc
AS := arm-none-eabi-as
LD := arm-none-eabi-ld
OC := arm-none-eabi-objcopy
OD := arm-none-eabi-objdump
SZ := arm-none-eabi-size

CFLAGS := -mthumb \
    -mcpu=cortex-m4 \
    -mfpu=fpv4-sp-d16 \
    -mfloat-abi=softfp \
		-ffunction-sections \
		-fdata-sections \
		-MD \
		-std=c99 \
		-Wall \
		-pedantic \
		-DPART_TM4C123GH6PM \
		-c \
		-Os \
		-DTARGET_IS_TM4C123_RB1 \
		-Dgcc

LDFLAGS := -T ./tm4c123/tm4c123.ld --entry ResetISR --gc-sections
OBJCOPYFLAGS := -O binary

# Use stdlibs or costum memory management
USE_STDLIB := 0
ifeq ($(USE_STDLIB),1)
	# If there is no stdlib in the project for FreeRTOS
	# Get the location of libgcc.a from the GCC front-end.
	LIBGCC:=${shell ${CC} ${CFLAGS} -print-libgcc-file-name}
	# Get the location of libc.a from the GCC front-end.
	LIBC:=${shell ${CC} ${CFLAGS} -print-file-name=libc.a}
	# Get the location of libm.a from the GCC front-end.
	LIBM:=${shell ${CC} ${CFLAGS} -print-file-name=libm.a}
	STDLIBS := '${LIBGCC}' '${LIBC}' '${LIBM}'
else
	STDLIBS :=
endif

MEMORY := ./freeRTOS/portable/MemMang/heap_4.c
PORT := ./freeRTOS/portable/GCC/ARM_CM4F/port.c

SOURCES := $(wildcard ./src/*.c) $(wildcard ./tm4c123/*.c) $(wildcard ./freeRTOS/*.c) $(PORT) $(MEMORY)
OBJECTS := $(patsubst %.c, build/%.o, $(SOURCES))
DEPENDS := $(patsubst %.c, build/%.d, $(SOURCES))

INCLUDES := -I./freeRTOS/include/ \
		-I./freeRTOS/portable/GCC/ARM_CM4F/ \
		-I./src/inc/ \
		-I./tm4c123/

all: main

flash: all
	lm4flash build/main.bin

clean:
	rm -rf build

-include $(DEPENDS)

build/%.o: %.c Makefile
	mkdir -p build
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o build/$(notdir $@)

main: $(OBJECTS)
	$(LD) $(LDFLAGS) $(addprefix build/, $(notdir $^)) -o build/main.elf $(STDLIBS)
	$(OC) $(OBJCOPYFLAGS) build/main.elf build/main.bin
