CONTIKI_PROJECT = button-hal-example
TARGET = simplelink
BOARD = launchpad/cc1310
CFLAGS += -g
MODULES += os/services/shell

CONTIKI = ../../..

all: $(CONTIKI_PROJECT)

PLATFORMS_ONLY += cc26x0-cc13x0 cc2538dk openmote zoul simplelink
PLATFORMS_ONLY += cooja native

include $(CONTIKI)/Makefile.include
