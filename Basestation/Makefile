# Contiki network configuration
MAKE_MAC = MAKE_MAC_TSCH #Mac as Tsch

CONTIKI_PROJECT = client basestation login
APPS=serial-shell

all: $(CONTIKI_PROJECT)

CONTIKI = ../contiki-ng-wcnes

# force Orchestra from command line
MAKE_WITH_ORCHESTRA ?= 0
# force Security from command line
MAKE_WITH_SECURITY ?= 0
# print #routes periodically, used for regression tests
MAKE_WITH_PERIODIC_ROUTES_PRINT = 1

MAKE_MAC = MAKE_MAC_TSCH

include $(CONTIKI)/Makefile.dir-variables
MODULES += $(CONTIKI_NG_SERVICES_DIR)/shell

ifeq ($(MAKE_WITH_ORCHESTRA),1)
MODULES += $(CONTIKI_NG_SERVICES_DIR)/orchestra
endif

ifeq ($(MAKE_WITH_SECURITY),1)
CFLAGS += -DWITH_SECURITY=1
endif

ifeq ($(MAKE_WITH_PERIODIC_ROUTES_PRINT),1)
CFLAGS += -DWITH_PERIODIC_ROUTES_PRINT=1
endif

include $(CONTIKI)/Makefile.include
