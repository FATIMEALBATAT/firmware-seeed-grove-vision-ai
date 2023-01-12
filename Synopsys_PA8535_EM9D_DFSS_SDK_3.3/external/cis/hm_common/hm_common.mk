DEV_CSRCDIR += $(EMBARC_ROOT)/external/cis/hm_common
DEV_INCDIR +=  $(EMBARC_ROOT)/external/cis/

DEV_DEFINES += -DHM_COMMON

##
#  CIS_XSHUTDOWN_PIN  SGPIO_0, SGPIO_1
##
ifeq ($(CIS_XSHUTDOWN_PIN_SEL), XSHUTDOWN_SGPIO0)
APPL_DEFINES += -DCIS_XSHUT_SGPIO0
else
APPL_DEFINES += -DCIS_XSHUT_SGPIO1
endif

ifeq ($(CIS_SEL), HM_COMMON)
APPL_DEFINES += -DCIS_COMMON
endif