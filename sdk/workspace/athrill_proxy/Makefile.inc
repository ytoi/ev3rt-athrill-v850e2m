mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_COBJS +=action_mech.o
APPL_LIBS += -lm

#ADDITIONAL_PRE_APPL=cat __ev3rt_bt_out &
