# Specifies the name of the exe file
APP        =  test

# Specifies to the compiler the files to compile
APP_SRCS    += Src/main.c

# Add here other sources if you have (i.e. APP_CFLAGS += my_conv.c)
APP_SRCS    += Src/pulp_nn_utils.c
APP_SRCS    += Src/pulp_nn_matmul.c
APP_SRCS    += Src/pulp_nn_conv.c

# Specifies c compiler's flags
APP_CFLAGS += -O3 -IInc

# Define the available cores for this application
APP_CFLAGS += -DNB_CORES=8

#
# EXERCISE 1.1
# - Add the file names
#

#
# PUT YOUR CODE HERE
#

READFS_FILES := $(FLASH_FILES)
PLPBRIDGE_FLAGS += -f

# Includes the rules to compile your code and the runtime functions and to run
# the exe file on simulation platforms such as RTL or GVSoC, or on real devices
# such as FPGAs and ASICs
CONFIG_HYPERFLASH=1
CONFIG_HYPERRAM=1

include $(RULES_DIR)/pmsis_rules.mk
