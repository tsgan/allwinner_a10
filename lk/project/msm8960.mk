# top level project rules for the msm8960_virtio project
#
LOCAL_DIR := $(GET_LOCAL_DIR)

TARGET := msm8960

MODULES += app/aboot

DEBUG := 1

#DEFINES += WITH_DEBUG_DCC=1
DEFINES += WITH_DEBUG_UART=1
#DEFINES += WITH_DEBUG_FBCON=1
