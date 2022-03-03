CONFIG_FILE_ANDROID = $(shell pwd)/productConfigs.mk
CONFIG_FILE_LINUX = $(call my-dir)/productConfigs.mk
ifeq ($(CONFIG_FILE_ANDROID), $(wildcard $(CONFIG_FILE_ANDROID)))
include $(CONFIG_FILE_ANDROID)
else ifeq ($(CONFIG_FILE_LINUX), $(wildcard $(CONFIG_FILE_LINUX)))
include $(CONFIG_FILE_LINUX)
endif

ifeq ($(IS_ANDROID_OS),true)
include $(call all-subdir-makefiles)
else
include $(call allSubdirMakefiles)
endif
