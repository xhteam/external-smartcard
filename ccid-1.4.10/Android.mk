LOCAL_PATH := $(call my-dir)

common_cflags := \
	-DANDROID \
	-DHAVE_CONFIG_H
	
COMMON_SOURCE := \
	src/ccid.c \
	src/commands.c \
	src/ifdhandler.c \
	src/utils.c \
	
USB_SOURCE := src/ccid_usb.c

SERIAL_SOURCE := src/ccid_serial_au9540.c

T1_SOURCE := \
	src/towitoko/atr.c \
	src/towitoko/pps.c \
	src/openct/buffer.c \
	src/openct/checksum.c \
	src/openct/proto-t1.c 

PROVIDED_BY_PCSC_SOURCE := src/debug.c	

TOKEN_PARSER_SOURCE := src/tokenparser.c src/strlcpy.c src/simclist.c
#
#libccid.so
#
include $(CLEAR_VARS)
LOCAL_C_INCLUDES := \
	$(LOCAL_PATH) \
	$(LOCAL_PATH)/src \
	$(LOCAL_PATH)/src/openct \
	$(LOCAL_PATH)/src/towitoko \
	$(LOCAL_PATH)/../pcsc-lite-1.8.8/src/PCSC \
	$(LOCAL_PATH)/../pcsc-lite-1.8.8/src/ \
	external/libusb/libusb \
	external/libusb/libusb/os

LOCAL_CFLAGS		:= $(common_cflags) -DSIMCLIST_NO_DUMPRESTORE

LOCAL_SRC_FILES:= $(COMMON_SOURCE) $(USB_SOURCE) $(TOKEN_PARSER_SOURCE) $(PROVIDED_BY_PCSC_SOURCE) $(T1_SOURCE)

LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/

LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:= libccid
LOCAL_SHARED_LIBRARIES := libc libdl libusb liblog
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/pcsc

include $(BUILD_SHARED_LIBRARY)


#
#libccidtwin.so
#
include $(CLEAR_VARS)
LOCAL_C_INCLUDES := \
	$(LOCAL_PATH) \
	$(LOCAL_PATH)/src \
	$(LOCAL_PATH)/src/openct \
	$(LOCAL_PATH)/src/towitoko \
	$(LOCAL_PATH)/../pcsc-lite-1.8.8/src/PCSC \
	$(LOCAL_PATH)/../pcsc-lite-1.8.8/src/ \
	external/libusb/libusb \
	external/libusb/libusb/os

LOCAL_SRC_FILES:= $(COMMON_SOURCE) $(SERIAL_SOURCE) $(TOKEN_PARSER_SOURCE) $(PROVIDED_BY_PCSC_SOURCE) $(T1_SOURCE)
LOCAL_CFLAGS		:= $(common_cflags) -DTWIN_SERIAL -DSIMCLIST_NO_DUMPRESTORE
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/

LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:= libccidtwin
LOCAL_SHARED_LIBRARIES := libc libdl libcutils
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/pcsc

include $(BUILD_SHARED_LIBRARY)

#
#For supporting usb smart card readers,disable this will disable usb ccid support
#
include $(CLEAR_VARS)
LOCAL_MODULE := Info.plist
LOCAL_SRC_FILES := src/Info.plist
LOCAL_MODULE_TAGS := optional
LCAL_MODULE_SUFFIX := .plist
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/lib/pcsc/ifd-ccid.bundle/Contents
include $(BUILD_PREBUILT)


