LOCAL_PATH := $(call my-dir)


common_cflags := \
	-DANDROID \
	-DHAVE_CONFIG_H

common_c_includes := \
	$(LOCAL_PATH) \
	$(LOCAL_PATH)/src/PCSC \
	$(LOCAL_PATH)/src \
	external/libusb/libusb


# ============ build libpcsclite.so ====================================

include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
	src/debug.c \
       	src/error.c \
       	src/simclist.c \
       	src/strlcat.c \
       	src/strlcpy.c \
       	src/sys_unix.c \
       	src/utils.c \
       	src/winscard_msg.c \
       	src/winscard_clnt.c 
	
LOCAL_CFLAGS		:= $(common_cflags) -DLIBPCSCLITE

LOCAL_C_INCLUDES	:= $(common_c_includes)
LOCAL_MODULE		:= libpcsclite
LOCAL_MODULE_TAGS	:= optional

include $(BUILD_SHARED_LIBRARY)

# ============ build pcscd =============================================

include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
	src/debuglog.c \
	src/atrhandler.c \
	src/configfile.c \
	src/dyn_hpux.c \
	src/dyn_macosx.c \
	src/dyn_unix.c \
	src/eventhandler.c \
	src/hotplug_generic.c \
	src/hotplug_libusb.c \
	src/hotplug_linux.c \
	src/hotplug_macosx.c \
	src/ifdwrapper.c \
	src/pcscdaemon.c \
	src/powermgt_generic.c \
	src/prothandler.c \
	src/readerfactory.c \
	src/simclist.c \
	src/strlcat.c \
	src/strlcpy.c \
	src/sys_unix.c \
	src/tokenparser.c \
	src/utils.c \
	src/winscard.c \
	src/winscard_msg.c \
	src/winscard_msg_srv.c \
	src/winscard_svc.c

LOCAL_C_INCLUDES	:= $(common_c_includes)
LOCAL_CFLAGS		:= $(common_cflags) \
	-DPCSCD \
	-DSIMCLIST_NO_DUMPRESTORE
LOCAL_LDLIBS		:= -ldl
LOCAL_SHARED_LIBRARIES := libc liblog libcrypto libdl libusb
LOCAL_PRELINK_MODULE	:= false
LOCAL_MODULE		:= pcscd
LOCAL_MODULE_TAGS	:= optional
LOCAL_REQUIRED_MODULES := libusb libpcsclite libccid reader.conf 
include $(BUILD_EXECUTABLE)


#testpcsc
include $(CLEAR_VARS)
LOCAL_LDLIBS		:= -ldl
LOCAL_SHARED_LIBRARIES := libc libcrypto libdl libpcsclite

LOCAL_SRC_FILES		:= src/testpcsc.c
LOCAL_C_INCLUDES	:= \
	$(LOCAL_PATH) \
	$(LOCAL_PATH)/src/PCSC \
	$(LOCAL_PATH)/src 
LOCAL_CFLAGS		:= $(common_cflags)
LOCAL_MODULE_TAGS       := optional
LOCAL_MODULE		:= testpcsc

include $(BUILD_EXECUTABLE)

#pcsc_demo
include $(CLEAR_VARS)
LOCAL_SHARED_LIBRARIES := libc libdl libpcsclite

LOCAL_SRC_FILES		:= doc/example/pcsc_demo.c
LOCAL_C_INCLUDES	:= \
	$(LOCAL_PATH) \
	$(LOCAL_PATH)/src/PCSC \
	$(LOCAL_PATH)/src 
LOCAL_CFLAGS		:= $(common_cflags)
LOCAL_MODULE_TAGS   := optional
LOCAL_MODULE		:= pcsc_demo

include $(BUILD_EXECUTABLE)

#AU9540 ccid twin serial smart reader configure file
include $(CLEAR_VARS)
LOCAL_MODULE := reader.conf
LOCAL_SRC_FILES := reader.conf
LOCAL_MODULE_TAGS := optional
LCAL_MODULE_SUFFIX := .conf
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_PATH := $(TARGET_OUT_VENDOR)/etc/pcsc
include $(BUILD_PREBUILT)


