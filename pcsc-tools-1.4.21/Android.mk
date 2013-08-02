LOCAL_PATH := $(call my-dir)

pcsc_tools_dir :=$(LOCAL_PATH)

common_cflags := \
	-DANDROID \
	-DHAVE_CONFIG_H

common_c_includes := \
	$(pcsc_tools_dir) \
	$(pcsc_tools_dir)/../pcsc-lite-1.8.8/src/PCSC \
	
include $(CLEAR_VARS)

LOCAL_SRC_FILES	 := pcsc_scan.c

	
LOCAL_CFLAGS		:= $(common_cflags) 
LOCAL_C_INCLUDES	:= $(common_c_includes)
LOCAL_SHARED_LIBRARIES := libc libcrypto libdl libpcsclite


LOCAL_MODULE_TAGS       := optional
LOCAL_MODULE		:= pcscscan

include $(BUILD_EXECUTABLE)
 
