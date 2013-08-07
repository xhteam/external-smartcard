LOCAL_PATH := $(call my-dir)

pcsc_tools_dir :=$(LOCAL_PATH)

common_cflags := \
	-DANDROID \
	-DHAVE_CONFIG_H

common_c_includes := \
	$(pcsc_tools_dir) \
	$(pcsc_tools_dir)/src \
	$(pcsc_tools_dir)/../pcsc-lite-1.8.8/src/PCSC \
	

	
# ============ build pcscd =============================================

include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
	src/SIMbrush.c src/pcsc-wrappers.c src/dirtree.c src/utils.c src/xml-results.c src/bail.c src/gsm-constants.c

LOCAL_C_INCLUDES	:= $(common_c_includes)
LOCAL_CFLAGS		:= $(common_cflags) 
	
LOCAL_LDLIBS		:= -ldl
LOCAL_SHARED_LIBRARIES := libc libpcsclite
LOCAL_PRELINK_MODULE	:= false
LOCAL_MODULE		:= simbrush
LOCAL_MODULE_TAGS	:= optional

include $(BUILD_EXECUTABLE)



