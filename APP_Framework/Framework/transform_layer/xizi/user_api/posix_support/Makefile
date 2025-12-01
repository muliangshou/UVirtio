SRC_FILES := pthread.c   
ifeq ($(CONFIG_KERNEL_SEMAPHORE),y)
	SRC_FILES += semaphore.c
endif

ifeq ($(CONFIG_KERNEL_MUTEX),y)
	SRC_FILES += pthread_mutex.c
endif

ifeq ($(CONFIG_KERNEL_MESSAGEQUEUE),y)
	SRC_FILES += mqueue.c
endif

include $(KERNEL_ROOT)/compiler.mk