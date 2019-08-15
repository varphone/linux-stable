
cflags-y += -I$(srctree)/drivers/crypto/hisi-cipher/src/drv/cipher_v1.0/osal/include
cflags-y += -I$(srctree)/drivers/crypto/hisi-cipher/include
cflags-y += -I$(srctree)/drivers/crypto/hisi-cipher/src/drv/cipher_v1.0/drivers/core/include
cflags-y += -I$(srctree)/drivers/crypto/hisi-cipher/src/drv/cipher_v1.0/compat/

ifeq ($(CONFIG_ARCH_HI3519AV100),y)
cflags-y += -DCHIP_TYPE_hi3519av100
endif

ifeq ($(CONFIG_ARCH_HI3559AV100),y)
cflags-y += -DCHIP_TYPE_hi3559av100
endif

ccflags-y  += $(cflags-y)
HOSTCFLAGS += $(cflags-y)
CPPFLAGS   += $(cflags-y)

obj-y    += hal_otp.o
