#ifndef __BOARD_MX6Q_PHYTEC_SD_H__
#define __BOARD_MX6Q_PHYTEC_SD_H__

#include <mach/iomux-mx6q.h>

#define MX6Q_USDHC_PAD_SETTING(id, speed)       \
mx6q_sd##id##_##speed##mhz[] = {                \
	MX6Q_PAD_SD##id##_CLK__USDHC##id##_CLK_##speed##MHZ,    \
	MX6Q_PAD_SD##id##_CMD__USDHC##id##_CMD_##speed##MHZ,    \
	MX6Q_PAD_SD##id##_DAT0__USDHC##id##_DAT0_##speed##MHZ,  \
	MX6Q_PAD_SD##id##_DAT1__USDHC##id##_DAT1_##speed##MHZ,  \
	MX6Q_PAD_SD##id##_DAT2__USDHC##id##_DAT2_##speed##MHZ,  \
	MX6Q_PAD_SD##id##_DAT3__USDHC##id##_DAT3_##speed##MHZ,  \
	MX6Q_PAD_SD##id##_DAT4__USDHC##id##_DAT4_##speed##MHZ,  \
	MX6Q_PAD_SD##id##_DAT5__USDHC##id##_DAT5_##speed##MHZ,  \
	MX6Q_PAD_SD##id##_DAT6__USDHC##id##_DAT6_##speed##MHZ,  \
	MX6Q_PAD_SD##id##_DAT7__USDHC##id##_DAT7_##speed##MHZ,  \
}

#define MX6Q_USDHC_PAD_SETTING_SHORT(id, speed) \
mx6q_sd##id##_##speed##mhz[] = {                \
	MX6Q_PAD_SD##id##_CLK__USDHC##id##_CLK_##speed##MHZ,    \
	MX6Q_PAD_SD##id##_CMD__USDHC##id##_CMD_##speed##MHZ,    \
	MX6Q_PAD_SD##id##_DAT0__USDHC##id##_DAT0_##speed##MHZ,  \
	MX6Q_PAD_SD##id##_DAT1__USDHC##id##_DAT1_##speed##MHZ,  \
	MX6Q_PAD_SD##id##_DAT2__USDHC##id##_DAT2_##speed##MHZ,  \
	MX6Q_PAD_SD##id##_DAT3__USDHC##id##_DAT3_##speed##MHZ,  \
}

extern void __init board_esdhc_init(char id, int cd_gpio, int wp_gpio);

#endif /* __BOARD_MX6Q_PHYTEC_SD_H__ */