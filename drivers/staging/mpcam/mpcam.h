/*
 * mpcam.h - Multiple Path Camera Driver
 *
 * Copyright (C) 2018 NanJing No.55 Research Institute
 *                    Technology Development CO.,LTD
 * Copyright (C) 2018 Varphone Wong <varphone@qq.com>
 *
 */

#ifndef MPCAP_H
#define MPCAP_H

#include <linux/io.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/videodev2.h>
#include <linux/wait.h>

// CONTROL_BUS
// 0x00 : Control signals
//        bit 0  - ap_start (Read/Write/COH)
//        bit 1  - ap_done (Read/COR)
//        bit 2  - ap_idle (Read)
//        bit 3  - ap_ready (Read)
//        bit 7  - auto_restart (Read/Write)
//        others - reserved
// 0x04 : Global Interrupt Enable Register
//        bit 0  - Global Interrupt Enable (Read/Write)
//        others - reserved
// 0x08 : IP Interrupt Enable Register (Read/Write)
//        bit 0  - Channel 0 (ap_done)
//        bit 1  - Channel 1 (ap_ready)
//        others - reserved
// 0x0c : IP Interrupt Status Register (Read/TOW)
//        bit 0  - Channel 0 (ap_done)
//        bit 1  - Channel 1 (ap_ready)
//        others - reserved
// 0x10 : Data signal of frame_buff
//        bit 31~0 - frame_buff[31:0] (Read/Write)
// 0x14 : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define MPCAM_FILLER_ADDR_AP_CTRL         0x00
#define MPCAM_FILLER_ADDR_GIE             0x04
#define MPCAM_FILLER_ADDR_IER             0x08
#define MPCAM_FILLER_ADDR_ISR             0x0c
#define MPCAM_FILLER_ADDR_FRAME_BUFF_DATA 0x10
#define MPCAM_FILLER_BITS_FRAME_BUFF_DATA 32

#define MPCAM_FILLER_FRAME_BUFF_BASE_ADDR 0x20000000
#define MPCAM_FILLER_MAX_BUFFERS          6

struct mpcam_filler {
    void __iomem* base;
    int irq;
};

struct mpcam_filler_buffer {
    struct list_head node;
    struct v4l2_buffer buf;
};

struct mpcam
{
    struct platform_device* pdev;
    struct miscdevice miscdev;

    /* File Operations */
    struct wait_queue_head poll_wait;

    /* ISP Modules */
    struct resource filler_fb_res;
    struct mpcam_filler filler_rd;
    struct mpcam_filler filler_wr;
    /* Buffers for read and write */
    struct mpcam_filler_buffer* filler_buffers;
    /* Current buffer used for RD */
    struct mpcam_filler_buffer* filler_rd_buf;
    /* Current buffer used for WR */
    struct mpcam_filler_buffer* filler_wr_buf;
    /* Buffers to transfer to sink */
    struct list_head filler_dirty_list;
    /* Buffers are not used */
    struct list_head filler_free_list;
    /* Buffers recieved from src */
    struct list_head filler_ready_list;
};

/**
 * mpcam_file - Context for fops
 */
struct mpcam_file
{
    struct mpcam* mpcam;          /* The master context */
    struct list_head dequed_list; /* Dequed buffers */
};

static inline struct mpcam* file_to_mpcam(struct file* file)
{
    struct miscdevice* miscdev = file->private_data;

    return container_of(miscdev, struct mpcam, miscdev);
}

#define file_to_mpcam_file(file) (struct mpcam_file*)(file->private_data)

/*****************************************************************************
 * Helpers for filler
 ****************************************************************************/
extern int mpcam_filler_init(struct mpcam* mpcam);
extern void mpcam_filler_exit(struct mpcam* mpcam);
extern int mpcam_filler_start(struct mpcam* mpcam);
extern int mpcam_filler_stop(struct mpcam* mpcam);

/**
 * mpcam_filler_add_dirty_buffer - Add buffer to dirty list
 * @mpcam: The mpcam context
 * @fb: The buffer to add
 */
extern int mpcam_filler_add_dirty_buffer(struct mpcam* mpcam,
                                         struct mpcam_filler_buffer* fb);

/**
 * mpcam_filler_find_buffer - Find buffer by index
 * @mpcam: The mpcam context.
 * @index: The index of the buffer.
 * @return: The pointer of the buffer or NULL.
 */
extern struct mpcam_filler_buffer* mpcam_filler_find_buffer(struct mpcam* mpcam,
                                                            int index);

extern void mpcam_filler_reclaim_buffers(struct mpcam* mpcam,
                                         struct list_head* list);

static inline u32 mpcam_filler_read_reg(struct mpcam_filler* filler, u32 reg)
{
    return readl(filler->base + reg);
}

static inline void mpcam_filler_write_reg(struct mpcam_filler* filler, u32 reg, u32 val)
{
    writel(val, filler->base + reg);
}

static inline void mpcam_filler_enable_irq(struct mpcam_filler* filler)
{
    mpcam_filler_write_reg(filler, MPCAM_FILLER_ADDR_GIE, 1);
}

static inline void mpcam_filler_disable_irq(struct mpcam_filler* filler)
{
    mpcam_filler_write_reg(filler, MPCAM_FILLER_ADDR_GIE, 0);
}

static inline void mpcam_filler_clear_irq_mask(struct mpcam_filler* filler, u32 mask)
{
    u32 val = mpcam_filler_read_reg(filler, MPCAM_FILLER_ADDR_IER);
    mpcam_filler_write_reg(filler, MPCAM_FILLER_ADDR_IER, val & (~mask));
}

static inline void mpcam_filler_set_irq_mask(struct mpcam_filler* filler, u32 mask)
{
    u32 val = mpcam_filler_read_reg(filler, MPCAM_FILLER_ADDR_IER);
    mpcam_filler_write_reg(filler, MPCAM_FILLER_ADDR_IER, val | mask);
}

static inline void mpcam_filler_clear_irq(struct mpcam_filler* filler, u32 mask)
{
    mpcam_filler_write_reg(filler, MPCAM_FILLER_ADDR_ISR, mask);
}

static inline u32 mpcam_filler_get_irq_status(struct mpcam_filler* filler)
{
    return mpcam_filler_read_reg(filler, MPCAM_FILLER_ADDR_ISR);
}

static inline u32 mpcam_filler_get_frame_buff(struct mpcam_filler* filler)
{
    return mpcam_filler_read_reg(filler, MPCAM_FILLER_ADDR_FRAME_BUFF_DATA);
}

static inline void mpcam_filler_set_frame_buff(struct mpcam_filler* filler, u32 phy_addr)
{
    mpcam_filler_write_reg(filler, MPCAM_FILLER_ADDR_FRAME_BUFF_DATA, phy_addr);
}

static inline void mpcam_filler_start_ap(struct mpcam_filler* filler)
{
    u32 val = mpcam_filler_read_reg(filler, MPCAM_FILLER_ADDR_AP_CTRL) & 0x80;
    mpcam_filler_write_reg(filler, MPCAM_FILLER_ADDR_AP_CTRL, val | 0x01);
}

static inline void mpcam_filler_stop_ap(struct mpcam_filler* filler)
{
    u32 val = mpcam_filler_read_reg(filler, MPCAM_FILLER_ADDR_AP_CTRL) & 0x80;
    mpcam_filler_write_reg(filler, MPCAM_FILLER_ADDR_AP_CTRL, val & (~0x01));
}

static inline void mpcam_filler_enable_auto_restart(struct mpcam_filler* filler)
{
    mpcam_filler_write_reg(filler, MPCAM_FILLER_ADDR_AP_CTRL, 0x80);
}

static inline void mpcam_filler_disable_auto_restart(struct mpcam_filler* filler)
{
    mpcam_filler_write_reg(filler, MPCAM_FILLER_ADDR_AP_CTRL, 0);
}

static inline u32 mpcam_filler_is_done(struct mpcam_filler* filler)
{
    u32 val = mpcam_filler_read_reg(filler, MPCAM_FILLER_ADDR_AP_CTRL);
    return (val >> 1) & 0x01;
}

/*****************************************************************************
 * File Operations
 ****************************************************************************/
extern int mpcam_fops_open(struct inode* inode, struct file* file);
extern int mpcam_fops_release(struct inode* inode, struct file* file);
extern unsigned int mpcam_fops_poll(struct file* file, poll_table* wait);
extern long mpcam_fops_ioctl(struct file* file, unsigned int cmd,
                             unsigned long arg);
extern int mpcam_fops_mmap(struct file* file, struct vm_area_struct* vma);

#endif /* MPCAP_H */
