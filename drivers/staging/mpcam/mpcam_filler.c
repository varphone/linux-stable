/*
 * mpcam_iller.c - Multiple Path Camera Driver
 *
 * Copyright (C) 2018 NanJing No.55 Research Institute
 *                    Technology Development CO.,LTD
 * Copyright (C) 2018 Varphone Wong <varphone@qq.com>
 *
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>

#include "mpcam.h"

static inline struct mpcam_filler_buffer*
buffer_pop_front(struct list_head* list)
{
    struct mpcam_filler_buffer* buf = NULL;

    buf = list_first_entry_or_null(list, struct mpcam_filler_buffer, node);
    if (buf)
        list_del(&buf->node);

    return buf;
}

static inline int buffer_push_back(struct list_head* list,
                                   struct mpcam_filler_buffer* buf)
{
    list_add_tail(&buf->node, list);
    return 0;
}

static irqreturn_t mpcam_filler_rd_intr(int in_irq, void* d)
{
    u32 phy;
    struct mpcam* mpcam = d;
    struct mpcam_filler_buffer* buf;

    mpcam_filler_clear_irq(&mpcam->filler_rd, 0x01);
    mpcam_filler_disable_irq(&mpcam->filler_rd);

    buf = buffer_pop_front(&mpcam->filler_dirty_list);
    if (buf) {
        buffer_push_back(&mpcam->filler_free_list, mpcam->filler_rd_buf);
        phy = mpcam->filler_fb_res.start + buf->buf.m.offset;
        mpcam_filler_set_frame_buff(&mpcam->filler_rd, phy);
        mpcam->filler_rd_buf = buf;
    }

    mpcam_filler_enable_irq(&mpcam->filler_rd);
    mpcam_filler_start_ap(&mpcam->filler_rd);

    return IRQ_HANDLED;
}

static irqreturn_t mpcam_filler_wr_intr(int in_irq, void* d)
{
    u32 phy;
    struct mpcam* mpcam = d;
    struct mpcam_filler_buffer* buf;

    mpcam_filler_clear_irq(&mpcam->filler_wr, 0x01);
    mpcam_filler_disable_irq(&mpcam->filler_wr);

    buf = buffer_pop_front(&mpcam->filler_free_list);
    if (buf) {
        buffer_push_back(&mpcam->filler_ready_list, mpcam->filler_wr_buf);
        phy = mpcam->filler_fb_res.start + buf->buf.m.offset;
        mpcam_filler_set_frame_buff(&mpcam->filler_wr, phy);
        mpcam->filler_wr_buf = buf;
        wake_up_interruptible(&mpcam->poll_wait);
    }

    mpcam_filler_enable_irq(&mpcam->filler_wr);
    mpcam_filler_start_ap(&mpcam->filler_wr);

    return IRQ_HANDLED;
}

static int mpcam_filler_init_buffers(struct mpcam* mpcam)
{
    int i = 0;
    u32 size, offset;
    struct mpcam_filler_buffer* list =
        kzalloc(sizeof(*list) * MPCAM_FILLER_MAX_BUFFERS, GFP_KERNEL);

    if (!list)
        return -ENOMEM;

    INIT_LIST_HEAD(&mpcam->filler_dirty_list);
    INIT_LIST_HEAD(&mpcam->filler_free_list);
    INIT_LIST_HEAD(&mpcam->filler_ready_list);

    size = 720 * 576;
    offset = 0;

    for (i = 0; i < MPCAM_FILLER_MAX_BUFFERS; i++) {
        // Fill v4l2_buffer.
        struct v4l2_buffer* vb = &list[i].buf;
        vb->index = i;
        vb->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        vb->bytesused = size;
        vb->flags = 0;
        vb->memory = V4L2_MEMORY_MMAP;
        vb->m.offset = offset;
        vb->length = size;
        // Initialzie the list.
        INIT_LIST_HEAD(&(list[i].node));
        // Add items to free list.
        list_add_tail(&(list[i].node), &mpcam->filler_free_list);
        // Move to next position.
        offset += size;
    }

    mpcam->filler_buffers = list;
    mpcam->filler_rd_buf = NULL;
    mpcam->filler_wr_buf = NULL;

    printk(KERN_INFO "initialize buffer done!");

    return 0;
}

int mpcam_filler_init(struct mpcam* mpcam)
{
    int ret;

    if (mpcam->filler_fb_res.start == 0) {
        mpcam->filler_fb_res.start = 0x20000000;
        mpcam->filler_fb_res.end = 0x23FFFFFF;
    }

    ret = devm_request_irq(
        &mpcam->pdev->dev, mpcam->filler_rd.irq, mpcam_filler_rd_intr,
        irq_get_trigger_type(mpcam->filler_rd.irq), "filler_rd", mpcam);

    dev_info(&mpcam->pdev->dev, "IRQ %d requested ? %d\n", mpcam->filler_rd.irq,
             ret);

    ret = devm_request_irq(
        &mpcam->pdev->dev, mpcam->filler_wr.irq, mpcam_filler_wr_intr,
        irq_get_trigger_type(mpcam->filler_wr.irq), "filler_wr", mpcam);

    dev_info(&mpcam->pdev->dev, "IRQ %d requested ? %d\n", mpcam->filler_wr.irq,
             ret);

    ret = mpcam_filler_init_buffers(mpcam);

    return ret;
}

void mpcam_filler_exit(struct mpcam* mpcam)
{
    mpcam_filler_stop(mpcam);
    kfree(mpcam->filler_buffers);
}

int mpcam_filler_start(struct mpcam* mpcam)
{
    u32 phy;

    mpcam->filler_rd_buf = buffer_pop_front(&mpcam->filler_free_list);
    phy = mpcam->filler_fb_res.start + mpcam->filler_rd_buf->buf.m.offset;

    mpcam_filler_set_frame_buff(&mpcam->filler_rd, phy);
    mpcam_filler_disable_auto_restart(&mpcam->filler_rd);
    mpcam_filler_set_irq_mask(&mpcam->filler_rd, 0x01);
    mpcam_filler_enable_irq(&mpcam->filler_rd);
    mpcam_filler_start_ap(&mpcam->filler_rd);

    mpcam->filler_wr_buf = buffer_pop_front(&mpcam->filler_free_list);
    phy = mpcam->filler_fb_res.start + mpcam->filler_wr_buf->buf.m.offset;

    mpcam_filler_set_frame_buff(&mpcam->filler_wr, phy);
    mpcam_filler_disable_auto_restart(&mpcam->filler_wr);
    mpcam_filler_set_irq_mask(&mpcam->filler_wr, 0x01);
    mpcam_filler_enable_irq(&mpcam->filler_wr);
    mpcam_filler_start_ap(&mpcam->filler_wr);

    return 0;
}

int mpcam_filler_stop(struct mpcam* mpcam)
{
    mpcam_filler_clear_irq(&mpcam->filler_wr, 0x01);
    mpcam_filler_disable_irq(&mpcam->filler_wr);
    mpcam_filler_clear_irq(&mpcam->filler_rd, 0x01);
    mpcam_filler_disable_irq(&mpcam->filler_rd);
    return 0;
}

int mpcam_filler_add_dirty_buffer(struct mpcam* mpcam,
                                  struct mpcam_filler_buffer* fb)
{
    return buffer_push_back(&mpcam->filler_dirty_list, fb);
}

struct mpcam_filler_buffer* mpcam_filler_find_buffer(struct mpcam* mpcam,
                                                     int index)
{
    if (index < 0 || index >= MPCAM_FILLER_MAX_BUFFERS)
        return NULL;
    return &mpcam->filler_buffers[index];
}

void mpcam_filler_reclaim_buffers(struct mpcam* mpcam, struct list_head* list)
{
    struct mpcam_filler_buffer* child;
    struct mpcam_filler_buffer* next;
    list_for_each_entry_safe(child, next, list, node)
    {
        (void)buffer_push_back(&mpcam->filler_free_list, child);
    }
}