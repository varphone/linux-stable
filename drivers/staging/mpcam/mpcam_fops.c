/*
 * mpcam_fops.c - Multiple Path Camera Driver
 *
 * Copyright (C) 2018 NanJing No.55 Research Institute
 *                    Technology Development CO.,LTD
 * Copyright (C) 2018 Varphone Wong <varphone@qq.com>
 *
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mman.h>
#include <linux/module.h>
#include <linux/pfn.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "mpcam.h"

int mpcam_fops_open(struct inode* inode, struct file* file)
{
    struct mpcam_file* mfile;
    struct mpcam* mpcam = file_to_mpcam(file);
    printk(KERN_INFO "mpcam_open() mpcam = %p\n", mpcam);

    mfile = kzalloc(sizeof(*mfile), GFP_KERNEL);
    if (mfile == NULL)
        return -ENOMEM;

    mfile->mpcam = container_of(file->private_data, struct mpcam, miscdev);

    // Initialize the list to save dequed buffers by VIDIOC_DQBUF
    INIT_LIST_HEAD(&mfile->dequed_list);

    // Replace the private data
    file->private_data = mfile;

    return nonseekable_open(inode, file);
}

int mpcam_fops_release(struct inode* inode, struct file* file)
{
    struct mpcam_file* mfile = file_to_mpcam_file(file);
    printk(KERN_INFO "mpcam_release() mpcam = %p\n", mfile);

    mpcam_filler_reclaim_buffers(mfile->mpcam, &mfile->dequed_list);

    kfree(mfile);

    return 0;
}

unsigned int mpcam_fops_poll(struct file* file, poll_table* wait)
{
    struct mpcam_file* mfile = file_to_mpcam_file(file);

    poll_wait(file, &mfile->mpcam->poll_wait, wait);

    return POLLIN | POLLRDNORM;
}

static struct mpcam_filler_buffer*
mpcam_file_find_buffer(struct list_head* list, int index)
{
    struct mpcam_filler_buffer* child;
    struct mpcam_filler_buffer* next;
    list_for_each_entry_safe(child, next, list, node)
    {
        if (child->buf.index == index)
            return child;
    }
    return NULL;
}

static int mpcam_file_deque_buffer(struct mpcam_file* mfile, void __user* argp)
{
    int ret = 0;
    struct mpcam_filler_buffer* fb;

    wait_event_interruptible(mfile->mpcam->poll_wait,
                             !list_empty(&mfile->mpcam->filler_ready_list));
    fb = list_first_entry_or_null(&mfile->mpcam->filler_ready_list,
                                  struct mpcam_filler_buffer, node);
    if (fb == NULL)
        return -ENOBUFS;

    // Take down from list.
    list_del(&fb->node);
    // Copy to user space.
    ret = copy_to_user(argp, &fb->buf, sizeof(fb->buf));
    // Return to free list if error.
    if (ret < 0)
        list_add_tail(&fb->node, &mfile->mpcam->filler_free_list);
    else
        list_add_tail(&fb->node, &mfile->dequed_list);

    return ret;
}

static int mpcam_file_queue_buffer(struct mpcam_file* mfile, void __user* argp)
{
    int ret;
    struct v4l2_buffer vb;
    struct mpcam_filler_buffer* fb;

    ret = copy_from_user(&vb, argp, sizeof(vb));
    if (ret < 0)
        return -EINVAL;

    fb = mpcam_file_find_buffer(&mfile->dequed_list, vb.index);
    if (fb == NULL)
        return -ENOBUFS;

    // Take down from list.
    list_del(&fb->node);

    mpcam_filler_add_dirty_buffer(mfile->mpcam, fb);

    return 0;
}

long mpcam_fops_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
    int ret;
    void __user* argp = (void __user*)arg;
    struct mpcam_file* mfile = file_to_mpcam_file(file);

    switch (cmd) {
    case VIDIOC_QUERYBUF: {
        struct v4l2_buffer vb;
        vb.index = -1;
        vb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        vb.bytesused = 0x04000000; // 64MiB
        vb.flags = 0;
        vb.memory = V4L2_MEMORY_MMAP;
        vb.m.offset = 0x20000000;
        vb.length = 0x04000000; // 64MiB
        ret = copy_to_user(argp, &vb, sizeof(vb));
        break;
    }
    case VIDIOC_DQBUF:
        ret = mpcam_file_deque_buffer(mfile, argp);
        break;

    case VIDIOC_QBUF:
        ret = mpcam_file_queue_buffer(mfile, argp);
        break;

    default:
        ret = -ENOTSUPP;
        break;
    }

    return ret;
}

#ifndef __HAVE_PHYS_MEM_ACCESS_PROT

/*
 * Architectures vary in how they handle caching for addresses
 * outside of main memory.
 *
 */
#ifdef pgprot_noncached
static int uncached_access(struct file* file, phys_addr_t addr)
{
#if defined(CONFIG_IA64)
    /*
     * On ia64, we ignore O_DSYNC because we cannot tolerate memory
     * attribute aliases.
     */
    return !(efi_mem_attributes(addr) & EFI_MEMORY_WB);
#elif defined(CONFIG_MIPS)
    {
        extern int __uncached_access(struct file * file, unsigned long addr);

        return __uncached_access(file, addr);
    }
#else
    /*
     * Accessing memory above the top the kernel knows about or through a
     * file pointer
     * that was marked O_DSYNC will be done non-cached.
     */
    if (file->f_flags & O_DSYNC)
        return 1;
    return addr >= __pa(high_memory);
#endif
}
#endif

static pgprot_t phys_mem_access_prot(struct file* file, unsigned long pfn,
                                     unsigned long size, pgprot_t vma_prot)
{
#ifdef pgprot_noncached
    phys_addr_t offset = pfn << PAGE_SHIFT;

    if (uncached_access(file, offset))
        return pgprot_noncached(vma_prot);
#endif
    return vma_prot;
}
#endif

static const struct vm_operations_struct mpcam_vm_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
    .access = generic_access_phys
#endif
};

int mpcam_fops_mmap(struct file* file, struct vm_area_struct* vma)
{
    size_t size = vma->vm_end - vma->vm_start;
    phys_addr_t offset = (phys_addr_t)vma->vm_pgoff << PAGE_SHIFT;

    /* It's illegal to wrap around the end of the physical address space. */
    if (offset + (phys_addr_t)size - 1 < offset)
        return -EINVAL;

    vma->vm_page_prot =
        phys_mem_access_prot(file, vma->vm_pgoff, size, vma->vm_page_prot);

    vma->vm_ops = &mpcam_vm_ops;

    /* Remap-pfn-range will mark the range VM_IO */
    if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size,
                        vma->vm_page_prot)) {
        return -EAGAIN;
    }

    return 0;
}
