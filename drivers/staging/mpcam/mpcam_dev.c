/*
 * mpcam_dev.c - Multiple Path Camera Driver
 *
 * Copyright (C) 2018 NanJing No.55 Research Institute
 *                    Technology Development CO.,LTD
 * Copyright (C) 2018 Varphone Wong <varphone@qq.com>
 *
 */

#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/types.h>

#include "mpcam.h"

#ifndef devm_of_iomap
/**
 * devm_of_iomap - Managed of_iomap()
 * @dev:	generic device to map range for
 * @np:		the device node whose io range will be mapped
 * @index:	index of the io range
 *
 * Returns a pointer to the mapped memory
 */
void __iomem* devm_of_iomap(struct device* dev, struct device_node* np,
                            int index)
{
    struct resource res;

    if (of_address_to_resource(np, index, &res))
        return NULL;

    return devm_ioremap(dev, res.start, resource_size(&res));
}
#endif /* devm_of_iomap */

static void devm_misc_dereg(struct device* dev, void* res)
{
    misc_deregister(*(struct miscdevice**)res);
}

#ifndef devm_misc_register
/**
 * devm_misc_register - Resource-managed misc_register
 * @dev:	Device to allocate miscdevice for
 * @misc:	Device structure filled by the device driver
 *
 * Managed misc_register. The miscdevice registered with this function is
 * automatically unregistered on driver detach. This function calls
 * misc_register() internally; refer to its documentation for more information.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int devm_misc_register(struct device* dev, struct miscdevice* misc)
{
    struct miscdevice** misc_ptr;
    int ret;

    misc_ptr = devres_alloc(devm_misc_dereg, sizeof(*misc_ptr), GFP_KERNEL);
    if (!misc_ptr)
        return -ENOMEM;

    *misc_ptr = misc;
    ret = misc_register(misc);
    if (!ret)
        devres_add(dev, misc_ptr);
    else
        devres_free(misc_ptr);

    return ret;
}
#endif /* devm_misc_register */

static const struct file_operations mpcam_fops = {
    .owner = THIS_MODULE,
    .open = mpcam_fops_open,
    .release = mpcam_fops_release,
    .mmap = mpcam_fops_mmap,
    .poll = mpcam_fops_poll,
    .unlocked_ioctl = mpcam_fops_ioctl,
};

static int mpcam_probe(struct platform_device* pdev)
{
    struct device_node* np = pdev->dev.of_node;
    struct device_node* child;
    struct mpcam* mpcam;
    int index, ret;

    mpcam = devm_kzalloc(&pdev->dev, sizeof(*mpcam), GFP_KERNEL);
    if (!mpcam)
        return -ENOMEM;

    platform_set_drvdata(pdev, mpcam);

    // Hold the platform device handle
    mpcam->pdev = pdev;

    // Initialize wait queue for poll or VIDIOC_DQBUF
    init_waitqueue_head(&mpcam->poll_wait);

    /* Get Filler Frame Buffers */
    if (of_address_to_resource(np, 0, &mpcam->filler_fb_res) < 0) {
        dev_err(&pdev->dev, "Filler: Frame Buffers does not exists");
        return -ENOENT;
    }

    index = 0;
    for_each_child_of_node(np, child)
    {
        switch (index) {
        case 0:
            mpcam->filler_rd.base = devm_of_iomap(&pdev->dev, child, 0);
            mpcam->filler_rd.irq = of_irq_get(child, 0);
            break;
        case 1:
            mpcam->filler_wr.base = devm_of_iomap(&pdev->dev, child, 0);
            mpcam->filler_wr.irq = of_irq_get(child, 0);
            break;
        }
        index++;
    }

    dev_info(&pdev->dev, "filler_rd base = %p, irq = %d\n",
             mpcam->filler_rd.base, mpcam->filler_rd.irq);
    dev_info(&pdev->dev, "filler_wr base = %p, irq = %d\n",
             mpcam->filler_wr.base, mpcam->filler_wr.irq);

    mpcam->miscdev.parent = &pdev->dev;
    mpcam->miscdev.fops = &mpcam_fops;
    mpcam->miscdev.minor = MISC_DYNAMIC_MINOR;
    mpcam->miscdev.name = "mpcam";

    ret = devm_misc_register(&pdev->dev, &mpcam->miscdev);
    if (ret) {
    }

    mpcam_filler_init(mpcam);

    mpcam_filler_start(mpcam);

    return 0;
}

static int mpcam_remove(struct platform_device* pdev)
{
    struct mpcam* mpcam = (struct mpcam*)platform_get_drvdata(pdev);

    mpcam_filler_exit(mpcam);

    return 0;
}

// clang-format off
static const struct platform_device_id mpcam_ids[] = {
    { .name = "mpcam", },
    {}
};

MODULE_DEVICE_TABLE(platform, mpcam_ids);

static const struct of_device_id mpcam_dt_ids[] = {
    { .compatible = "tdc,mpcam", },
    {}
};

MODULE_DEVICE_TABLE(of, mpcam_dt_ids);

static struct platform_driver mpcam_driver = {
    .driver =
        {
            .name = "mpcam",
            .of_match_table = mpcam_dt_ids,
        },
    .probe = mpcam_probe,
    .remove = mpcam_remove,
    .id_table = mpcam_ids,
};
// clang-format on

static int __init mpcam_init(void)
{
    return platform_driver_register(&mpcam_driver);
}

static void __exit mpcam_exit(void)
{
    platform_driver_unregister(&mpcam_driver);
}

module_init(mpcam_init);
module_exit(mpcam_exit);

MODULE_AUTHOR("Varphone Wong <varphone@qq.com>");
MODULE_DESCRIPTION("Multiple Path Camera");
MODULE_LICENSE("GPL v2");
