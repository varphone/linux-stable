#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>

#define DRV_NAME "hisi-gpio"
/*
#define  GPIO_0_BASE_ADDR  0x20150000
#define  GPIO_1_BASE_ADDR  0x20160000
#define  GPIO_2_BASE_ADDR  0x20170000
#define  GPIO_3_BASE_ADDR  0x20180000
#define  GPIO_4_BASE_ADDR  0x20190000
#define  GPIO_5_BASE_ADDR  0x201a0000
#define  GPIO_6_BASE_ADDR  0x201b0000
#define  GPIO_7_BASE_ADDR  0x201c0000
#define  GPIO_8_BASE_ADDR  0x201d0000
 changed for 3531
#define  GPIO_9_BASE_ADDR  0x201e0000
#define  GPIO_10_BASE_ADDR  0x201f0000
#define  GPIO_11_BASE_ADDR  0x20200000
#define  GPIO_12_BASE_ADDR  0x20210000
#define  GPIO_13_BASE_ADDR  0x20220000
#define  GPIO_14_BASE_ADDR  0x20230000
#define  GPIO_15_BASE_ADDR  0x20240000
#define  GPIO_16_BASE_ADDR  0x20250000
#define  GPIO_17_BASE_ADDR  0x20260000
#define  GPIO_18_BASE_ADDR  0x20270000


#define  GPIO_DIR_BASE   (groupbase+0x400)
#define  GPIO_INTR_MASK  (groupbase+0x410)
#define  GPIO_DATA_BASE   data_reg_base


#define WRITE_REG(Addr, Value) ((*(volatile unsigned int *)(Addr)) = (Value))
#define READ_REG(Addr)         (*(volatile unsigned int *)(Addr))
*/

/* pin mux ctrl:
 * GPIO12_3 - GPIO12_4: PAD 8
 */

#define HISI_GPIO_MAX_CHIPS	19
#define HISI_GPIO_MUXCTRL_BASE	0x200F0000
#define HISI_GPIO_MUXCTRL_SIZE	0x10000
#define HISI_GPIO_MUXCTRL_GPIO	0x00000001

struct hisi_gpio_chip {
	struct gpio_chip	chip;
	unsigned long		base_virt;
	unsigned int		group_id;
	unsigned int		mux_vals[8];
	char			label[16];
	const char* 		names[9];
	char			name_bufs[8][16];
};

static struct hisi_gpio {
	struct hisi_gpio_chip chips[HISI_GPIO_MAX_CHIPS];
	spinlock_t lock;
} hisi_gpio;

/*
 * Hi3531 GPIO Muxing value:
 *   GPIO = 1 muxctrl_reg0 ~ muxctrl_reg77
 *   GPIO = 0 muxctrl_reg78 ~ muxctrl_reg107
 *   GPIO = 1 muxctrl_reg108
 *   GPIO = 0 muxctrl_reg109 ~ muxctrl_reg125
 *   GPIO = 1 muxctrl_reg126 ~ muxctrl_reg140
 *   GPIO = 0 muxctrl_reg141 ~ muxctrl_reg150
 *
 * Hi3531 Muxctrl reg offset:
 *   0x000 muxctrl_reg0
 *   0x134 muxctrl_reg77
 *   0x138 muxctrl_reg78
 *   0x1AC muxctrl_reg107
 *   0x1B0 muxctrl_reg108
 *   0x1B4 muxctrl_reg109
 *   0x1F4 muxctrl_reg125
 *   0x1F8 muxctrl_reg126
 *   0x230 muxctrl_reg140
 *   0x234 muxctrl_reg141
 *   0x258 muxctrl_reg150
 */
static long hisi_gpio_mux_val(unsigned long reg_offset)
{
	if (reg_offset <= 0x134)
		return 1;
	if (reg_offset >= 0x138 && reg_offset <= 0x1AC)
		return 0;
	if (reg_offset == 0x1B0)
		return 1;
	if (reg_offset >= 0x1B4 && reg_offset <= 0x1F4)
		return 0;
	if (reg_offset >= 0x1F8 && reg_offset <= 0x230)
		return 1;
	if (reg_offset >= 0x234 && reg_offset <= 0x258)
		return 0;
	return 0;
}

static int hisi_gpio_mux_setup(struct gpio_chip *chip, unsigned offset)
{
	void* base;
	unsigned long reg;
	struct hisi_gpio_chip *hic = container_of(chip, struct hisi_gpio_chip, chip);
	base = ioremap_nocache(HISI_GPIO_MUXCTRL_BASE, HISI_GPIO_MUXCTRL_SIZE);
	if (base == 0) {
		printk(KERN_ERR "%s: Map MUXCTRL [0x%x-0x%x] failed!\n", DRV_NAME,
			HISI_GPIO_MUXCTRL_BASE,
			HISI_GPIO_MUXCTRL_BASE + HISI_GPIO_MUXCTRL_SIZE);
		return -EIO;
	}
	reg = (unsigned long)base;
	if (hic->group_id >= 12 && offset > 3)
		reg += 0x08;
	reg += (hic->group_id * 8 + offset) * 4;
	hic->mux_vals[offset] = readl(reg);
	writel(hisi_gpio_mux_val(reg - (unsigned long)base), reg);
	iounmap(base);
	return 0;
}

static int hisi_gpio_mux_restore(struct gpio_chip *chip, unsigned offset)
{
	void* base;
	unsigned long reg;
	struct hisi_gpio_chip *hic = container_of(chip, struct hisi_gpio_chip, chip);
	base = ioremap_nocache(HISI_GPIO_MUXCTRL_BASE, HISI_GPIO_MUXCTRL_SIZE);
	if (base == 0) {
		printk(KERN_ERR "%s: Map MUXCTRL [0x%x-0x%x] failed!\n", DRV_NAME,
			HISI_GPIO_MUXCTRL_BASE,
			HISI_GPIO_MUXCTRL_BASE + HISI_GPIO_MUXCTRL_SIZE);
		return -EIO;
	}
	reg = (unsigned long)base;
	if (hic->group_id >= 12 && offset > 3)
		reg += 0x08;
	reg += (hic->group_id * 8 + offset) * 4;
	writel(hic->mux_vals[offset], reg);
	iounmap(base);
	return 0;
}

static int hisi_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	printk(KERN_DEBUG "%s:%d\n", __FILE__, __LINE__);
	hisi_gpio_mux_setup(chip, offset);
	return 0;
}

static void hisi_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	printk(KERN_DEBUG "%s:%d\n", __FILE__, __LINE__);
	hisi_gpio_mux_restore(chip, offset);
}

/*
 * configures signal "offset" as input, or returns error
 */
static int hisi_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	unsigned int reg;
	unsigned int val;
	struct hisi_gpio_chip *hic = container_of(chip, struct hisi_gpio_chip, chip);
	/* config as input */
	reg = hic->base_virt + 0x400;
	val = readl(reg);
	val &= ~(1 << offset);
	writel(val, reg);
	/* read the value */
	reg =  hic->base_virt + (1 << (offset + 2));
	readl(reg);
	return 0;
}

/*
 * returns value for signal "offset"; for output signals this
 *	returns either the value actually sensed, or zero
 */
static int hisi_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	unsigned int reg;
	unsigned int val;
	struct hisi_gpio_chip *hic = container_of(chip, struct hisi_gpio_chip, chip);
	reg = hic->base_virt + (1 << (offset + 2));
	val = readl(reg);
	return val ? 1 : 0;
}

/*
 * configures signal "offset" as output, or returns error
 */
static int hisi_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
					int value)
{
	unsigned int reg;
	unsigned int val;
	struct hisi_gpio_chip *hic = container_of(chip, struct hisi_gpio_chip, chip);
	/* set output bit */
	reg  = hic->base_virt + 0x400;
	val  = readl(reg);
	val |= 1 << offset;
	writel(val, reg);
	/* write the value */
	reg = hic->base_virt + (1 << (offset + 2));
	val = value ? 1 << offset : 0;
	writel(val, reg);
	return 0;
}

/*
 * assigns output value for signal "offset"
 */
static void hisi_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	unsigned int reg;
	unsigned int val;
	struct hisi_gpio_chip *hic = container_of(chip, struct hisi_gpio_chip, chip);
	reg = hic->base_virt + (1 << (offset + 2));
	val = value ? 1 << offset : 0;
	writel(val, reg);
}

static void hisi_gpio_fill_gpio_chip(struct gpio_chip* chip)
{
	chip->owner = THIS_MODULE;
	chip->request = hisi_gpio_request;
	chip->free = hisi_gpio_free;
	chip->direction_input = hisi_gpio_direction_input;
	chip->get = hisi_gpio_get;
	chip->direction_output = hisi_gpio_direction_output;
	chip->set = hisi_gpio_set;

	chip->can_sleep = 0;
	chip->exported = 0;
}

static void hisi_gpio_gen_names(struct hisi_gpio_chip *hic)
{
	int i;
	for (i = 0; i < 8; i++) {
		sprintf(hic->name_bufs[i], "%s_%d", hic->label, i);
		hic->names[i] = hic->name_bufs[i];
	}
	hic->names[i] = NULL;
}

static unsigned int hisi_gpio_map_resource(struct platform_device *pdev, int index)
{
	unsigned int area;
	unsigned int virt;
	struct resource *res;
	res = platform_get_resource(pdev, IORESOURCE_MEM, index);
	area = request_mem_region(res->start, (res->end - res->start) + 1, pdev->name);
	if (area == NULL) {
		dev_err(&pdev->dev, "request memory on [%x,%x] failed!\n", res->start, res->end);
		return 0;
	}
	virt = ioremap_nocache(res->start, (res->end - res->start) + 1);
	if (virt == NULL) {
		release_mem_region(res->start, (res->end - res->start) + 1);
		dev_err(&pdev->dev, "map memory on [%x,%x] failed!\n", res->start, res->end);
		return 0;
	}
	dev_info(&pdev->dev, "map [%x,%x] -> %x\n", res->start, res->end, virt);
	return virt;
}

static void hisi_gpio_unmap_resource(struct platform_device *pdev,
				       int index, unsigned int virt)
{
}

static void hisi_gpio_add_chip(struct platform_device *pdev,
				 struct hisi_gpio_chip* hic,
				 unsigned long virt)
{
	int			err;
	struct gpio_chip	*chip;
	hic->base_virt = virt;
	hic->chip.label = hic->label;
	hic->chip.base = hic->group_id * 8;
	hic->chip.ngpio = 8;
	hic->chip.names = hic->names;
	sprintf(hic->label, "gpio%d", hic->group_id);
	hisi_gpio_gen_names(hic);
	hisi_gpio_fill_gpio_chip(&hic->chip);

	chip = &hic->chip;
	err = gpiochip_add(chip);
	if (err < 0) {
		dev_err(&pdev->dev, "GPIO chip [%s] add failed: %x\n", chip->label, err);
	}

	dev_info(&pdev->dev, "GPIO chip [%s] added.\n", chip->label);

}

static int __devinit hisi_gpio_probe(struct platform_device *pdev)
{
	int 			i;
	int 			err = -EIO;
	unsigned long		virt;
	unsigned long 		virts[8];
	struct hisi_gpio_chip *hic;
	struct gpio_chip	*chip;

	for (i = 0; i < 5; i++)
		virts[i] = hisi_gpio_map_resource(pdev, i);

	virt = virts[0];

	for (i = 0; i < 19; i++) {
		if ((i % 4) == 0)
			virt = virts[i / 4];

		hic = &hisi_gpio.chips[i];
		hic->group_id = i;
		hisi_gpio_add_chip(pdev, &hisi_gpio.chips[i], virt);

		virt += 0x10000;
	}

	dev_info(&pdev->dev, "GPIO support loaded.\n");

	return 0;
error:
	return err;
}

static int __devexit hisi_gpio_remove(struct platform_device *pdev)
{
	int i;
	int err;
	struct gpio_chip *chip;

	for (i = 0; i < HISI_GPIO_MAX_CHIPS; i++) {
		chip = &hisi_gpio.chips[i].chip;
		if (chip->label) {
			err = gpiochip_remove(chip);
			if (err < 0) {
				dev_err(&pdev->dev, "GPIO chip [%s] remove failed: %x\n", chip->label, err);
			}
			dev_info(&pdev->dev, "GPIO chip [%s] remove.\n", chip->label);
		}
	}

	if (hisi_gpio.chips[0].base_virt)
		hisi_gpio_unmap_resource(pdev, 0, hisi_gpio.chips[0].base_virt);

	dev_info(&pdev->dev, "GPIO support removed.\n");
	return 0;
}

static void hisi_gpio_shutdown(struct platform_device *pdev)
{
}

static struct platform_driver hisi_gpio_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = DRV_NAME,
	},
	.probe = hisi_gpio_probe,
	.remove = hisi_gpio_remove,
	.shutdown = hisi_gpio_shutdown,
};

static int __init hisi_gpio_init(void)
{
	return platform_driver_register(&hisi_gpio_driver);
}

static void __exit hisi_gpio_exit(void)
{
	printk(KERN_DEBUG "%s:%d\n", __FILE__, __LINE__);
	platform_driver_unregister(&hisi_gpio_driver);
}

module_init(hisi_gpio_init);
module_exit(hisi_gpio_exit);

MODULE_AUTHOR("Varphone Wong <varphone@qq.com>");
MODULE_DESCRIPTION("Hisilicon 3XXX GPIO driver");
MODULE_LICENSE("GPL");

