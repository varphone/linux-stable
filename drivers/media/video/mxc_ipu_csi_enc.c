/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file ipu_csi_enc_camsoc.c
 *
 * @brief CSI Use case for video capture
 *
 * @ingroup IPU
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/ipu.h>
#include <mach/mipi_csi2.h>
#include "mxc_camera.h"

#ifdef CAMERA_DBG
	#define CAMERA_TRACE(x) (printk)x
#else
	#define CAMERA_TRACE(x)
#endif

/*
 * Function definitions
 */


struct csi_enc_data {
	struct mxc_v4l_frame dummy_frame;
};


#define mxc_cam_to_enc_data(cam) ((struct csi_enc_data *)cam->enc_private)


/*!
 * csi ENC callback function.
 *
 * @param irq       int irq line
 * @param dev_id    void * device id
 *
 * @return status   IRQ_HANDLED for handled
 */
static irqreturn_t csi_enc_callback(int irq, void *dev_id)
{
	struct mxc_camera_dev *cam = (struct mxc_camera_dev *) dev_id;

	if (cam->enc_callback == NULL)
		return IRQ_HANDLED;

	cam->enc_callback(irq, dev_id);
	return IRQ_HANDLED;
}

/*!
 * CSI ENC enable channel setup function
 *
 * @param cam       struct struct mxc_camera_dev * mxc capture instance
 *
 * @return  status
 */
static int csi_enc_setup(struct mxc_camera_dev *cam)
{
	ipu_channel_params_t params;
	u32 pixel_fmt;
	int err = 0, sensor_protocol = 0;
	struct csi_enc_data *enc_data = mxc_cam_to_enc_data(cam);
	struct soc_camera_device *icd = cam->icd;
	__u32 host_pixelfmt = icd->current_fmt->host_fmt->fourcc;
	dma_addr_t dummy = enc_data->dummy_frame.buffer.m.offset;

	CAMERA_TRACE("In csi_enc_setup\n");
	if (!cam) {
		printk(KERN_ERR "cam private is NULL\n");
		return -ENXIO;
	}

	memset(&params, 0, sizeof(ipu_channel_params_t));
	params.csi_mem.csi = cam->csi;

	sensor_protocol = ipu_csi_get_sensor_protocol(cam->ipu, cam->csi);
	switch (sensor_protocol) {
	case IPU_CSI_CLK_MODE_GATED_CLK:
	case IPU_CSI_CLK_MODE_NONGATED_CLK:
	case IPU_CSI_CLK_MODE_CCIR656_PROGRESSIVE:
	case IPU_CSI_CLK_MODE_CCIR1120_PROGRESSIVE_DDR:
	case IPU_CSI_CLK_MODE_CCIR1120_PROGRESSIVE_SDR:
		params.csi_mem.interlaced = false;
		break;
	case IPU_CSI_CLK_MODE_CCIR656_INTERLACED:
	case IPU_CSI_CLK_MODE_CCIR1120_INTERLACED_DDR:
	case IPU_CSI_CLK_MODE_CCIR1120_INTERLACED_SDR:
		params.csi_mem.interlaced = true;
		break;
	default:
		printk(KERN_ERR "sensor protocol unsupported\n");
		return -EINVAL;
	}

	pixel_fmt = fourcc_to_ipu_pixfmt(host_pixelfmt);

	ipu_csi_enable_mclk_if(cam->ipu, CSI_MCLK_ENC, cam->csi, true, true);

	err = ipu_init_channel(cam->ipu, CSI_MEM, &params);
	if (err != 0) {
		printk(KERN_ERR "ipu_init_channel %d\n", err);
		return err;
	}

	err = ipu_init_channel_buffer(cam->ipu, CSI_MEM, IPU_OUTPUT_BUFFER,
				      pixel_fmt, icd->user_width,
				      icd->user_height,
				      icd->user_width, IPU_ROTATE_NONE,
				      dummy, dummy, 0,
				      0,
				      0);
	if (err != 0) {
		printk(KERN_ERR "CSI_MEM output buffer\n");
		return err;
	}
	err = ipu_enable_channel(cam->ipu, CSI_MEM);
	if (err < 0) {
		printk(KERN_ERR "ipu_enable_channel CSI_MEM\n");
		return err;
	}

	return err;
}

/*!
 * function to update physical buffer address for encorder IDMA channel
 *
 * @param eba         physical buffer address for encorder IDMA channel
 * @param buffer_num  int buffer 0 or buffer 1
 *
 * @return  status
 */
static int csi_enc_eba_update(struct ipu_soc *ipu, dma_addr_t eba, int *buffer_num)
{
	int err = 0;

	pr_debug("eba %x\n", eba);
	err = ipu_update_channel_buffer(ipu, CSI_MEM, IPU_OUTPUT_BUFFER,
					*buffer_num, eba);
	if (err != 0) {
		ipu_clear_buffer_ready(ipu, CSI_MEM, IPU_OUTPUT_BUFFER,
				       *buffer_num);

		err = ipu_update_channel_buffer(ipu, CSI_MEM, IPU_OUTPUT_BUFFER,
						*buffer_num, eba);
		if (err != 0) {
			pr_err("ERROR: v4l2 capture: fail to update "
			       "buf%d\n", *buffer_num);
			return err;
		}
	}

	ipu_select_buffer(ipu, CSI_MEM, IPU_OUTPUT_BUFFER, *buffer_num);

	*buffer_num = (*buffer_num == 0) ? 1 : 0;

	return 0;
}

/*!
 * Enable encoder task
 * @param private       struct struct mxc_camera_dev * mxc capture instance
 *
 * @return  status
 */
static int csi_enc_enabling_tasks(struct mxc_camera_dev *cam)
{
	int err = 0;
	struct csi_enc_data *enc_data = mxc_cam_to_enc_data(cam);

	CAMERA_TRACE("IPU:In csi_enc_enabling_tasks\n");

	enc_data->dummy_frame.vaddress = dma_alloc_coherent(0,
			       PAGE_ALIGN(cam->icd->sizeimage),
			       &enc_data->dummy_frame.paddress,
			       GFP_DMA | GFP_KERNEL);
	if (enc_data->dummy_frame.vaddress == 0) {
		pr_err("ERROR: v4l2 capture: Allocate dummy frame "
		       "failed.\n");
		return -ENOBUFS;
	}
	enc_data->dummy_frame.buffer.type = V4L2_BUF_TYPE_PRIVATE;
	enc_data->dummy_frame.buffer.length =
	    PAGE_ALIGN(cam->icd->sizeimage);
	enc_data->dummy_frame.buffer.m.offset = enc_data->dummy_frame.paddress;

	ipu_clear_irq(cam->ipu, IPU_IRQ_CSI0_OUT_EOF);
	err = ipu_request_irq(cam->ipu, IPU_IRQ_CSI0_OUT_EOF,
			      csi_enc_callback, 0, "Mxc Camera", cam);
	if (err != 0) {
		printk(KERN_ERR "Error registering rot irq\n");
		return err;
	}

	err = csi_enc_setup(cam);
	if (err != 0) {
		printk(KERN_ERR "csi_enc_setup %d\n", err);
		return err;
	}

	return err;
}

/*!
 * Disable encoder task
 * @param private       struct struct mxc_camera_dev * mxc capture instance
 *
 * @return  int
 */
static int csi_enc_disabling_tasks(struct mxc_camera_dev *cam)
{
	int err = 0;
	struct csi_enc_data *enc_data = mxc_cam_to_enc_data(cam);

	ipu_free_irq(cam->ipu, IPU_IRQ_CSI0_OUT_EOF, cam);

	err = ipu_disable_channel(cam->ipu, CSI_MEM, true);

	ipu_uninit_channel(cam->ipu, CSI_MEM);

	if (enc_data->dummy_frame.vaddress != 0) {
		dma_free_coherent(0, enc_data->dummy_frame.buffer.length,
				  enc_data->dummy_frame.vaddress,
				  enc_data->dummy_frame.paddress);
		enc_data->dummy_frame.vaddress = 0;
	}

	ipu_csi_enable_mclk_if(cam->ipu, CSI_MCLK_ENC, cam->csi, false, false);

	return err;
}

/*!
 * Enable csi
 * @param private       struct struct mxc_camera_dev * mxc capture instance
 *
 * @return  status
 */
static int csi_enc_enable_csi(struct mxc_camera_dev *cam)
{
	return ipu_enable_csi(cam->ipu, cam->csi);
}

/*!
 * Disable csi
 * @param private       struct struct mxc_camera_dev * mxc capture instance
 *
 * @return  status
 */
static int csi_enc_disable_csi(struct mxc_camera_dev *cam)
{
	return ipu_disable_csi(cam->ipu, cam->csi);
}

static struct mxc_cam_enc_ops csi_enc_ops = {
	.enc_update_eba		= csi_enc_eba_update,
	.enc_enable		= csi_enc_enabling_tasks,
	.enc_disable		= csi_enc_disabling_tasks,
	.enc_enable_csi		= csi_enc_enable_csi,
	.enc_disable_csi	= csi_enc_disable_csi,
};


/*!
 * function to select CSI ENC as the working path
 *
 * @param cam       struct struct mxc_camera_dev * mxc capture instance
 *
 * @return  int
 */
int csi_enc_select(struct mxc_camera_dev *cam)
{
	struct csi_enc_data *enc_data;

	enc_data = kmalloc(GFP_KERNEL, sizeof(*enc_data));
	if (!enc_data) {
		dev_err(cam->icd->dev.parent, "Fail to allocate csi_enc private data\n");
		return -ENOMEM;
	}

	cam->enc_ops = &csi_enc_ops;
	cam->enc_private = enc_data;

	return 0;
}

/*!
 * function to de-select CSI ENC as the working path
 *
 * @param cam 	struct struct mxc_camera_dev * mxc capture instance
 *
 * @return  int
 */
int csi_enc_deselect(struct mxc_camera_dev *cam)
{
	if (cam->enc_private) {
		kfree(cam->enc_private);
		cam->enc_private = NULL;
	}

	cam->enc_ops = NULL;

	return 0;
}

/*!
 * Init the Encorder channels
 *
 * @return  Error code indicating success or failure
 */
__init int csi_enc_init(void)
{
	return 0;
}

/*!
 * Deinit the Encorder channels
 *
 */
void __exit csi_enc_exit(void)
{
}

module_init(csi_enc_init);
module_exit(csi_enc_exit);

EXPORT_SYMBOL(csi_enc_select);
EXPORT_SYMBOL(csi_enc_deselect);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("CSI ENC Driver");
MODULE_LICENSE("GPL");
