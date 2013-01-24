
#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/videodev2.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/sched.h>

#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/soc_camera.h>
#include <media/soc_mediabus.h>

#include "mxc_camera.h"

#define MXC_CAM_DRV_NAME	"mxc-camera"

u32 fourcc_to_ipu_pixfmt(u32 fourcc)
{
	u32 ipu_pixfmt;

	switch (fourcc) {

	case V4L2_PIX_FMT_YUV420:
		ipu_pixfmt = IPU_PIX_FMT_YUV420P;
		break;
	case V4L2_PIX_FMT_YUV422P:
		ipu_pixfmt = IPU_PIX_FMT_YUV422P;
		break;
	case V4L2_PIX_FMT_UYVY:
		ipu_pixfmt = IPU_PIX_FMT_UYVY;
		break;
	case V4L2_PIX_FMT_YUYV:
		ipu_pixfmt = IPU_PIX_FMT_YUYV;
		break;
	case V4L2_PIX_FMT_NV12:
		ipu_pixfmt = IPU_PIX_FMT_NV12;
		break;
	case V4L2_PIX_FMT_BGR24:
		ipu_pixfmt = IPU_PIX_FMT_BGR24;
		break;
	case V4L2_PIX_FMT_RGB24:
		ipu_pixfmt = IPU_PIX_FMT_RGB24;
		break;
	case V4L2_PIX_FMT_RGB565:
		ipu_pixfmt = IPU_PIX_FMT_RGB565;
		break;
	case V4L2_PIX_FMT_BGR32:
		ipu_pixfmt = IPU_PIX_FMT_BGR32;
		break;
	case V4L2_PIX_FMT_RGB32:
		ipu_pixfmt = IPU_PIX_FMT_RGB32;
		break;
	case V4L2_PIX_FMT_GREY:
		ipu_pixfmt = IPU_PIX_FMT_GREY;
		break;
	case V4L2_PIX_FMT_Y10:
		ipu_pixfmt = IPU_PIX_FMT_Y16;
		break;
	default:
		printk(KERN_ERR "Format is not supported, use generic\n");
		ipu_pixfmt = IPU_PIX_FMT_GENERIC;
	}

	return ipu_pixfmt;
}
EXPORT_SYMBOL_GPL(fourcc_to_ipu_pixfmt);


static struct mxc_camera_buffer *to_mxc_vb(struct vb2_buffer *vb)
{
	return container_of(vb, struct mxc_camera_buffer, vb);
}

static void mxc_cam_enc_callback(u32 mask, struct mxc_camera_dev *mxc_cam)
{

	dev_dbg(mxc_cam->icd->dev.parent, "callback, active DMA 0x%08x\n",
		mxc_cam->active ? sg_dma_address(&mxc_cam->active->sg) : 0);

	spin_lock(&mxc_cam->lock);
	if (mxc_cam->active) {
		struct vb2_buffer *vb = &mxc_cam->active->vb;
		struct mxc_camera_buffer *buf = to_mxc_vb(vb);

		list_del_init(&buf->queue);
		do_gettimeofday(&vb->v4l2_buf.timestamp);
		vb->v4l2_buf.field = mxc_cam->field;
		vb->v4l2_buf.sequence = mxc_cam->sequence++;
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
	}

	if (list_empty(&mxc_cam->capture)) {
		mxc_cam->active = NULL;
		spin_unlock(&mxc_cam->lock);

		/*
		 * stop capture - without further buffers IPU_CHA_BUF0_RDY will
		 * not get updated
		 */
		return;
	}

	mxc_cam->active = list_entry(mxc_cam->capture.next,
				     struct mxc_camera_buffer, queue);
	spin_unlock(&mxc_cam->lock);
}


static void mxc_camera_activate(struct mxc_camera_dev *mxc_cam,
				struct soc_camera_device *icd)
{
	u32 conf;
	long rate;
	ipu_csi_signal_cfg_t csi_param;

	csi_param.data_width = 0;
	csi_param.clk_mode = IPU_CSI_CLK_MODE_GATED_CLK;
	csi_param.ext_vsync = 0;
	csi_param.Vsync_pol = 0;
	csi_param.Hsync_pol = 0;
	csi_param.pixclk_pol = 0;
	csi_param.data_pol = 0;
	csi_param.sens_clksrc = 0;
	csi_param.pack_tight = 0;
	csi_param.force_eof = 0;
	csi_param.data_en_pol = 0;
	csi_param.data_fmt = 0;
	csi_param.csi = mxc_cam->csi;
	csi_param.mclk = 0;

	if (mxc_cam->platform_flags & MXC_CAMERA_DATAWIDTH_16)
		csi_param.data_width = IPU_CSI_DATA_WIDTH_16;
	else if (mxc_cam->platform_flags & MXC_CAMERA_DATAWIDTH_15)
		csi_param.data_width = IPU_CSI_DATA_WIDTH_15;
	else if (mxc_cam->platform_flags & MXC_CAMERA_DATAWIDTH_14)
		csi_param.data_width = IPU_CSI_DATA_WIDTH_14;
	else if (mxc_cam->platform_flags & MXC_CAMERA_DATAWIDTH_13)
		csi_param.data_width = IPU_CSI_DATA_WIDTH_13;
	else if (mxc_cam->platform_flags & MXC_CAMERA_DATAWIDTH_12)
		csi_param.data_width = IPU_CSI_DATA_WIDTH_12;
	else if (mxc_cam->platform_flags & MXC_CAMERA_DATAWIDTH_11)
		csi_param.data_width = IPU_CSI_DATA_WIDTH_11;
	else if (mxc_cam->platform_flags & MXC_CAMERA_DATAWIDTH_10)
		csi_param.data_width = IPU_CSI_DATA_WIDTH_10;
	else if (mxc_cam->platform_flags & MXC_CAMERA_DATAWIDTH_9)
		csi_param.data_width = IPU_CSI_DATA_WIDTH_9;
	else if (mxc_cam->platform_flags & MXC_CAMERA_DATAWIDTH_8)
		csi_param.data_width = IPU_CSI_DATA_WIDTH_8;
	else if (mxc_cam->platform_flags & MXC_CAMERA_DATAWIDTH_4)
		csi_param.data_width = IPU_CSI_DATA_WIDTH_4;

	if (mxc_cam->platform_flags & MXC_CAMERA_CLK_SRC)
		csi_param.sens_clksrc = 1;
	if (mxc_cam->platform_flags & MXC_CAMERA_EXT_VSYNC)
		csi_param.ext_vsync = 1;
	if (mxc_cam->platform_flags & MXC_CAMERA_DP)
		csi_param.data_pol = 1;
	if (mxc_cam->platform_flags & MXC_CAMERA_PCP)
		csi_param.pixclk_pol = 1;
	if (mxc_cam->platform_flags & MXC_CAMERA_HSP)
		csi_param.Hsync_pol = 1;
	if (mxc_cam->platform_flags & MXC_CAMERA_VSP)
		csi_param.Vsync_pol = 1;

	ipu_csi_init_interface(mxc_cam->ipu, 640,
			       480,
			       IPU_PIX_FMT_RGB565, csi_param);
}

/*
 * Videobuf operations
 */

/*
 * Calculate the __buffer__ (not data) size and number of buffers.
 */
static int mxc_videobuf_setup(struct vb2_queue *vq,
			unsigned int *count, unsigned int *num_planes,
			unsigned long sizes[], void *alloc_ctxs[])
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mxc_camera_dev *mxc_cam = ici->priv;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);

	if (bytes_per_line < 0)
		return bytes_per_line;

	*num_planes = 1;

	mxc_cam->sequence = 0;
	sizes[0] = bytes_per_line * icd->user_height;
	alloc_ctxs[0] = mxc_cam->alloc_ctx;

	if (!*count)
		*count = 32;

	if (sizes[0] * *count > MAX_VIDEO_MEM * 1024 * 1024)
		*count = MAX_VIDEO_MEM * 1024 * 1024 / sizes[0];

	return 0;
}

static int mxc_videobuf_prepare(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mxc_camera_dev *mxc_cam = ici->priv;
	struct scatterlist *sg;
	struct mxc_camera_buffer *buf;
	size_t new_size;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);

	if (bytes_per_line < 0)
		return bytes_per_line;

	buf = to_mxc_vb(vb);
	sg = &buf->sg;

	new_size = bytes_per_line * icd->user_height;

	if (vb2_plane_size(vb, 0) < new_size) {
		dev_err(icd->dev.parent, "Buffer too small (%lu < %zu)\n",
			vb2_plane_size(vb, 0), new_size);
		return -ENOBUFS;
	}

	if (buf->state == CSI_BUF_NEEDS_INIT) {
		sg_dma_address(sg)	= vb2_dma_contig_plane_paddr(vb, 0);
		sg_dma_len(sg)		= new_size;

		buf->state = CSI_BUF_PREPARED;
	}

	vb2_set_plane_payload(vb, 0, new_size);

	return 0;
}

static void mxc_videobuf_queue(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mxc_camera_dev *mxc_cam = ici->priv;
	struct mxc_camera_buffer *buf = to_mxc_vb(vb);
	u32 fourcc = icd->current_fmt->host_fmt->fourcc;
	unsigned long flags;
	int ret = 0;

	/* TODO */
#if 0
	/* This is the configuration of one sg-element */
	video->out_pixel_fmt	= fourcc_to_ipu_pixfmt(fourcc);

	if (video->out_pixel_fmt == IPU_PIX_FMT_GENERIC) {
		/*
		 * If the IPU DMA channel is configured to transport
		 * generic 8-bit data, we have to set up correctly the
		 * geometry parameters upon the current pixel format.
		 * So, since the DMA horizontal parameters are expressed
		 * in bytes not pixels, convert these in the right unit.
		 */
		int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						icd->current_fmt->host_fmt);
		BUG_ON(bytes_per_line <= 0);

		video->out_width	= bytes_per_line;
		video->out_height	= icd->user_height;
		video->out_stride	= bytes_per_line;
	} else {
		/*
		 * For IPU known formats the pixel unit will be managed
		 * successfully by the IPU code
		 */
		video->out_width	= icd->user_width;
		video->out_height	= icd->user_height;
		video->out_stride	= icd->user_width;
	}
#endif
#ifdef DEBUG
	/* helps to see what DMA actually has written */
	if (vb2_plane_vaddr(vb, 0))
		memset(vb2_plane_vaddr(vb, 0), 0xaa, vb2_get_plane_payload(vb, 0));
#endif

	spin_lock_irqsave(&mxc_cam->lock, flags);
	list_add_tail(&buf->queue, &mxc_cam->capture);

	if (!mxc_cam->active)
		mxc_cam->active = buf;

	spin_unlock_irq(&mxc_cam->lock);

	ret = mxc_cam->enc_ops->enc_update_eba(mxc_cam->ipu, sg_dma_address(&buf->sg),
			&mxc_cam->csi_buf_num);

	dev_dbg(icd->dev.parent, "Submitted DMA 0x%08x\n",
		sg_dma_address(&buf->sg));

	if (!ret)
		return;

	spin_lock_irq(&mxc_cam->lock);

	/* Submit error */
	list_del_init(&buf->queue);

	if (mxc_cam->active == buf)
		mxc_cam->active = NULL;

	spin_unlock_irqrestore(&mxc_cam->lock, flags);
	vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
}

static void mxc_videobuf_release(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mxc_camera_dev *mxc_cam = ici->priv;
	struct mxc_camera_buffer *buf = to_mxc_vb(vb);
	unsigned long flags;

	dev_dbg(icd->dev.parent,
		"Release%s DMA 0x%08x, queue %sempty\n",
		mxc_cam->active == buf ? " active" : "", sg_dma_address(&buf->sg),
		list_empty(&buf->queue) ? "" : "not ");

	spin_lock_irqsave(&mxc_cam->lock, flags);

	if (mxc_cam->active == buf)
		mxc_cam->active = NULL;

	/* Doesn't hurt also if the list is empty */
	list_del_init(&buf->queue);
	buf->state = CSI_BUF_NEEDS_INIT;

	spin_unlock_irqrestore(&mxc_cam->lock, flags);
}

static int mxc_videobuf_init(struct vb2_buffer *vb)
{
	struct mxc_camera_buffer *buf = to_mxc_vb(vb);
	/* This is for locking debugging only */
	INIT_LIST_HEAD(&buf->queue);
	sg_init_table(&buf->sg, 1);

	buf->state = CSI_BUF_NEEDS_INIT;

	return 0;
}

static int mxc_stop_streaming(struct vb2_queue *q)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(q);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mxc_camera_dev *mxc_cam = ici->priv;
	struct mxc_camera_buffer *buf, *tmp;
	unsigned long flags;

	mxc_cam->enc_ops->enc_disable_csi(mxc_cam);

	mxc_cam->enc_ops->enc_disable(mxc_cam);

	spin_lock_irqsave(&mxc_cam->lock, flags);

	mxc_cam->active = NULL;

	list_for_each_entry_safe(buf, tmp, &mxc_cam->capture, queue) {
		buf->state = CSI_BUF_NEEDS_INIT;
		list_del_init(&buf->queue);
	}

	spin_unlock_irqrestore(&mxc_cam->lock, flags);

	return 0;
}

static int mxc_start_streaming(struct vb2_queue *q)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(q);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mxc_camera_dev *mxc_cam = ici->priv;
	struct mxc_camera_buffer *buf, *tmp;
	unsigned long flags;
	int err = 0;

	if (mxc_cam->enc_ops->enc_enable) {
		err = mxc_cam->enc_ops->enc_enable(mxc_cam);
		if (err != 0) {
			return err;
		}
	}

	if (mxc_cam->enc_ops->enc_enable_csi) {
		err = mxc_cam->enc_ops->enc_enable_csi(mxc_cam);
		if (err != 0)
			return err;
	}


	return 0;
}

static struct vb2_ops mxc_videobuf_ops = {
	.queue_setup	= mxc_videobuf_setup,
	.buf_prepare	= mxc_videobuf_prepare,
	.buf_queue	= mxc_videobuf_queue,
	.buf_cleanup	= mxc_videobuf_release,
	.buf_init	= mxc_videobuf_init,
	.wait_prepare	= soc_camera_unlock,
	.wait_finish	= soc_camera_lock,
	.start_streaming = mxc_start_streaming,
	.stop_streaming	= mxc_stop_streaming,
};


static int mxc_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mxc_camera_dev *mxc_cam = ici->priv;

	if (mxc_cam->icd)
		return -EBUSY;

	mxc_camera_activate(mxc_cam, icd);

	mxc_cam->icd = icd;

	dev_info(icd->dev.parent, "MXC Camera driver attached to camera %d\n",
		 icd->devnum);

	return 0;
}

static void mxc_camera_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mxc_camera_dev *mxc_cam = ici->priv;

	BUG_ON(icd != mxc_cam->icd);

	mxc_cam->icd = NULL;

	dev_info(icd->dev.parent, "MXC Camera driver detached from camera %d\n",
		 icd->devnum);
}

static void configure_geometry(struct mxc_camera_dev *cam,
			       unsigned int width, unsigned int height,
			       unsigned int left, unsigned int top,
			       const struct soc_mbus_pixelfmt *fmt)
{
	u32 ctrl, width_field, height_field;

	if (fourcc_to_ipu_pixfmt(fmt->fourcc) == IPU_PIX_FMT_GENERIC) {
		/*
		 * As the CSI will be configured to output BAYER, here
		 * the width parameter count the number of samples to
		 * capture to complete the whole image width.
		 */
		unsigned int num, den;
		int ret = soc_mbus_samples_per_pixel(fmt, &num, &den);
		BUG_ON(ret < 0);
		width = width * num / den;
	}

	ipu_csi_set_window_size(cam->ipu, width, height, cam->csi);
	ipu_csi_set_window_pos(cam->ipu, left, top, cam->csi);
}

/*
 * FIXME: learn to use stride != width, then we can keep stride properly aligned
 * and support arbitrary (even) widths.
 */
static inline void stride_align(__u32 *width)
{
	if (ALIGN(*width, 8) < 4096)
		*width = ALIGN(*width, 8);
	else
		*width = *width &  ~7;
}


/*
 * As long as we don't implement host-side cropping and scaling, we can use
 * default g_crop and cropcap from soc_camera.c
 */
static int mxc_camera_set_crop(struct soc_camera_device *icd,
		struct v4l2_crop *a)
{
	struct v4l2_rect *rect = &a->c;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mxc_camera_dev *mxc_cam = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct v4l2_mbus_framefmt mf;
	int ret;

	soc_camera_limit_side(&rect->left, &rect->width, 0, 2, 4096);
	soc_camera_limit_side(&rect->top, &rect->height, 0, 2, 4096);

	ret = v4l2_subdev_call(sd, video, s_crop, a);
	if (ret < 0)
		return ret;

	/* The capture device might have changed its output sizes */
	ret = v4l2_subdev_call(sd, video, g_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	if (mf.code != icd->current_fmt->code)
		return -EINVAL;

	if (mf.width & 7) {
		/* Ouch! We can only handle 8-byte aligned width... */
		stride_align(&mf.width);
		ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);
		if (ret < 0)
			return ret;
	}

	if (mf.width != icd->user_width || mf.height != icd->user_height)
		configure_geometry(mxc_cam, mf.width, mf.height,
				0, 0, icd->current_fmt->host_fmt);

	dev_dbg(icd->dev.parent, "Sensor cropped %dx%d\n",
			mf.width, mf.height);

	icd->user_width		= mf.width;
	icd->user_height	= mf.height;

	return ret;
}

static int mxc_camera_set_fmt(struct soc_camera_device *icd,
			      struct v4l2_format *f)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mxc_camera_dev *mxc_cam = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_warn(icd->dev.parent, "Format %x not found\n",
				pix->pixelformat);
		return -EINVAL;
	}

	stride_align(&pix->width);
	dev_dbg(icd->dev.parent, "Set format %dx%d\n", pix->width, pix->height);

	configure_geometry(mxc_cam, pix->width, pix->height,
			0, 0, xlate->host_fmt);

	mf.width	= pix->width;
	mf.height	= pix->height;
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;
	mf.code		= xlate->code;

	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	if (mf.code != xlate->code)
		return -EINVAL;

	pix->width		= mf.width;
	pix->height		= mf.height;
	pix->field		= mf.field;
	mxc_cam->field		= mf.field;
	pix->colorspace		= mf.colorspace;
	icd->current_fmt	= xlate;

	pix->bytesperline = soc_mbus_bytes_per_line(pix->width,
			xlate->host_fmt);
	if (pix->bytesperline < 0)
		return pix->bytesperline;
	pix->sizeimage = pix->height * pix->bytesperline;

	dev_dbg(icd->dev.parent, "Sensor set %dx%d\n", pix->width, pix->height);

	return ret;
}

static int mxc_camera_try_fmt(struct soc_camera_device *icd,
			      struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	__u32 pixfmt = pix->pixelformat;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (pixfmt && !xlate) {
		dev_warn(icd->dev.parent, "Format %x not found\n", pixfmt);
		return -EINVAL;
	}

	/* limit to mxc hardware capabilities */
	if (pix->height > 4096)
		pix->height = 4096;
	if (pix->width > 4096)
		pix->width = 4096;

	pix->bytesperline = soc_mbus_bytes_per_line(pix->width,
						    xlate->host_fmt);
	if (pix->bytesperline < 0)
		return pix->bytesperline;
	pix->sizeimage = pix->height * pix->bytesperline;

	/* limit to sensor capabilities */
	mf.width	= pix->width;
	mf.height	= pix->height;
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;
	mf.code		= xlate->code;

	ret = v4l2_subdev_call(sd, video, try_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	pix->width	= mf.width;
	pix->height	= mf.height;
	pix->colorspace	= mf.colorspace;

	switch (mf.field) {
	case V4L2_FIELD_ANY:
		pix->field = V4L2_FIELD_NONE;
		break;
	case V4L2_FIELD_NONE:
		break;
	default:
		dev_err(icd->dev.parent, "Field type %d unsupported.\n",
			mf.field);
		ret = -EINVAL;
	}

	return ret;
}

static int test_platform_param(struct mxc_camera_dev *mxc_cam,
			       unsigned char buswidth, unsigned long *flags)
{
	/*
	 * Platform specified synchronization and pixel clock polarities are
	 * only a recommendation and are only used during probing. MXCx
	 * camera interface only works in master mode, i.e., uses HSYNC and
	 * VSYNC signals from the sensor
	 */
	*flags = SOCAM_MASTER |
		SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_HSYNC_ACTIVE_LOW |
		SOCAM_VSYNC_ACTIVE_HIGH |
		SOCAM_VSYNC_ACTIVE_LOW |
		SOCAM_PCLK_SAMPLE_RISING |
		SOCAM_PCLK_SAMPLE_FALLING |
		SOCAM_DATA_ACTIVE_HIGH |
		SOCAM_DATA_ACTIVE_LOW;

	/*
	 * If requested data width is supported by the platform, use it or any
	 * possible lower value - i.MXC1 is smart enough to schift bits
	 */
	if (mxc_cam->platform_flags & MXC_CAMERA_DATAWIDTH_15)
		*flags |= SOCAM_DATAWIDTH_15 | SOCAM_DATAWIDTH_10 |
			SOCAM_DATAWIDTH_8 | SOCAM_DATAWIDTH_4;
	else if (mxc_cam->platform_flags & MXC_CAMERA_DATAWIDTH_10)
		*flags |= SOCAM_DATAWIDTH_10 | SOCAM_DATAWIDTH_8 |
			SOCAM_DATAWIDTH_4;
	else if (mxc_cam->platform_flags & MXC_CAMERA_DATAWIDTH_8)
		*flags |= SOCAM_DATAWIDTH_8 | SOCAM_DATAWIDTH_4;
	else if (mxc_cam->platform_flags & MXC_CAMERA_DATAWIDTH_4)
		*flags |= SOCAM_DATAWIDTH_4;

	switch (buswidth) {
	case 15:
		if (!(*flags & SOCAM_DATAWIDTH_15))
			return -EINVAL;
		break;
	case 10:
		if (!(*flags & SOCAM_DATAWIDTH_10))
			return -EINVAL;
		break;
	case 8:
		if (!(*flags & SOCAM_DATAWIDTH_8))
			return -EINVAL;
		break;
	case 4:
		if (!(*flags & SOCAM_DATAWIDTH_4))
			return -EINVAL;
		break;
	default:
		dev_warn(mxc_cam->soc_host.v4l2_dev.dev,
			 "Unsupported bus width %d\n", buswidth);
		return -EINVAL;
	}

	return 0;
}

static int mxc_camera_try_bus_param(struct soc_camera_device *icd,
				    const unsigned int depth)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mxc_camera_dev *mxc_cam = ici->priv;
	unsigned long bus_flags, camera_flags;
	int ret = test_platform_param(mxc_cam, depth, &bus_flags);

	dev_dbg(icd->dev.parent, "request bus width %d bit: %d\n", depth, ret);

	if (ret < 0)
		return ret;

	camera_flags = icd->ops->query_bus_param(icd);

	ret = soc_camera_bus_param_compatible(camera_flags, bus_flags);
	if (ret < 0)
		dev_warn(icd->dev.parent,
			 "Flags incompatible: camera %lx, host %lx\n",
			 camera_flags, bus_flags);

	return ret;
}

static const struct soc_mbus_pixelfmt mxc_camera_formats[] = {
	{
		.fourcc			= V4L2_PIX_FMT_SBGGR8,
		.name			= "Bayer BGGR (sRGB) 8 bit",
		.bits_per_sample	= 8,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	}, {
		.fourcc			= V4L2_PIX_FMT_GREY,
		.name			= "Monochrome 8 bit",
		.bits_per_sample	= 8,
		.packing		= SOC_MBUS_PACKING_NONE,
		.order			= SOC_MBUS_ORDER_LE,
	},
};

/* This will be corrected as we get more formats */
static bool mxc_camera_packing_supported(const struct soc_mbus_pixelfmt *fmt)
{
	return	fmt->packing == SOC_MBUS_PACKING_NONE ||
		(fmt->bits_per_sample == 8 &&
		 fmt->packing == SOC_MBUS_PACKING_2X8_PADHI) ||
		(fmt->bits_per_sample > 8 &&
		 fmt->packing == SOC_MBUS_PACKING_EXTEND16);
}

static int mxc_camera_get_formats(struct soc_camera_device *icd, unsigned int idx,
				  struct soc_camera_format_xlate *xlate)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->dev.parent;
	int formats = 0, ret;
	enum v4l2_mbus_pixelcode code;
	const struct soc_mbus_pixelfmt *fmt;

	ret = v4l2_subdev_call(sd, video, enum_mbus_fmt, idx, &code);
	if (ret < 0)
		/* No more formats */
		return 0;

	fmt = soc_mbus_get_fmtdesc(code);
	if (!fmt) {
		dev_warn(icd->dev.parent,
			 "Unsupported format code #%u: %d\n", idx, code);
		return 0;
	}

	/* This also checks support for the requested bits-per-sample */
	ret = mxc_camera_try_bus_param(icd, fmt->bits_per_sample);
	if (ret < 0)
		return 0;

	switch (code) {
	case V4L2_MBUS_FMT_SBGGR10_1X10:
		formats++;
		if (xlate) {
			xlate->host_fmt	= &mxc_camera_formats[0];
			xlate->code	= code;
			xlate++;
			dev_dbg(dev, "Providing format %s using code %d\n",
				mxc_camera_formats[0].name, code);
		}
		break;
	case V4L2_MBUS_FMT_Y10_1X10:
		formats++;
		if (xlate) {
			xlate->host_fmt	= &mxc_camera_formats[1];
			xlate->code	= code;
			xlate++;
			dev_dbg(dev, "Providing format %s using code %d\n",
				mxc_camera_formats[1].name, code);
		}
		break;
	default:
		if (!mxc_camera_packing_supported(fmt))
			return 0;
	}

	/* Generic pass-through */
	formats++;
	if (xlate) {
		xlate->host_fmt	= fmt;
		xlate->code	= code;
		dev_dbg(dev, "Providing format %c%c%c%c in pass-through mode\n",
			(fmt->fourcc >> (0*8)) & 0xFF,
			(fmt->fourcc >> (1*8)) & 0xFF,
			(fmt->fourcc >> (2*8)) & 0xFF,
			(fmt->fourcc >> (3*8)) & 0xFF);
		xlate++;
	}

	return formats;
}
static int mxc_camera_init_videobuf(struct vb2_queue *q,
				     struct soc_camera_device *icd)
{
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->drv_priv = icd;
	q->ops = &mxc_videobuf_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct mxc_camera_buffer);

	return vb2_queue_init(q);
}

static int mxc_camera_reqbufs(struct soc_camera_device *icd,
			      struct v4l2_requestbuffers *p)
{
	return 0;
}

static unsigned int mxc_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_device *icd = file->private_data;

	return vb2_poll(&icd->vb2_vidq, file, pt);
}

static int mxc_camera_querycap(struct soc_camera_host *ici,
			       struct v4l2_capability *cap)
{
	/* cap->name is set by the firendly caller:-> */
	strlcpy(cap->card, "i.MX3x Camera", sizeof(cap->card));
	cap->version = KERNEL_VERSION(0, 2, 2);
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

static int mxc_camera_set_bus_param(struct soc_camera_device *icd, __u32 pixfmt)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mxc_camera_dev *mxc_cam = ici->priv;
	unsigned long bus_flags, camera_flags, common_flags;
	u32 dw, sens_conf;
	const struct soc_mbus_pixelfmt *fmt;
	int buswidth;
	int ret;
	const struct soc_camera_format_xlate *xlate;
	struct device *dev = icd->dev.parent;
	ipu_csi_signal_cfg_t csi_param;

	fmt = soc_mbus_get_fmtdesc(icd->current_fmt->code);
	if (!fmt)
		return -EINVAL;

	buswidth = fmt->bits_per_sample;
	ret = test_platform_param(mxc_cam, buswidth, &bus_flags);

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate) {
		dev_warn(dev, "Format %x not found\n", pixfmt);
		return -EINVAL;
	}

	dev_dbg(dev, "requested bus width %d bit: %d\n", buswidth, ret);

	if (ret < 0)
		return ret;

	camera_flags = icd->ops->query_bus_param(icd);

	common_flags = soc_camera_bus_param_compatible(camera_flags, bus_flags);
	dev_dbg(dev, "Flags cam: 0x%lx host: 0x%lx common: 0x%lx\n",
		camera_flags, bus_flags, common_flags);
	if (!common_flags) {
		dev_dbg(dev, "no common flags");
		return -EINVAL;
	}

	/* Make choices, based on platform preferences */
	if ((common_flags & SOCAM_HSYNC_ACTIVE_HIGH) &&
	    (common_flags & SOCAM_HSYNC_ACTIVE_LOW)) {
		if (mxc_cam->platform_flags & MXC_CAMERA_HSP)
			common_flags &= ~SOCAM_HSYNC_ACTIVE_HIGH;
		else
			common_flags &= ~SOCAM_HSYNC_ACTIVE_LOW;
	}

	if ((common_flags & SOCAM_VSYNC_ACTIVE_HIGH) &&
	    (common_flags & SOCAM_VSYNC_ACTIVE_LOW)) {
		if (mxc_cam->platform_flags & MXC_CAMERA_VSP)
			common_flags &= ~SOCAM_VSYNC_ACTIVE_HIGH;
		else
			common_flags &= ~SOCAM_VSYNC_ACTIVE_LOW;
	}

	if ((common_flags & SOCAM_DATA_ACTIVE_HIGH) &&
	    (common_flags & SOCAM_DATA_ACTIVE_LOW)) {
		if (mxc_cam->platform_flags & MXC_CAMERA_DP)
			common_flags &= ~SOCAM_DATA_ACTIVE_HIGH;
		else
			common_flags &= ~SOCAM_DATA_ACTIVE_LOW;
	}

	if ((common_flags & SOCAM_PCLK_SAMPLE_RISING) &&
	    (common_flags & SOCAM_PCLK_SAMPLE_FALLING)) {
		if (mxc_cam->platform_flags & MXC_CAMERA_PCP)
			common_flags &= ~SOCAM_PCLK_SAMPLE_RISING;
		else
			common_flags &= ~SOCAM_PCLK_SAMPLE_FALLING;
	}

	/*
	 * Make the camera work in widest common mode, we'll take care of
	 * the rest
	 */
	if (common_flags & SOCAM_DATAWIDTH_15)
		common_flags = (common_flags & ~SOCAM_DATAWIDTH_MASK) |
			SOCAM_DATAWIDTH_15;
	else if (common_flags & SOCAM_DATAWIDTH_10)
		common_flags = (common_flags & ~SOCAM_DATAWIDTH_MASK) |
			SOCAM_DATAWIDTH_10;
	else if (common_flags & SOCAM_DATAWIDTH_8)
		common_flags = (common_flags & ~SOCAM_DATAWIDTH_MASK) |
			SOCAM_DATAWIDTH_8;
	else
		common_flags = (common_flags & ~SOCAM_DATAWIDTH_MASK) |
			SOCAM_DATAWIDTH_4;

	ret = icd->ops->set_bus_param(icd, common_flags);
	if (ret < 0) {
		dev_dbg(dev, "camera set_bus_param(%lx) returned %d\n",
			common_flags, ret);
		return ret;
	}

	csi_param.data_width = 0;
	csi_param.clk_mode = IPU_CSI_CLK_MODE_GATED_CLK;
	csi_param.ext_vsync = 0;
	csi_param.Vsync_pol = 0;
	csi_param.Hsync_pol = 0;
	csi_param.pixclk_pol = 0;
	csi_param.data_pol = 0;
	csi_param.sens_clksrc = 0;
	csi_param.pack_tight = 0;
	csi_param.force_eof = 0;
	csi_param.data_en_pol = 0;
	csi_param.data_fmt = 0;
	csi_param.csi = mxc_cam->csi;
	csi_param.mclk = 0;

	if (common_flags & SOCAM_PCLK_SAMPLE_FALLING)
		csi_param.pixclk_pol = 1;
	if (common_flags & SOCAM_HSYNC_ACTIVE_LOW)
		csi_param.Hsync_pol = 1;
	if (common_flags & SOCAM_VSYNC_ACTIVE_LOW)
		csi_param.Vsync_pol = 1;
	if (common_flags & SOCAM_DATA_ACTIVE_LOW)
		csi_param.data_pol = 1;

	/* Just do what we're asked to do */
	switch (xlate->host_fmt->bits_per_sample) {
	case 4:
		csi_param.data_width = IPU_CSI_DATA_WIDTH_4;
		break;
	case 8:
		csi_param.data_width = IPU_CSI_DATA_WIDTH_8;
		break;
	case 9:
		csi_param.data_width = IPU_CSI_DATA_WIDTH_9;
		break;
	case 10:
		csi_param.data_width = IPU_CSI_DATA_WIDTH_10;
		break;
	case 11:
		csi_param.data_width = IPU_CSI_DATA_WIDTH_11;
		break;
	case 12:
		csi_param.data_width = IPU_CSI_DATA_WIDTH_12;
		break;
	case 13:
		csi_param.data_width = IPU_CSI_DATA_WIDTH_13;
		break;
	case 14:
		csi_param.data_width = IPU_CSI_DATA_WIDTH_14;
		break;
	case 15:
		csi_param.data_width = IPU_CSI_DATA_WIDTH_15;
		break;
	case 16:
		csi_param.data_width = IPU_CSI_DATA_WIDTH_16;
		break;
	default:
		dev_err(dev, "Unsupported bits_per_sample: %u\n", xlate->host_fmt->bits_per_sample);
		break;
	}

	ipu_csi_init_interface(mxc_cam->ipu, icd->user_width,
			       icd->user_height,
			       fourcc_to_ipu_pixfmt(xlate->host_fmt->fourcc),
			       csi_param);

	return 0;
}

static struct soc_camera_host_ops mxc_soc_camera_host_ops = {
	.owner		= THIS_MODULE,
	.add		= mxc_camera_add_device,
	.remove		= mxc_camera_remove_device,
	.set_crop	= mxc_camera_set_crop,
	.set_fmt	= mxc_camera_set_fmt,
	.try_fmt	= mxc_camera_try_fmt,
	.get_formats	= mxc_camera_get_formats,
	.init_videobuf2	= mxc_camera_init_videobuf,
	.reqbufs	= mxc_camera_reqbufs,
	.poll		= mxc_camera_poll,
	.querycap	= mxc_camera_querycap,
	.set_bus_param	= mxc_camera_set_bus_param,
};


static int __devinit mxc_camera_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct soc_camera_host *soc_host;
	struct mxc_camera_dev *mxc_cam;
	int err = 0;

	dev_dbg(&pdev->dev, "Probing mxc_camera...\n");

	mxc_cam = vzalloc(sizeof(*mxc_cam));
	if (!mxc_cam) {
		dev_err(&pdev->dev, "Could not allocate mxc camera object\n");
		err = -ENOMEM;
		goto ealloc;
	}

	mxc_cam->pdata = pdev->dev.platform_data;
	mxc_cam->platform_flags = mxc_cam->pdata->flags;

	/* TODO: add other data widths */
	if (!(mxc_cam->platform_flags & (MXC_CAMERA_DATAWIDTH_4 |
			MXC_CAMERA_DATAWIDTH_8 | MXC_CAMERA_DATAWIDTH_10 |
			MXC_CAMERA_DATAWIDTH_15))) {
		/*
		 * Platform hasn't set available data widths. This is bad.
		 * Warn and use a default.
		 */
		dev_warn(&pdev->dev, "WARNING! Platform hasn't set available "
			 "data widths, using default 8 bit\n");
		mxc_cam->platform_flags |= MXC_CAMERA_DATAWIDTH_8;
	}

	mxc_cam->mclk = mxc_cam->pdata->mclk_default;
	if (!mxc_cam->mclk) {
		dev_warn(&pdev->dev,
			 "mclk_default == 0! Please, fix your platform data. "
			 "Using default 20MHz\n");
		mxc_cam->mclk = 20000000;
	}

	/* list of video-buffers */
	INIT_LIST_HEAD(&mxc_cam->capture);
	spin_lock_init(&mxc_cam->lock);

	mxc_cam->ipu = ipu_get_soc(mxc_cam->pdata->ipu);
	if (mxc_cam->ipu == NULL) {
		pr_err("ERROR: v4l2 capture: failed to get ipu\n");
		err = -EINVAL;
		goto erripu;
	} else if (mxc_cam->ipu == ERR_PTR(-ENODEV)) {
		pr_err("ERROR: v4l2 capture: get invalid ipu\n");
		err = -ENODEV;
		goto erripu;
	}

	mxc_cam->csi = mxc_cam->pdata->csi;

	soc_host		= &mxc_cam->soc_host;
	soc_host->drv_name	= MXC_CAM_DRV_NAME;
	soc_host->ops		= &mxc_soc_camera_host_ops;
	soc_host->priv		= mxc_cam;
	soc_host->v4l2_dev.dev	= &pdev->dev;
	soc_host->nr		= pdev->id;

	mxc_cam->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(mxc_cam->alloc_ctx)) {
		err = PTR_ERR(mxc_cam->alloc_ctx);
		goto eallocctx;
	}

	mxc_cam->enc_callback = mxc_cam_enc_callback;
	csi_enc_select(mxc_cam);

	err = soc_camera_host_register(soc_host);
	if (err)
		goto ecamhostreg;

	return 0;

ecamhostreg:
	vb2_dma_contig_cleanup_ctx(mxc_cam->alloc_ctx);
eallocctx:
erripu:
	vfree(mxc_cam);
ealloc:

	dev_err(&pdev->dev, "Probe error: %d\n", err);

	return err;

}

static int __devexit mxc_camera_remove(struct platform_device *pdev)
{
	struct soc_camera_host *soc_host = to_soc_camera_host(&pdev->dev);
	struct mxc_camera_dev *mxc_cam = container_of(soc_host,
					struct mxc_camera_dev, soc_host);

	soc_camera_host_unregister(soc_host);
	vb2_dma_contig_cleanup_ctx(mxc_cam->alloc_ctx);
	vfree(mxc_cam);

	dev_info(&pdev->dev, "i.MXC Camera driver unloaded\n");

	return 0;
}

static struct platform_driver mxc_camera_driver = {
	.driver = {
		.name = MXC_CAM_DRV_NAME,
	},
	.probe = mxc_camera_probe,
	.remove = __devexit_p(mxc_camera_remove),
};

static int __init mxc_camera_init(void)
{
	return platform_driver_register(&mxc_camera_driver);
}

static void __exit mxc_camera_exit(void)
{
	platform_driver_unregister(&mxc_camera_driver);
}

module_init(mxc_camera_init);
module_exit(mxc_camera_exit);

MODULE_DESCRIPTION("MXC SoC Camera Host driver");
MODULE_AUTHOR("Yauhen Kharuzhy <y.kharuzhy@sam-solutions.net>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mxc-camera");

