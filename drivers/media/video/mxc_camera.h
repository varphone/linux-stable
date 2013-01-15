#ifndef __MXC_V4L2_CAPTURE_CAMSOC_H__
#define __MXC_V4L2_CAPTURE_CAMSOC_H__

#include <asm/uaccess.h>
#include <linux/list.h>
#include <linux/ipu.h>
#include <linux/mxc_v4l2.h>
#include <mach/ipu-v3.h>

#include <media/v4l2-dev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <mach/mxc_camera.h>


#define MAX_VIDEO_MEM 16

enum csi_buffer_state {
	CSI_BUF_NEEDS_INIT,
	CSI_BUF_PREPARED,
};

struct mxc_camera_buffer {
	/* common v4l buffer stuff -- must be first */
	struct vb2_buffer			vb;
	enum csi_buffer_state			state;
	struct list_head			queue;

	/* We have to "build" a scatterlist ourselves - one element per frame */
	struct scatterlist			sg;
};

/*!
 * v4l2 frame structure.
 */
struct mxc_v4l_frame {
	u32 paddress;
	void *vaddress;
	int count;
	int width;
	int height;

	struct v4l2_buffer buffer;
	struct list_head queue;
	int index;
	int ipu_buf_num;
};

struct mxc_camera_dev;

struct mxc_cam_enc_ops {
	int (*enc_update_eba) (struct ipu_soc *ipu, dma_addr_t eba, int *bufferNum);
	int (*enc_enable) (struct mxc_camera_dev *);
	int (*enc_disable) (struct mxc_camera_dev *);
	int (*enc_enable_csi) (struct mxc_camera_dev *);
	int (*enc_disable_csi) (struct mxc_camera_dev *);
};

struct mxc_camera_dev {

	struct soc_camera_device *icd;
	struct soc_camera_host	soc_host;
	struct mxc_camera_pdata	*pdata;

	unsigned long		platform_flags;
	unsigned long		mclk;
	int			mclk_source;
	struct list_head	capture;
	spinlock_t		lock;		/* Protects video buffer lists */
	struct mxc_camera_buffer *active;
	struct vb2_alloc_ctx	*alloc_ctx;
	enum v4l2_field		field;
	int			sequence;
	int			csi_buf_num;

	struct ipu_soc *ipu;
	unsigned int csi; //TODO

	struct mxc_cam_enc_ops	*enc_ops;
	void			*enc_private;
	void (*enc_callback) (u32 mask, struct mxc_camera_dev *mxc_cam);
};

extern u32 fourcc_to_ipu_pixfmt(u32 fourcc);

extern int csi_enc_select(struct mxc_camera_dev *);
extern int csi_enc_deselect(struct mxc_camera_dev *);


#endif
