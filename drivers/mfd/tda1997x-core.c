/*
 * Copyright (C) 2015 Jasbir Matharu.
 * Copyright (C) Texas Instruments
 * 
 * This is a short term workaround for initialising the tda19988 
 * on the UDO Neo. We rely on the MX28 LCD controller for creating the 
 * framebuffer and setting the resolution. This driver piggy backs 
 * from those settings. Longer term this needs to be replaced with a
 * KMS driver for MX28 LCD and use the existing tda998x kernel driver.
 * A large portion of this code is taken from the tda998x_drv.c kernel 
 * driver originally developed by Rob Clark <robdclark@gmail.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
 
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/fsl_devices.h>
#include <linux/interrupt.h>
#include <linux/reset.h>
#include <linux/hdmi.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <sound/asoundef.h>

#include <drm/drm_bridge.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_of.h>
#include <drm/drm_edid.h>

#include <drm/i2c/tda998x.h>
#define DBG(fmt, ...) DRM_DEBUG(fmt"\n", ##__VA_ARGS__)

static const struct drm_connector_funcs tda998x_connector_funcs;
static const struct drm_connector_helper_funcs tda998x_connector_helper_funcs;

struct tda998x_priv {
	struct i2c_client *cec;
	struct i2c_client *hdmi;
	struct mutex mutex;
	u16 rev;
	u8 current_page;
	bool is_on;
	bool supports_infoframes;
	bool sink_has_audio;
	int dpms;
	bool is_hdmi_sink;
	enum hdmi_quantization_range rgb_quant_range;
	u8 vip_cntrl_0;
	u8 vip_cntrl_1;
	u8 vip_cntrl_2;
	unsigned long tmds_clock;
	struct tda998x_encoder_params params;

	struct mutex audio_mutex;

	wait_queue_head_t wq_edid;
	volatile int wq_edid_wait;

	struct work_struct detect_work;
	struct timer_list edid_delay_timer;
	wait_queue_head_t edid_delay_waitq;
	bool edid_delay_active;

	struct drm_encoder encoder;
	struct drm_bridge bridge;
	struct drm_connector connector;
};

#define conn_to_tda998x_priv(x) \
	container_of(x, struct tda998x_priv, connector)

#define enc_to_tda998x_priv(x) \
	container_of(x, struct tda998x_priv, encoder)

#define bridge_to_tda998x_priv(x) \
	container_of(x, struct tda998x_priv, bridge)
#define REG(page, addr) (((page) << 8) | (addr))
#define REG2ADDR(reg)   ((reg) & 0xff)
#define REG2PAGE(reg)   (((reg) >> 8) & 0xff)

#define REG_CURPAGE               0xff                /* write */


/* Page 00h: General Control */
#define REG_VERSION_LSB           REG(0x00, 0x00)     /* read */
#define REG_MAIN_CNTRL0           REG(0x00, 0x01)     /* read/write */
# define MAIN_CNTRL0_SR           (1 << 0)
# define MAIN_CNTRL0_DECS         (1 << 1)
# define MAIN_CNTRL0_DEHS         (1 << 2)
# define MAIN_CNTRL0_CECS         (1 << 3)
# define MAIN_CNTRL0_CEHS         (1 << 4)
# define MAIN_CNTRL0_SCALER       (1 << 7)
#define REG_VERSION_MSB           REG(0x00, 0x02)     /* read */
#define REG_SOFTRESET             REG(0x00, 0x0a)     /* write */
# define SOFTRESET_AUDIO          (1 << 0)
# define SOFTRESET_I2C_MASTER     (1 << 1)
#define REG_DDC_DISABLE           REG(0x00, 0x0b)     /* read/write */
#define REG_CCLK_ON               REG(0x00, 0x0c)     /* read/write */
#define REG_I2C_MASTER            REG(0x00, 0x0d)     /* read/write */
# define I2C_MASTER_DIS_MM        (1 << 0)
# define I2C_MASTER_DIS_FILT      (1 << 1)
# define I2C_MASTER_APP_STRT_LAT  (1 << 2)
#define REG_FEAT_POWERDOWN        REG(0x00, 0x0e)     /* read/write */
# define FEAT_POWERDOWN_PREFILT   BIT(0)
# define FEAT_POWERDOWN_CSC       BIT(1)
# define FEAT_POWERDOWN_SPDIF     (1 << 3)
#define REG_INT_FLAGS_0           REG(0x00, 0x0f)     /* read/write */
#define REG_INT_FLAGS_1           REG(0x00, 0x10)     /* read/write */
#define REG_INT_FLAGS_2           REG(0x00, 0x11)     /* read/write */
# define INT_FLAGS_2_EDID_BLK_RD  (1 << 1)
#define REG_ENA_ACLK              REG(0x00, 0x16)     /* read/write */
#define REG_ENA_VP_0              REG(0x00, 0x18)     /* read/write */
#define REG_ENA_VP_1              REG(0x00, 0x19)     /* read/write */
#define REG_ENA_VP_2              REG(0x00, 0x1a)     /* read/write */
#define REG_ENA_AP                REG(0x00, 0x1e)     /* read/write */
#define REG_VIP_CNTRL_0           REG(0x00, 0x20)     /* write */
# define VIP_CNTRL_0_MIRR_A       (1 << 7)
# define VIP_CNTRL_0_SWAP_A(x)    (((x) & 7) << 4)
# define VIP_CNTRL_0_MIRR_B       (1 << 3)
# define VIP_CNTRL_0_SWAP_B(x)    (((x) & 7) << 0)
#define REG_VIP_CNTRL_1           REG(0x00, 0x21)     /* write */
# define VIP_CNTRL_1_MIRR_C       (1 << 7)
# define VIP_CNTRL_1_SWAP_C(x)    (((x) & 7) << 4)
# define VIP_CNTRL_1_MIRR_D       (1 << 3)
# define VIP_CNTRL_1_SWAP_D(x)    (((x) & 7) << 0)
#define REG_VIP_CNTRL_2           REG(0x00, 0x22)     /* write */
# define VIP_CNTRL_2_MIRR_E       (1 << 7)
# define VIP_CNTRL_2_SWAP_E(x)    (((x) & 7) << 4)
# define VIP_CNTRL_2_MIRR_F       (1 << 3)
# define VIP_CNTRL_2_SWAP_F(x)    (((x) & 7) << 0)
#define REG_VIP_CNTRL_3           REG(0x00, 0x23)     /* write */
# define VIP_CNTRL_3_X_TGL        (1 << 0)
# define VIP_CNTRL_3_H_TGL        (1 << 1)
# define VIP_CNTRL_3_V_TGL        (1 << 2)
# define VIP_CNTRL_3_EMB          (1 << 3)
# define VIP_CNTRL_3_SYNC_DE      (1 << 4)
# define VIP_CNTRL_3_SYNC_HS      (1 << 5)
# define VIP_CNTRL_3_DE_INT       (1 << 6)
# define VIP_CNTRL_3_EDGE         (1 << 7)
#define REG_VIP_CNTRL_4           REG(0x00, 0x24)     /* write */
# define VIP_CNTRL_4_BLC(x)       (((x) & 3) << 0)
# define VIP_CNTRL_4_BLANKIT(x)   (((x) & 3) << 2)
# define VIP_CNTRL_4_CCIR656      (1 << 4)
# define VIP_CNTRL_4_656_ALT      (1 << 5)
# define VIP_CNTRL_4_TST_656      (1 << 6)
# define VIP_CNTRL_4_TST_PAT      (1 << 7)
#define REG_VIP_CNTRL_5           REG(0x00, 0x25)     /* write */
# define VIP_CNTRL_5_CKCASE       (1 << 0)
# define VIP_CNTRL_5_SP_CNT(x)    (((x) & 3) << 1)
#define REG_MUX_AP                REG(0x00, 0x26)     /* read/write */
# define MUX_AP_SELECT_I2S	  0x64
# define MUX_AP_SELECT_SPDIF	  0x40
#define REG_MUX_VP_VIP_OUT        REG(0x00, 0x27)     /* read/write */
#define REG_MAT_CONTRL            REG(0x00, 0x80)     /* write */
# define MAT_CONTRL_MAT_SC(x)     (((x) & 3) << 0)
# define MAT_CONTRL_MAT_BP        (1 << 2)
#define REG_VIDFORMAT             REG(0x00, 0xa0)     /* write */
#define REG_REFPIX_MSB            REG(0x00, 0xa1)     /* write */
#define REG_REFPIX_LSB            REG(0x00, 0xa2)     /* write */
#define REG_REFLINE_MSB           REG(0x00, 0xa3)     /* write */
#define REG_REFLINE_LSB           REG(0x00, 0xa4)     /* write */
#define REG_NPIX_MSB              REG(0x00, 0xa5)     /* write */
#define REG_NPIX_LSB              REG(0x00, 0xa6)     /* write */
#define REG_NLINE_MSB             REG(0x00, 0xa7)     /* write */
#define REG_NLINE_LSB             REG(0x00, 0xa8)     /* write */
#define REG_VS_LINE_STRT_1_MSB    REG(0x00, 0xa9)     /* write */
#define REG_VS_LINE_STRT_1_LSB    REG(0x00, 0xaa)     /* write */
#define REG_VS_PIX_STRT_1_MSB     REG(0x00, 0xab)     /* write */
#define REG_VS_PIX_STRT_1_LSB     REG(0x00, 0xac)     /* write */
#define REG_VS_LINE_END_1_MSB     REG(0x00, 0xad)     /* write */
#define REG_VS_LINE_END_1_LSB     REG(0x00, 0xae)     /* write */
#define REG_VS_PIX_END_1_MSB      REG(0x00, 0xaf)     /* write */
#define REG_VS_PIX_END_1_LSB      REG(0x00, 0xb0)     /* write */
#define REG_VS_LINE_STRT_2_MSB    REG(0x00, 0xb1)     /* write */
#define REG_VS_LINE_STRT_2_LSB    REG(0x00, 0xb2)     /* write */
#define REG_VS_PIX_STRT_2_MSB     REG(0x00, 0xb3)     /* write */
#define REG_VS_PIX_STRT_2_LSB     REG(0x00, 0xb4)     /* write */
#define REG_VS_LINE_END_2_MSB     REG(0x00, 0xb5)     /* write */
#define REG_VS_LINE_END_2_LSB     REG(0x00, 0xb6)     /* write */
#define REG_VS_PIX_END_2_MSB      REG(0x00, 0xb7)     /* write */
#define REG_VS_PIX_END_2_LSB      REG(0x00, 0xb8)     /* write */
#define REG_HS_PIX_START_MSB      REG(0x00, 0xb9)     /* write */
#define REG_HS_PIX_START_LSB      REG(0x00, 0xba)     /* write */
#define REG_HS_PIX_STOP_MSB       REG(0x00, 0xbb)     /* write */
#define REG_HS_PIX_STOP_LSB       REG(0x00, 0xbc)     /* write */
#define REG_VWIN_START_1_MSB      REG(0x00, 0xbd)     /* write */
#define REG_VWIN_START_1_LSB      REG(0x00, 0xbe)     /* write */
#define REG_VWIN_END_1_MSB        REG(0x00, 0xbf)     /* write */
#define REG_VWIN_END_1_LSB        REG(0x00, 0xc0)     /* write */
#define REG_VWIN_START_2_MSB      REG(0x00, 0xc1)     /* write */
#define REG_VWIN_START_2_LSB      REG(0x00, 0xc2)     /* write */
#define REG_VWIN_END_2_MSB        REG(0x00, 0xc3)     /* write */
#define REG_VWIN_END_2_LSB        REG(0x00, 0xc4)     /* write */
#define REG_DE_START_MSB          REG(0x00, 0xc5)     /* write */
#define REG_DE_START_LSB          REG(0x00, 0xc6)     /* write */
#define REG_DE_STOP_MSB           REG(0x00, 0xc7)     /* write */
#define REG_DE_STOP_LSB           REG(0x00, 0xc8)     /* write */
#define REG_TBG_CNTRL_0           REG(0x00, 0xca)     /* write */
# define TBG_CNTRL_0_TOP_TGL      (1 << 0)
# define TBG_CNTRL_0_TOP_SEL      (1 << 1)
# define TBG_CNTRL_0_DE_EXT       (1 << 2)
# define TBG_CNTRL_0_TOP_EXT      (1 << 3)
# define TBG_CNTRL_0_FRAME_DIS    (1 << 5)
# define TBG_CNTRL_0_SYNC_MTHD    (1 << 6)
# define TBG_CNTRL_0_SYNC_ONCE    (1 << 7)
#define REG_TBG_CNTRL_1           REG(0x00, 0xcb)     /* write */
# define TBG_CNTRL_1_H_TGL        (1 << 0)
# define TBG_CNTRL_1_V_TGL        (1 << 1)
# define TBG_CNTRL_1_TGL_EN       (1 << 2)
# define TBG_CNTRL_1_X_EXT        (1 << 3)
# define TBG_CNTRL_1_H_EXT        (1 << 4)
# define TBG_CNTRL_1_V_EXT        (1 << 5)
# define TBG_CNTRL_1_DWIN_DIS     (1 << 6)
#define REG_ENABLE_SPACE          REG(0x00, 0xd6)     /* write */
#define REG_HVF_CNTRL_0           REG(0x00, 0xe4)     /* write */
# define HVF_CNTRL_0_SM           (1 << 7)
# define HVF_CNTRL_0_RWB          (1 << 6)
# define HVF_CNTRL_0_PREFIL(x)    (((x) & 3) << 2)
# define HVF_CNTRL_0_INTPOL(x)    (((x) & 3) << 0)
#define REG_HVF_CNTRL_1           REG(0x00, 0xe5)     /* write */
# define HVF_CNTRL_1_FOR          (1 << 0)
# define HVF_CNTRL_1_YUVBLK       (1 << 1)
# define HVF_CNTRL_1_VQR(x)       (((x) & 3) << 2)
# define HVF_CNTRL_1_PAD(x)       (((x) & 3) << 4)
# define HVF_CNTRL_1_SEMI_PLANAR  (1 << 6)
#define REG_RPT_CNTRL             REG(0x00, 0xf0)     /* write */
# define RPT_CNTRL_REPEAT(x)      ((x) & 15)
#define REG_I2S_FORMAT            REG(0x00, 0xfc)     /* read/write */
# define I2S_FORMAT(x)            (((x) & 3) << 0)
#define REG_AIP_CLKSEL            REG(0x00, 0xfd)     /* write */
# define AIP_CLKSEL_AIP_SPDIF	  (0 << 3)
# define AIP_CLKSEL_AIP_I2S	  (1 << 3)
# define AIP_CLKSEL_FS_ACLK	  (0 << 0)
# define AIP_CLKSEL_FS_MCLK	  (1 << 0)
# define AIP_CLKSEL_FS_FS64SPDIF  (2 << 0)

/* Page 02h: PLL settings */
#define REG_PLL_SERIAL_1          REG(0x02, 0x00)     /* read/write */
# define PLL_SERIAL_1_SRL_FDN     (1 << 0)
# define PLL_SERIAL_1_SRL_IZ(x)   (((x) & 3) << 1)
# define PLL_SERIAL_1_SRL_MAN_IZ  (1 << 6)
#define REG_PLL_SERIAL_2          REG(0x02, 0x01)     /* read/write */
# define PLL_SERIAL_2_SRL_NOSC(x) ((x) << 0)
# define PLL_SERIAL_2_SRL_PR(x)   (((x) & 0xf) << 4)
#define REG_PLL_SERIAL_3          REG(0x02, 0x02)     /* read/write */
# define PLL_SERIAL_3_SRL_CCIR    (1 << 0)
# define PLL_SERIAL_3_SRL_DE      (1 << 2)
# define PLL_SERIAL_3_SRL_PXIN_SEL (1 << 4)
#define REG_SERIALIZER            REG(0x02, 0x03)     /* read/write */
#define REG_BUFFER_OUT            REG(0x02, 0x04)     /* read/write */
#define REG_PLL_SCG1              REG(0x02, 0x05)     /* read/write */
#define REG_PLL_SCG2              REG(0x02, 0x06)     /* read/write */
#define REG_PLL_SCGN1             REG(0x02, 0x07)     /* read/write */
#define REG_PLL_SCGN2             REG(0x02, 0x08)     /* read/write */
#define REG_PLL_SCGR1             REG(0x02, 0x09)     /* read/write */
#define REG_PLL_SCGR2             REG(0x02, 0x0a)     /* read/write */
#define REG_AUDIO_DIV             REG(0x02, 0x0e)     /* read/write */
# define AUDIO_DIV_SERCLK_1       0
# define AUDIO_DIV_SERCLK_2       1
# define AUDIO_DIV_SERCLK_4       2
# define AUDIO_DIV_SERCLK_8       3
# define AUDIO_DIV_SERCLK_16      4
# define AUDIO_DIV_SERCLK_32      5
#define REG_SEL_CLK               REG(0x02, 0x11)     /* read/write */
# define SEL_CLK_SEL_CLK1         (1 << 0)
# define SEL_CLK_SEL_VRF_CLK(x)   (((x) & 3) << 1)
# define SEL_CLK_ENA_SC_CLK       (1 << 3)
#define REG_ANA_GENERAL           REG(0x02, 0x12)     /* read/write */


/* Page 09h: EDID Control */
#define REG_EDID_DATA_0           REG(0x09, 0x00)     /* read */
/* next 127 successive registers are the EDID block */
#define REG_EDID_CTRL             REG(0x09, 0xfa)     /* read/write */
#define REG_DDC_ADDR              REG(0x09, 0xfb)     /* read/write */
#define REG_DDC_OFFS              REG(0x09, 0xfc)     /* read/write */
#define REG_DDC_SEGM_ADDR         REG(0x09, 0xfd)     /* read/write */
#define REG_DDC_SEGM              REG(0x09, 0xfe)     /* read/write */


/* Page 10h: information frames and packets */
#define REG_IF1_HB0               REG(0x10, 0x20)     /* read/write */
#define REG_IF2_HB0               REG(0x10, 0x40)     /* read/write */
#define REG_IF3_HB0               REG(0x10, 0x60)     /* read/write */
#define REG_IF4_HB0               REG(0x10, 0x80)     /* read/write */
#define REG_IF5_HB0               REG(0x10, 0xa0)     /* read/write */


/* Page 11h: audio settings and content info packets */
#define REG_AIP_CNTRL_0           REG(0x11, 0x00)     /* read/write */
# define AIP_CNTRL_0_RST_FIFO     (1 << 0)
# define AIP_CNTRL_0_SWAP         (1 << 1)
# define AIP_CNTRL_0_LAYOUT       (1 << 2)
# define AIP_CNTRL_0_ACR_MAN      (1 << 5)
# define AIP_CNTRL_0_RST_CTS      (1 << 6)
#define REG_CA_I2S                REG(0x11, 0x01)     /* read/write */
# define CA_I2S_CA_I2S(x)         (((x) & 31) << 0)
# define CA_I2S_HBR_CHSTAT        (1 << 6)
#define REG_LATENCY_RD            REG(0x11, 0x04)     /* read/write */
#define REG_ACR_CTS_0             REG(0x11, 0x05)     /* read/write */
#define REG_ACR_CTS_1             REG(0x11, 0x06)     /* read/write */
#define REG_ACR_CTS_2             REG(0x11, 0x07)     /* read/write */
#define REG_ACR_N_0               REG(0x11, 0x08)     /* read/write */
#define REG_ACR_N_1               REG(0x11, 0x09)     /* read/write */
#define REG_ACR_N_2               REG(0x11, 0x0a)     /* read/write */
#define REG_CTS_N                 REG(0x11, 0x0c)     /* read/write */
# define CTS_N_K(x)               (((x) & 7) << 0)
# define CTS_N_M(x)               (((x) & 3) << 4)
#define REG_ENC_CNTRL             REG(0x11, 0x0d)     /* read/write */
# define ENC_CNTRL_RST_ENC        (1 << 0)
# define ENC_CNTRL_RST_SEL        (1 << 1)
# define ENC_CNTRL_CTL_CODE(x)    (((x) & 3) << 2)
#define REG_DIP_FLAGS             REG(0x11, 0x0e)     /* read/write */
# define DIP_FLAGS_ACR            (1 << 0)
# define DIP_FLAGS_GC             (1 << 1)
#define REG_DIP_IF_FLAGS          REG(0x11, 0x0f)     /* read/write */
# define DIP_IF_FLAGS_IF1         (1 << 1)
# define DIP_IF_FLAGS_IF2         (1 << 2)
# define DIP_IF_FLAGS_IF3         (1 << 3)
# define DIP_IF_FLAGS_IF4         (1 << 4)
# define DIP_IF_FLAGS_IF5         (1 << 5)
#define REG_CH_STAT_B(x)          REG(0x11, 0x14 + (x)) /* read/write */


/* Page 12h: HDCP and OTP */
#define REG_TX3                   REG(0x12, 0x9a)     /* read/write */
#define REG_TX4                   REG(0x12, 0x9b)     /* read/write */
# define TX4_PD_RAM               (1 << 1)
#define REG_TX33                  REG(0x12, 0xb8)     /* read/write */
# define TX33_HDMI                (1 << 1)


/* Page 13h: Gamut related metadata packets */

/* CEC registers: (not paged)
 */
#define REG_CEC_INTSTATUS	  0xee		      /* read */
# define CEC_INTSTATUS_CEC	  (1 << 0)
# define CEC_INTSTATUS_HDMI	  (1 << 1)
#define REG_CEC_FRO_IM_CLK_CTRL   0xfb                /* read/write */
# define CEC_FRO_IM_CLK_CTRL_GHOST_DIS (1 << 7)
# define CEC_FRO_IM_CLK_CTRL_ENA_OTP   (1 << 6)
# define CEC_FRO_IM_CLK_CTRL_IMCLK_SEL (1 << 1)
# define CEC_FRO_IM_CLK_CTRL_FRO_DIV   (1 << 0)
#define REG_CEC_RXSHPDINTENA	  0xfc		      /* read/write */
#define REG_CEC_RXSHPDINT	  0xfd		      /* read */
#define REG_CEC_RXSHPDLEV         0xfe                /* read */
# define CEC_RXSHPDLEV_RXSENS     (1 << 0)
# define CEC_RXSHPDLEV_HPD        (1 << 1)

#define REG_CEC_ENAMODS           0xff                /* read/write */
# define CEC_ENAMODS_DIS_FRO      (1 << 6)
# define CEC_ENAMODS_DIS_CCLK     (1 << 5)
# define CEC_ENAMODS_EN_RXSENS    (1 << 2)
# define CEC_ENAMODS_EN_HDMI      (1 << 1)
# define CEC_ENAMODS_EN_CEC       (1 << 0)

#define REG_AVI_IF                REG(0x10, 0x40)   /* AVI Infoframe packet */
#define REG_AUDIO_IF              REG(0x10, 0x80)   /* AVI Infoframe packet */
#define SEL_AIP_I2S              (1 << 3)  /* I2S Clk */

#define FB_VMODE_ASPECT_4_3	0x10
#define FB_VMODE_ASPECT_16_9	0x20
#define FB_VMODE_ASPECT_MASK	(FB_VMODE_ASPECT_4_3 | FB_VMODE_ASPECT_16_9)

/* Device versions: */
#define TDA9989N2                 0x0101
#define TDA19989                  0x0201
#define TDA19989N2                0x0202
#define TDA19988                  0x0301

#define DRV_NAME "tda19988"


struct tda19988_data {
	struct i2c_client *cec;
	struct i2c_client *hdmi;
	struct mutex mutex;
	struct delayed_work det_work;
	struct fb_info *fbi;
	bool waiting_for_fb;
	bool dft_mode_set;
	const char *mode_str;
	int bits_per_pixel;
	uint8_t current_page;

	wait_queue_head_t wq_edid;
	volatile int wq_edid_wait;

	struct work_struct detect_work;
	struct timer_list edid_delay_timer;
	wait_queue_head_t edid_delay_waitq;
	bool edid_delay_active;
};

static struct tda19988_data *data = NULL;

const struct fb_videomode mxc_cea_mode[64] = {
	/* #1: 1920x1080p@60Hz 16:9 */
	[1] = {
		NULL, 60, 1920, 1080, 6734, 148, 88, 36, 4, 44, 5,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_NONINTERLACED | FB_VMODE_ASPECT_16_9, 0,
	},
};
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////


static void
cec_write(struct tda998x_priv *priv, u16 addr, u8 val)
{
	struct i2c_client *client = priv->cec;
	u8 buf[] = {addr, val};
	int ret;

	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret < 0)
		dev_err(&client->dev, "Error %d writing to cec:0x%x\n", ret, addr);
}

static int
set_page(struct tda998x_priv *priv, u16 reg)
{
	if (REG2PAGE(reg) != priv->current_page) {
		struct i2c_client *client = priv->hdmi;
		u8 buf[] = {
				REG_CURPAGE, REG2PAGE(reg)
		};
		int ret = i2c_master_send(client, buf, sizeof(buf));
		if (ret < 0) {
			dev_err(&client->dev, "%s %04x err %d\n", __func__,
					reg, ret);
			return ret;
		}

		priv->current_page = REG2PAGE(reg);
	}
	return 0;
}

static int
reg_read_range(struct tda998x_priv *priv, u16 reg, char *buf, int cnt)
{
	struct i2c_client *client = priv->hdmi;
	u8 addr = REG2ADDR(reg);
	int ret;

	mutex_lock(&priv->mutex);
	ret = set_page(priv, reg);
	if (ret < 0)
		goto out;

	ret = i2c_master_send(client, &addr, sizeof(addr));
	if (ret < 0)
		goto fail;

	ret = i2c_master_recv(client, buf, cnt);
	if (ret < 0)
		goto fail;

	goto out;

fail:
	dev_err(&client->dev, "Error %d reading from 0x%x\n", ret, reg);
out:
	mutex_unlock(&priv->mutex);
	return ret;
}

#define MAX_WRITE_RANGE_BUF 32

static void
reg_write_range(struct tda998x_priv *priv, u16 reg, u8 *p, int cnt)
{
	struct i2c_client *client = priv->hdmi;
	/* This is the maximum size of the buffer passed in */
	u8 buf[MAX_WRITE_RANGE_BUF + 1];
	int ret;

	if (cnt > MAX_WRITE_RANGE_BUF) {
		dev_err(&client->dev, "Fixed write buffer too small (%d)\n",
				MAX_WRITE_RANGE_BUF);
		return;
	}

	buf[0] = REG2ADDR(reg);
	memcpy(&buf[1], p, cnt);

	mutex_lock(&priv->mutex);
	ret = set_page(priv, reg);
	if (ret < 0)
		goto out;

	ret = i2c_master_send(client, buf, cnt + 1);
	if (ret < 0)
		dev_err(&client->dev, "Error %d writing to 0x%x\n", ret, reg);
out:
	mutex_unlock(&priv->mutex);
}

static int
reg_read(struct tda998x_priv *priv, u16 reg)
{
	u8 val = 0;
	int ret;

	ret = reg_read_range(priv, reg, &val, sizeof(val));
	if (ret < 0)
		return ret;
	return val;
}

static void
reg_write(struct tda998x_priv *priv, u16 reg, u8 val)
{
	struct i2c_client *client = priv->hdmi;
	u8 buf[] = {REG2ADDR(reg), val};
	int ret;

	mutex_lock(&priv->mutex);
	ret = set_page(priv, reg);
	if (ret < 0)
		goto out;

	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret < 0)
		dev_err(&client->dev, "Error %d writing to 0x%x\n", ret, reg);
out:
	mutex_unlock(&priv->mutex);
}

#if 1
static void
reg_write16(struct tda998x_priv *priv, u16 reg, u16 val)
{
	struct i2c_client *client = priv->hdmi;
	u8 buf[] = {REG2ADDR(reg), val >> 8, val};
	int ret;

	mutex_lock(&priv->mutex);
	ret = set_page(priv, reg);
	if (ret < 0)
		goto out;

	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret < 0)
		dev_err(&client->dev, "Error %d writing to 0x%x\n", ret, reg);
out:
	mutex_unlock(&priv->mutex);
}
#endif

static void
reg_set(struct tda998x_priv *priv, u16 reg, u8 val)
{
	int old_val;

	old_val = reg_read(priv, reg);
	if (old_val >= 0)
		reg_write(priv, reg, old_val | val);
}

static void
reg_clear(struct tda998x_priv *priv, u16 reg, u8 val)
{
	int old_val;

	old_val = reg_read(priv, reg);
	if (old_val >= 0)
		reg_write(priv, reg, old_val & ~val);
}

static void
tda998x_reset(struct tda998x_priv *priv)
{
	/* reset audio and i2c master: */
	reg_write(priv, REG_SOFTRESET, SOFTRESET_AUDIO | SOFTRESET_I2C_MASTER);
	msleep(50);
	reg_write(priv, REG_SOFTRESET, 0);
	msleep(50);

	/* reset transmitter: */
	reg_set(priv, REG_MAIN_CNTRL0, MAIN_CNTRL0_SR);
	reg_clear(priv, REG_MAIN_CNTRL0, MAIN_CNTRL0_SR);

	/* PLL registers common configuration */
	reg_write(priv, REG_PLL_SERIAL_1, 0x00);
	reg_write(priv, REG_PLL_SERIAL_2, PLL_SERIAL_2_SRL_NOSC(1));
	reg_write(priv, REG_PLL_SERIAL_3, 0x00);
	reg_write(priv, REG_SERIALIZER,   0x00);
	reg_write(priv, REG_BUFFER_OUT,   0x00);
	reg_write(priv, REG_PLL_SCG1,     0x00);
	reg_write(priv, REG_AUDIO_DIV,    AUDIO_DIV_SERCLK_8);
	reg_write(priv, REG_SEL_CLK,      SEL_CLK_SEL_CLK1 | SEL_CLK_ENA_SC_CLK);
	reg_write(priv, REG_PLL_SCGN1,    0xfa);
	reg_write(priv, REG_PLL_SCGN2,    0x00);
	reg_write(priv, REG_PLL_SCGR1,    0x5b);
	reg_write(priv, REG_PLL_SCGR2,    0x00);
	reg_write(priv, REG_PLL_SCG2,     0x10);

	/* Write the default value MUX register */
	reg_write(priv, REG_MUX_VP_VIP_OUT, 0x24);
}

/*
 * The TDA998x has a problem when trying to read the EDID close to a
 * HPD assertion: it needs a delay of 100ms to avoid timing out while
 * trying to read EDID data.
 *
 * However, tda998x_encoder_get_modes() may be called at any moment
 * after tda998x_connector_detect() indicates that we are connected, so
 * we need to delay probing modes in tda998x_encoder_get_modes() after
 * we have seen a HPD inactive->active transition.  This code implements
 * that delay.
 */
static void tda998x_edid_delay_done(struct timer_list *t)
{
	struct tda998x_priv *priv = from_timer(priv, t, edid_delay_timer);

	priv->edid_delay_active = false;
	wake_up(&priv->edid_delay_waitq);
	schedule_work(&priv->detect_work);
}

static int tda998x_edid_delay_wait(struct tda998x_priv *priv)
{
	return wait_event_killable(priv->edid_delay_waitq, !priv->edid_delay_active);
}


////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
/*
 * Ignore sync value when matching
 */
int mxc_lcd_fb_mode_is_equal(const struct fb_videomode *mode1,
			const struct fb_videomode *mode2)
{
	return (mode1->xres         == mode2->xres &&
		mode1->yres         == mode2->yres &&
		mode1->hsync_len    == mode2->hsync_len &&
		mode1->vsync_len    == mode2->vsync_len &&
		mode1->left_margin  == mode2->left_margin &&
		mode1->right_margin == mode2->right_margin &&
		mode1->upper_margin == mode2->upper_margin &&
		mode1->lower_margin == mode2->lower_margin &&
		/* refresh check, 59.94Hz and 60Hz have the same parameter
		 * in struct of mxc_cea_mode */
		abs(mode1->refresh - mode2->refresh) <= 1
	);
};

/*
 * Crude attempt at matching LCD mode to CEA mode
 */
int mxc_lcd_fb_var_to_vic(struct fb_var_screeninfo *var)
{
	int i;
	struct fb_videomode m;

	for (i = 0; i < ARRAY_SIZE(mxc_cea_mode); i++) {
		fb_var_to_videomode(&m, var);
		if (mxc_lcd_fb_mode_is_equal(&m, &mxc_cea_mode[i]))
			break;
	}

	if (i == ARRAY_SIZE(mxc_cea_mode))
		return 0;

	return i;
}
#define HB(x) (x)
#define PB(x) (HB(2) + 1 + (x))

static void
tda998x_write_if(struct tda998x_priv *priv, u8 bit, u16 addr,
		 union hdmi_infoframe *frame)
{
	u8 buf[32];
	ssize_t len;

	len = hdmi_infoframe_pack(frame, buf, sizeof(buf));
	if (len < 0) {
		dev_err(&priv->hdmi->dev,
			"hdmi_infoframe_pack() type=0x%02x failed: %zd\n",
			frame->any.type, len);
		return;
	}

	reg_clear(priv, REG_DIP_IF_FLAGS, bit);
	reg_write_range(priv, addr, buf, len);
	reg_set(priv, REG_DIP_IF_FLAGS, bit);
}

static void
tda998x_write_aif(struct tda998x_priv *priv, struct tda998x_encoder_params *p)
{
	union hdmi_infoframe frame;

	hdmi_audio_infoframe_init(&frame.audio);

	frame.audio.channels = p->audio_frame[1] & 0x07;
	frame.audio.channel_allocation = p->audio_frame[4];
	frame.audio.level_shift_value = (p->audio_frame[5] & 0x78) >> 3;
	frame.audio.downmix_inhibit = (p->audio_frame[5] & 0x80) >> 7;

	/*
	 * L-PCM and IEC61937 compressed audio shall always set sample
	 * frequency to "refer to stream".  For others, see the HDMI
	 * specification.
	 */
	frame.audio.sample_frequency = (p->audio_frame[2] & 0x1c) >> 2;

	tda998x_write_if(priv, DIP_IF_FLAGS_IF4, REG_IF4_HB0, &frame);
}

static void
tda998x_write_avi(struct tda998x_priv *priv, struct drm_display_mode *mode)
{
	union hdmi_infoframe frame1;
	struct hdmi_avi_infoframe frame2;

	drm_hdmi_avi_infoframe_from_display_mode(&frame2, &priv->connector, mode);
	frame1.avi.quantization_range = HDMI_QUANTIZATION_RANGE_FULL;

	tda998x_write_if(priv, DIP_IF_FLAGS_IF2, REG_IF2_HB0, &frame1);
}

static void tda998x_write_vsi(struct tda998x_priv *priv,
			      const struct drm_display_mode *mode)
{
	union hdmi_infoframe frame1;
	struct hdmi_vendor_infoframe frame2;

	if (drm_hdmi_vendor_infoframe_from_display_mode(&frame2, &priv->connector, mode))
		reg_clear(priv, REG_DIP_IF_FLAGS, DIP_IF_FLAGS_IF1);
	else
		tda998x_write_if(priv, DIP_IF_FLAGS_IF1, REG_IF1_HB0, &frame1);
}

/* Audio support */

static void tda998x_audio_mute(struct tda998x_priv *priv, bool on)
{
	if (on) {
		reg_set(priv, REG_SOFTRESET, SOFTRESET_AUDIO);
		reg_clear(priv, REG_SOFTRESET, SOFTRESET_AUDIO);
		reg_set(priv, REG_AIP_CNTRL_0, AIP_CNTRL_0_RST_FIFO);
	} else {
		reg_clear(priv, REG_AIP_CNTRL_0, AIP_CNTRL_0_RST_FIFO);
	}
}

static void
tda998x_configure_audio(struct tda998x_priv *priv,
		struct drm_display_mode *mode, struct tda998x_encoder_params *p)
{
	u8 buf[6], clksel_aip, clksel_fs, cts_n, adiv;
	u32 n;

	/* Enable audio ports */
	reg_write(priv, REG_ENA_AP, p->audio_cfg);
	reg_write(priv, REG_ENA_ACLK, p->audio_clk_cfg);

	/* Set audio input source */
	switch (p->audio_params.format) {
	case AFMT_SPDIF:
		reg_write(priv, REG_MUX_AP, MUX_AP_SELECT_SPDIF);
		clksel_aip = AIP_CLKSEL_AIP_SPDIF;
		clksel_fs = AIP_CLKSEL_FS_FS64SPDIF;
		cts_n = CTS_N_M(3) | CTS_N_K(3);
		break;

	case AFMT_I2S:
		reg_write(priv, REG_MUX_AP, MUX_AP_SELECT_I2S);
		clksel_aip = AIP_CLKSEL_AIP_I2S;
		clksel_fs = AIP_CLKSEL_FS_ACLK;
		cts_n = CTS_N_M(3) | CTS_N_K(3);
		break;

	default:
		BUG();
		return;
	}

	reg_write(priv, REG_AIP_CLKSEL, clksel_aip);
	reg_clear(priv, REG_AIP_CNTRL_0, AIP_CNTRL_0_LAYOUT |
					AIP_CNTRL_0_ACR_MAN);	/* auto CTS */
	reg_write(priv, REG_CTS_N, cts_n);

	/*
	 * Audio input somehow depends on HDMI line rate which is
	 * related to pixclk. Testing showed that modes with pixclk
	 * >100MHz need a larger divider while <40MHz need the default.
	 * There is no detailed info in the datasheet, so we just
	 * assume 100MHz requires larger divider.
	 */
	adiv = AUDIO_DIV_SERCLK_8;
	if (148500 > 100000)
		adiv++;			/* AUDIO_DIV_SERCLK_16 */

	/* S/PDIF asks for a larger divider */
	if (p->audio_params.format == AFMT_SPDIF)
		adiv++;			/* AUDIO_DIV_SERCLK_16 or _32 */

	reg_write(priv, REG_AUDIO_DIV, adiv);

	/*
	 * This is the approximate value of N, which happens to be
	 * the recommended values for non-coherent clocks.
	 */
	n = 128 * p->audio_sample_rate / 1000;

	/* Write the CTS and N values */
	buf[0] = 0x44;
	buf[1] = 0x42;
	buf[2] = 0x01;
	buf[3] = n;
	buf[4] = n >> 8;
	buf[5] = n >> 16;
	reg_write_range(priv, REG_ACR_CTS_0, buf, 6);

	/* Set CTS clock reference */
	reg_write(priv, REG_AIP_CLKSEL, clksel_aip | clksel_fs);

	/* Reset CTS generator */
	reg_set(priv, REG_AIP_CNTRL_0, AIP_CNTRL_0_RST_CTS);
	reg_clear(priv, REG_AIP_CNTRL_0, AIP_CNTRL_0_RST_CTS);

	/* Write the channel status */
	buf[0] = IEC958_AES0_CON_NOT_COPYRIGHT;
	buf[1] = 0x00;
	buf[2] = IEC958_AES3_CON_FS_NOTID;
	buf[3] = IEC958_AES4_CON_ORIGFS_NOTID |
			IEC958_AES4_CON_MAX_WORDLEN_24;
	reg_write_range(priv, REG_CH_STAT_B(0), buf, 4);

	tda998x_audio_mute(priv, true);
	msleep(20);
	tda998x_audio_mute(priv, false);

	/* Write the audio information packet */
	tda998x_write_aif(priv, p);
}

static int tda998x_connector_mode_valid(struct drm_connector *connector,
					struct drm_display_mode *mode)
{
	if (148500 > 150000)
		return MODE_CLOCK_HIGH;
	if (2322 >= BIT(13))
		return MODE_BAD_HVALUE;
	if (1134 >= BIT(11))
		return MODE_BAD_VVALUE;
	return MODE_OK;
}


static int read_edid_block(void *data, u8 *buf, unsigned int blk, size_t length)
{
	struct tda998x_priv *priv = data;
	u8 offset, segptr;
	int ret, i;

	offset = (blk & 1) ? 128 : 0;
	segptr = blk / 2;

	reg_write(priv, REG_DDC_ADDR, 0xa0);
	reg_write(priv, REG_DDC_OFFS, offset);
	reg_write(priv, REG_DDC_SEGM_ADDR, 0x60);
	reg_write(priv, REG_DDC_SEGM, segptr);

	/* enable reading EDID: */
	priv->wq_edid_wait = 1;
	reg_write(priv, REG_EDID_CTRL, 0x1);

	/* flag must be cleared by sw: */
	reg_write(priv, REG_EDID_CTRL, 0x0);

	/* wait for block read to complete: */
	if (priv->hdmi->irq) {
		i = wait_event_timeout(priv->wq_edid,
					!priv->wq_edid_wait,
					msecs_to_jiffies(100));
		if (i < 0) {
			dev_err(&priv->hdmi->dev, "read edid wait err %d\n", i);
			return i;
		}
	} else {
		for (i = 100; i > 0; i--) {
			msleep(1);
			ret = reg_read(priv, REG_INT_FLAGS_2);

			if (ret < 0)
				return ret;
			if (ret & INT_FLAGS_2_EDID_BLK_RD)
				break;
		}
	}

	if (i == 0) {
		dev_err(&priv->hdmi->dev, "read edid timeout\n");
		return -ETIMEDOUT;
	}

	ret = reg_read_range(priv, REG_EDID_DATA_0, buf, length);

	if (ret != length) {
		dev_err(&priv->hdmi->dev, "failed to read edid block %d: %d\n",
			blk, ret);
		return ret;
	}

	return 0;
}

static int tda998x_connector_get_modes(struct drm_connector *connector)
{
	struct tda998x_priv *priv = conn_to_tda998x_priv(connector);
	struct edid *edid;
	int n;

	/*
	 * If we get killed while waiting for the HPD timeout, return
	 * no modes found: we are not in a restartable path, so we
	 * can't handle signals gracefully.
	 */
	if (tda998x_edid_delay_wait(priv))
		return 0;

	if (priv->rev == TDA19988)
		reg_clear(priv, REG_TX4, TX4_PD_RAM);

	edid = drm_do_get_edid(connector, read_edid_block, priv);

	if (priv->rev == TDA19988)
		reg_set(priv, REG_TX4, TX4_PD_RAM);

	if (!edid) {

		dev_warn(&priv->hdmi->dev, "failed to read EDID\n");
		return 0;
	}

	drm_connector_update_edid_property(connector, edid);

	n = drm_add_edid_modes(connector, edid);
	priv->is_hdmi_sink = drm_detect_hdmi_monitor(edid);
	kfree(edid);

	return n;
}

static int tda998x_connector_init(struct tda998x_priv *priv,
				  struct drm_device *drm)
{
	struct drm_connector *connector = &priv->connector;
	int ret;

	connector->interlace_allowed = 1;

	if (priv->hdmi->irq)
		connector->polled = DRM_CONNECTOR_POLL_HPD;
	else
		connector->polled = DRM_CONNECTOR_POLL_CONNECT |
			DRM_CONNECTOR_POLL_DISCONNECT;

	drm_connector_helper_add(connector, &tda998x_connector_helper_funcs);
	ret = drm_connector_init(drm, connector, &tda998x_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);
	if (ret)
		return ret;

	drm_connector_attach_encoder(&priv->connector,
				     priv->bridge.encoder);

	return 0;
}

/* DRM bridge functions */
int tda19988_bridge_attach(struct drm_bridge *bridge)
{
	struct tda998x_priv *priv = bridge_to_tda998x_priv(bridge);

	return tda998x_connector_init(priv, bridge->dev);
}

void tda19988_bridge_enable(struct drm_bridge *bridge)
{
	struct tda998x_priv *priv = bridge_to_tda998x_priv(bridge);

	if (!priv->is_on) {
		/* enable video ports, audio will be enabled later */
		reg_write(priv, REG_ENA_VP_0, 0xff);
		reg_write(priv, REG_ENA_VP_1, 0xff);
		reg_write(priv, REG_ENA_VP_2, 0xff);
		/* set muxing after enabling ports: */
		reg_write(priv, REG_VIP_CNTRL_0, priv->vip_cntrl_0);
		reg_write(priv, REG_VIP_CNTRL_1, priv->vip_cntrl_1);
		reg_write(priv, REG_VIP_CNTRL_2, priv->vip_cntrl_2);

		priv->is_on = true;
	}
}

void tda19988_bridge_disable(struct drm_bridge *bridge)
{
	struct tda998x_priv *priv = bridge_to_tda998x_priv(bridge);

	if (priv->is_on) {
		/* disable video ports */
		reg_write(priv, REG_ENA_VP_0, 0x00);
		reg_write(priv, REG_ENA_VP_1, 0x00);
		reg_write(priv, REG_ENA_VP_2, 0x00);

		priv->is_on = false;
	}
}

void tda19988_bridge_mode_set(struct tda998x_priv *priv)
{
	struct drm_display_mode *mode;
	struct drm_display_mode *adjusted_mode;
	unsigned long tmds_clock;
	u16 ref_pix, ref_line, n_pix, n_line;
	u16 hs_pix_s, hs_pix_e;
	u16 vs1_pix_s, vs1_pix_e, vs1_line_s, vs1_line_e;
	u16 vs2_pix_s, vs2_pix_e, vs2_line_s, vs2_line_e;
	u16 vwin1_line_s, vwin1_line_e;
	u16 vwin2_line_s, vwin2_line_e;
	u16 de_pix_s, de_pix_e;
	u8 reg, div, rep, sel_clk;

	mode = kzalloc(sizeof(struct drm_display_mode), GFP_KERNEL);
	adjusted_mode = kzalloc(sizeof(struct drm_display_mode), GFP_KERNEL);

	mode->clock=148500;
	mode->vsync_start=1100;
	mode->vsync_end=1107;
	mode->vtotal=1125;
	mode->vdisplay=1080;
	mode->hsync_start=2000;
	mode->hsync_end=2080;
	mode->htotal=2160;
	mode->hdisplay=1920;
	adjusted_mode->flags=0x0a;
	adjusted_mode->hskew=0;

	/*
	 * Internally TDA998x is using ITU-R BT.656 style sync but
	 * we get VESA style sync. TDA998x is using a reference pixel
	 * relative to ITU to sync to the input frame and for output
	 * sync generation. Currently, we are using reference detection
	 * from HS/VS, i.e. REFPIX/REFLINE denote frame start sync point
	 * which is position of rising VS with coincident rising HS.
	 *
	 * Now there is some issues to take care of:
	 * - HDMI data islands require sync-before-active
	 * - TDA998x register values must be > 0 to be enabled
	 * - REFLINE needs an additional offset of +1
	 * - REFPIX needs an addtional offset of +1 for UYUV and +3 for RGB
	 *
	 * So we add +1 to all horizontal and vertical register values,
	 * plus an additional +3 for REFPIX as we are using RGB input only.
	 */
	n_pix        = mode->htotal;
	n_line       = mode->vtotal;

	hs_pix_e     = mode->hsync_end - mode->hdisplay;
	hs_pix_s     = mode->hsync_start - mode->hdisplay;
	de_pix_e     = mode->htotal;
	de_pix_s     = mode->htotal - mode->hdisplay;
	ref_pix      = 3 + hs_pix_s;

	/*
	 * Attached LCD controllers may generate broken sync. Allow
	 * those to adjust the position of the rising VS edge by adding
	 * HSKEW to ref_pix.
	 */
	if (adjusted_mode->flags & DRM_MODE_FLAG_HSKEW)
		ref_pix += adjusted_mode->hskew;

	if ((mode->flags & DRM_MODE_FLAG_INTERLACE) == 0) {
		ref_line     = 1 + mode->vsync_start - mode->vdisplay;
		vwin1_line_s = mode->vtotal - mode->vdisplay - 1;
		vwin1_line_e = vwin1_line_s + mode->vdisplay;
		vs1_pix_s    = vs1_pix_e = hs_pix_s;
		vs1_line_s   = mode->vsync_start - mode->vdisplay;
		vs1_line_e   = vs1_line_s +
			       mode->vsync_end - mode->vsync_start;
		vwin2_line_s = vwin2_line_e = 0;
		vs2_pix_s    = vs2_pix_e  = 0;
		vs2_line_s   = vs2_line_e = 0;
	} else {
		ref_line     = 1 + (mode->vsync_start - mode->vdisplay)/2;
		vwin1_line_s = (mode->vtotal - mode->vdisplay)/2;
		vwin1_line_e = vwin1_line_s + mode->vdisplay/2;
		vs1_pix_s    = vs1_pix_e = hs_pix_s;
		vs1_line_s   = (mode->vsync_start - mode->vdisplay)/2;
		vs1_line_e   = vs1_line_s +
			       (mode->vsync_end - mode->vsync_start)/2;
		vwin2_line_s = vwin1_line_s + mode->vtotal/2;
		vwin2_line_e = vwin2_line_s + mode->vdisplay/2;
		vs2_pix_s    = vs2_pix_e = hs_pix_s + mode->htotal/2;
		vs2_line_s   = vs1_line_s + mode->vtotal/2 ;
		vs2_line_e   = vs2_line_s +
			       (mode->vsync_end - mode->vsync_start)/2;
	}

	/*
	 * Select pixel repeat depending on the double-clock flag
	 * (which means we have to repeat each pixel once.)
	 */
	rep = mode->flags & DRM_MODE_FLAG_DBLCLK ? 1 : 0;
	sel_clk = SEL_CLK_ENA_SC_CLK | SEL_CLK_SEL_CLK1 |
		  SEL_CLK_SEL_VRF_CLK(rep ? 2 : 0);

	/* the TMDS clock is scaled up by the pixel repeat */
	tmds_clock = mode->clock * (1 + rep);

	/*
	 * The divisor is power-of-2. The TDA9983B datasheet gives
	 * this as ranges of Msample/s, which is 10x the TMDS clock:
	 *   0 - 800 to 1500 Msample/s
	 *   1 - 400 to 800 Msample/s
	 *   2 - 200 to 400 Msample/s
	 *   3 - as 2 above
	 */
	for (div = 0; div < 3; div++)
		if (80000 >> div <= tmds_clock)
			break;

	/* mute the audio FIFO: */
	reg_set(priv, REG_AIP_CNTRL_0, AIP_CNTRL_0_RST_FIFO);

	/* set HDMI HDCP mode off: */
	reg_write(priv, REG_TBG_CNTRL_1, TBG_CNTRL_1_DWIN_DIS);
	reg_clear(priv, REG_TX33, TX33_HDMI);
	reg_write(priv, REG_ENC_CNTRL, ENC_CNTRL_CTL_CODE(0));

	/* no pre-filter or interpolator: */
	reg_write(priv, REG_HVF_CNTRL_0, HVF_CNTRL_0_PREFIL(0) |
			HVF_CNTRL_0_INTPOL(0));
	reg_set(priv, REG_FEAT_POWERDOWN, FEAT_POWERDOWN_PREFILT);
	reg_write(priv, REG_VIP_CNTRL_5, VIP_CNTRL_5_SP_CNT(0));
	reg_write(priv, REG_VIP_CNTRL_4, VIP_CNTRL_4_BLANKIT(0) |
			VIP_CNTRL_4_BLC(0));

	reg_clear(priv, REG_PLL_SERIAL_1, PLL_SERIAL_1_SRL_MAN_IZ);
	reg_clear(priv, REG_PLL_SERIAL_3, PLL_SERIAL_3_SRL_CCIR |
					  PLL_SERIAL_3_SRL_DE);
	reg_write(priv, REG_SERIALIZER, 0);
	reg_write(priv, REG_HVF_CNTRL_1, HVF_CNTRL_1_VQR(0));

	reg_write(priv, REG_RPT_CNTRL, RPT_CNTRL_REPEAT(rep));
	reg_write(priv, REG_SEL_CLK, sel_clk);
	reg_write(priv, REG_PLL_SERIAL_2, PLL_SERIAL_2_SRL_NOSC(div) |
			PLL_SERIAL_2_SRL_PR(rep));

	/* set color matrix according to output rgb quant range */
	if (priv->rgb_quant_range == HDMI_QUANTIZATION_RANGE_LIMITED) {
		static u8 tda998x_full_to_limited_range[] = {
			MAT_CONTRL_MAT_SC(2),
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x03, 0x6f, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x03, 0x6f, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x03, 0x6f,
			0x00, 0x40, 0x00, 0x40, 0x00, 0x40
		};
		reg_clear(priv, REG_FEAT_POWERDOWN, FEAT_POWERDOWN_CSC);
		reg_write_range(priv, REG_MAT_CONTRL,
				tda998x_full_to_limited_range,
				sizeof(tda998x_full_to_limited_range));
	} else {
		reg_write(priv, REG_MAT_CONTRL, MAT_CONTRL_MAT_BP |
					MAT_CONTRL_MAT_SC(1));
		reg_set(priv, REG_FEAT_POWERDOWN, FEAT_POWERDOWN_CSC);
	}

	/* set BIAS tmds value: */
	reg_write(priv, REG_ANA_GENERAL, 0x09);

	reg_write(priv, REG_VIDFORMAT, 0x00);
	reg_write16(priv, REG_REFPIX_MSB, ref_pix);
	reg_write16(priv, REG_REFLINE_MSB, ref_line);
	reg_write16(priv, REG_NPIX_MSB, n_pix);
	reg_write16(priv, REG_NLINE_MSB, n_line);
	reg_write16(priv, REG_VS_LINE_STRT_1_MSB, vs1_line_s);
	reg_write16(priv, REG_VS_PIX_STRT_1_MSB, vs1_pix_s);
	reg_write16(priv, REG_VS_LINE_END_1_MSB, vs1_line_e);
	reg_write16(priv, REG_VS_PIX_END_1_MSB, vs1_pix_e);
	reg_write16(priv, REG_VS_LINE_STRT_2_MSB, vs2_line_s);
	reg_write16(priv, REG_VS_PIX_STRT_2_MSB, vs2_pix_s);
	reg_write16(priv, REG_VS_LINE_END_2_MSB, vs2_line_e);
	reg_write16(priv, REG_VS_PIX_END_2_MSB, vs2_pix_e);
	reg_write16(priv, REG_HS_PIX_START_MSB, hs_pix_s);
	reg_write16(priv, REG_HS_PIX_STOP_MSB, hs_pix_e);
	reg_write16(priv, REG_VWIN_START_1_MSB, vwin1_line_s);
	reg_write16(priv, REG_VWIN_END_1_MSB, vwin1_line_e);
	reg_write16(priv, REG_VWIN_START_2_MSB, vwin2_line_s);
	reg_write16(priv, REG_VWIN_END_2_MSB, vwin2_line_e);
	reg_write16(priv, REG_DE_START_MSB, de_pix_s);
	reg_write16(priv, REG_DE_STOP_MSB, de_pix_e);

	if (priv->rev == TDA19988) {
		/* let incoming pixels fill the active space (if any) */
		reg_write(priv, REG_ENABLE_SPACE, 0x00);
	}


	/*
	 * Always generate sync polarity relative to input sync and
	 * revert input stage toggled sync at output stage
	 */
	reg = TBG_CNTRL_1_DWIN_DIS | TBG_CNTRL_1_TGL_EN;
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		reg |= TBG_CNTRL_1_H_TGL;
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		reg |= TBG_CNTRL_1_V_TGL;
	reg_write(priv, REG_TBG_CNTRL_1, reg);

	/* must be last register set: */
	reg_write(priv, REG_TBG_CNTRL_0, 0);

	/* CEA-861B section 6 says that:
	 * CEA version 1 (CEA-861) has no support for infoframes.
	 * CEA version 2 (CEA-861A) supports version 1 AVI infoframes,
	 * and optional basic audio.
	 * CEA version 3 (CEA-861B) supports version 1 and 2 AVI infoframes,
	 * and optional digital audio, with audio infoframes.
	 *
	 * Since we only support generation of version 2 AVI infoframes,
	 * ignore CEA version 2 and below (iow, behave as if we're a
	 * CEA-861 source.)
	 */

	if (priv->supports_infoframes) {
		/* We need to turn HDMI HDCP stuff on to get audio through */
		reg &= ~TBG_CNTRL_1_DWIN_DIS;
		reg_write(priv, REG_TBG_CNTRL_1, reg);
		reg_write(priv, REG_ENC_CNTRL, ENC_CNTRL_CTL_CODE(1));
		reg_set(priv, REG_TX33, TX33_HDMI);

		tda998x_write_avi(priv, adjusted_mode);
		tda998x_write_vsi(priv, adjusted_mode);

		if (priv->sink_has_audio)
			tda998x_configure_audio(priv, adjusted_mode,
						&priv->params);
	}

	reg_write(priv, REG_VIP_CNTRL_0, priv->vip_cntrl_0);
	reg_write(priv, REG_VIP_CNTRL_1, priv->vip_cntrl_1);
	reg_write(priv, REG_VIP_CNTRL_2, priv->vip_cntrl_2);

}

/* I2C driver functions */
static int tda998x_create(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct device_node *np = client->dev.of_node;
	struct tda998x_priv *priv;
	u32 video;
	int rev_lo, rev_hi, ret, i;
	unsigned short cec_addr;
	char buf[128];

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);

	priv->vip_cntrl_0 = VIP_CNTRL_0_SWAP_A(2) | VIP_CNTRL_0_SWAP_B(3);
	priv->vip_cntrl_1 = VIP_CNTRL_1_SWAP_C(0) | VIP_CNTRL_1_SWAP_D(1);
	priv->vip_cntrl_2 = VIP_CNTRL_2_SWAP_E(4) | VIP_CNTRL_2_SWAP_F(5);

	priv->current_page = 0xff;
	priv->hdmi = client;
	/* CEC I2C address bound to TDA998x I2C addr by configuration pins */
	cec_addr = 0x34 + (client->addr & 0x03);
	priv->cec = i2c_new_dummy_device(client->adapter, cec_addr);
	if (!priv->cec)
		return -ENODEV;

	mutex_init(&priv->mutex);	/* protect the page access */
	init_waitqueue_head(&priv->edid_delay_waitq);
	timer_setup(&priv->edid_delay_timer, tda998x_edid_delay_done, 0);

	/* wake up the device: */
	cec_write(priv, REG_CEC_ENAMODS,
			CEC_ENAMODS_EN_RXSENS | CEC_ENAMODS_EN_HDMI);

	tda998x_reset(priv);

	/* read version: */
	rev_lo = reg_read(priv, REG_VERSION_LSB);
	rev_hi = reg_read(priv, REG_VERSION_MSB);
	if (rev_lo < 0 || rev_hi < 0) {
		ret = rev_lo < 0 ? rev_lo : rev_hi;
		goto fail;
	}

	priv->rev = rev_lo | rev_hi << 8;

	/* mask off feature bits: */
	priv->rev &= ~0x30; /* not-hdcp and not-scalar bit */

	switch (priv->rev) {
	case TDA9989N2:
		dev_info(&client->dev, "found TDA9989 n2");
		break;
	case TDA19989:
		dev_info(&client->dev, "found TDA19989");
		break;
	case TDA19989N2:
		dev_info(&client->dev, "found TDA19989 n2");
		break;
	case TDA19988:
		dev_info(&client->dev, "found TDA19988");
		break;
	default:
		dev_err(&client->dev, "found unsupported device: %04x\n",
			priv->rev);
		goto fail;
	}

	/* after reset, enable DDC: */
	reg_write(priv, REG_DDC_DISABLE, 0x00);

	/* set clock on DDC channel: */
	reg_write(priv, REG_TX3, 39);

	/* if necessary, disable multi-master: */
	if (priv->rev == TDA19989)
		reg_set(priv, REG_I2C_MASTER, I2C_MASTER_DIS_MM);

	cec_write(priv, REG_CEC_FRO_IM_CLK_CTRL,
			CEC_FRO_IM_CLK_CTRL_GHOST_DIS | CEC_FRO_IM_CLK_CTRL_IMCLK_SEL);

	/* initialize the optional IRQ */
	if (client->irq) {

		/* init read EDID waitqueue and HDP work */
		init_waitqueue_head(&priv->wq_edid);

		/* clear pending interrupts */
		reg_read(priv, REG_INT_FLAGS_0);
		reg_read(priv, REG_INT_FLAGS_1);
		reg_read(priv, REG_INT_FLAGS_2);

		/* enable HPD irq */
		cec_write(priv, REG_CEC_RXSHPDINTENA, CEC_RXSHPDLEV_HPD);
	}

	/* enable EDID read irq: */
	reg_set(priv, REG_INT_FLAGS_2, INT_FLAGS_2_EDID_BLK_RD);

	if (!np)
		return 0;		/* non-DT */

	/* get the optional video properties */
	ret = of_property_read_u32(np, "video-ports", &video);
	if (ret == 0) {
		priv->vip_cntrl_0 = video >> 16;
		priv->vip_cntrl_1 = video >> 8;
		priv->vip_cntrl_2 = video;
	}

	read_edid_block(priv, buf, 0, 128);
	for ( i = 0; i < 8; i++ ) {
		printk(KERN_INFO
		"%02x %02x %02x %02x  %02x %02x %02x %02x  "
		"%02x %02x %02x %02x  %02x %02x %02x %02x  ",
		buf[0+16*i], buf[1+16*i], buf[2+16*i], buf[3+16*i],
		buf[4+16*i], buf[5+16*i], buf[6+16*i], buf[7+16*i],
		buf[8+16*i], buf[9+16*i], buf[10+16*i], buf[11+16*i],
		buf[12+16*i], buf[13+16*i], buf[14+16*i], buf[15+16*i]);
	}

	read_edid_block(priv, buf, 1, 128);
	for ( i = 0; i < 8; i++ ) {
		printk(KERN_INFO
		"%02x %02x %02x %02x  %02x %02x %02x %02x  "
		"%02x %02x %02x %02x  %02x %02x %02x %02x  ",
		buf[0+16*i], buf[1+16*i], buf[2+16*i], buf[3+16*i],
		buf[4+16*i], buf[5+16*i], buf[6+16*i], buf[7+16*i],
		buf[8+16*i], buf[9+16*i], buf[10+16*i], buf[11+16*i],
		buf[12+16*i], buf[13+16*i], buf[14+16*i], buf[15+16*i]);
	}

	tda19988_bridge_mode_set(priv);

	return 0;

fail:
	/* if encoder_init fails, the encoder slave is never registered,
	 * so cleanup here:
	 */
	if (priv->cec)
		i2c_unregister_device(priv->cec);
	return -ENXIO;
}


static struct drm_encoder *
tda998x_connector_best_encoder(struct drm_connector *connector)
{
	struct tda998x_priv *priv = conn_to_tda998x_priv(connector);


	return &priv->encoder;
}

static
const struct drm_connector_helper_funcs tda998x_connector_helper_funcs = {
	.get_modes = tda998x_connector_get_modes,
	.mode_valid = tda998x_connector_mode_valid,
	.best_encoder = tda998x_connector_best_encoder,
};


static int tda19988_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	
	int ret;
	
	ret = tda998x_create(&client->dev);
	if (ret)
		return ret;

	return 0;
}

static int tda19988_remove(struct i2c_client *client)
{

	if (!data->hdmi) {
		i2c_unregister_device(data->hdmi);
	}

	if (!data) {
		kfree(data);
	}

	return 0;
}

static const struct i2c_device_id tda19988_id[] = {
	{ DRV_NAME, 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, tda19988_id);

static const struct of_device_id tda19988_dt_ids[] = {
	{ .compatible = "udoo,tda19988", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tda19988_dt_ids);

static struct i2c_driver tda19988_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tda19988_dt_ids,
	},
	.probe		= tda19988_probe,
	.remove		= tda19988_remove,
	.id_table	= tda19988_id,
};

module_i2c_driver(tda19988_driver);

MODULE_AUTHOR("Jasbir Matharu");
MODULE_DESCRIPTION("UDOO NEO TDA19988 HDMI driver");
MODULE_LICENSE("GPL");
