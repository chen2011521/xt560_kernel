
EXTRA_CFLAGS += -Idrivers/video/msm

obj-$(CONFIG_FB_MSM_MIPI_DSI_NOVATEK_FWVGA) += mipi_novatek_fwvga.o
obj-$(CONFIG_FB_MSM_MIPI_NOVATEK_VIDEO_FWVGA_PT) += mipi_novatek_video_fwvga_pt.o
obj-$(CONFIG_FB_MSM_MIPI_TOSHIBA_CMD_FWVGA_PT)  += mipi_toshiba_cmd_fwvga_pt.o
obj-$(CONFIG_FB_MSM_MIPI_TOSHIBA_FWVGA)         += mipi_toshiba_fwvga.o
obj-$(CONFIG_FB_MSM_MIPI_RENESAS_CMD_QVGA_PT)   += mipi_renesas_cmd_qvga_pt.o
obj-$(CONFIG_FB_MSM_MIPI_RENESAS_QVGA)          += mipi_renesas_qvga.o
obj-$(CONFIG_FB_MSM_LCDC_S6E63M0_WVGA_PT)          += lcdc_s6e63m0_wvga_pt.o
obj-y += panel.o

ifeq ($(CONFIG_FB_MSM_MDP40),y)
else
obj-y += mdp_hw_init.o
endif

obj-$(CONFIG_FB_MSM_MDP303) += mdp_dma_dsi_video.o

# MIPI gereric
msm_mipi-objs := mipi_dsi.o mipi_dsi_host.o
obj-$(CONFIG_FB_MSM_MIPI_DSI) += msm_mipi.o

ifeq ($(CONFIG_FB_MSM_OVERLAY),y)
else
obj-y += mdp_dma_lcdc.o
endif

# LCDC
obj-$(CONFIG_FB_MSM_LCDC) += lcdc.o

clean:
	rm *.o .*cmd
