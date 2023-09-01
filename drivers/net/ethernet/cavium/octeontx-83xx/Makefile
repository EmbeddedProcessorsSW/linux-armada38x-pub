#
# Makefile for Cavium's Octeontx ethernet device
#

obj-$(CONFIG_OCTEONTX_FPA_PF) += fpapf.o
obj-$(CONFIG_OCTEONTX_FPA_VF) += fpavf.o
obj-$(CONFIG_OCTEONTX_RST) += rst.o
obj-$(CONFIG_OCTEONTX_SSO_PF) += ssopf.o
obj-$(CONFIG_OCTEONTX_SSOW_PF) += ssowpf.o
obj-$(CONFIG_OCTEONTX_PKO_PF) += pkopf.o
obj-$(CONFIG_OCTEONTX_LBK) += lbk.o
obj-$(CONFIG_OCTEONTX_TIM_PF) += timpf.o
obj-$(CONFIG_OCTEONTX_PKI) += pki.o
obj-$(CONFIG_OCTEONTX_DPI) += dpi.o
obj-$(CONFIG_OCTEONTX_SLI_PF) += slipf.o
obj-$(CONFIG_OCTEONTX) += octeontx.o

fpapf-objs := fpapf_main.o
fpavf-objs := fpavf_main.o
rst-objs := rst_main.o
ssopf-objs := ssopf_main.o octeontx_mbox.o
ssowpf-objs := ssowpf_main.o
pkopf-objs := pkopf_main.o
octeontx-objs := octeontx_main.o bgx.o
lbk-objs := lbk_main.o
timpf-objs := timpf_main.o
pki-objs := pki_main.o pki_ucode.o pki_config.o
dpi-objs := dpipf_main.o
slipf-objs := slipf_main.o
