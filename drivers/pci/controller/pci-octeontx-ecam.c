// SPDX-License-Identifier: GPL-2.0
/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2015, 2016 Cavium, Inc.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/of_pci.h>
#include <linux/of.h>
#include <linux/pci-ecam.h>
#include <linux/platform_device.h>

static int pki_bus = -1;

static void set_val(u32 v, int where, int size, u32 *val)
{
	int shift = (where & 3) * 8;

	pr_debug("set_val %04x: %08x\n", (unsigned int)(where & ~3), v);
	v >>= shift;
	if (size == 1)
		v &= 0xff;
	else if (size == 2)
		v &= 0xffff;
	*val = v;
}

static int octeontx_pki_sriov_read(struct pci_bus *bus, unsigned int devfn,
				    int where, int size, u32 *val)
{
	int offset = where - 0x180;
	u32 v;

	switch (offset) {
	case 0x0:
	case 0x1:
	case 0x2:
	case 0x3:
		v = 0x10000 | PCI_EXT_CAP_ID_SRIOV;
		set_val(v, where, size, val);
		break;
	case 0x4:
	case 0x5:
	case 0x6:
	case 0x7:
		v = 2;
		set_val(v, where, size, val);
		break;
	case 0x8:
	case 0x9:
	case 0xA:
	case 0xB:
		v = 0x19;
		set_val(v, where, size, val);
		break;
	case 0xc:
	case 0xd:
	case 0xe:
	case 0xf:
		v = 32 | (32 << 16);
		set_val(v, where, size, val);
		break;
	case 0x10:
	case 0x11:
	case 0x12:
	case 0x13:
		v = 32;
		set_val(v, where, size, val);
		break;
	case 0x14:
	case 0x15:
	case 0x16:
	case 0x17:
		v = 1 | (1 << 16);
		set_val(v, where, size, val);
		break;
	case 0x18:
	case 0x19:
	case 0x1a:
	case 0x1b:
		v = (0xa0dd << 16);
		set_val(v, where, size, val);
		break;
	case 0x1c:
	case 0x1d:
	case 0x1e:
	case 0x1f:
		v = 0x553;
		set_val(v, where, size, val);
		break;
	case 0x20:
	case 0x21:
	case 0x22:
	case 0x23:
		v = (0xa0dd << 16);
		set_val(v, where, size, val);
		break;
	default:
		*val = 0;
	}

	return PCIBIOS_SUCCESSFUL;
}

static int octtx_handle_pkivf_read(struct pci_bus *bus, unsigned int devfn,
				    int where, int size, u32 *val)
{
	u32 v;

	if (where >= 0x10 && where < 0x2c) {
		*val = 0;
		return PCIBIOS_SUCCESSFUL;
	}

	if (where >= 0x30 && where < 0x40) {
		*val = 0;
		return PCIBIOS_SUCCESSFUL;
	}

	switch (where) {
	case 0x0:
	case 0x1:
	case 0x2:
	case 0x3:
	case 0xc:
	case 0xd:
	case 0xe:
	case 0xf:
		*val = 0;
		break;
	case 0x4:
	case 0x5:
	case 0x6:
	case 0x7:
		v = 0x100400;
		set_val(v, where, size, val);
		break;
	case 0x8:
	case 0x9:
	case 0xa:
	case 0xb:
		v = 0x8800000;
		set_val(v, where, size, val);
		break;
	case 0x2c:
	case 0x2d:
	case 0x2e:
	case 0x2f:
		v = 0xa3dd177d;
		set_val(v, where, size, val);
		break;
	default:
		*val = ~0;
	}

	return PCIBIOS_SUCCESSFUL;
}

static int octtx_handle_pkipf_read(struct pci_bus *bus, unsigned int devfn,
				    int where, int size, u32 *val)
{
	u32 v;
	void __iomem *addr;

	if (where >= 0x98 && where < 0x9c) {
		addr = bus->ops->map_bus(bus, devfn, 0x98);
		if (!addr) {
			*val = ~0;
			return PCIBIOS_DEVICE_NOT_FOUND;
		}
		v = readl(addr);
		v = v & 0xffff;
		v = v | (0x3 << 16);
		set_val(v, where, size, val);
		return PCIBIOS_SUCCESSFUL;
	}

	if (where >= 0xC4 && where < 0xc8) {
		v = 0x80ff0494;
		set_val(v, where, size, val);
		return PCIBIOS_SUCCESSFUL;
	}

	if (where >= 0xC8 && where < 0xcc) {
		v = 0x1E00002;
		set_val(v, where, size, val);
		return PCIBIOS_SUCCESSFUL;
	}

	if (where >= 0xCC && where < 0xD0) {
		v = 0xfffe;
		set_val(v, where, size, val);
		return PCIBIOS_SUCCESSFUL;
	}

	if (where >= 0x140 && where < 0x144) {
		addr = bus->ops->map_bus(bus, devfn, 0x140);
		if (!addr) {
			*val = ~0;
			return PCIBIOS_DEVICE_NOT_FOUND;
		}
		v = readl(addr);
		v |= (0x180 <<  20);
		set_val(v, where, size, val);
		return PCIBIOS_SUCCESSFUL;
	}

	if (where >= 0x180 && where < 0x1bc)
		return octeontx_pki_sriov_read(bus, devfn, where, size, val);

	return pci_generic_config_read(bus, devfn, where, size, val);
}

static int octeontx_ecam_config_read(struct pci_bus *bus, unsigned int devfn,
				    int where, int size, u32 *val)
{
	u32 vendor_device;
	void __iomem *addr;

	if (pki_bus == bus->number && devfn > 0)
		return octtx_handle_pkivf_read(bus, devfn, where, size, val);

	addr = bus->ops->map_bus(bus, devfn, 0x0);
	if (!addr) {

		*val = ~0;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	vendor_device = readl(addr);
	if (vendor_device == 0xa047177d) {
		pki_bus = bus->number;
		return octtx_handle_pkipf_read(bus, devfn, where, size, val);
	}

	return pci_generic_config_read(bus, devfn, where, size, val);
}

static int octeontx_ecam_config_write(struct pci_bus *bus, unsigned int devfn,
				     int where, int size, u32 val)
{
	/* If required trap PKI SRIOV config writes here */
	return pci_generic_config_write(bus, devfn, where, size, val);
}

static struct pci_ecam_ops pci_octeontx_ecam_ops = {
	.bus_shift	= 20,
	.pci_ops	= {
		.map_bus        = pci_ecam_map_bus,
		.read           = octeontx_ecam_config_read,
		.write          = octeontx_ecam_config_write,
	}
};

static const struct of_device_id octeontx_ecam_of_match[] = {
	{
		.compatible = "cavium,pci-host-octeontx-ecam",
		.data = &pci_octeontx_ecam_ops,
	},
	{ },
};

static struct platform_driver octeontx_ecam_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = octeontx_ecam_of_match,
	},
	.probe = pci_host_common_probe,
};
builtin_platform_driver(octeontx_ecam_driver);
