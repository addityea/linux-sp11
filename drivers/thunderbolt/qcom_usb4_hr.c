// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm X1E USB4 host-router bring-up scaffold.
 *
 * By default this only validates that the DT describes the complete
 * host-router resource set.  The activation path is explicitly opt-in while
 * the firmware and Qualcomm-specific initialization sequence are incomplete.
 *
 * Controller ownership contract between this driver and the QMP PHY driver
 * (qualification area c, updated with 2026-11-14 REA evidence in
 * evidence/rea-usb4-20261114.md):
 *
 * - The PHY driver (phy-qcom-qmp-combo, #24 series) owns the QMP block,
 *   its "phy_nocsr" reset (we take it only as an optional SHARED reset),
 *   and the USB4 pipe-clock/ownership state machine it acquired in
 *   #24/#87. The DP tunnel clock also stays entirely in the QMP-combo
 *   domain; this frontend never touches it. We consume the PHY
 *   exclusively through phy_get/phy_set_mode_ext/phy_init/phy_exit and
 *   never write its registers -- the one historical exception, the AON
 *   clamp release at PHY base +0x104, is default-off behind
 *   uc_skip_clamp=0.
 * - This driver owns the host-router NHI block: its named MMIO regions,
 *   the remaining resets, the clocks, and the NHI rings/interrupts once
 *   activated. Firmware is NOT host-owned: the router boots autonomously
 *   and the host only reads version/capabilities (the old experimental
 *   request_firmware upload path is removed; it targeted a window that
 *   aliases host DRAM).
 * - MMIO discovery (REA 1.1, QcUsb4Filter8380 FUN_140018470): the router
 *   MMIO base is NOT in _CRS. Windows scans the DSDT AML for the literal
 *   "SOSI" signature and reads the u64 physical base at SOSI+5; the
 *   header is 0xb8 bytes and the full region size is
 *   (u32@0xa4 + u32@0x9c + 0x2e) * 4 bytes; register access uses an
 *   indexed u32 window at SOSI_base + u32@0xa0. Until a DT binding
 *   publishes these regions, this scaffold validates named DT resources
 *   only (fail-closed) with sizes from DT, not fixed constants.
 * - Stop/restart ownership (REA 1.4/1.5): on disconnect the router
 *   sequence is PHY standby (RXSTANDBY) -> drive SBTX low -> sideband
 *   slot reset ("CS/SB registers are reset. Disconnect completed"); on
 *   resume it is wake FSM -> HSE retrain phases (lnk_hse_p1..p5) ->
 *   SB reconnect ("Enable SBTX") -> DROM revalidation. This frontend
 *   implements NONE of that physical sequence yet: stop() only unwinds
 *   host-side state in strict reverse activation order (nhi_remove ->
 *   devres groups -> phy_exit -> resets -> clocks) and the physical
 *   disconnect/resume sequence remains unqualified.
 * - Activation is serialised under hr->lock and one-shot. After
 *   qcom_usb4_hr_stop() (remove/shutdown), state is STOPPED and cannot
 *   be re-entered; if any physical unwind step fails, cleanup_error is
 *   recorded, state becomes FAILED and restart is refused. The PHY
 *   itself may still be held by its own driver for the USB/DP alternate
 *   paths, which is why we do not touch its resources in remove()
 *   beyond phy_exit().
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/iopoll.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/soc/qcom/smem.h>
#include <linux/soc/qcom/socinfo.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "nhi.h"
#include "qcom_usb4_hr.h"
#include "tb.h"
#include "tb_regs.h"

struct qcom_usb4_hr_reg {
	const char *name;
	resource_size_t min_size;
};

/* Cover existing accesses, including the optional frontend diagnostics. */
static const struct qcom_usb4_hr_reg qcom_usb4_hr_regs[] = {
	{ "router", 0x20ac },
	{ "router_config", 0x1c },
	{ "tmu_config", 0 },
	{ "port_group", 0x68 },
	{ "sideband", 0x40 },
	{ "uc_ram", 56 * SZ_1K },
	{ "uc_per", 0x1004 },
	{ "uc_mbox", 0 },
	/* Retain the complete Qualcomm NHI register aperture for the core. */
	{ "nhi", SZ_256K },
	{ "cfg", 0 },
	{ "debug", 0 },
	{ "usbap_config", 0x18 },
	{ "pcieap_config", 0 },
	{ "dpap0_aux", 0 },
	{ "dpap0_config", 0 },
	{ "dpap1_aux", 0 },
	{ "dpap1_config", 0 },
};

static const char * const qcom_usb4_hr_clocks[] = {
	"sys", "tmu", "ahb", "axi", "master", "phy_rx0", "phy_rx1",
	"sb", "dp0", "dp1",
};

static const char * const qcom_usb4_hr_resets[] = {
	"core", "phy_nocsr", "sys", "rx0", "rx1", "usb_pipe",
	"pcie_pipe", "tmu", "sideband_iface", "hia_master", "ahb",
	"dp0", "dp1",
};

enum qcom_usb4_hr_state {
	QCOM_USB4_HR_RESOURCE_ONLY,
	QCOM_USB4_HR_PREPARED,
	QCOM_USB4_HR_ACTIVATING,
	QCOM_USB4_HR_ACTIVE,
	QCOM_USB4_HR_FAILED,
	/* Frontend entry points stopped, not a physical power-state guarantee. */
	QCOM_USB4_HR_STOPPED,
};

struct qcom_usb4_hr {
	/* Serializes preparation, activation and all frontend diagnostics. */
	struct mutex lock;
	enum qcom_usb4_hr_state state;
	void *prepare_group;
	void *activation_group;
	bool stopping;
	bool cleanup_attempted;
	int cleanup_error;
	void __iomem *regs[ARRAY_SIZE(qcom_usb4_hr_regs)];
	void __iomem *ucsb;      /* UCS0 mailbox @ 0x81F20040, ACPI QCOM0CA4 */
	int uc_irq;
	void __iomem *win;       /* ACPI PRT0 flat window, window0 base */
	struct clk_bulk_data clocks[ARRAY_SIZE(qcom_usb4_hr_clocks)];
	struct reset_control_bulk_data resets[ARRAY_SIZE(qcom_usb4_hr_resets)];
	struct phy *usb4_phy;
	struct tb_nhi nhi;
	int ring_irq;
	int fw_irq;
	bool ring_irq_requested;
	bool fw_irq_requested;
	bool uc_irq_requested;
	bool clocks_enabled;
	bool resets_deasserted;
	bool phy_initialized;
};

static int qcom_usb4_hr_map_resources(struct qcom_usb4_hr *hr,
				      struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int i;

	/*
	 * MMIO contract (qualification area a):
	 *
	 * Every named region is validated for presence, size and alignment
	 * BEFORE any part of the controller is mapped, then mapped through
	 * devm_platform_ioremap_resource_byname() so the mapping is devres
	 * owned and released with the platform device. All accesses in this
	 * driver are 32-bit readl/writel, so each region must start on a
	 * 4-byte boundary and cover a whole number of words; min_size comes
	 * from the highest offset+access-width actually used per region.
	 *
	 * There is no in-tree DT node for the host router (the binding is
	 * research-only), so a mismatch here fails probe of this scaffold
	 * only and cannot affect a normal boot.
	 */
	for (i = 0; i < ARRAY_SIZE(qcom_usb4_hr_regs); i++) {
		const struct qcom_usb4_hr_reg *reg = &qcom_usb4_hr_regs[i];
		struct resource *res;
		resource_size_t size;

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   reg->name);
		if (!res || res->end < res->start)
			return dev_err_probe(dev, -EINVAL,
					     "invalid MMIO resource %s\n", reg->name);

		size = resource_size(res);
		if (!size || size < reg->min_size)
			return dev_err_probe(dev, -EINVAL,
					     "MMIO resource %s is too small\n",
					     reg->name);
		if (!IS_ALIGNED(res->start, sizeof(u32)) ||
		    size % sizeof(u32))
			return dev_err_probe(dev, -EINVAL,
					     "MMIO resource %s is not 32-bit aligned\n",
					     reg->name);
	}

	for (i = 0; i < ARRAY_SIZE(qcom_usb4_hr_regs); i++) {
		hr->regs[i] = devm_platform_ioremap_resource_byname(
			pdev, qcom_usb4_hr_regs[i].name);
		if (IS_ERR(hr->regs[i]))
			return dev_err_probe(dev, PTR_ERR(hr->regs[i]),
					     "cannot map MMIO resource %s\n",
					     qcom_usb4_hr_regs[i].name);
	}

	return 0;
}

static void qcom_usb4_hr_context_release(struct device *dev, void *res)
{
	/* devres frees this allocation; hardware teardown is separate. */
}

static struct qcom_usb4_hr *qcom_usb4_hr_get(struct device *dev)
{
	/* The common NHI core owns dev->driver_data once activated. */
	return devres_find(dev, qcom_usb4_hr_context_release, NULL, NULL);
}

static inline struct qcom_usb4_hr *qcom_usb4_hr_from_nhi(struct tb_nhi *nhi)
{
	return container_of(nhi, struct qcom_usb4_hr, nhi);
}

static irqreturn_t qcom_usb4_hr_fw_irq(int irq, void *data);
static irqreturn_t qcom_usb4_hr_fw_thread(int irq, void *data);
static irqreturn_t qcom_usb4_hr_uc_irq(int irq, void *data);

static void qcom_usb4_hr_shutdown_interrupts(struct tb_nhi *nhi)
{
	struct qcom_usb4_hr *hr = qcom_usb4_hr_from_nhi(nhi);

	/* Quiesce every producer before draining work or releasing its storage. */
	if (hr->uc_irq_requested) {
		devm_free_irq(nhi->dev, hr->uc_irq, hr);
		hr->uc_irq_requested = false;
	}
	if (hr->fw_irq_requested) {
		devm_free_irq(nhi->dev, hr->fw_irq, hr);
		hr->fw_irq_requested = false;
	}
	if (hr->ring_irq_requested) {
		devm_free_irq(nhi->dev, hr->ring_irq, nhi);
		hr->ring_irq_requested = false;
	}
	flush_work(&nhi->interrupt_work);
}

static int qcom_usb4_hr_init_interrupts(struct tb_nhi *nhi)
{
	struct qcom_usb4_hr *hr = qcom_usb4_hr_from_nhi(nhi);
	int ret;

	ret = devm_request_irq(nhi->dev, hr->ring_irq, nhi_msi,
			       IRQF_NO_SUSPEND, "thunderbolt", nhi);
	if (ret)
		return ret;
	hr->ring_irq_requested = true;

	/*
	 * The UC's event/notification line; the common NHI code has no use
	 * for it yet, but log activity so connect events become visible.
	 */
	ret = devm_request_threaded_irq(nhi->dev, hr->fw_irq,
					qcom_usb4_hr_fw_irq,
					qcom_usb4_hr_fw_thread,
					IRQF_NO_SUSPEND | IRQF_ONESHOT,
					"thunderbolt-fw", hr);
	if (ret)
		goto err_shutdown;
	hr->fw_irq_requested = true;

	if (hr->uc_irq >= 0) {
		ret = devm_request_irq(nhi->dev, hr->uc_irq,
				       qcom_usb4_hr_uc_irq, 0,
				       "qcom-usb4-hr-uc", hr);
		if (ret)
			goto err_shutdown;
		hr->uc_irq_requested = true;
	}
	return 0;

err_shutdown:
	qcom_usb4_hr_shutdown_interrupts(nhi);
	return ret;
}

static bool qcom_usb4_hr_is_present(struct tb_nhi *nhi)
{
	return !nhi->going_away;
}

static const struct tb_nhi_ops qcom_usb4_hr_nhi_ops = {
	.is_present = qcom_usb4_hr_is_present,
	.init_interrupts = qcom_usb4_hr_init_interrupts,
	.shutdown = qcom_usb4_hr_shutdown_interrupts,
};

/*
 * ACPI PRT0 _CRS gives ONE flat window (default 0x1563F000, 768K) and the
 * Windows filter addresses everything relative to it: fw at +0x13000,
 * uc ctl at +0x22000, gates at +0xd064/+0x18, ready at +0x18. The DTB's
 * separate low-block regions (0x15600000..0x15624fff) have no ACPI
 * backing; reads there alias host DRAM.
 */
/*
 * Windows arms the UC's interrupt controller as a separate step after the
 * UC boots (decoded from QcUsb4Filter function 0x14000ba7c): five
 * interrupt sources at router+0x20a8, port_group bits 21/22/23 at
 * +0xd000, and router config bits. Without it the UC never signals the
 * host - which is why the uc IRQ stayed silent through every plug test.
 * Runtime-toggleable because a near-identical write once regressed the
 * port1 USB path.
 */
static bool arm_int;
module_param_named(arm_int, arm_int, bool, 0644);
MODULE_PARM_DESC(arm_int, "apply the Windows UC interrupt-arming sequence after ready");

static unsigned long long window0 = 0x1563F000;
module_param(window0, ullong, 0444);
MODULE_PARM_DESC(window0, "USB4 router flat window physical base");

static int __qcom_usb4_hr_nhi_preflight(struct qcom_usb4_hr *hr,
					struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	void __iomem *ucsb;
	struct phy *usb4_phy;
	void *group;
	int ring_irq, fw_irq, uc_irq;
	int ret;

	lockdep_assert_held(&hr->lock);
	if (hr->state == QCOM_USB4_HR_PREPARED)
		return 0;
	if (hr->state != QCOM_USB4_HR_RESOURCE_ONLY)
		return -EBUSY;

	group = devres_open_group(dev, NULL, GFP_KERNEL);
	if (!group)
		return -ENOMEM;

	ring_irq = platform_get_irq_byname(pdev, "ring");
	if (ring_irq < 0) {
		ret = ring_irq;
		goto err_release;
	}
	fw_irq = platform_get_irq_byname(pdev, "fw");
	if (fw_irq < 0) {
		ret = fw_irq;
		goto err_release;
	}

	usb4_phy = devm_phy_get(dev, "usb4");
	if (IS_ERR(usb4_phy)) {
		ret = dev_err_probe(dev, PTR_ERR(usb4_phy),
				    "USB4 PHY is not available\n");
		goto err_release;
	}
	uc_irq = platform_get_irq_byname_optional(pdev, "uc");
	if (uc_irq < 0 && uc_irq != -ENXIO) {
		ret = uc_irq;
		goto err_release;
	}
	if (uc_irq < 0)
		dev_warn(dev, "no optional UC interrupt\n");

	/*
	 * ACPI QCOM0CA4 declares the UCS0 shared mailbox as a separate
	 * OperationRegion (USBC @ 0x81F20040, 0x2F bytes). The DT node for
	 * the host router does not carry it, so map it through a named
	 * optional resource when present and otherwise fall back to the
	 * fixed ACPI-declared physical window. Never map outside devres
	 * management: the mapping lives in the prepare group and is torn
	 * down with the rest of the preflight state.
	 */
	{
		struct resource *ucsb_res;
		void __iomem *map;

		ucsb_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"ucsb");
		if (ucsb_res) {
			map = devm_ioremap_resource(dev, ucsb_res);
			if (IS_ERR(map)) {
				ret = PTR_ERR(map);
				goto err_release;
			}
		} else {
			map = devm_ioremap(dev, 0x81F20040, 0x40);
			if (!map)
				dev_warn(dev,
					 "could not map UCS0 mailbox\n");
		}
		ucsb = map;
	}

	devres_close_group(dev, group);
	hr->prepare_group = group;
	hr->ring_irq = ring_irq;
	hr->fw_irq = fw_irq;
	hr->uc_irq = uc_irq;
	hr->usb4_phy = usb4_phy;
	hr->ucsb = ucsb;
	hr->nhi.iobase = hr->regs[8];
	hr->nhi.ops = &qcom_usb4_hr_nhi_ops;
	hr->state = QCOM_USB4_HR_PREPARED;

	dev_info(dev,
		 "NHI preflight passed: aperture=%p ring_irq=%d fw_irq=%d; activation=%s\n",
		 hr->nhi.iobase, hr->ring_irq, hr->fw_irq,
		 device_property_read_bool(dev, "qcom,nhi-activate") ?
		 "requested" : "disabled");
	return 0;

err_release:
	devres_release_group(dev, group);
	return ret;
}

static int qcom_usb4_hr_nhi_preflight(struct qcom_usb4_hr *hr,
				      struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	/*
	 * The common NHI code is ready for platform users, but Qualcomm's
	 * router-specific firmware upload and power-on sequence is not. Keep
	 * this check explicitly opt-in so a DT node cannot accidentally turn
	 * the resource validator into a boot-critical driver.
	 */
	if (!device_property_read_bool(dev, "qcom,nhi-preflight") &&
	    !device_property_read_bool(dev, "qcom,nhi-activate"))
		return 0;

	return __qcom_usb4_hr_nhi_preflight(hr, pdev);
}

static irqreturn_t qcom_usb4_hr_fw_irq(int irq, void *data)
{
	struct qcom_usb4_hr *hr = data;

	dev_dbg(hr->nhi.dev, "fw_irq fired (irq %d)\n", irq);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t qcom_usb4_hr_fw_thread(int irq, void *data)
{
	struct qcom_usb4_hr *hr = data;

	dev_info(hr->nhi.dev, "fw_irq thread: uc_per status %08x %08x %08x\n",
		 readl(hr->regs[6] + 0x00), readl(hr->regs[6] + 0x04),
		 readl(hr->regs[6] + 0x1000));
	return IRQ_HANDLED;
}

static u32 qcom_usb4_hr_rmw(void __iomem *addr, u32 value, u32 mask)
{
	u32 val = readl(addr);

	val &= ~mask;
	val |= value & mask;
	writel(val, addr);
	return val;
}

/*
 * 2026-08-26: fires here hard-lock the SoC silently (verified by netconsole
 * capture). Off until the correct X1P-aware way to release the clamp exists.
 */
static bool uc_skip_clamp = true;
module_param_named(skip_clamp, uc_skip_clamp, bool, 0444);
MODULE_PARM_DESC(skip_clamp,
		 "do not touch the raw USB4 AON clamp register (deadly on X1P)");

/*
 * 2026-08-26: a plain readl(usbap_config+0x00) also hard-locks the SoC
 * (verified by netconsole) - the aperture is unpowered in our state and
 * Windows only reaches it behind platform glue we don't run. Default off.
 */
static bool uc_usbap_allow;
module_param_named(usbap_allow, uc_usbap_allow, bool, 0444);
MODULE_PARM_DESC(usbap_allow,
		 "allow the deadly raw usbap_config RMW experiment");

/*
 * 2026-11-14 REA evidence (evidence/rea-usb4-20261114.md, QcUsb4Filter8380
 * full-corpus decompile): there is NO host-side firmware loading for this
 * router. The router boots autonomously from its own storage; the host only
 * reads version/capabilities from the SOSI-published MMIO region. The old
 * experimental upload path is removed: it wrote into a window that aliases
 * host DRAM (2026-08-26 round-9 finding) and no upload path exists in the
 * Windows drivers either.
 */


/*
 * === 2026-11-28 correction to the 2026-11-14 REA note above ===
 * The claim "there is NO host-side firmware loading" is now REFUTED by
 * direct evidence: (1) we hold the actual X1E80100 router MCU image,
 * carved out of the Microsoft-published QcUsb4Filter8380.sys at 0x4b9c0
 * (40,816 B, sha256 cd4f5929b51f2dbb0b583693ff8d024521c87f0d2e45c7adc142fed976650b99,
 * records (0x0000, 7840 words) + (0x8000, 2360 words)); (2) the
 * Glymur-series vendor-derived router driver shows the exact host load
 * path (memcpy into the uc_ram aperture, DROM at ram+0xdf20, RUN bit at
 * uc_per+0x1000). uc_ctl BIT(0) == 0 at every boot we ever observed is
 * consistent with "nobody ever loaded the firmware", not with an
 * autonomous boot. See evidence/usb4-hr-stage1-2026-11.md.
 *
 * arch1-18: first MCU bring-up attempt, minimal vendor sequence on top
 * of an already clocked, domain-restored router. Legacy MCU layout
 * offsets (absolute from the router root at 0x15600000):
 *   RAM 0x13000        -> regs[5] "uc_ram"  (window 0x15613000, len 0xe000)
 *   CTRL 0x22000       -> regs[6] "uc_per" (0x15621000) + 0x1000
 *   mailbox CTRL+0xc, shared-offset reg CTRL+0x10
 *   STATUS 0x18        -> regs[0] "router" + 0x18, BIT(24) = ready
 *   HOST_CONTROL 0x8018-> regs[1] "router_config" + 0x18, BIT(24)
 *   UID_HIGH/LOW 0x801c/0x8020 -> regs[1] + 0x1c / +0x20
 * Deliberately NOT done yet (x1e offsets unconfirmed on our board):
 * GCC rx-mux selects, CBCR mem-force, TCSR 0xb004, router 0x81000-pad
 * pulses and the adapter timer writes. Add them only if READY never
 * arrives here.
 */
#define UC_FW_NAME		"qcom/x1e80100/usb4-router-legacy.bin"
#define UC_RAM_SIZE		0xdf20u /* stop before the DROM area */
#define UC_DROM_OFFSET		0xdf20u
#define UC_DROM_SIZE		116
#define UC_MCU_READY		BIT(24)
#define UC_MCU_RUN		BIT(0)
#define UC_PRESET_CMD		(2U | (0x4421U << 8))

/* Vendor DROM body: revision/length, legacy IDs, adapters, product strings. */
static const u8 uc_drom_body[UC_DROM_SIZE - 13] = {
	0x01, 0x67, 0x00, 0x28, 0x03, 0x00, 0x00, 0x00, 0x00, 0x05,
	0x86, 0x00, 0x00, 0x00, 0x05, 0x87, 0x00, 0x00, 0x00, 0x1e,
	0x01, 0x51, 0x75, 0x61, 0x6c, 0x63, 0x6f, 0x6d, 0x6d, 0x20,
	0x54, 0x65, 0x63, 0x68, 0x6e, 0x6f, 0x6c, 0x6f, 0x67, 0x69,
	0x65, 0x73, 0x2c, 0x20, 0x49, 0x6e, 0x63, 0x2e, 0x00, 0x11,
	0x02, 0x53, 0x6e, 0x61, 0x70, 0x64, 0x72, 0x61, 0x67, 0x6f,
	0x6e, 0x20, 0x30, 0x31, 0x42, 0x00, 0x0f, 0x09, 0x10, 0x04,
	0xc6, 0x05, 0x38, 0x09, 0x00, 0x00, 0xeb, 0x34, 0x00, 0x00,
	0x00, 0x08, 0x82, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x08,
	0x83, 0x90, 0x02, 0x00, 0x00, 0x00, 0x00, 0x03, 0x84, 0x00,
	0x03, 0x08, 0x06,
};

static u8 uc_crc8(const u8 *data)
{
	u8 crc = 0xff;
	unsigned int i, bit;

	for (i = 0; i < 8; i++) {
		crc ^= data[i];
		for (bit = 0; bit < 8; bit++)
			crc = (crc << 1) ^ ((crc & 0x80) ? 0x07 : 0);
	}
	return crc;
}

/* Table-less crc32c (Castagnoli, reflected), 103 bytes: cost irrelevant. */
static u32 uc_crc32c(const u8 *data, size_t len)
{
	u32 crc = ~0U;
	size_t i;
	unsigned int bit;

	for (i = 0; i < len; i++) {
		crc ^= *data++;
		for (bit = 0; bit < 8; bit++)
			crc = (crc >> 1) ^ (0x82f63b78U & -(int)(crc & 1));
	}
	return ~crc;
}

static u32 uc_soc_serial(struct device *dev)
{
	struct socinfo *info;
	size_t item_size = 0;
	void *p;

	p = qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_HW_SW_BUILD_ID, &item_size);
	if (IS_ERR(p) ||
	    offsetofend(struct socinfo, serial_num) > item_size) {
		dev_warn(dev, "hr-mcu: no SMEM serial (size %zu), using 0\n",
			 item_size);
		return 0;
	}
	info = p;
	return le32_to_cpu(info->serial_num);
}

static int qcom_usb4_hr_uc_load(struct qcom_usb4_hr *hr, struct device *dev)
{
	const struct firmware *fw;
	void __iomem *ram = hr->regs[5];
	void __iomem *ctrl = hr->regs[6] + 0x1000;
	void __iomem *status = hr->regs[0] + 0x18;
	void __iomem *host_ctrl = hr->regs[1] + 0x18;
	u8 drom[UC_DROM_SIZE];
	u64 uid;
	u32 serial, val, shared;
	size_t pos = 0, n, fwsize;
	int ret;

	if (readl(ctrl) & UC_MCU_RUN) {
		dev_info(dev, "hr-mcu: MCU already running (ctrl=%#x)\n",
			 readl(ctrl));
		return 0;
	}

	ret = request_firmware(&fw, UC_FW_NAME, dev);
	if (ret)
		return dev_err_probe(dev, ret, "hr-mcu: firmware %s not found\n",
				     UC_FW_NAME);

	/* Upload: zero control, then walk [addr, words, data] records. */
	fwsize = fw->size;
	writel(0, ctrl);
	while (pos + 8 <= fw->size) {
		u32 addr = get_unaligned_le32(fw->data + pos);
		u32 words = get_unaligned_le32(fw->data + pos + 4);

		if (!words || words > 0x8000 ||
		    addr > UC_RAM_SIZE || words * 4 > UC_RAM_SIZE - addr ||
		    pos + 8 + (size_t)words * 4 > fw->size) {
			ret = -EPROTO;
			break;
		}
		dev_info(dev, "hr-mcu: record addr=%#x words=%u\n", addr, words);
		for (n = 0; n < words; n++)
			writel(get_unaligned_le32(fw->data + pos + 8 + n * 4),
			       ram + addr + n * 4);
		pos += 8 + (size_t)words * 4;
	}
	if (ret || pos != fw->size) {
		release_firmware(fw);
		return dev_err_probe(dev, ret ? ret : -EPROTO,
				     "hr-mcu: firmware records malformed at %zu\n",
				     pos);
	}
	release_firmware(fw);
	readl(ctrl); /* flush posted writes */

	/* DROM: uid = 0x5c6:serial:router with crc8 and a crc32c over the body. */
	serial = uc_soc_serial(dev);
	uid = (0x5c6ULL << 48) | ((u64)serial << 4) | 0; /* router index 0 */
	put_unaligned_le64(uid, drom + 1);
	drom[0] = uc_crc8(drom + 1);
	memcpy(drom + 13, uc_drom_body, sizeof(uc_drom_body));
	put_unaligned_le32(uc_crc32c(drom + 13, sizeof(uc_drom_body)), drom + 9);

	writel(get_unaligned_le32(drom + 5), hr->regs[1] + 0x1c);
	writel(get_unaligned_le32(drom + 1), hr->regs[1] + 0x20);
	for (n = 0; n < UC_DROM_SIZE; n += 4)
		writel(get_unaligned_le32(drom + n), ram + UC_DROM_OFFSET + n);
	readl(ctrl);
	dev_info(dev, "hr-mcu: uploaded %zu B, drom uid=%#llx serial=%u\n",
		 fwsize, uid, serial);

	/* Start: clear ready, run, poll STATUS.BIT(24). */
	qcom_usb4_hr_rmw(host_ctrl, 0, UC_MCU_READY);
	qcom_usb4_hr_rmw(ctrl, UC_MCU_RUN, UC_MCU_RUN);
	ret = readl_poll_timeout(status, val, val & UC_MCU_READY, 5000,
				 1000000);
	dev_info(dev, "hr-mcu: start poll status=%#x ctrl=%#x (%d)\n",
		 val, readl(ctrl), ret);
	if (ret) {
		qcom_usb4_hr_rmw(ctrl, 0, UC_MCU_RUN);
		qcom_usb4_hr_rmw(host_ctrl, 0, UC_MCU_READY);
		return ret;
	}

	shared = readl(ctrl + 0x10);
	if (shared % 4 || shared > UC_RAM_SIZE - 12) {
		dev_warn(dev, "hr-mcu: shared offset implausible %#x\n", shared);
		return -EPROTO;
	}
	dev_info(dev, "hr-mcu: MCU running, shared offset=%#x\n", shared);

	/* Preset command via mailbox; MCU zeroes it when consumed. */
	writel(UC_PRESET_CMD, ctrl + 0xc);
	ret = readl_poll_timeout(ctrl + 0xc, val, !val, 1000, 1000000);
	dev_info(dev, "hr-mcu: preset 0x4421 mailbox=%#x (%d)\n", val, ret);
	return ret;
}

static int qcom_usb4_hr_uc_bringup(struct qcom_usb4_hr *hr)
{
	void __iomem *uc_ctl = hr->regs[6] + 0x1000; /* uc_per + 0x1000 */
	u32 val;

	/*
	 * Corrected 2026-11-28: the old 2026-11-14 REA note claiming the UC
	 * boots autonomously is refuted. The MCU image rides inside the
	 * vendor Windows driver (QcUsb4Filter8380.sys) and must be uploaded
	 * by the host; uc_ctl BIT(0) reading 0 on every Linux boot simply
	 * meant nobody ever loaded it. With qcom_usb4_hr_uc_load() run at
	 * step4b, BIT(0) here is the post-load verdict.
	 */
	val = readl(uc_ctl);
	dev_info(hr->nhi.dev, "hr-bring: uc_ctl status=%#x\n", val);
	if (val & BIT(0)) {
		/*
		 * Warm start: the UC is already executing (it booted from
		 * its own storage - we never load firmware). Do NOT halt
		 * it; just fall through to gate clearing so the router
		 * reaches a ring-protocol-accessible state.
		 */
		dev_info(hr->nhi.dev,
			 "USB4 UC already running (autonomous boot); not halting\n");
	} else {
		/*
		 * [sp11] Round 3 (2026-11-27): the domain power-cycle now runs
		 * earlier, in qcom_usb4_hr_activate() step2a, at the one moment
		 * this GDSC provably has no other consumers. What is left here
		 * is the verdict: gates reset on domain power-on, so clear the
		 * two known gates and poll for the UC.
		 */
		qcom_usb4_hr_rmw(hr->regs[3] + 0x64, 0, BIT(6));   /* port_group */
		qcom_usb4_hr_rmw(hr->regs[1] + 0x18, 0, BIT(24));  /* router_config */
		if (readl_poll_timeout(uc_ctl, val, val & BIT(0),
				       250000, 5000000)) {
			dev_warn(hr->nhi.dev,
				 "hr-bring: UC not executing (uc_ctl=%#x after 5s poll)\n",
				 val);
			return -ENODEV;
		}
		dev_info(hr->nhi.dev, "hr-bring: UC is executing (uc_ctl=%#x)\n",
			 val);
	}
	/* Clear the two gates, then release the UC. */
	dev_info(hr->nhi.dev, "hr-bring: clearing port_group gate\n");
	/*
	 * Round-17: the two extra writes from the Windows trace (port_group
	 * 0x7800 config + router_config BIT0 enable) did NOT unlock the UC
	 * sideband and BROKE the port1 USB3 path (usb4-port1 enable fail).
	 * REVERTED to the minimal proven flow: two gates only.
	 */
	qcom_usb4_hr_rmw(hr->regs[3] + 0x64, 0, BIT(6));   /* port_group */
	dev_info(hr->nhi.dev, "hr-bring: clearing router_config gate\n");
	qcom_usb4_hr_rmw(hr->regs[1] + 0x18, 0, BIT(24));  /* router_config */
	/*
	 * The UC was already running, so no GO reissue and no ready poll:
	 * ready (window0+0x18 BIT24) is a boot-done pulse consumed at the
	 * autonomous boot, not something the host can wait for here.
	 */

	/*
	 * UC interrupt/config arming (Windows order, values exact):
	 * +0x4/+0x2/+0x14 config bits, +0xd000 port_group bits 21-23,
	 * and the five interrupt sources at router+0x20a8.
	 */
	if (arm_int) {
		dev_info(hr->nhi.dev,
			 "hr-bring: arming UC interrupts (Windows sequence)\n");
		qcom_usb4_hr_rmw(hr->regs[0] + 0x4, 0x0, 0x1);
		qcom_usb4_hr_rmw(hr->regs[0] + 0x2, 0x5, 0x1);
		qcom_usb4_hr_rmw(hr->regs[3] + 0x0, 0x0, 0x800000);
		qcom_usb4_hr_rmw(hr->regs[3] + 0x0, 0x0, 0x400000);
		qcom_usb4_hr_rmw(hr->regs[3] + 0x0, 0x0, 0x200000);
		qcom_usb4_hr_rmw(hr->regs[0] + 0x4, 0x0, 0x3f00);
		qcom_usb4_hr_rmw(hr->regs[0] + 0x14, 0x0, 0x4);
		qcom_usb4_hr_rmw(hr->regs[0] + 0x14, 0x0, 0x2);
		qcom_usb4_hr_rmw(hr->regs[0] + 0x20a8, 0x0, 0x80000);
		qcom_usb4_hr_rmw(hr->regs[0] + 0x20a8, 0x0, 0x20000);
		qcom_usb4_hr_rmw(hr->regs[0] + 0x20a8, 0x0, 0x10000);
		qcom_usb4_hr_rmw(hr->regs[0] + 0x20a8, 0x0, 0x10);
		qcom_usb4_hr_rmw(hr->regs[0] + 0x20a8, 0x0, 0x8);
		qcom_usb4_hr_rmw(hr->regs[0] + 0x2, 0xc, 0xc);
		dev_info(hr->nhi.dev, "hr-bring: UC interrupts armed\n");
	}
	return 0;
}

static void qcom_usb4_hr_release_activation(struct qcom_usb4_hr *hr)
{
	struct device *dev = hr->nhi.dev;
	int ret;

	lockdep_assert_held(&hr->lock);
	if (hr->cleanup_attempted)
		return;
	hr->cleanup_attempted = true;
	if (hr->state == QCOM_USB4_HR_ACTIVE)
		nhi_remove(&hr->nhi);

	/* Domain release and IRQ/work quiescence precede managed storage. */
	if (hr->activation_group) {
		devres_release_group(dev, hr->activation_group);
		hr->activation_group = NULL;
	}
	hr->nhi.tx_rings = NULL;
	hr->nhi.rx_rings = NULL;
	hr->nhi.hop_count = 0;

	/* Keep the existing PHY/reset/clock order; no physical recovery claim. */
	if (hr->phy_initialized) {
		ret = phy_exit(hr->usb4_phy);
		if (ret) {
			dev_warn(dev, "USB4 PHY exit failed: %d\n", ret);
			if (!hr->cleanup_error)
				hr->cleanup_error = ret;
		} else {
			hr->phy_initialized = false;
		}
	}
	if (hr->resets_deasserted) {
		ret = reset_control_bulk_assert(ARRAY_SIZE(hr->resets), hr->resets);
		if (ret) {
			dev_warn(dev, "host-router reset assertion failed: %d\n", ret);
			if (!hr->cleanup_error)
				hr->cleanup_error = ret;
		} else {
			hr->resets_deasserted = false;
		}
	}
	if (hr->clocks_enabled) {
		clk_bulk_disable_unprepare(ARRAY_SIZE(hr->clocks), hr->clocks);
		hr->clocks_enabled = false;
	}
}

/*
 * [sp11] Cold UC experiment, round 3 (2026-11-27). The SP11 ACPI tables bind
 * the router to the power-engine provider (_DEP: PEP0 in \_SB.UBF0): Windows
 * power-cycles the domain and the UC announces a cold boot at that power-on
 * (QcUsb4Filter "(%s boot)" log, no host-side FW load). arch1-5 proved the
 * syscore-oriented dev_pm_genpd_suspend/resume pair forces the domain OFF
 * (UC cold-boot moment reached) but leaves the QCOM clock framework
 * inconsistent (EBUSY on re-enable), so cycle through runtime PM instead:
 * at this point in activation no other consumer holds a GCC_USB4_0_*
 * reference (our clocks were the only ones, the QMP PHY is not yet
 * initialized), so pm_runtime_put_sync() on the single reference probe
 * holds lets the core idle-suspend the device (genpd powers the domain
 * down; arch1-6 proved this transition works), and pm_runtime_get_sync()
 * powers it back up with correct bookkeeping. The UC restarts its boot at
 * that edge.
 */
static int qcom_usb4_hr_domain_power_cycle(struct qcom_usb4_hr *hr,
					   struct device *dev)
{
	void __iomem *gcc;
	u32 val, sel, bcr;
	int ret;

	/*
	 * arch1-15: the Yoga Slim 7x bring-up (same X1E80100 silicon)
	 * published exactly our stuck signature (gdscr=0x6822f800,
	 * cfg_gdscr=0x050e0000, "status stuck at 'off'", start -110) and
	 * the two conditions under which the domain powers up again:
	 *   1) GCC_PCIE_0_TUNNEL_BCR (GCC 0xa0000) is released, not held,
	 *   2) the gcc_usb4_0_phy_sys_clk_src selector (GCC 0x9f0e4,
	 *      bits [1:0]) reads input 0, not 2. The in-tree phy-mux clock
	 *      driver parks this mux at 2 whenever the clock is off, and
	 *      with 2 selected the GDSC ON handshake never completes.
	 * Their recovered runs also kept the router's own resets released
	 * across the power-off, so we no longer assert them here. We poke
	 * both registers directly with the same field encodings the gcc
	 * and reset drivers use; nothing in this DT consumes the tunnel
	 * BCR or the mux, so there is no other owner to disturb.
	 */
	gcc = ioremap(0x00100000, 0xb0000);	/* full GCC block */
	if (!gcc)
		return -ENOMEM;

	bcr = readl(gcc + 0xa0000);
	sel = readl(gcc + 0x9f0e4);
	dev_info(dev, "hr-act: pre-edge gcc gdscr=%#x cfg=%#x phy_sys=%#x tunnel_bcr=%#x\n",
		 readl(gcc + 0x9f004), readl(gcc + 0x9f008), sel, bcr);

	if (bcr & BIT(0)) {
		writel(0, gcc + 0xa0000);
		dev_info(dev, "hr-act: tunnel BCR %#x -> released\n", bcr);
	}
	if (sel & 0x3) {
		writel(sel & ~0x3u, gcc + 0x9f0e4);
		dev_info(dev, "hr-act: phy_sys_clk_src selector %u -> 0\n",
			 sel & 0x3);
	}

	/*
	 * Clean-edge recipe (Yoga restart flow): the domain must go off
	 * with NO loaded consumers. Their recovered runs drop every clock
	 * and vote before power-off and never power-on with clocks enabled.
	 * We currently hold ten prepared+enabled interface clocks at this
	 * point, and this device shows the damage signature: cfg_gdscr
	 * walks from 0x04070000 (healthy ON) to 0x050e0000 (bits 24/19 up,
	 * 16 down) exactly across the forced collapse, then the ON
	 * handshake never completes. Unprepare everything first.
	 */
	clk_bulk_disable(ARRAY_SIZE(hr->clocks), hr->clocks);
	clk_bulk_unprepare(ARRAY_SIZE(hr->clocks), hr->clocks);

	pm_runtime_put_sync(dev);
	usleep_range(3000000, 3100000);
	dev_info(dev, "hr-act: post-off gdscr=%#x cfg=%#x\n",
		 readl(gcc + 0x9f004), readl(gcc + 0x9f008));

	/*
	 * arch1-17: arch1-16 proved the clean edge completes (post-off
	 * gdscr=0x22f801, a real off-ack) but the power-up request still
	 * stalls, and the failure dump showed phy_sys parked at 2: the
	 * phy-mux clock driver RE-PARKS the selector to 2 when we
	 * unprepare the clocks, i.e. after the arch1-15 pre-edge poke. The mux must
	 * read 0 at the moment the ON request is issued, so force it
	 * here, on the clean-off domain, immediately before
	 * pm_runtime_get_sync(). Vendor restart likewise does PCIe prep
	 * first: the pcie_0 tunnel genpd (gcc_pcie_0_tunnel_gdsc,
	 * gdscr 0xa0004) is powered and held by the tiny
	 * qcom,x1e80100-usb4-tunnel-pd consumer below before we ask for
	 * the router domain.
	 */
	writel(readl(gcc + 0x9f0e4) & ~0x3u, gcc + 0x9f0e4);
	dev_info(dev, "hr-act: pre-on tunnels gdscr=%#x/%#x/%#x phy_sys=%#x bcr=%#x\n",
		 readl(gcc + 0xa0004), readl(gcc + 0x2c004),
		 readl(gcc + 0x13004), readl(gcc + 0x9f0e4),
		 readl(gcc + 0xa0000));

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		dev_warn(dev, "hr-act: restore failed (%d) gdscr=%#x cfg=%#x phy_sys=%#x bcr=%#x\n",
			 ret, readl(gcc + 0x9f004), readl(gcc + 0x9f008),
			 readl(gcc + 0x9f0e4), readl(gcc + 0xa0000));
		iounmap(gcc);
		/* Leave clocks unprepared on failure; teardown is idempotent. */
		return ret;
	}
	dev_info(dev, "hr-act: domain restored gdscr=%#x cfg=%#x phy_sys=%#x\n",
		 readl(gcc + 0x9f004), readl(gcc + 0x9f008),
		 readl(gcc + 0x9f0e4));
	iounmap(gcc);

	/* Re-acquire our clock references on the now-powered domain. */
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(hr->clocks), hr->clocks);
	if (ret) {
		dev_warn(dev, "hr-act: clock re-enable after restore failed (%d)\n", ret);
		return ret;
	}

	/* On a clean edge the UC boots autonomously from its own storage. */
	if (readl_poll_timeout(hr->regs[6] + 0x1000, val, val & BIT(0),
			       250000, 8000000))
		dev_warn(dev, "hr-act: UC still not executing after edge (uc_ctl=%#x)\n",
			 val);
	else
		dev_info(dev, "hr-act: UC BOOTED (uc_ctl=%#x)\n", val);

	return 0;
}

static int qcom_usb4_hr_activate(struct qcom_usb4_hr *hr,
					struct platform_device *pdev, bool force)
{
	struct device *dev = &pdev->dev;
	int ret;

	lockdep_assert_held(&hr->lock);
	if (hr->stopping)
		return -ENODEV;
	/*
	 * This is deliberately separate from the resource preflight.  The
	 * common NHI probe resets and touches the router, so only an explicitly
	 * named DT experiment may reach it.  No Qualcomm mailbox or firmware
	 * upload sequence is inferred here.  force=true bypasses the DT gate
	 * for the deferred sysfs-triggered activation.
	 */
	if (!force && !device_property_read_bool(dev, "qcom,nhi-activate"))
		return 0;
	if (hr->state == QCOM_USB4_HR_ACTIVE)
		return 0;
	if (hr->state == QCOM_USB4_HR_FAILED)
		return -EIO;
	if (hr->state == QCOM_USB4_HR_STOPPED)
		return -ENODEV;
	ret = __qcom_usb4_hr_nhi_preflight(hr, pdev);
	if (ret)
		return ret;

	hr->activation_group = devres_open_group(dev, NULL, GFP_KERNEL);
	if (!hr->activation_group)
		return -ENOMEM;
	hr->state = QCOM_USB4_HR_ACTIVATING;

	dev_info(dev, "hr-act: step0 activation entered%s\n",
		 force ? " (sysfs)" : "");

	/*
	 * Self-heal: anything that force-suspended us between probe and
	 * activation (system sleep, genpd unused sweep) is undone here
	 * through the framework, so step1 clocks always run with the
	 * domain actually powered. put_noidle brings the usage count back
	 * to the single probe reference (status stays active).
	 */
	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		goto err_release;
	pm_runtime_put_noidle(dev);

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(hr->clocks), hr->clocks);
	if (ret)
		goto err_release;
	hr->clocks_enabled = true;
	dev_info(dev, "hr-act: step1 clocks enabled\n");

	ret = reset_control_bulk_deassert(ARRAY_SIZE(hr->resets), hr->resets);
	if (ret)
		goto err_release;
	hr->resets_deasserted = true;
	dev_info(dev, "hr-act: step2 resets deasserted\n");

	/*
	 * [sp11] If the UC is not executing, cycle the router power domain
	 * now, before the PHY is engaged, while this GDSC provably has no
	 * other consumers (see qcom_usb4_hr_domain_power_cycle). The uc_ctl
	 * read needs our clocks, hence it sits after step1/step2.
	 */
	if (!(readl(hr->regs[6] + 0x1000) & BIT(0))) {
		/*
		 * [sp11 arch1-19] arch1-14..17 proved the domain cycle is a
		 * dead lever: with a clean off-ack, phy_sys pinned to 0 and
		 * the PCIe-0 tunnel genpd held ON, the GDSC ON handshake
		 * still dies (-110). The vendor cold-start flow never
		 * re-powers this domain either: it runs with the domain
		 * still ON as firmware handed it over and boots the MCU
		 * entirely in software (upload, DROM, RUN). So only cycle
		 * if the domain is actually OFF; when it is ON, go
		 * straight to the step4b software MCU boot.
		 */
		void __iomem *gcc = ioremap(0x00100000, 0xb0000);
		bool domain_on = false;

		if (gcc) {
			/* gdscr BIT(0) is the collapse state: 0 = ON */
			domain_on = !(readl(gcc + 0x9f004) & BIT(0));
			iounmap(gcc);
		}
		if (domain_on) {
			dev_info(dev, "hr-act: step2a UC down, domain ON: software MCU boot only\n");
		} else {
			dev_info(dev, "hr-act: step2a UC down, cycling router domain\n");
			ret = qcom_usb4_hr_domain_power_cycle(hr, dev);
			if (ret)
				goto err_release;
			dev_info(dev, "hr-act: step2d uc_ctl after cycle=%#x\n",
				 readl(hr->regs[6] + 0x1000));
		}
	}

	dev_info(dev, "hr-act: step3 setting PHY mode TBT3\n");
	ret = phy_set_mode_ext(hr->usb4_phy, PHY_MODE_TBT, PHY_SUBMODE_TBT3);
	if (ret)
		goto err_release;
	dev_info(dev, "hr-act: step3a phy_set_mode_ext done\n");

	ret = phy_init(hr->usb4_phy);
	if (ret)
		goto err_release;
	hr->phy_initialized = true;
	dev_info(dev, "hr-act: step4 phy_init done\n");

	if (!(readl(hr->regs[6] + 0x1000) & BIT(0))) {
		dev_info(dev, "hr-act: step4b UC firmware load (MCU bring-up)\n");
		ret = qcom_usb4_hr_uc_load(hr, dev);
		if (ret)
			goto err_release;
		dev_info(dev, "hr-act: step4b MCU ready (fw loaded)\n");
	}

	ret = qcom_usb4_hr_uc_bringup(hr);
	if (ret)
		goto err_release;
	dev_info(dev, "hr-act: step5 UC bringup done\n");

	dev_info(dev, "hr-act: step6 calling nhi_probe\n");
	ret = nhi_probe(&hr->nhi);
	if (ret)
		goto err_release;

	devres_close_group(dev, hr->activation_group);
	hr->state = QCOM_USB4_HR_ACTIVE;
	dev_info(dev,
		 "experimental Qualcomm USB4 NHI activated with UC firmware\n");
	return 0;

err_release:
	devres_close_group(dev, hr->activation_group);
	qcom_usb4_hr_release_activation(hr);
	/* A software unwind does not establish safe physical UC restart. */
	hr->state = QCOM_USB4_HR_FAILED;
	return dev_err_probe(dev, ret, "host-router activation failed\n");
}

static int qcom_usb4_hr_find_tb_domain(struct device *child, const void *data)
{
	return child->bus == &tb_bus_type;
}

/*
 * A child domain is an additional reason not to repeat bring-up. The
 * frontend state remains the readiness/ownership contract; child lookup
 * alone does not serialize activation against teardown.
 */
static bool qcom_usb4_hr_is_live(struct platform_device *pdev)
{
	struct device *tb_dev = device_find_child(&pdev->dev, NULL,
					qcom_usb4_hr_find_tb_domain);

	if (!tb_dev)
		return false;
	put_device(tb_dev);
	return true;
}

static int qcom_usb4_hr_lock_active(struct qcom_usb4_hr *hr)
{
	int ret;

	ret = mutex_lock_interruptible(&hr->lock);
	if (ret)
		return ret;
	if (hr->stopping || hr->state != QCOM_USB4_HR_ACTIVE) {
		ret = hr->state == QCOM_USB4_HR_FAILED ? -EIO : -ENODEV;
		mutex_unlock(&hr->lock);
		return ret;
	}
	return 0;
}

/*
 * Writes issue a raw config-space read to the host router, testing whether
 * the UC's command processor still answers long after the boot-time
 * handshake. Read ROUTER_CS_1 (router vendor ID) and report latency.
 */
static ssize_t uc_ping_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct qcom_usb4_hr *hr = qcom_usb4_hr_get(dev);
	struct device *tb_dev;
	struct tb *tb;
	ktime_t t0;
	int i;
	s64 us;
	u32 val = 0;
	int ret;

	if (!hr)
		return -ENODEV;
	ret = qcom_usb4_hr_lock_active(hr);
	if (ret)
		return ret;
	tb_dev = device_find_child(dev, NULL, qcom_usb4_hr_find_tb_domain);
	if (!tb_dev) {
		ret = -ENODEV;
		goto out_hr_unlock;
	}
	tb = container_of(tb_dev, struct tb, dev);
	ret = mutex_lock_interruptible(&tb->lock);
	if (ret)
		goto out_put;
	if (!tb->root_switch) {
		ret = -ENODEV;
		goto out_unlock;
	}

	t0 = ktime_get();
	ret = tb_sw_read(tb->root_switch, &val, TB_CFG_SWITCH, ROUTER_CS_1, 1);
	us = ktime_us_delta(ktime_get(), t0);

	dev_info(dev, "uc_ping: ret=%d val=%#x latency=%lld us\n",
		 ret, val, us);

	for (i = 0; i <= tb->root_switch->config.max_port_number; i++) {
		struct tb_port *port = &tb->root_switch->ports[i];
		u32 cs18 = 0, cs19 = 0;

		if (!port->cap_usb4)
			continue;
		tb_port_read(port, &cs18, TB_CFG_PORT,
			     port->cap_usb4 + PORT_CS_18, 1);
		tb_port_read(port, &cs19, TB_CFG_PORT,
			     port->cap_usb4 + PORT_CS_19, 1);
		dev_info(dev,
			 "uc_ping: port%d cap_usb4=%#x CS18=%#x CS19=%#x TIP=%d CPS=%d BE=%d\n",
			 i, port->cap_usb4, cs18, cs19,
			 !!(cs18 & PORT_CS_18_TIP), !!(cs18 & PORT_CS_18_CPS),
			 !!(cs18 & PORT_CS_18_BE));
	}
out_unlock:
	mutex_unlock(&tb->lock);
out_put:
	put_device(tb_dev);
out_hr_unlock:
	mutex_unlock(&hr->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(uc_ping);

/*
 * Replicates the Windows QcUsb4Filter post-bring-up flow: RMW of
 * usbap_config+0x10 (0x15681010) with value/mask 0xc/0xc. The static
 * trace shows this happens once after "router ready" and it is the best
 * candidate for what arms UC link training. Bracketed with markers so a
 * wedge here is attributable, like every other hardware step.
 */
static ssize_t uc_usbap_rmw_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct qcom_usb4_hr *hr = qcom_usb4_hr_get(dev);
	void __iomem *usbap;
	u32 val;
	int ret;

	if (!hr)
		return -ENODEV;
	ret = qcom_usb4_hr_lock_active(hr);
	if (ret)
		return ret;
	if (!uc_usbap_allow) {
		dev_info(dev,
			 "uc_usbap: BLOCKED (usbap_allow=0; raw read is a known instant lockup)\n");
		ret = -EPERM;
		goto out_unlock;
	}

	usbap = hr->regs[11]; /* usbap_config @ 0x15681000 */
	dev_info(dev, "hr-usbap: dumping before RMW\n");
	val = readl(usbap + 0x00);
	dev_info(dev, "hr-usbap: 0x00=%#x\n", val);
	val = readl(usbap + 0x10);
	dev_info(dev, "hr-usbap: 0x10 before=%#x\n", val);
	val = readl(usbap + 0x14);
	dev_info(dev, "hr-usbap: 0x14=%#x\n", val);

	val = qcom_usb4_hr_rmw(usbap + 0x10, 0xc, 0xc);
	dev_info(dev, "hr-usbap: RMW 0xc/0xc done, now=%#x\n",
		 readl(usbap + 0x10));
	ret = 0;
out_unlock:
	mutex_unlock(&hr->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(uc_usbap_rmw);

/*
 * Sideband-aperture dump (0x15612000, "sideband" region): the UC<->ps883x
 * SB interface state. A previous session's ucpeek read this region while
 * the domain was live without a lockup, unlike usbap_config. Each word is
 * logged immediately after its read so a fatal access is attributable to
 * one offset.
 */
static ssize_t uc_sb_dump_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct qcom_usb4_hr *hr = qcom_usb4_hr_get(dev);
	void __iomem *sb;
	int i, ret;

	if (!hr)
		return -ENODEV;
	ret = qcom_usb4_hr_lock_active(hr);
	if (ret)
		return ret;

	sb = hr->regs[4]; /* sideband @ 0x15612000 (reads may alias) */
	dev_info(dev, "hr-sb: dump start\n");
	for (i = 0; i < 0x40; i += 4) {
		u32 v = readl(sb + i);

		dev_info(dev, "hr-sb: [%#02x] = %#010x\n", i, v);
	}
	dev_info(dev, "hr-sb: dump done\n");
	mutex_unlock(&hr->lock);
	return count;
}
static DEVICE_ATTR_WO(uc_sb_dump);

/*
 * Scan the experimental UC RAM aperture for printable runs. Keep MMIO
 * reads aligned while preserving the logical byte offsets.
 *
 * Physical aperture ownership and host-read safety are not established
 * by this diagnostic.
 */
static ssize_t uc_ram_dump_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct qcom_usb4_hr *hr = qcom_usb4_hr_get(dev);
	void __iomem *ram;
	size_t off, run;
	char line[96];
	int ret;

	if (!hr)
		return -ENODEV;
	ret = qcom_usb4_hr_lock_active(hr);
	if (ret)
		return ret;

	ram = hr->regs[5]; /* uc_ram (reads may alias DRAM) */
	dev_info(dev, "hr-ram: scan start (0xe000 bytes)\n");
	for (off = 0; off < 0xe000; off += max_t(size_t, run, 4)) {
		u32 word = readl(ram + (off & ~3u));
		u8 c0 = (word >> ((off & 3) * 8)) & 0xff;

		if (c0 < 0x20 || c0 > 0x7e) {
			run = 4;
			continue;
		}
		run = 0;
		while (off + run < 0xe000 && run < sizeof(line) - 1) {
			u32 w = readl(ram + ((off + run) & ~3u));
			u8 c = (w >> (((off + run) & 3) * 8)) & 0xff;

			if (c < 0x20 || c > 0x7e)
				break;
			line[run++] = c;
		}
		line[run] = '\0';
		if (run >= 6)
			dev_info(dev, "hr-ram: +%#06zx: %s\n", off, line);
	}
	dev_info(dev, "hr-ram: scan done\n");
	mutex_unlock(&hr->lock);
	return count;
}
static DEVICE_ATTR_WO(uc_ram_dump);


/*
 * ACPI PRT0 declares a third interrupt (GSI 287 / SPI 255, edge +
 * wake-capable) that the DT never had. Log every firing: if this is the
 * UC firmware/completion line we will see it on plug/boot events.
 */
static irqreturn_t qcom_usb4_hr_uc_irq(int irq, void *data)
{
	struct qcom_usb4_hr *hr = data;

	dev_info(hr->nhi.dev, "hr-uc: uc irq %d fired\n", irq);
	return IRQ_HANDLED;
}

/*
 * Read-only dump of the UCS0 shared mailbox (ACPI QCOM0CA4, OperationRegion
 * USBC @ 0x81F20040, 0x2F bytes): INFO, UPDT, then 3 x {connected, mux,
 * res, vid16, sid16, state64}. Watch UPDT bits change on plug if the ADSP
 * populates this on Linux too.
 */
static ssize_t uc_mailbox_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct qcom_usb4_hr *hr = qcom_usb4_hr_get(dev);
	int i, n = 0, ret;

	if (!hr)
		return -ENODEV;
	ret = qcom_usb4_hr_lock_active(hr);
	if (ret)
		return ret;
	if (!hr->ucsb) {
		ret = -ENODEV;
		goto out_unlock;
	}
	for (i = 0; i < 0x2f; i += 4) {
		u32 v = readl(hr->ucsb + i);

		n += scnprintf(buf + n, PAGE_SIZE - n, "[%#02x] %#010x\n",
			       i, v);
	}
	ret = n;
out_unlock:
	mutex_unlock(&hr->lock);
	return ret;
}
static DEVICE_ATTR_RO(uc_mailbox);

/*
 * Deferred activation: runs the full bring-up (clocks, resets, PHY, clamp
 * release, UC firmware load/start, NHI probe) on demand. Used to start the
 * UC only after the Type-C stack has brought the retimer out of reset, so
 * the UC's init-time sideband discovery sees a live target.
 */
static ssize_t uc_activate_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct qcom_usb4_hr *hr = qcom_usb4_hr_get(dev);
	int ret;

	if (!hr)
		return -ENODEV;
	ret = mutex_lock_interruptible(&hr->lock);
	if (ret)
		return ret;
	if (hr->stopping) {
		ret = -ENODEV;
		goto out_unlock;
	}
	if (hr->state == QCOM_USB4_HR_ACTIVE) {
		ret = 0;
		goto out_unlock;
	}
	if (qcom_usb4_hr_is_live(pdev)) {
		ret = -EBUSY;
		goto out_unlock;
	}

	ret = qcom_usb4_hr_activate(hr, pdev, true);
out_unlock:
	mutex_unlock(&hr->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(uc_activate);

static struct attribute *qcom_usb4_hr_attrs[] = {
	&dev_attr_uc_ping.attr,
	&dev_attr_uc_activate.attr,
	&dev_attr_uc_usbap_rmw.attr,
	&dev_attr_uc_sb_dump.attr,
	&dev_attr_uc_ram_dump.attr,
	&dev_attr_uc_mailbox.attr,
	NULL,
};
ATTRIBUTE_GROUPS(qcom_usb4_hr);

static void qcom_usb4_hr_stop(struct qcom_usb4_hr *hr)
{
	struct device *dev = hr->nhi.dev;

	mutex_lock(&hr->lock);
	hr->stopping = true;
	if (hr->state == QCOM_USB4_HR_STOPPED)
		goto out_unlock;
	if (hr->state == QCOM_USB4_HR_ACTIVE)
		qcom_usb4_hr_release_activation(hr);
	if (hr->cleanup_error)
		dev_warn(dev, "frontend stopped; physical cleanup failed: %d\n",
			 hr->cleanup_error);
	hr->state = QCOM_USB4_HR_STOPPED;
	if (hr->prepare_group) {
		devres_release_group(dev, hr->prepare_group);
		hr->prepare_group = NULL;
	}
	hr->usb4_phy = NULL;
	hr->ucsb = NULL;
	hr->nhi.ops = NULL;
	hr->nhi.iobase = NULL;
	/* Failed cleanup flags remain historical evidence, not usable handles. */
out_unlock:
	mutex_unlock(&hr->lock);
}

static int qcom_usb4_hr_probe(struct platform_device *pdev)
{
	struct qcom_usb4_hr *hr;
	int i, ret;

	hr = devres_alloc(qcom_usb4_hr_context_release, sizeof(*hr), GFP_KERNEL);
	if (!hr)
		return -ENOMEM;
	devres_add(&pdev->dev, hr);
	mutex_init(&hr->lock);
	hr->nhi.dev = &pdev->dev;

	ret = qcom_usb4_hr_map_resources(hr, pdev);
	if (ret)
		return ret;

	/*
	 * arch1-14: read uc_ctl ONCE at probe, before any reset/clock/genpd
	 * action ever runs. Every prior round only ever sampled uc_ctl after
	 * we had already pulsed resets, so we never knew whether UEFI handed
	 * the UC over already executing and we were killing it ourselves.
	 * Read-only; no new MMIO region is touched.
	 */
	dev_info(&pdev->dev, "hr-probe: uc_ctl at boot=%#x (pre-touch)\n",
		 readl(hr->regs[6] + 0x1000));

	for (i = 0; i < ARRAY_SIZE(qcom_usb4_hr_clocks); i++)
		hr->clocks[i].id = qcom_usb4_hr_clocks[i];

	ret = devm_clk_bulk_get(&pdev->dev, ARRAY_SIZE(hr->clocks),
					hr->clocks);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "missing host-router clocks\n");

	for (i = 0; i < ARRAY_SIZE(qcom_usb4_hr_resets); i++)
		hr->resets[i].id = qcom_usb4_hr_resets[i];

	/* Identify reset ownership failures individually while bringing this up. */
	for (i = 0; i < ARRAY_SIZE(hr->resets); i++) {
		/* The QMP PHY owns this reset on the DT wiring used here. */
		if (!strcmp(hr->resets[i].id, "phy_nocsr"))
			hr->resets[i].rstc = devm_reset_control_get_optional_shared(
				&pdev->dev, hr->resets[i].id);
		else
			hr->resets[i].rstc = devm_reset_control_get_shared(&pdev->dev,
								 hr->resets[i].id);
		if (IS_ERR(hr->resets[i].rstc))
			return dev_err_probe(&pdev->dev, PTR_ERR(hr->resets[i].rstc),
					     "reset %s unavailable\n",
					     hr->resets[i].id);
	}

	mutex_lock(&hr->lock);
	ret = qcom_usb4_hr_nhi_preflight(hr, pdev);
	if (!ret)
		ret = qcom_usb4_hr_activate(hr, pdev, false);
	mutex_unlock(&hr->lock);
	if (ret) {
		qcom_usb4_hr_stop(hr);
		return ret;
	}

	/*
	 * [sp11] The router's power domain (gcc_usb4_0_gdsc via genpd) is
	 * cycled deliberately in qcom_usb4_hr_domain_power_cycle(); enable
	 * runtime PM so those calls are legal. No autosuspend: the domain
	 * only moves when the driver asks.
	 */
	pm_runtime_enable(&pdev->dev);
	/*
	 * Do NOT claim the device is active: genpd_power_off_unused()
	 * (late_initcall_sync) powers off domains whose consumers are RPM-
	 * managed once this driver enables runtime PM, so the domain is
	 * likely OFF by the time we get here (arch1-6/7: set_active was a
	 * lie; activation then toggled branch clocks inside a dead domain,
	 * 'status stuck at off' + EBUSY). Mark suspended and genuinely
	 * resume through genpd so framework state matches hardware, and
	 * hold that reference: the domain only moves again inside
	 * qcom_usb4_hr_domain_power_cycle().
	 */
	pm_runtime_set_suspended(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(&pdev->dev);
		pm_runtime_disable(&pdev->dev);
		return dev_err_probe(&pdev->dev, ret,
				     "power-domain resume failed\n");
	}

	dev_info(&pdev->dev,
		 hr->state == QCOM_USB4_HR_ACTIVE ?
		 "resource validation passed; host-router NHI is active\n" :
		 "resource validation passed; host-router hardware access is disabled\n");
	return 0;
}

static void qcom_usb4_hr_remove(struct platform_device *pdev)
{
	struct qcom_usb4_hr *hr = qcom_usb4_hr_get(&pdev->dev);

	/*
	 * Full unwind (qualification area c): driver core has removed and
	 * drained the sysfs dev_groups and any tb domain children before this
	 * callback, so nothing can re-enter activation afterwards. stop()
	 * sets STOPPED (one-shot; no restart), releases the activation in
	 * reverse order (nhi_remove -> domain group -> phy_exit -> resets ->
	 * clocks), releases the preflight devres group, and leaves the PHY
	 * driver's own resources untouched.
	 */
	qcom_usb4_hr_stop(hr);

	/* [sp11] Undo the runtime PM hookup from probe (drop our reference). */
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
}

static void qcom_usb4_hr_shutdown(struct platform_device *pdev)
{
	struct qcom_usb4_hr *hr = qcom_usb4_hr_get(&pdev->dev);

	/* Shutdown, unlike unbind, does not drain the driver's sysfs groups. */
	mutex_lock(&hr->lock);
	hr->stopping = true;
	mutex_unlock(&hr->lock);
	device_remove_groups(&pdev->dev, qcom_usb4_hr_groups);
	qcom_usb4_hr_stop(hr);
}

/*
 * arch1-17: gcc_pcie_0_tunnel_gdsc has no DT consumer on our tree, so
 * Linux's genpd has it off from early boot (pm_genpd_summary: off-0 for
 * all three tunnel domains), while UEFI had the tunnel powered at
 * handoff. Vendor restart flows power the tunnel block before asking
 * the router domain to come up. We cannot dev_pm_domain_attach_by_name()
 * on the router itself (its first power-domain is already auto-attached
 * and the API returns -EEXIST), so the DT carries a dedicated consumer
 * node for the tunnel genpd; the driver core auto-attaches it and this
 * probe powers the domain and holds the runtime PM reference for the
 * lifetime of the boot. Failure to power it is logged, not fatal.
 */
static int usb4_tunnel_pd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		dev_warn(dev, "hr-pd: tunnel power-domain power-on failed (%d)\n",
			 ret);
	} else {
		dev_info(dev, "hr-pd: tunnel power-domain powered up and held\n");
	}
	/* Built-in experiment consumer: reference intentionally held. */
	return 0;
}

static const struct of_device_id usb4_tunnel_pd_of_match[] = {
	{ .compatible = "qcom,x1e80100-usb4-tunnel-pd" },
	{ }
};
MODULE_DEVICE_TABLE(of, usb4_tunnel_pd_of_match);

static struct platform_driver usb4_tunnel_pd_driver = {
	.probe = usb4_tunnel_pd_probe,
	.driver = {
		.name = "usb4-tunnel-pd",
		.of_match_table = usb4_tunnel_pd_of_match,
	},
};

static const struct of_device_id qcom_usb4_hr_of_match[] = {
	{ .compatible = "qcom,x1e80100-usb4-hr" },
	{ .compatible = "qcom,usb4-hr" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_usb4_hr_of_match);

static struct platform_driver qcom_usb4_hr_driver = {
	.probe = qcom_usb4_hr_probe,
	.remove = qcom_usb4_hr_remove,
	.shutdown = qcom_usb4_hr_shutdown,
	.driver = {
		.name = "qcom-usb4-hr",
		.of_match_table = qcom_usb4_hr_of_match,
		.dev_groups = qcom_usb4_hr_groups,
	},
};
int __init qcom_usb4_hr_register(void)
{
	int ret;

	ret = platform_driver_register(&usb4_tunnel_pd_driver);
	if (ret)
		return ret;
	ret = platform_driver_register(&qcom_usb4_hr_driver);
	if (ret)
		platform_driver_unregister(&usb4_tunnel_pd_driver);
	return ret;
}

void __exit qcom_usb4_hr_unregister(void)
{
	platform_driver_unregister(&qcom_usb4_hr_driver);
	platform_driver_unregister(&usb4_tunnel_pd_driver);
}

MODULE_DESCRIPTION("Qualcomm X1E USB4 host-router bring-up scaffold");
MODULE_LICENSE("GPL");
