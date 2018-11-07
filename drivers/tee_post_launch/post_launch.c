// SPDX-License-Identifier: GPL-2.0
/*
 * TEE Post Launch Kernel Module Driver
 *
 * Copyright (c) 2018 Intel Corporation. All rights reserved.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ":%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/smp.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <asm/msr.h>

#include "libelfloader/libelfloader.h"

//#define LAUNCH_TRUSTY 1
#define LAUNCH_OPTEE 1

#define TEE_ELF_PATH "/boot/tee.elf"

#define TEE_RUNTIME_SIZE (16*1024*1024)

struct smc_interface {
	uint64_t rax;
	uint64_t rdi;
	uint64_t rsi;
	uint64_t rdx;
	uint64_t rbx;
	uint64_t rcx;
};

static char *tee_elf_path = "/boot/tee.elf";
module_param(tee_elf_path, charp, 0000);
MODULE_PARM_DESC(tee_elf_path, "Path to tee elf binary");

static void acrn_hc_init_tee(void *param)
{
	struct smc_interface *args = param;
#ifdef LAUNCH_OPTEE
	#define OPTEE_HCALL_LAUNCH 0x80000073UL
	register signed long smc_id asm("r8") = OPTEE_HCALL_LAUNCH;
	__asm__ __volatile__(
			"vmcall;"
			: "=a" (args->rax), "=D" (args->rdi), "=S" (args->rsi),
			  "=d" (args->rdx), "=b" (args->rbx), "=c" (args->rcx)
			: "r" (smc_id), "D" (args->rdi)
	);
#elif defined LAUNCH_TRUSTY
	#define TRUSTY_HCALL_LAUNCH 0x80000070UL
	register signed long ret asm("rax");
	register signed long smc_id asm("r8") = TRUSTY_HCALL_LAUNCH;
	ulong r0;

	asm volatile (
			"vmcall;"
			: "=a"(ret), "=D"(args->rdi), "=S"(args->rsi), "=d"(args->rdx), "=b"(args->rbx)
			: "r"(smc_id), "D"(args->rdi)
		     );

	if (ret < 0) {
		pr_err("VMCALL failed to launch trusty!\n");
	}
	pr_err("VMCALL success to launch trusty!\n");
#endif

}

#define HC_PRINT_VMCS 0x8000007FUL
static void acrn_print_vmcs(void) __attribute__((unused));
static void acrn_print_vmcs(void)
{
	register signed long ret asm("rax");
	register signed long hc_id asm("r8") = HC_PRINT_VMCS;
	asm volatile (
			"vmcall;"
			: "=a"(ret)
			: "r"(hc_id)
		     );
	if (ret == 0)
		pr_info("VMCS print succeed!\n");
}

#ifdef LAUNCH_OPTEE
struct tee_boot_param {
	uint64_t version;
	uint64_t base;
	uint64_t size;
	uint64_t entry;
} __attribute__((aligned(8)));

static int launch_optee(long rt_mem_pa, long entry)
{
	int ret;
	struct tee_boot_param boot_param;
	struct smc_interface args;

	boot_param.version = 1ULL;
	boot_param.base = rt_mem_pa;
	boot_param.size = TEE_RUNTIME_SIZE;
	boot_param.entry = entry;

	args.rax = 0;
	args.rdi = (uint64_t)(virt_to_phys(&boot_param));
	ret = smp_call_function_single(0, acrn_hc_init_tee, (void *)&args, 1);
	if (ret)
		pr_err("smp call function(acrn_hc) failed!\n");

	return (int)args.rax;
}

#elif defined LAUNCH_TRUSTY
struct trusty_boot_param {
	uint32_t size_of_this_struct;
	uint32_t version;
	uint32_t base_addr;
	uint32_t entry_point;
	uint32_t mem_size;
	uint32_t padding;
	uint32_t base_addr_high;
	uint32_t entry_point_high;
	uint8_t rpmb_key[64];
} __attribute__((aligned(8)));

static int launch_trusty(long rt_mem_pa, long entry)
{
	int ret;
	struct trusty_boot_param boot_param;
	struct smc_interface args;

	boot_param.size_of_this_struct = sizeof(struct trusty_boot_param);
	boot_param.version = 1;
	boot_param.base_addr = rt_mem_pa & 0xFFFFFFFF;
	boot_param.base_addr_high = (rt_mem_pa >> 32) & 0xFFFFFFFF;
	boot_param.mem_size = TEE_RUNTIME_SIZE;
	boot_param.entry_point = entry & 0xFFFFFFFF;
	boot_param.entry_point_high = (entry >> 32) & 0xFFFFFFFF;
	memset(boot_param.rpmb_key, 0xDE, 64);

	args.rax = 0;
	args.rdi = (uint64_t)(virt_to_phys(&boot_param));

	ret = smp_call_function_single(0, acrn_hc_init_tee, (void *)&args, 1);
	if (ret)
		pr_err("smp call function(acrn_hc) failed!\n");

	return (int)args.rax;
}

#endif

static ulong alloc_tee_mem(void)
{
#if 0
	struct page *pg;
	uint32_t count;
	ulong tee_rt_mem;
	count = PAGE_ALIGN(TEE_RUNTIME_SIZE) >> PAGE_SHIFT;
	pg = dma_alloc_from_contiguous(NULL, count, 512, false);
	tee_rt_mem = page_to_phys(pg);

	pr_info("before align size=%x, %llx(%lx), p2v=%lx\n", TEE_RUNTIME_SIZE, (u64)tee_rt_mem, (ulong)phys_to_virt(tee_rt_mem), (ulong)page_to_virt(pg));
	tee_rt_mem = ALIGN((ulong)tee_rt_mem, 0x1000);
	pr_info("malloc rt mem success! size=%x, %llx(%lx), p2v=%lx\n", TEE_RUNTIME_SIZE, (u64)tee_rt_mem, (ulong)phys_to_virt(tee_rt_mem), (ulong)page_to_virt(pg));
	return tee_rt_mem;
#endif

	ulong tee_pa = 0x20000000ULL;
	return tee_pa;
}

static int read_tee_binary(const char *path, ulong *bin_addr, loff_t *size, struct device *dev, bool *need_free)
{
	int rc;
	loff_t file_size;
	const struct firmware *fw;
	void *out;

	rc = kernel_read_file_from_path(path, &out, &file_size, 0, READING_MODULE);
	if (rc < 0) {
		pr_err("Failed to read file from path, try firmware!\n");

		rc = request_firmware(&fw, "tee.elf", dev);
		pr_info("%s: requested fw: size=%lx, addr=%lx\n", __func__, fw->size, (ulong)fw->data);

		if (bin_addr)
			*bin_addr = (ulong)fw->data;
		if (size)
			*size = fw->size;
		if (need_free)
			*need_free = false;
		return rc;
	}

	if (bin_addr)
		*bin_addr = (ulong)out;

	if (size)
		*size = file_size;

	if (need_free)
		*need_free = true;

	return rc;
}

static int relocate_tee_binary(void *in, size_t in_size, void *out, size_t out_size, uint64_t *entry)
{
	bool rc;
#ifdef LAUNCH_TRUSTY
	void *bin_out = out + 0x1000;
#else
	void *bin_out = out;
#endif

	/* Can be replaced simply copy except elf header */
	rc = relocate_elf_image((u64)in, (u64)in_size, (u64)bin_out, (u64)out_size, entry);
	if (!rc) {
		pr_err("Failed to relocate image!\n");
		return rc;
	}
	pr_err("relocate succeed, entry=%llx(%llx), pa=%llx\n", *entry, virt_to_phys((void *)(*entry)), virt_to_phys(out));
	return rc;

	return 0;
}

#if 1
static void tee_post_launch_dev_release(struct device *dev)
{
	dev_dbg(dev, "%s() called\n", __func__);
	return;
}

static struct platform_device tpl_dev = {
	.name = "tee_post_launch",
	.id   = -1,
	.num_resources = 0,
	.dev = {
		.release = tee_post_launch_dev_release,
	},
};

static int init_tpl_device(void)
{
	int ret = 0;

	ret = platform_device_register(&tpl_dev);
	if (ret) {
		pr_err("%s: Failed to register tee_post_launch device\n", __func__);
	}
	return ret;
}
#endif

static int __init post_launch_init(void)
{
	int rc;
	ulong data = 0;
	loff_t size;
	void *tee_rt_mem_va = NULL;
	ulong tee_rt_mem_pa;
	uint64_t entry;
	unsigned long flags;
	bool need_free;

	rc = init_tpl_device();
	if (rc) {
		pr_err("%s: Failed to init tee_post_launch device\n", __func__);
		return rc;
	}

	rc = read_tee_binary(tee_elf_path, &data, &size, &tpl_dev.dev, &need_free);
	if (rc < 0) {
		pr_err("Unable to open file: %s (%d)\n", tee_elf_path, rc);
		goto err;
	}

	tee_rt_mem_pa = alloc_tee_mem();
	if (!tee_rt_mem_pa) {
		pr_err("Failed to alloc tee runtime mem!\n");
		goto err;
	}

	tee_rt_mem_va = memremap(tee_rt_mem_pa, TEE_RUNTIME_SIZE, MEMREMAP_WB);
	if (!tee_rt_mem_va) {
		pr_err("remap tee memory failed!\n");
		goto err;
	}

	rc = relocate_tee_binary((void *)data, size, tee_rt_mem_va, TEE_RUNTIME_SIZE, &entry);
	if (rc < 0) {
		pr_err("Failed ro relocate tee binary!\n");
		goto err;
	}

	if (tee_rt_mem_va)
		memunmap(tee_rt_mem_va);

#ifdef LAUNCH_OPTEE
	local_irq_save(flags);
	rc = launch_optee(tee_rt_mem_pa, tee_rt_mem_pa + entry - (ulong)tee_rt_mem_va);
	local_irq_restore(flags);
	if(rc < 0) {
		pr_err("launch tee failed!\n");
		goto err;
	}
	pr_info("launch optee succeed\n");
#elif defined LAUNCH_TRUSTY
	local_irq_save(flags);
	rc = launch_trusty(tee_rt_mem_pa, tee_rt_mem_pa + entry - (ulong)tee_rt_mem_va);
	local_irq_restore(flags);
	if(rc < 0) {
		pr_err("launch trusty failed!\n");
		goto err;
	}

	pr_info("launch trusty succeed\n");
#endif

	if (data && need_free)
		vfree((void *)data);
	return 0;

err:
	if (data && need_free)
		vfree((void *)data);
	if (tee_rt_mem_va)
		memunmap(tee_rt_mem_va);
	//if (tee_rt_mem_va)
	//	kfree(tee_rt_mem_va);
	return rc;
}

static void __exit post_launch_exit(void)
{
	pr_info("module unloaded.\n");
}


module_init(post_launch_init);
module_exit(post_launch_exit);

MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TEE Post launch Module");
