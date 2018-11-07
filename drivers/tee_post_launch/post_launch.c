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

#include "libelfloader/libelfloader.h"

#define TEE_ELF_PATH "/boot/tee.elf"

#define TEE_RUNTIME_SIZE (16*1024*1024)

struct tee_boot_param {
	uint64_t version;
	uint64_t base;
	uint64_t size;
	uint64_t entry;
} __attribute__((aligned(8)));

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

#define OPTEE_HCALL_LAUNCH 0x80000073UL
static void acrn_hc_init_tee(void *param)
{
	struct smc_interface *args = param;
	register signed long smc_id asm("r8") = OPTEE_HCALL_LAUNCH;
	__asm__ __volatile__(
			"vmcall;"
			: "=a" (args->rax), "=D" (args->rdi), "=S" (args->rsi),
			  "=d" (args->rdx), "=b" (args->rbx), "=c" (args->rcx)
			: "r" (smc_id), "D" (args->rdi)
	);
}

static int launch_optee(uint64_t param_addr)
{
	int ret;

	struct smc_interface args = {
		.rax = 0,
		.rdi = param_addr
	};

	ret = smp_call_function_single(0, acrn_hc_init_tee, (void *)&args, 1);
	if (ret)
		pr_err("smp call function(acrn_hc) failed!\n");

	return (int)args.rax;
}

static ulong alloc_tee_mem(void)
{
#if 0
	tee_rt_mem = kzalloc(TEE_RUNTIME_SIZE, GFP_KERNEL | GFP_DMA);
	if (!tee_rt_mem) {
		pr_err("failed to alloc runtime memory!\n");
		rc = -EINVAL;
		goto err;
	}
	pr_info("malloc rt mem success! size=%x, %llx(%llx)\n", TEE_RUNTIME_SIZE, (u64)tee_rt_mem, virt_to_phys(tee_rt_mem));
#endif

#if 1
	ulong tee_pa = 0x12300000ULL;
	return tee_pa;
#endif
}

static int read_tee_binary(const char *path, void *out, loff_t *size)
{
	int rc;
	loff_t file_size;
	rc = kernel_read_file_from_path(path, &out, &file_size, 0, READING_MODULE);

	if (size)
		*size = file_size;

	return rc;
}

static int relocate_tee_binary(void *in, size_t in_size, void *out, size_t out_size, uint64_t *entry)
{
#if 0
	rc = relocate_elf_image((u64)in, (u64)in_size, (u64)out, (u64)out_size, entry);
	if (!rc) {
		pr_err("Failed to relocate image!\n");
	}
	return rc;
#endif
	uint8_t *ptr = (uint8_t *)in;
	memcpy(out, ptr + 0x100000, in_size - 0x100000);
	*entry = 0x12300000ULL;
	pr_err("relocate succeed, entry=%llx(%llx), pa=%llx\n", *entry, virt_to_phys((void *)(*entry)), virt_to_phys(out));

	return 0;
}

static int __init post_launch_init(void)
{
	int rc;
	void *data = NULL;
	loff_t size;
	void *tee_rt_mem_va = NULL;
	ulong tee_rt_mem_pa;
	uint64_t entry;
	struct tee_boot_param boot_param;

	rc = read_tee_binary(tee_elf_path, data, &size);
	if (rc < 0) {
		pr_err("Unable to open file: %s (%d)\n", tee_elf_path, rc);
		goto err;
	}
	pr_info("read tee file succeed from %s, (%llx)\n", tee_elf_path, size);

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

	rc = relocate_tee_binary(data, size, tee_rt_mem_va, TEE_RUNTIME_SIZE, &entry);
	if (rc < 0) {
		pr_err("Failed ro relocate tee binary!\n");
		goto err;
	}

	boot_param.version = 1ULL;
	boot_param.base = tee_rt_mem_pa;
	boot_param.size = TEE_RUNTIME_SIZE;
	boot_param.entry = entry;

	if (tee_rt_mem_va)
		memunmap(tee_rt_mem_va);
#if 1
	rc = launch_optee((uint64_t)virt_to_phys(&boot_param));
	if(rc < 0) {
		pr_err("launch tee failed!\n");
		goto err;
	}
#endif

	if (data)
		vfree(data);
	return 0;

err:
	if (data)
		vfree(data);
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
