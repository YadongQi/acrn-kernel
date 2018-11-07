// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause

#define pr_fmt(fmt) KBUILD_MODNAME ":%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/string.h>

#include "elf32_ld.h"
#include "elf64_ld.h"
#include "elf_ld.h"
#include "libelfloader.h"

void *image_offset(module_file_info_t *file_info,
				uint64_t src_offset, uint64_t bytes_to_read)
{
	if ((src_offset + bytes_to_read) > file_info->loadtime_size) {
		return NULL; /* read no more than size */
	}
	if ((src_offset + bytes_to_read) <= src_offset) {
		return NULL; /* overflow or bytes_to_read == 0 */
	}

	return (void *)(file_info->loadtime_addr+ src_offset);
}

bool image_copy(void *dest, module_file_info_t *file_info,
				uint64_t src_offset, uint64_t bytes_to_copy)
{
	void *src;
	src = image_offset(file_info, src_offset, bytes_to_copy);
	if (!src) {
		return false;
	}
	if (((uint64_t)dest < file_info->runtime_addr) ||
		(((uint64_t)dest + bytes_to_copy) >
		 (file_info->runtime_addr + file_info->runtime_image_size))) {
		return false;
	}
	memcpy(dest, src, bytes_to_copy);
	return true;
}

/*------------------------- Exported Interface --------------------------*/

/*----------------------------------------------------------------------
 *
 * relocate image in memory
 *
 * Input:
 * uint64_t ld_addr - loadtime address, where the image has been load to.
 * uint64_t ld_size - loadtime size, the image size.
 * uint64_t rt_addr - runtime address, where the image will be relocated.
 * uint64_t rt_size - runtime size.
 *
 * Output:
 * uint64_t* p_entry - address of the uint64_t that will be filled
 * with the address of image entry point if all is ok
 *
 * Output:
 * Return value - false on any error
 *---------------------------------------------------------------------- */
bool relocate_elf_image(uint64_t ld_addr,
			uint64_t ld_size,
			uint64_t rt_addr,
			uint64_t rt_size,
			uint64_t *p_entry)
{
	uint8_t *p_buffer;
	module_file_info_t file_info;

	file_info.loadtime_addr = ld_addr;
	file_info.loadtime_size = ld_size;
	file_info.runtime_addr = rt_addr;
	file_info.runtime_total_size = rt_size;

	p_buffer = (uint8_t *)image_offset(&file_info, 0,
			sizeof(elf64_ehdr_t));
	if (!p_buffer){
		pr_err("failed to read file's header\n");
		return false;
	}
	if (!elf_header_is_valid((elf64_ehdr_t *)p_buffer)) {
		pr_err("not an elf binary\n");
		return false;
	}

	if (is_elf64((elf64_ehdr_t *)p_buffer)) {
		pr_info("ELF 64 relocate\n");
		return elf64_load_executable(&file_info, p_entry);
	} else if (is_elf32((elf32_ehdr_t *)p_buffer)) {
		pr_info("ELF 32 relocate\n");
		return elf32_load_executable(&file_info, p_entry);
	} else {
		pr_err("not an elf32 or elf64 binary\n");
		return false;
	}
}
