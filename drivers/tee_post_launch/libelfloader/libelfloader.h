/* SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause */

#ifndef __LIBELFLOADER_H_
#define __LIBELFLOADER_H_

bool relocate_elf_image(uint64_t ld_addr,
			uint64_t ld_size,
			uint64_t rt_addr,
			uint64_t rt_size,
			uint64_t *p_entry);

#endif
