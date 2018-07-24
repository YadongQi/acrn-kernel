/*
 * HOB(Hand-off-block) parse
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright (c) 2018 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Contact Information: Qi Yadong <yadong.qi@intel.com>
 *
 * BSD LICENSE
 *
 * Copyright (C) 2018 Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Qi Yadong <yadong.qi@intel.com>
 *
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/io.h>

#include "sbl_seed_parse.h"
#include "rpmb_key.h"

#define SEED_ENTRY_TYPE_SVNSEED         0x1U
#define SEED_ENTRY_TYPE_RPMBSEED        0x2U

#define SEED_ENTRY_USAGE_BASE_ON_SERIAL          0x1U
#define SEED_ENTRY_USAGE_NOT_BASE_ON_SERIAL          0x2U

struct image_boot_params {
	uint32_t size_of_this_struct;
	uint32_t version;
	uint64_t p_seed_list;
	uint64_t p_platform_info;
	uint64_t reserved;
};

struct seed_list_hob {
	uint8_t revision;
	uint8_t rsvd0[3];
	uint32_t buffer_size;
	uint8_t total_seed_count;
	uint8_t rsvd1[3];
};

struct seed_entry {
	/* SVN based seed or RPMB seed or attestation key_box */
	uint8_t type;
	/* For SVN seed: useed or dseed
	 * For RPMB seed: serial number based or not
	 */
	uint8_t usage;
	/* index for the same type and usage seed */
	uint8_t index;
	uint8_t reserved;
	/* reserved for future use */
	uint16_t flags;
	/* Total size of this seed entry */
	uint16_t seed_entry_size;
	/* SVN seed: struct seed_info
	 * RPMB seed: uint8_t rpmb_seed[key_len]
	 */
	uint8_t seed[0];
};

static void parse_seed_list(struct seed_list_hob *seed_hob, uint8_t rpmb_seed[][64])
{
	uint8_t i;
	uint8_t index = 0U;
	struct seed_entry *entry;
	uint8_t rpmb_usage = 0U;

	if (!seed_hob) {
		pr_warn("Invalid seed_list hob pointer. Use fake seed!");
		goto fail;
	}

	if (seed_hob->total_seed_count == 0U) {
		pr_warn("Total seed count is 0. Use fake seed!");
		goto fail;
	}

	entry = (struct seed_entry *)((uint8_t *)seed_hob +
					sizeof(struct seed_list_hob));

	for (i = 0U; i < seed_hob->total_seed_count; i++) {

		if ((uint64_t)entry >=
			((uint64_t)seed_hob + seed_hob->buffer_size)) {
			pr_warn("Exceed memory boundray!\n");
			goto fail;
		}

		/* retrieve rpmb seed */
		if (SEED_ENTRY_TYPE_RPMBSEED == entry->type) {
			if (entry->index == 0) {
				rpmb_usage = entry->usage;
			} else {
				pr_warn("RPMB usage mismatch!\n");
				goto fail;
			}

			/* The seed_entry with same type/usage are always
			 * arranged by index in order of 0~3.
			 */
			if (entry->index != index) {
				pr_warn("Index mismatch. Use fake seed!");
				goto fail;
			}

			if (entry->index > RPMB_MAX_PARTITION_NUMBER) {
				pr_warn("Index exceed max number!");
				goto fail;
			}

			memcpy(&rpmb_seed[index], entry->seed,
						RPMB_SEED_LENGTH);
			index++;

			/* erase original seed in seed entry */
			memset(entry->seed, 0U, RPMB_SEED_LENGTH);
		}

		entry = (struct seed_entry *)((uint8_t *)entry +
						entry->seed_entry_size);
	}

	return;

fail:
	memset(rpmb_seed, 0U, RPMB_MAX_PARTITION_NUMBER * RPMB_SEED_LENGTH);
	/* Use fake rpmb seed, and only first entry is valid */
	memset(rpmb_seed[0], 0xA6U, RPMB_SEED_LENGTH);
}

void parse_sbl_rpmb_key(unsigned long params_addr, uint8_t rpmb_key[][64])
{
	struct image_boot_params *boot_params = NULL;
	struct seed_list_hob* seed_list = NULL;
	uint32_t remap_buffer_size = 0;

	//pr_err("%s: bpa=%lx\n", __func__, params_addr);
	if (params_addr == 0) {
		pr_err("Invalid params addr\n");
		goto fail;
	}

	boot_params = memremap(params_addr,
					sizeof(struct image_boot_params), MEMREMAP_WB);
	if (boot_params) {
		seed_list = memremap(boot_params->p_seed_list,
					sizeof(struct seed_list_hob), MEMREMAP_WB);
		if (seed_list) {
			remap_buffer_size = seed_list->buffer_size;
			memunmap(seed_list);

			/* Remap with actual buffer size */
			seed_list = memremap(boot_params->p_seed_list,
							remap_buffer_size, MEMREMAP_WB);
		}
	}

	parse_seed_list(seed_list, rpmb_key);

fail:
	if (seed_list)
		memunmap(seed_list);

	if (boot_params)
		memunmap(boot_params);
}

MODULE_LICENSE("DUAL BSD/GPL");
