/*
 * ABL SEED parse
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

#include "abl_seed_parse.h"

#define BUP_MKHI_BOOTLOADER_SEED_LEN    64U
/* Structure of seed info */
struct seed_info {
	uint8_t cse_svn;
	uint8_t bios_svn;
	uint8_t padding[2];
	uint8_t seed[BUP_MKHI_BOOTLOADER_SEED_LEN];
};


#define ABL_SEED_LEN 32U
struct abl_seed_info {
	uint8_t svn;
	uint8_t reserved[3];
	uint8_t seed[ABL_SEED_LEN];
};

#define ABL_SEED_LIST_MAX 4U
struct dev_sec_info {
	uint32_t size_of_this_struct;
	uint32_t version;
	uint32_t num_seeds;
	struct abl_seed_info seed_list[ABL_SEED_LIST_MAX];
};


void parse_abl_rpmb_key(unsigned long params_addr, uint8_t rpmb_key[][64])
{
	uint32_t i, legacy_seed_index = 0;
	struct dev_sec_info *sec_info;

	pr_err("params_addr=0x%lx\n", params_addr);
	if (params_addr == 0) {
		pr_err("Invalid params_addr!\n");
		goto fail;
	}

	sec_info = memremap(params_addr, sizeof(struct dev_sec_info), MEMREMAP_WB);

	if (sec_info == NULL) {
		pr_err("params_addr: remap addr failed!\n");
		goto fail;
	}

	for (i = 1; i < sec_info->num_seeds; i++) {
		if (sec_info->seed_list[i].svn <
			sec_info->seed_list[legacy_seed_index].svn) {
			legacy_seed_index = i;
		}
	}
	/* TODO: derive rpmb_key from legacy_seed with serial number and copy to rpmb_key[0][0] */
	memcpy(&rpmb_key[0][0], &sec_info->seed_list[legacy_seed_index].seed[0], 64);

	memset(&sec_info->seed_list[legacy_seed_index], 0, sizeof(sec_info->seed_list[legacy_seed_index]));
	memunmap(sec_info);
	return;
fail:
	/* USE fake rpmb key? */
	;
}


MODULE_LICENSE("DUAL BSD/GPL");
