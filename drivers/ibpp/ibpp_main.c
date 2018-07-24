/*
 * Image boot params parse module
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
#include <linux/init.h>
#include <linux/kernel.h>

#include "sbl_seed_parse.h"
#include "abl_seed_parse.h"
#include "rpmb_key.h"

static void print_seed(uint8_t *seed, uint32_t len)
{
	int i = 0;
	char buf[1024];

	memset(buf, 0, sizeof(buf));
	for(i = 0; i < len; i++) {
		snprintf(buf+(i*3), 1024-i, "%02X-", seed[i]);
	}
	pr_err("SEED:%s\n", buf);
}

static unsigned long sbl_params_addr = 0;
static unsigned long abl_params_addr = 0;

static int __init get_sbl_params_addr(char * str)
{

	if (kstrtoul(str, 16, &sbl_params_addr)) {
		pr_err("%s: failed to parse ImageBootParamsAddr\n", __func__);
		return -EINVAL;
	}

	return 0;
}
__setup("ImageBootParamsAddr=", get_sbl_params_addr);

static int __init get_abl_params_addr(char * str)
{

	if (kstrtoul(str, 16, &abl_params_addr)) {
		pr_err("%s: failed to parse dev_sec_info.param\n", __func__);
		return -EINVAL;
	}

	return 0;
}
__setup("dev_sec_info.param_addr=", get_abl_params_addr);


static uint8_t rpmb_key[RPMB_MAX_PARTITION_NUMBER][64];


static int __init ibpp_init(void)
{
	pr_err("ibpp module loaded.\n");

	if (sbl_params_addr) {
		pr_err("ibpp: seed from sbl\n");
		parse_sbl_rpmb_key(sbl_params_addr, rpmb_key);
	} else if (abl_params_addr) {
		pr_err("ibpp: seed from abl\n");
		parse_abl_rpmb_key(abl_params_addr, rpmb_key);
	} else {
		pr_err("Failed to get boot_params from Commandline!\n");
	}

	print_seed(&rpmb_key[0][0], 64);

	return 0;
}

static void __exit ibpp_exit(void)
{
	pr_err("ibpp module unloaded.\n");
}

module_init(ibpp_init);
module_exit(ibpp_exit);

MODULE_LICENSE("DUAL BSD/GPL");
MODULE_DESCRIPTION("IBPP Module");
