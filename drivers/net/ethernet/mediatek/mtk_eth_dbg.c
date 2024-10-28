/*
 *   Copyright (C) 2018 MediaTek Inc.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; version 2 of the License
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   Copyright (C) 2009-2016 John Crispin <blogic@openwrt.org>
 *   Copyright (C) 2009-2016 Felix Fietkau <nbd@openwrt.org>
 *   Copyright (C) 2013-2016 Michael Lee <igvtee@gmail.com>
 */
#include <linux/trace_seq.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/u64_stats_sync.h>
#include <linux/dma-mapping.h>
#include <linux/netdevice.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/of_mdio.h>
#include <linux/of_address.h>
#include "mtk_eth_soc.h"
#include "mtk_eth_dbg.h"

struct proc_dir_entry *proc_reg_dir;
static struct proc_dir_entry *proc_rss_ctrl;
u32 cur_rss_num;
struct mtk_eth_debug {
	struct dentry *root;
	void __iomem *base;
	int direct_access;
};
struct mtk_eth *g_eth;
struct mtk_eth_debug eth_debug;

static int mtk_rss_set_indr_tbl(struct mtk_eth *eth, int num)
{
	struct mtk_rss_params *rss_params = &eth->rss_params;
	const struct mtk_reg_map *reg_map = eth->soc->reg_map;
	u32 i;
	if (num <= 0 || num > MTK_RX_NAPI_NUM)
		return -EOPNOTSUPP;
	for (i = 0; i < MTK_RSS_MAX_INDIRECTION_TABLE; i++)
		rss_params->indirection_table[i] = i % num;
	for (i = 0; i < MTK_RSS_MAX_INDIRECTION_TABLE / 16; i++)
		mtk_w32(eth, mtk_rss_indr_table(rss_params, i),
			MTK_RSS_INDR_TABLE_DW(i));
			//eth->soc->reg_map->pdma.rss_indr_table_dw0 + (i * 0x4));
	return 0;
}

static ssize_t rss_ctrl_write(struct file *file, const char __user *buffer,
		       size_t count, loff_t *data)
{
	char buf[32];
	char *p_buf;
	char *p_token = NULL;
	char *p_delimiter = " \t";
	long num = 4;
	u32 len = count;
	int ret;
	if (len >= sizeof(buf)) {
		pr_info("Input handling fail!\n");
		return -1;
	}
	if (copy_from_user(buf, buffer, len))
		return -EFAULT;
	buf[len] = '\0';
	p_buf = buf;
	p_token = strsep(&p_buf, p_delimiter);
	if (!p_token)
		num = 4;
	else
		ret = kstrtol(p_token, 10, &num);
	if (!mtk_rss_set_indr_tbl(g_eth, num))
		cur_rss_num = num;
	return count;
}

static int rss_ctrl_read(struct seq_file *seq, void *v)
{
	pr_info("ADMA is using %d-RSS.\n", cur_rss_num);
	return 0;
}

static int rss_ctrl_open(struct inode *inode, struct file *file)
{
	return single_open(file, rss_ctrl_read, 0);
}

static const struct proc_ops rss_ctrl_fops = {
	.proc_open = rss_ctrl_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_write = rss_ctrl_write,
	.proc_release = single_release
};

int mtketh_debugfs_init(struct mtk_eth *eth)
{
	//char name[16];
	//long i;
	int ret = 0;
	eth_debug.root = debugfs_create_dir("mtketh", NULL);
	if (!eth_debug.root) {
		dev_notice(eth->dev, "%s:err at %d\n", __func__, __LINE__);
		return -ENOMEM;
	}
	return 0;
//err:
	debugfs_remove_recursive(eth_debug.root);
	return ret;
}

int debug_proc_init(struct mtk_eth *eth)
{
	g_eth = eth;
	if (!proc_reg_dir)
		proc_reg_dir = proc_mkdir(PROCREG_DIR, NULL);

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_RSS)) {
		proc_rss_ctrl =
			proc_create(PROCREG_RSS_CTRL, 0, proc_reg_dir,
				    &rss_ctrl_fops);
		if (!proc_rss_ctrl)
			pr_info("!! FAIL to create %s PROC !!\n",
				PROCREG_RSS_CTRL);
		cur_rss_num = g_eth->soc->rss_num;
	}
	return 0;
}

void debug_proc_exit(void)
{
	if (proc_reg_dir)
		remove_proc_entry(PROCREG_DIR, 0);

	if (proc_rss_ctrl)
		remove_proc_entry(PROCREG_RSS_CTRL, proc_reg_dir);

}
