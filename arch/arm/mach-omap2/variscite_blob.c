
#include <linux/proc_fs.h>
#include <linux/string.h>
#include "variscite_blob.h"

#ifdef CONFIG_PROC_FS
struct proc_dir_entry *variscite_blob_proc;
#endif	

static vatiscite_blob g_variscite_blob;

#ifdef CONFIG_PROC_FS
static int variscite_blob_proc_show(struct seq_file *m, void *v)
{
	u8 som_rev_str[32] = {0};
	u8 eth_mac_base[32] = {0};
	u8 wlan_mac_base[32] = {0};

	if (g_variscite_blob.som_rev_str[0] != 0xff)
		memcpy(som_rev_str, g_variscite_blob.som_rev_str, 
				sizeof(g_variscite_blob.som_rev_str));
	else
		strcpy(som_rev_str, "N/A");

	if (g_variscite_blob.eth_mac_base[0] != 0xff)
		sprintf(eth_mac_base, "%02x:%02x:%02x:%02x:%02x:%02x", 
				g_variscite_blob.eth_mac_base[0],g_variscite_blob.eth_mac_base[1],
				g_variscite_blob.eth_mac_base[2],g_variscite_blob.eth_mac_base[3],
				g_variscite_blob.eth_mac_base[4],g_variscite_blob.eth_mac_base[5]);
	else
		strcpy(eth_mac_base, "N/A");

	if (g_variscite_blob.wlan_mac_base[0] != 0xff)
		sprintf(wlan_mac_base, "%02x:%02x:%02x:%02x:%02x:%02x", 
				g_variscite_blob.wlan_mac_base[0],g_variscite_blob.wlan_mac_base[1],
				g_variscite_blob.wlan_mac_base[2],g_variscite_blob.wlan_mac_base[3],
				g_variscite_blob.wlan_mac_base[4],g_variscite_blob.wlan_mac_base[5]);
	else
		strcpy(wlan_mac_base, "N/A");


	seq_printf(m, "SOM Revision:\t\t%s\n", som_rev_str);
	seq_printf(m, "Eth MAC Base:\t\t%s\n", eth_mac_base);
	seq_printf(m, "WLAN MAC Base:\t\t%s\n", wlan_mac_base);

	return 0;
}

static int variscite_blob_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, variscite_blob_proc_show, NULL);
}

static const struct file_operations variscite_blob_proc_fops = {
	.open		= variscite_blob_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

int is_vatiscite_blob_valid(vatiscite_blob* var_blob)
{
	if (var_blob == NULL) 
		var_blob = &g_variscite_blob;

	return (strncmp(var_blob->magic, VAR_BLOB_MAGIC, VAR_BLOB_MAGIC_SIZE) == 0);
}

int vatiscite_blob_setup(vatiscite_blob* var_blob)
{
	if (!is_vatiscite_blob_valid(var_blob)) {
		pr_err("Can't setup Variscite BLOB, invalid BLOB\n");
		return -1;
	}

	memcpy(&g_variscite_blob, var_blob, sizeof(vatiscite_blob));

#ifdef CONFIG_PROC_FS
	if(variscite_blob_proc)
		remove_proc_entry("var_board_info", NULL);

	variscite_blob_proc = proc_create("var_board_info", S_IWUSR | S_IRUGO, NULL,
			  &variscite_blob_proc_fops);
	if (!variscite_blob_proc)
		pr_err("Failed to create Variscite BLOB proc\n");
#endif

	return 0;
}

u8 *vatiscite_blob_get_eth_mac_base(void)
{
	if (!is_vatiscite_blob_valid(&g_variscite_blob) ||
			g_variscite_blob.eth_mac_base[0] == 0xff)
		return NULL;

	return g_variscite_blob.eth_mac_base;
}

u8 *vatiscite_blob_get_wlan_mac_base(void)
{
	if (!is_vatiscite_blob_valid(&g_variscite_blob) ||
			g_variscite_blob.wlan_mac_base[0] == 0xff)
		return NULL;

	return g_variscite_blob.wlan_mac_base;
}

u8 *vatiscite_blob_get_som_rev_str(void)
{
	if (!is_vatiscite_blob_valid(&g_variscite_blob) ||
			g_variscite_blob.som_rev_str[0] == 0xff)
		return NULL;

	return g_variscite_blob.som_rev_str;
}
