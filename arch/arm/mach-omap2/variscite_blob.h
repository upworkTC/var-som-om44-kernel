/*
 * arch/arm/mach-omap2/variscite_blob.h
 *
 * Variscite LTD.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _VARISCITE_BLOB_H
#define _VARISCITE_BLOB_H

#include <linux/if_ether.h>
#include <linux/etherdevice.h>

#define VAR_BLOB_MAGIC "VRSC"
#define VAR_BLOB_MAGIC_SIZE (sizeof(VAR_BLOB_MAGIC)-1)

typedef struct vatiscite_blob {
	u8 magic[VAR_BLOB_MAGIC_SIZE];
	u8 som_rev_str[4];
	u8 eth_mac_base[ETH_ALEN]; 
	u8 wlan_mac_base[ETH_ALEN]; 
}vatiscite_blob;

int vatiscite_blob_setup(vatiscite_blob* var_blob);
int is_vatiscite_blob_valid(vatiscite_blob* var_blob);

u8 *vatiscite_blob_get_eth_mac_base(void);
u8 *vatiscite_blob_get_wlan_mac_base(void);
u8 *vatiscite_blob_get_som_rev_str(void);

#endif /* _VARISCITE_BLOB_H */
