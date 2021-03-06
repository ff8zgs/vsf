/***************************************************************************
 *   Copyright (C) 2009 - 2010 by Simon Qian <SimonQian@SimonQian.com>     *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef __VSFUSBD_RNDIS_H_INCLUDED__
#define __VSFUSBD_RNDIS_H_INCLUDED__

#ifndef VSFUSBD_CDCCFG_TRANSACT
#error "vsfusbd_RNDIS need vsfusbd_CDC run in TRANSACT mode"
#endif

#if VSFIP_CFG_NETIF_HEADLEN < 64
#error "RNDIS require minimum 64 bytes netif header size"
#endif

#include "../../common/CDC/vsfusb_RNDIS.h"

#ifndef VSFCFG_STANDALONE_MODULE
extern const struct vsfusbd_class_protocol_t vsfusbd_RNDISControl_class;
extern const struct vsfusbd_class_protocol_t vsfusbd_RNDISData_class;
#endif

#ifndef VSFUSBD_RNDIS_CFG_OIDNUM
#define VSFUSBD_RNDIS_CFG_OIDNUM				25
#endif

struct vsfusbd_RNDIS_param_t
{
	struct vsfusbd_CDCACM_param_t CDCACM;

	uint8_t encapsulated_buf[4 * VSFUSBD_RNDIS_CFG_OIDNUM + 32];

	struct vsfip_addr_t mac;

	// private
	struct vsfip_netif_t netif;
	struct vsfip_netdrv_t netdrv;
	struct vsfip_dhcpd_t dhcpd;
	bool netif_inited;
	struct vsf_bufstream_t tx_bufstream;
	struct vsf_bufstream_t rx_bufstream;
	struct vsfip_buffer_t *tx_buffer;
	struct vsfip_buffer_t *rx_buffer;

	// thread to receive netif.output_sem
	struct vsfsm_t sm;

	struct
	{
		uint32_t txok;
		uint32_t rxok;
		uint32_t txbad;
		uint32_t rxbad;
		uint32_t rx_nobuf;
	} statistics;
	uint32_t oid_packet_filter;
};

#endif	// __VSFUSBD_RNDIS_H_INCLUDED__
