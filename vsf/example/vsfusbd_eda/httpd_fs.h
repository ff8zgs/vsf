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
#ifndef __HTTPD_FS_H_INCLUDED__
#define __HTTPD_FS_H_INCLUDED__

static struct vsfile_memfile_t httpd_root_dir[] =
{
	{
		.file.name = NULL,
	},
};

static struct vsfile_memfile_t httpd_root[2] =
{
	{
		.file.name = "ROOT",
		.file.attr = VSFILE_ATTR_DIRECTORY,
		.file.parent = NULL,
		.d.child = httpd_root_dir,
	},
	{
		.file.name = NULL,
		.d.child = NULL,
	},
};

#endif		// __HTTPD_FS_H_INCLUDED__
