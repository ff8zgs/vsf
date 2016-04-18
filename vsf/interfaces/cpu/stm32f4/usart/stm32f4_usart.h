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
#ifndef __STM32F4_USART_H__
#define __STM32F4_USART_H__


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

vsf_err_t stm32f4_usart_init(uint8_t index);
vsf_err_t stm32f4_usart_fini(uint8_t index);
vsf_err_t stm32f4_usart_config(uint8_t index, uint32_t baudrate, uint32_t mode);
vsf_err_t stm32f4_usart_config_callback(uint8_t index, uint32_t int_priority,
				void *p, void (*ontx)(void *), void (*onrx)(void *, uint16_t));
vsf_err_t stm32f4_usart_tx(uint8_t index, uint16_t data);
uint16_t stm32f4_usart_rx(uint8_t index);
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */



