/*
 * Copyright (c) 2015 Intel Corporation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3) Neither the name of Intel Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file Contains configuration for ia32 platforms.
 */

#include <stdio.h>
#include <stdint.h>
#include <device.h>
#include <init.h>

#include "board.h"

#ifdef CONFIG_NS16550
#include <drivers/uart.h>
#include <serial/ns16550.h>

static struct uart_device_config_t uart_dev_cfg_info[] = {
	{
		.port = CONFIG_UART_PORT_0_REGS,
		.irq = CONFIG_UART_PORT_0_IRQ,
		.int_pri = CONFIG_UART_PORT_0_IRQ_PRIORITY,
	},
	{
		.port = CONFIG_UART_PORT_1_REGS,
		.irq = CONFIG_UART_PORT_1_IRQ,
		.int_pri = CONFIG_UART_PORT_1_IRQ_PRIORITY,
	},
};

static struct device_config uart_dev_cfg[] = {
	{
		.name = CONFIG_UART_PORT_0_NAME,
		.init = NULL,
		.config_info = &uart_dev_cfg_info[0],
	},
	{
		.name = CONFIG_UART_PORT_1_NAME,
		.init = NULL,
		.config_info = &uart_dev_cfg_info[1],
	},
};

static struct uart_ns16550_dev_data_t uart_dev_data[2];

struct device uart_devs[] = {
	{
		.config = &uart_dev_cfg[0],
		.driver_api = NULL,
		.driver_data = &uart_dev_data[0],
	},
	{
		.config = &uart_dev_cfg[1],
		.driver_api = NULL,
		.driver_data = &uart_dev_data[1],
	},
};

#if defined(CONFIG_UART_CONSOLE_INDEX)
struct device * const uart_console_dev = &uart_devs[CONFIG_UART_CONSOLE_INDEX];
#endif

#endif
