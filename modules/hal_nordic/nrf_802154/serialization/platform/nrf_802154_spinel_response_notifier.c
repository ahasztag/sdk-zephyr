/*
 * Copyright (c) 2020 - 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "nrf_802154_spinel_response_notifier.h"

#include <assert.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/zephyr.h>

#include "../spinel_base/spinel.h"
#include "nrf_802154_spinel_log.h"

#define LOG_LEVEL LOG_LEVEL_INFO
#define LOG_MODULE_NAME spinel_ipc_backend_rsp_ntf
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/* TODO: Current implementation doesn't support reentrancy or multiple awaits. */

/**
 * Valid property IDs are from 0 to SPINEL_MAX_UINT_PACKED (2097151).
 * We use UINT32_MAX which is out of this range to indicate that we are
 * not waiting for any property.
 */
#define AWAITED_PROPERTY_NONE UINT32_MAX

struct spinel_notify_buff_internal {
	nrf_802154_spinel_notify_buff_t buff;
	bool                            free;
};

static K_SEM_DEFINE(notify_sem, 0, 1);
static struct k_mutex await_mutex;

static struct spinel_notify_buff_internal notify_buff;

static spinel_prop_key_t awaited_property = AWAITED_PROPERTY_NONE;

extern volatile int arha14;
extern volatile int arha15;
volatile uint32_t arha_curprop1 = 0;
volatile uint32_t arha_curprop2 = 0;
volatile uint32_t arha_curprop3 = 0;

static void set_arha14(int val)
{
	if(0x3c1b == arha_curprop1)
    {
		arha14 = val;
	}
}

static void set_arha15(int val)
{
	if(0x3c1b == arha_curprop2)
    {
		arha15 = val;
	}
}

void nrf_802154_spinel_response_notifier_init(void)
{
	notify_buff.free = true;
	k_mutex_init(&await_mutex);
}

void nrf_802154_spinel_response_notifier_lock_before_request(spinel_prop_key_t property)
{
	/*
	 * Only one thread can await response.
	 * TODO: Implement matching responses to requests and allow multiple threads
	 *       to await simultaneously
	 */

	LOG_DBG("Locking response notifier");
	int ret = k_mutex_lock(&await_mutex, K_FOREVER);

	assert(ret == 0);
	(void)ret;

	assert(awaited_property == AWAITED_PROPERTY_NONE);
	awaited_property = property;
}

nrf_802154_spinel_notify_buff_t *nrf_802154_spinel_response_notifier_property_await(
	uint32_t timeout)
{
	nrf_802154_spinel_notify_buff_t *result = NULL;
	int ret;

	k_timeout_t k_timeout;
	set_arha14(1);
	if (timeout == 0) {
		k_timeout = K_NO_WAIT;
	} else if (timeout == SPINEL_RESPONSE_NOTIFIER_INF_TIMEOUT) {
		k_timeout = K_FOREVER;
	} else {
		k_timeout = K_MSEC(timeout);
	}
	set_arha14(2);

	if (k_sem_take(&notify_sem, k_timeout) == 0) {
		set_arha14(3);
		result = &notify_buff.buff;
	} else {
		set_arha14(4);
		LOG_ERR("No response within timeout %u", timeout);
	}

	set_arha14(5);
	ret = k_mutex_unlock(&await_mutex);
	set_arha14(6);
	assert(ret == 0);
	set_arha14(7);
	(void)ret;
	LOG_DBG("Unlocking response notifier");

	return result;
}

void nrf_802154_spinel_response_notifier_free(nrf_802154_spinel_notify_buff_t *p_notify)
{
	struct spinel_notify_buff_internal *p_notify_buff_free;

	p_notify_buff_free = CONTAINER_OF(p_notify,
					  struct spinel_notify_buff_internal,
					  buff);

	assert(p_notify_buff_free == &notify_buff);

	p_notify_buff_free->free = true;
}

volatile int arha11 = 0;
volatile uint32_t arha11_prop = 0;
volatile uint32_t arha11_aw_prop = 0;
volatile uint32_t arha11_prop_2 = 0;

void nrf_802154_spinel_response_notifier_property_notify(spinel_prop_key_t property,
					      const void       *p_data,
					      size_t            data_len)
{
	arha11 = 1;
    arha11_prop = property;
    arha11_aw_prop = awaited_property;
    arha_curprop2 = arha11_prop;
    if(arha11_prop == 0x3c1b)
    {
		arha_curprop3 = arha_curprop1;
	}
	set_arha15(1);
	if (property == awaited_property) {
		set_arha15(2);
		arha11 = 2;
		assert(notify_buff.free);

		set_arha15(3);
		arha11 = 5;
		notify_buff.free = false;
        arha11_prop_2 = property;
		awaited_property = AWAITED_PROPERTY_NONE;

		assert(data_len <= sizeof(notify_buff.buff.data));
		set_arha15(4);
		arha11 = 6;

		memcpy(notify_buff.buff.data, p_data, data_len);
		set_arha15(5);
		notify_buff.buff.data_len = data_len;

		if(notify_sem.count == 1)
        {
			set_arha15(6);
			while (true);
		}

		set_arha15(7);
		k_sem_give(&notify_sem);
		set_arha15(8);
	} else {
		arha11 = 3;
		set_arha15(9);
		/* TODO: Determine if this is an error condition. */
		NRF_802154_SPINEL_LOG_RAW("Received property that noone is waiting for\n");
	}
	set_arha15(10);
    arha_curprop2 = 0;
}
