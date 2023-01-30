/**
 * @file      mender-glue.c
 * @brief     Example glue code to use Mender in an ESP32 project
 *
 * MIT License
 *
 * Copyright (c) 2022-2023 joelguittet and mender-mcu-client contributors
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"

#include "mender-client.h"
#include "mender-ota.h"

/**
 * @brief Tag used for logging
 */
static const char *TAG = "mender";

/**
 * @brief Authentication success callback
 * @return MENDER_OK if application is marked valid and success deployment status should be reported to the server, error code otherwise
 */
static mender_err_t
authentication_success_cb(void) {

    ESP_LOGI(TAG, "Mender client authenticated");

    /* Validate the image if it is still pending */
    /* Note it is possible to do multiple diagnosic tests before validating the image */
    /* In this example, authentication success with the mender-server is enough */
    return mender_ota_mark_app_valid_cancel_rollback();
}

/**
 * @brief Authentication failure callback
 * @return MENDER_OK if nothing to do, error code if the mender client should restart the application
 */
static mender_err_t
authentication_failure_cb(void) {

    static int   tries = 0;
    mender_err_t ret   = MENDER_OK;

    /* Increment number of failures */
    tries++;
    ESP_LOGI(TAG, "Mender client authentication failed (%d/%d)", tries, CONFIG_EXAMPLE_AUTHENTICATION_FAILS_MAX_TRIES);

    /* Invalidate the image if it is still pending */
    /* Note it is possible to invalid the image later to permit clean closure before reboot */
    /* In this example, several authentication failures with the mender-server is enough */
    if (tries >= CONFIG_EXAMPLE_AUTHENTICATION_FAILS_MAX_TRIES) {
        ret = mender_ota_mark_app_invalid_rollback_and_reboot();
    }

    return ret;
}

/**
 * @brief Deployment status callback
 * @param status Deployment status value
 * @param desc Deployment status description as string
 * @return MENDER_OK if the function succeeds, error code otherwise
 */
static mender_err_t
deployment_status_cb(mender_deployment_status_t status, char *desc) {

    /* We can do something else if required */
    ESP_LOGI(TAG, "Deployment status is '%s'", desc);

    return MENDER_OK;
}

/**
 * @brief Restart callback
 * @return MENDER_OK if the function succeeds, error code otherwise
 */
static mender_err_t
restart_cb(void) {

    /* Restart */
    /* Note it is return to not restart the system right now depending of the application */
    /* In this example, immediate restart is enough */
    ESP_LOGI(TAG, "Restarting system");
    esp_restart();

    return MENDER_OK;
}

/**
 * @brief run the Mender client
 * @return MENDER_OK if the function succeeds, error code otherwise
 */
mender_err_t
init_mender_client() {
	/* Read base MAC address of the ESP32 */
    uint8_t mac[6];
    char    mac_address[18];
    ESP_ERROR_CHECK(esp_base_mac_addr_get(mac));
    sprintf(mac_address, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    /* Retrieve running version of the ESP32 */
    esp_app_desc_t         running_app_info;
    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_ERROR_CHECK(esp_ota_get_partition_description(running, &running_app_info));
    ESP_LOGI(TAG, "Running project '%s' version '%s'", running_app_info.project_name, running_app_info.version);

    /* Compute artifact name */
    char artifact_name[128];
    sprintf(artifact_name, "%s-v%s", running_app_info.project_name, running_app_info.version);

	/* Initialize mender-client */
    mender_client_config_t    mender_client_config    = {
		.mac_address                  = mac_address,
		.artifact_name                = artifact_name,
		.device_type                  = CONFIG_MENDER_DEVICE_TYPE,
		.host                         = CONFIG_MENDER_SERVER_HOST,
		.tenant_token                 = CONFIG_MENDER_SERVER_TENANT_TOKEN,
		.authentication_poll_interval = CONFIG_MENDER_CLIENT_AUTHENTICATION_POLL_INTERVAL,
		.inventory_poll_interval      = CONFIG_MENDER_CLIENT_INVENTORY_POLL_INTERVAL,
		.update_poll_interval         = CONFIG_MENDER_CLIENT_UPDATE_POLL_INTERVAL,
		.restart_poll_interval        = CONFIG_MENDER_CLIENT_RESTART_POLL_INTERVAL,
		.recommissioning              = false
	};
    mender_client_callbacks_t mender_client_callbacks = {
		.authentication_success = authentication_success_cb,
		.authentication_failure = authentication_failure_cb,
		.deployment_status      = deployment_status_cb,
		.ota_begin              = mender_ota_begin,
		.ota_write              = mender_ota_write,
		.ota_abort              = mender_ota_abort,
		.ota_end                = mender_ota_end,
		.ota_set_boot_partition = mender_ota_set_boot_partition,
		.restart                = restart_cb
	};
    ESP_ERROR_CHECK(mender_client_init(&mender_client_config, &mender_client_callbacks));
    ESP_LOGI(TAG, "Mender client initialized");

    /* Set mender inventory (this is just an example) */
    mender_inventory_t inventory[] = { { .name = "latitude", .value = "45.8325" }, { .name = "longitude", .value = "6.864722" } };
    if (MENDER_OK != mender_client_set_inventory(inventory, sizeof(inventory) / sizeof(inventory[0]))) {
        ESP_LOGE(TAG, "Unable to set mender inventory");
    }

	return MENDER_OK;
}