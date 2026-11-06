/*
 * SPDX-FileCopyrightText: 2015-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_log.h"
#include "inttypes.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "esp_check.h"
#include "esp_log.h"
#include "msc_common.h"
#include "msc_scsi_bot.h"
#include "usb/msc_host.h"

static const char *TAG = "USB_MSC_SCSI";

/* --------------------------- SCSI Definitions ----------------------------- */
#define CMD_SENSE_VALID_BIT (1 << 7)
#define SCSI_FLAG_DPO (1<<4)
#define SCSI_FLAG_FUA (1<<3)

#define SCSI_CMD_FORMAT_UNIT 0x04
#define SCSI_CMD_INQUIRY 0x12
#define SCSI_CMD_MODE_SELECT 0x55
#define SCSI_CMD_MODE_SENSE 0x5A
#define SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL 0x1E
#define SCSI_CMD_READ10 0x28
#define SCSI_CMD_READ12 0xA8
#define SCSI_CMD_READ_CAPACITY 0x25
#define SCSI_CMD_READ_FORMAT_CAPACITIES 0x23
#define SCSI_CMD_REQUEST_SENSE 0x03
#define SCSI_CMD_REZERO 0x01
#define SCSI_CMD_SEEK10 0x2B
#define SCSI_CMD_SEND_DIAGNOSTIC 0x1D
#define SCSI_CMD_START_STOP Unit 0x1B
#define SCSI_CMD_TEST_UNIT_READY 0x00
#define SCSI_CMD_VERIFY 0x2F
#define SCSI_CMD_WRITE10 0x2A
#define SCSI_CMD_WRITE12 0xAA
#define SCSI_CMD_WRITE_AND_VERIFY 0x2E

#define IN_DIR   CWB_FLAG_DIRECTION_IN
#define OUT_DIR  0

#define INQUIRY_VID_SIZE    8
#define INQUIRY_PID_SIZE    16
#define INQUIRY_REV_SIZE    4

#define CBW_CMD_SIZE(cmd) (sizeof(cmd) - sizeof(msc_cbw_t))

#define CBW_BASE_INIT(dir, cbw_len, data_len)   \
    .base = {                                   \
        .signature = 0x43425355,                \
        .tag = ++cbw_tag,                       \
        .flags = dir,                           \
        .lun = 0,                               \
        .data_length = data_len,                \
        .cbw_length = cbw_len,                  \
    }

#define CSW_SIGNATURE   0x53425355
#define CBW_SIZE        31

#define CWB_FLAG_DIRECTION_IN (1<<7) // device -> host

/**
 * @brief LUT with error codes and descriptions
 *
 * @see USB Mass Storage Class – UFI Command Specification, Revision 1.0
 * Table 51 - Sense Keys, ASC/ASCQ Listing for All Commands (sorted by Key)
 *
 */
typedef struct {
    uint8_t sense_key;
    uint8_t asc;
    uint8_t ascq;
    const char *description;
} sense_errors_t;

const sense_errors_t sense_errors_lut[] = {
    {0x00, 0x00, 0x00, "NO SENSE"},
    {0x07, 0x27, 0x00, "WRITE PROTECTED MEDIA"},
    // add more items as needed
};

#define SENSE_ERROR_COUNT (sizeof(sense_errors_lut) / sizeof(sense_errors_t))

/**
 * @brief Command Block Wrapper structure
 *
 * @see USB Mass Storage Class – Bulk Only Transport, Table 5.1
 */
typedef struct __attribute__((packed))
{
    uint32_t signature;
    uint32_t tag;
    uint32_t data_length;
    uint8_t flags;
    uint8_t lun;
    uint8_t cbw_length;
} msc_cbw_t;

/**
 * @brief Command Status Wrapper structure
 *
 * @see USB Mass Storage Class – Bulk Only Transport, Table 5.2
 */
typedef struct __attribute__((packed))
{
    uint32_t signature;
    uint32_t tag;
    uint32_t dataResidue;
    uint8_t status;
} msc_csw_t;

typedef struct __attribute__((packed))
{
    msc_cbw_t base;
    uint8_t opcode;
    uint8_t flags;
    uint32_t address;
    uint8_t reserved1;
    uint16_t length;
    uint8_t reserved2[3];
} cbw_read10_t;

typedef struct __attribute__((packed))
{
    msc_cbw_t base;
    uint8_t opcode;
    uint8_t flags;
    uint32_t address;
    uint8_t reserved1;
    uint16_t length;
    uint8_t reserved2[1];
} cbw_write10_t;

typedef struct __attribute__((packed))
{
    msc_cbw_t base;
    uint8_t opcode;
    uint8_t flags;
    uint32_t address;
    uint8_t reserved[6];
} cbw_read_capacity_t;

typedef struct __attribute__((packed))
{
    uint32_t last_lba;      // last accessible LBA (NOT a block count, per SCSI spec)
    uint32_t block_size;
} cbw_read_capacity_response_t;

typedef struct __attribute__((packed))
{
    msc_cbw_t base;
    uint8_t opcode;
    uint8_t flags;
    uint8_t reserved[10];
} cbw_unit_ready_t;

typedef struct __attribute__((packed))
{
    msc_cbw_t base;
    uint8_t opcode;
    uint8_t flags;
    uint8_t reserved_0[2];
    uint8_t allocation_length;
    uint8_t reserved_1[7];
} cbw_sense_t;

typedef struct __attribute__((packed))
{
    uint8_t error_code;
    uint8_t reserved_0;
    uint8_t sense_key;
    uint32_t info;
    uint8_t sense_len;
    uint32_t reserved_1;
    uint8_t sense_code;
    uint8_t sense_code_qualifier;
    uint32_t reserved_2;
} cbw_sense_response_t;

typedef struct __attribute__((packed))
{
    msc_cbw_t base;
    uint8_t opcode;
    uint8_t flags;
    uint8_t page_code;
    uint8_t reserved_0;
    uint8_t allocation_length;
    uint8_t reserved_1[7];
} cbw_inquiry_t;

typedef struct __attribute__((packed))
{
    msc_cbw_t base;
    uint8_t opcode;
    uint8_t flags;
    uint8_t pc_page_code;
    uint8_t reserved_1[4];
    uint16_t parameter_list_length;
    uint8_t reserved_2[3];
} mode_sense_t;

typedef struct __attribute__((packed))
{
    uint8_t data[8];
} mode_sense_response_t;

typedef struct __attribute__((packed))
{
    msc_cbw_t base;
    uint8_t opcode;
    uint8_t flags;
    uint8_t reserved_1[2];
    uint8_t prevent;
    uint8_t reserved_2[7];
} prevent_allow_medium_removal_t;

typedef struct __attribute__((packed))
{
    uint8_t data[36];
} cbw_inquiry_response_t;

// Unique number based on which MSC protocol pairs request and response
static uint32_t cbw_tag;

typedef struct {
    bool transport_ok;
    bool csw_valid;
    uint8_t csw_status;
    size_t data_bytes;
} bot_command_result_t;

static esp_err_t bot_reset_after_error(msc_device_t *device, esp_err_t command_err)
{
    esp_err_t recovery_err = msc_bot_reset_recovery(device);
    return recovery_err == ESP_OK ? command_err : recovery_err;
}

static esp_err_t check_csw(msc_csw_t *csw, uint32_t tag)
{
    const bool csw_ok = csw->signature == CSW_SIGNATURE && csw->tag == tag &&
                        csw->dataResidue == 0 && csw->status == 0;

    if (!csw_ok) {
        ESP_LOGV(TAG, "CSW failed: dCSWSignature = 0x%02"PRIx32", dCSWTag = 0x%02"PRIx32", dCSWDataResidue = 0x%02"PRIx32"",
                 csw->signature, csw->tag, csw->dataResidue);
        ESP_LOGD(TAG, "CSW failed: bCSWStatus 0x%02"PRIx8"", csw->status);
    }

    return csw_ok ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Execute BOT command
 *
 * There are multiple stages in BOT command:
 * 1. Command transport
 * 2. Data transport (optional)
 * 3. Status transport
 * 3.1. Error recovery (in case of error)
 *
 * This function is not 'static' so it could be called from unit test
 *
 * @see USB Mass Storage Class – Bulk Only Transport, Chapter 5.3
 *
 * @param[in] device MSC device handle
 * @param[in] cbw    Command Block Wrapper
 * @param[in] data   Data (optional)
 * @param[in] size   Size of data in bytes
 * @return esp_err_t
 */
static esp_err_t bot_execute_command_with_result(msc_device_t *device,
                                                  msc_cbw_t *cbw, void *data,
                                                  size_t size,
                                                  bot_command_result_t *result)
{
    msc_csw_t csw = { 0 };
    msc_endpoint_t ep = (cbw->flags & CWB_FLAG_DIRECTION_IN) ? MSC_EP_IN : MSC_EP_OUT;
    msc_lun_status_t *lun_status = device->lun_status;
    bool transport_error = lun_status && lun_status->transport_error;
    bool command_transport_error = false;
    size_t data_bytes = 0;

    if (result) {
        *result = (bot_command_result_t) { 0 };
    }

    cbw->lun = device->lun;
    if (lun_status) {
        // Observe initialization without changing the established command or recovery sequence.
        lun_status->transport_error = true;
        lun_status->command_failed = false;
        lun_status->data_bytes = 0;
        if (((const uint8_t *)cbw)[sizeof(*cbw)] != SCSI_CMD_REQUEST_SENSE) {
            lun_status->unavailable = false;
        }
    }

    // 1. Command transport
    esp_err_t err = msc_bulk_transfer(device, (uint8_t *)cbw, CBW_SIZE, MSC_EP_OUT);
    if (err != ESP_OK) {
        return bot_reset_after_error(device, err);
    }
    if (device->xfer->actual_num_bytes != CBW_SIZE) {
        return bot_reset_after_error(device, ESP_ERR_INVALID_RESPONSE);
    }

    // 2. Optional data transport
    if (data) {
        err = msc_bulk_transfer(device, (uint8_t *)data, size, ep);
        data_bytes = device->xfer->actual_num_bytes;
        if (lun_status) {
            lun_status->data_bytes = data_bytes;
        }
        if (err == ESP_ERR_MSC_STALL) {
            transport_error = true;
            command_transport_error = true;
            uint8_t endpoint = ep == MSC_EP_IN ? device->config.bulk_in_ep : device->config.bulk_out_ep;
            esp_err_t clear_err = clear_feature(device, endpoint);
            if (clear_err != ESP_OK) {
                return bot_reset_after_error(device, clear_err);
            }
        } else if (err != ESP_OK) {
            return bot_reset_after_error(device, err);
        }
    }

    // 3. Status transport
    err = msc_bulk_transfer(device, (uint8_t *)&csw, sizeof(msc_csw_t), MSC_EP_IN);

    // 3.1 Error recovery
    if (err == ESP_ERR_MSC_STALL) {
        // BOT 5.3.3 permits one CSW retry after clearing a Bulk-In STALL. A
        // valid CSW from that retry restores a known command result; only a
        // failed retry leaves the status unknown and requires Reset Recovery.
        esp_err_t clear_err = clear_feature(device, device->config.bulk_in_ep);
        if (clear_err != ESP_OK) {
            return bot_reset_after_error(device, clear_err);
        }
        err = msc_bulk_transfer(device, (uint8_t *)&csw, sizeof(msc_csw_t), MSC_EP_IN);
    }

    if (err != ESP_OK) {
        // The command status is unknown. Restore BOT synchronization before
        // allowing another CBW, but do not issue SCSI commands from recovery.
        if (result) {
            result->data_bytes = data_bytes;
        }
        return bot_reset_after_error(device, err);
    }

    const bool csw_valid = device->xfer->actual_num_bytes == sizeof(csw) &&
                           csw.signature == CSW_SIGNATURE &&
                           csw.tag == cbw->tag &&
                           csw.dataResidue <= cbw->data_length &&
                           csw.status <= 2;
    if (result) {
        result->transport_ok = !command_transport_error;
        result->csw_valid = csw_valid;
        result->csw_status = csw.status;
        result->data_bytes = data_bytes;
    }

    if (!csw_valid || csw.status == 2) {
        // Invalid CSWs and Phase Error require Reset Recovery before the next CBW.
        return bot_reset_after_error(device, ESP_FAIL);
    }

    if (lun_status && device->xfer->actual_num_bytes == sizeof(csw) &&
            csw.signature == CSW_SIGNATURE && csw.tag == cbw->tag &&
            csw.dataResidue <= cbw->data_length && csw.status <= 1) {
        lun_status->transport_error = transport_error;
        lun_status->command_failed = csw.status == 1;
    }

    esp_err_t csw_err = check_csw(&csw, cbw->tag);
    if (csw_err == ESP_OK && data != NULL && data_bytes != size) {
        return ESP_ERR_INVALID_SIZE;
    }
    return csw_err;
}

esp_err_t bot_execute_command(msc_device_t *device, msc_cbw_t *cbw,
                              void *data, size_t size)
{
    return bot_execute_command_with_result(device, cbw, data, size, NULL);
}

static const char *decode_sense_keys(cbw_sense_response_t *sense_response)
{
    // Only decode WRITE_PROTECTED_MEDIA sense key, other keys are not implemented
    for (int i = 0; i < SENSE_ERROR_COUNT; i++) {
        if (sense_errors_lut[i].sense_key == sense_response->sense_key &&
                sense_errors_lut[i].asc == sense_response->sense_code &&
                sense_errors_lut[i].ascq == sense_response->sense_code_qualifier) {
            return sense_errors_lut[i].description;
        }
    }

    return "not found, refer to USB Mass Storage Class – UFI Command Specification (Table 51)";
}

static bool is_retryable_write_crc_error(const cbw_sense_response_t *response)
{
    return (response->sense_key & 0x0F) == 0x0B &&
           response->sense_code == 0x47 &&
           response->sense_code_qualifier == 0x01;
}

#ifndef MSC_WRITE10_CRC_RETRY_COUNT
#define MSC_WRITE10_CRC_RETRY_COUNT 3
#endif

static esp_err_t scsi_cmd_current_write_sense(
    msc_device_t *device, cbw_sense_response_t *response)
{
    memset(response, 0, sizeof(*response));
    cbw_sense_t cbw = {
        CBW_BASE_INIT(IN_DIR, CBW_CMD_SIZE(cbw_sense_t), sizeof(*response)),
        .opcode = SCSI_CMD_REQUEST_SENSE,
        .allocation_length = sizeof(*response),
    };
    bot_command_result_t result;
    esp_err_t err = bot_execute_command_with_result(
        device, &cbw.base, response, sizeof(*response), &result);
    if (err != ESP_OK) {
        return err;
    }

    const uint8_t response_code = response->error_code & ~CMD_SENSE_VALID_BIT;
    if (!result.transport_ok || !result.csw_valid ||
            result.csw_status != 0 || result.data_bytes < 14 ||
            response_code != 0x70 || response->sense_len < 6) {
        ESP_LOGW(TAG,
                 "WRITE10 retry suppressed: response=0x%02"PRIx8
                 " bytes=%u sense_len=%u transport=%d csw_valid=%d status=%u",
                 response_code, (unsigned)result.data_bytes,
                 (unsigned)response->sense_len, result.transport_ok,
                 result.csw_valid, (unsigned)result.csw_status);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}


esp_err_t scsi_cmd_read10(msc_host_device_handle_t dev,
                          uint8_t *data,
                          uint32_t sector_address,
                          uint32_t num_sectors,
                          uint32_t sector_size)
{
    if (num_sectors != 0 && sector_size > UINT32_MAX / num_sectors) {
        return ESP_ERR_INVALID_SIZE;
    }

    msc_device_t *device = (msc_device_t *)dev;
    cbw_read10_t cbw = {
        CBW_BASE_INIT(IN_DIR, CBW_CMD_SIZE(cbw_read10_t), num_sectors * sector_size),
        .opcode = SCSI_CMD_READ10,
        .flags = 0, // lun
        .address = __builtin_bswap32(sector_address),
        .length = __builtin_bswap16(num_sectors),
    };

    esp_err_t ret = bot_execute_command(device, &cbw.base, data, num_sectors * sector_size);

    // In case of an error, get an error code
    if (unlikely(ret != ESP_OK)) {
        MSC_RETURN_ON_ERROR( scsi_cmd_sense(device, NULL));
    }
    return ret;
}

esp_err_t scsi_cmd_write10(msc_host_device_handle_t dev,
                           const uint8_t *data,
                           uint32_t sector_address,
                           uint32_t num_sectors,
                           uint32_t sector_size)
{
    if (num_sectors != 0 && sector_size > UINT32_MAX / num_sectors) {
        return ESP_ERR_INVALID_SIZE;
    }

    msc_device_t *device = (msc_device_t *)dev;
    cbw_write10_t cbw = {
        CBW_BASE_INIT(OUT_DIR, CBW_CMD_SIZE(cbw_write10_t), num_sectors * sector_size),
        .opcode = SCSI_CMD_WRITE10,
        .address = __builtin_bswap32(sector_address),
        .length = __builtin_bswap16(num_sectors),
    };

    const size_t data_size = num_sectors * sector_size;
    bot_command_result_t write_result;
    esp_err_t ret = bot_execute_command_with_result(
        device, &cbw.base, (void *)data, data_size, &write_result);
    if (unlikely(ret != ESP_OK)) {
        const bool command_failed = write_result.transport_ok &&
                                    write_result.csw_valid &&
                                    write_result.csw_status == 1 &&
                                    write_result.data_bytes == data_size;
        if (!command_failed) {
            return ret;
        }

        cbw_sense_response_t sense;
        esp_err_t sense_err = scsi_cmd_current_write_sense(device, &sense);
        if (sense_err != ESP_OK) {
            return ret;
        }

        /* A valid Command Failed CSW leaves BOT synchronized, so retry the
         * same LBA and payload directly after an exact current CRC sense.
         * Keep this bounded below FatFS so a successful retry is transparent. */
        if (is_retryable_write_crc_error(&sense)) {
            for (unsigned retry = 0; retry < MSC_WRITE10_CRC_RETRY_COUNT; ++retry) {
                cbw.base.tag = ++cbw_tag;
                ret = bot_execute_command_with_result(
                    device, &cbw.base, (void *)data, data_size, &write_result);
                if (ret == ESP_OK) {
                    ESP_LOGD(TAG, "WRITE10 recovered after data-phase CRC error");
                    return ESP_OK;
                }

                const bool retry_command_failed = write_result.transport_ok &&
                                                  write_result.csw_valid &&
                                                  write_result.csw_status == 1 &&
                                                  write_result.data_bytes == data_size;
                if (!retry_command_failed) {
                    return ret;
                }

                if (retry + 1 >= MSC_WRITE10_CRC_RETRY_COUNT) {
                    MSC_RETURN_ON_ERROR(scsi_cmd_sense(device, NULL));
                    return ret;
                }

                sense_err = scsi_cmd_current_write_sense(device, &sense);
                if (sense_err != ESP_OK) {
                    return ret;
                }
                if (!is_retryable_write_crc_error(&sense)) {
                    break;
                }
            }
        }
        ESP_LOGE(TAG, "Sense error codes: Sense Key 0x%02"PRIx8", ASC: 0x%02"PRIx8", ASCQ: 0x%02"PRIx8"",
                 sense.sense_key, sense.sense_code, sense.sense_code_qualifier);
        ESP_LOGE(TAG, "Sense error description: %s", decode_sense_keys(&sense));
    }
    return ret;
}

esp_err_t scsi_cmd_read_capacity(msc_host_device_handle_t dev, uint32_t *block_size, uint32_t *block_count)
{
    msc_device_t *device = (msc_device_t *)dev;
    cbw_read_capacity_response_t response;

    cbw_read_capacity_t cbw = {
        CBW_BASE_INIT(IN_DIR, CBW_CMD_SIZE(cbw_read_capacity_t), sizeof(response)),
        .opcode = SCSI_CMD_READ_CAPACITY,
    };

    esp_err_t ret = bot_execute_command(device, &cbw.base, &response, sizeof(response));

    // In case of an error, get an error code
    if (unlikely(ret != ESP_OK)) {
        MSC_RETURN_ON_ERROR( scsi_cmd_sense(device, NULL));
    }

    uint32_t last_lba = __builtin_bswap32(response.last_lba);

    if (unlikely(last_lba == UINT32_MAX)) {
        // Sentinel: capacity needs READ CAPACITY(16), which is unsupported
        ESP_LOGE(TAG, "Device capacity exceeds READ CAPACITY(10) range");
        return ESP_ERR_NOT_SUPPORTED;
    }

    // last_lba is the LAST block's LBA, not a count -> +1 (safe: UINT32_MAX rejected above)
    *block_count = last_lba + 1;
    *block_size = __builtin_bswap32(response.block_size);

    return ret;
}

esp_err_t scsi_cmd_unit_ready(msc_host_device_handle_t dev)
{
    msc_device_t *device = (msc_device_t *)dev;
    cbw_unit_ready_t cbw = {
        CBW_BASE_INIT(IN_DIR, CBW_CMD_SIZE(cbw_unit_ready_t), 0),
        .opcode = SCSI_CMD_TEST_UNIT_READY,
    };

    esp_err_t ret = bot_execute_command(device, &cbw.base, NULL, 0);

    // In case of an error, get an error code
    if (unlikely(ret != ESP_OK)) {
        /* The ready-state loop requests sense again to decide whether to
         * wait or select another LUN. Preserve that command sequence, but do
         * not print this expected transitional sense as a device error. */
        scsi_sense_data_t ignored_sense;
        MSC_RETURN_ON_ERROR(scsi_cmd_sense(device, &ignored_sense));
    }
    return ret;
}

esp_err_t scsi_cmd_sense(msc_host_device_handle_t dev, scsi_sense_data_t *sense)
{
    msc_device_t *device = (msc_device_t *)dev;
    cbw_sense_response_t response;
    msc_lun_status_t *lun_status = device->lun_status;
    const bool command_failed = lun_status && lun_status->command_failed;

    cbw_sense_t cbw = {
        CBW_BASE_INIT(IN_DIR, CBW_CMD_SIZE(cbw_sense_t), sizeof(response)),
        .opcode = SCSI_CMD_REQUEST_SENSE,
        .allocation_length = sizeof(response),
    };

    MSC_RETURN_ON_ERROR( bot_execute_command(device, &cbw.base, &response, sizeof(response)) );

    if (command_failed && !lun_status->transport_error && lun_status->data_bytes >= 14) {
        const uint8_t response_code = response.error_code & ~CMD_SENSE_VALID_BIT;
        if ((response_code == 0x70 || response_code == 0x71) && response.sense_len >= 6) {
            const uint8_t sense_key = response.sense_key & 0x0F;
            // A second REQUEST SENSE in the ready-state loop must not erase the first result.
            lun_status->unavailable = (sense_key == 0x02 && response.sense_code == 0x3A) ||
                                      (sense_key == 0x05 && response.sense_code == 0x25 && response.sense_code_qualifier == 0);
        }
    }

    if (sense == NULL) {
        ESP_LOGE(TAG, "Sense error codes: Sense Key 0x%02"PRIx8", ASC: 0x%02"PRIx8", ASCQ: 0x%02"PRIx8"",
                 response.sense_key, response.sense_code, response.sense_code_qualifier);
        const char *error_description = decode_sense_keys(&response);
        ESP_LOGE(TAG, "Sense error description: %s", error_description);
        return ESP_OK;
    }

    sense->key = response.sense_key;
    sense->code = response.sense_code;
    sense->code_q = response.sense_code_qualifier;

    return ESP_OK;
}

esp_err_t scsi_cmd_inquiry(msc_host_device_handle_t dev)
{
    msc_device_t *device = (msc_device_t *)dev;
    cbw_inquiry_response_t response = { 0 };

    cbw_inquiry_t cbw = {
        CBW_BASE_INIT(IN_DIR, CBW_CMD_SIZE(cbw_inquiry_t), sizeof(response)),
        .opcode = SCSI_CMD_INQUIRY,
        .allocation_length = sizeof(response),
    };

    esp_err_t ret = bot_execute_command(device, &cbw.base, &response, sizeof(response) );

    // In case of an error, get an error code
    if (unlikely(ret != ESP_OK)) {
        MSC_RETURN_ON_ERROR( scsi_cmd_sense(device, NULL));
    }
    return ret;
}

esp_err_t scsi_cmd_mode_sense(msc_host_device_handle_t dev)
{
    msc_device_t *device = (msc_device_t *)dev;
    mode_sense_response_t response = { 0 };

    mode_sense_t cbw = {
        CBW_BASE_INIT(IN_DIR, CBW_CMD_SIZE(mode_sense_t), sizeof(response)),
        .opcode = SCSI_CMD_MODE_SENSE,
        .pc_page_code = 0x3F,
        .parameter_list_length = sizeof(response),
    };

    esp_err_t ret = bot_execute_command(device, &cbw.base, &response, sizeof(response) );

    // In case of an error, get an error code
    if (unlikely(ret != ESP_OK)) {
        MSC_RETURN_ON_ERROR( scsi_cmd_sense(device, NULL));
    }
    return ret;
}

esp_err_t scsi_cmd_prevent_removal(msc_host_device_handle_t dev, bool prevent)
{
    msc_device_t *device = (msc_device_t *)dev;
    prevent_allow_medium_removal_t cbw = {
        CBW_BASE_INIT(OUT_DIR, CBW_CMD_SIZE(prevent_allow_medium_removal_t), 0),
        .opcode = SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL,
        .prevent = (uint8_t) prevent,
    };

    esp_err_t ret = bot_execute_command(device, &cbw.base, NULL, 0);

    // In case of an error, get an error code
    if (unlikely(ret != ESP_OK)) {
        MSC_RETURN_ON_ERROR( scsi_cmd_sense(device, NULL));
    }
    return ret;
}
