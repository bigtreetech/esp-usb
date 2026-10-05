/*
 * SPDX-FileCopyrightText: 2015-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Mount USB MSC storage on VFS/FatFS.
 *
 * ESP-IDF 6.0+:
 *   MSC supplies an esp_blockdev handle from msc_host_get_blockdev().
 *   IDF FatFS owns diskio: ff_diskio_register_bdl() -> diskio_bdl.c -> BDL ops.
 *
 *   fopen / VFS -> FatFS -> diskio_bdl.c (IDF) -> msc_bdl_read/write -> SCSI
 *
 * Pre-6.0:
 *   This component registers SCSI-backed FatFS callbacks itself
 *   (ff_diskio_register_msc in diskio_usb.c), then mounts FatFS.
 *
 *   fopen / VFS -> FatFS -> diskio_usb.c -> scsi_cmd_read10/write10
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include "msc_common.h"
#include "msc_scsi_bot.h"
#include "usb/msc_host_vfs.h"
#include "diskio_impl.h"
#include "ffconf.h"
#include "ff.h"
#include "esp_idf_version.h"
#ifdef MSC_HOST_BDL_API_SUPPORTED
#include "diskio_bdl.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 2, 0)
#include "esp_vfs.h"
#elif ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 1, 0)
// IDF 6.1's BDL mount helper uses this function before its public declaration.
extern esp_err_t esp_vfs_set_readonly_flag(const char *base_path);
#endif
#endif // MSC_HOST_BDL_API_SUPPORTED

#define DRIVE_STR_LEN 3

/* This component owns diskio registration, so drive/pdrv
 * must be cached here for later use in msc_host_vfs_unregister(). */
typedef struct msc_host_vfs {
    char drive[DRIVE_STR_LEN];
    char *base_path;
    uint8_t pdrv;
#ifdef MSC_HOST_BDL_API_SUPPORTED
    msc_device_t *dev;         /* owner; used to clear dev->bdl_vfs_registered on unregister */
#endif
} msc_host_vfs_t;

static const char *TAG = "MSC VFS";

static esp_err_t msc_format_storage(size_t block_size, size_t allocation_size, const msc_host_vfs_t *vfs,
                                    BYTE num_fats, UINT root_entries)
{
    void *workbuf = NULL;
    const size_t workbuf_size = 4096;

    MSC_RETURN_ON_FALSE(workbuf = ff_memalloc(workbuf_size), ESP_ERR_NO_MEM);

    // Valid value of cluster size is between sector_size and 128 * sector_size.
    size_t cluster_size = MIN(MAX(allocation_size, block_size), 128 * block_size);

    // Formatting retains the existing whole-device FM_SFD behavior.
    BYTE previous_partition = VolToPart[vfs->pdrv].pt;
    VolToPart[vfs->pdrv].pt = 0;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    FRESULT err = f_mkfs(vfs->drive, FM_ANY | FM_SFD, cluster_size, workbuf, workbuf_size);
#else
    const MKFS_PARM opt = {(BYTE)(FM_ANY | FM_SFD), num_fats, 0, root_entries, cluster_size};
    FRESULT err = f_mkfs(vfs->drive, &opt, workbuf, workbuf_size);
#endif

    if (err) {
        VolToPart[vfs->pdrv].pt = previous_partition;
        ESP_LOGE(TAG, "Formatting failed with error: %d", err);
        free(workbuf);
        return ESP_ERR_MSC_FORMAT_FAILED;
    }

    free(workbuf);
    return ESP_OK;
}

esp_err_t msc_host_vfs_format(msc_host_device_handle_t device, const esp_vfs_fat_mount_config_t *mount_config, const msc_host_vfs_handle_t vfs_handle)
{
    MSC_RETURN_ON_INVALID_ARG(device);
    MSC_RETURN_ON_INVALID_ARG(mount_config);
    MSC_RETURN_ON_INVALID_ARG(vfs_handle);

    size_t block_size = ((msc_device_t *)device)->disk.block_size;
    size_t alloc_size = mount_config->allocation_unit_size;

    return msc_format_storage(block_size, alloc_size, vfs_handle, 0, 0);
}

static void dealloc_msc_vfs(msc_host_vfs_t *vfs)
{
    free(vfs->base_path);
    free(vfs);
}

static uint32_t read_le32(const uint8_t *data)
{
    return (uint32_t)data[0] | (uint32_t)data[1] << 8 |
           (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
}

static uint8_t get_mbr_partitions(const uint8_t *sector, uint32_t block_count)
{
    if (sector[510] != 0x55 || sector[511] != 0xaa) {
        return 0;
    }

    uint8_t partitions = 0;
    for (unsigned i = 0; i < 4; i++) {
        const uint8_t *entry = sector + 446 + i * 16;
        if (entry[4] == 0) {
            continue;
        }
        const uint32_t start = read_le32(entry + 8);
        const uint32_t count = read_le32(entry + 12);
        // Leave GPT to FatFs and avoid mistaking unpartitioned boot code for an MBR.
        if (entry[4] == 0xee || (entry[0] != 0 && entry[0] != 0x80) ||
                start == 0 || count == 0 || (uint64_t)start + count > block_count) {
            return 0;
        }
        partitions |= (uint8_t)(1U << i);
    }
    return partitions;
}

static esp_err_t probe_partitions(msc_device_t *dev, uint8_t *candidates)
{
    uint8_t *sector = malloc(dev->disk.block_size);
    MSC_RETURN_ON_FALSE(sector != NULL, ESP_ERR_NO_MEM);
    esp_err_t ret = scsi_cmd_read10(dev, sector, 0, 1, dev->disk.block_size);
    if (ret == ESP_OK) {
        *candidates = get_mbr_partitions(sector, dev->disk.block_count);
    }
    free(sector);
    return ret == ESP_OK ? ESP_OK : ESP_ERR_MSC_MOUNT_FAILED;
}

static FRESULT mount_volume(FATFS *fs, const msc_host_vfs_t *vfs, uint8_t candidates)
{
    VolToPart[vfs->pdrv].pd = vfs->pdrv;
    VolToPart[vfs->pdrv].pt = 0;
    if (candidates) {
        // Explicit partitions prevent a stale sector-zero BPB from winning.
        FRESULT result = FR_NO_FILESYSTEM;
        for (uint8_t partition = 1; partition <= 4; partition++) {
            if (!(candidates & (1U << (partition - 1)))) {
                continue;
            }
            VolToPart[vfs->pdrv].pt = partition;
            result = f_mount(fs, vfs->drive, 1);
            if (result != FR_NO_FILESYSTEM) {
                return result;
            }
        }
        return result;
    }
    return f_mount(fs, vfs->drive, 1);
}

static void unregister_disk(msc_host_vfs_t *vfs)
{
    VolToPart[vfs->pdrv].pd = vfs->pdrv;
    VolToPart[vfs->pdrv].pt = 0;
    ff_diskio_unregister(vfs->pdrv);
#ifdef MSC_HOST_BDL_API_SUPPORTED
    ff_diskio_clear_pdrv_bdl(vfs->dev->bdl);
#endif
}

esp_err_t msc_host_vfs_register(msc_host_device_handle_t device,
                                const char *base_path,
                                const esp_vfs_fat_mount_config_t *mount_config,
                                msc_host_vfs_handle_t *vfs_handle)
{
    MSC_RETURN_ON_INVALID_ARG(device);
    MSC_RETURN_ON_INVALID_ARG(base_path);
    MSC_RETURN_ON_INVALID_ARG(mount_config);
    MSC_RETURN_ON_INVALID_ARG(vfs_handle);

    FATFS *fs = NULL;
    BYTE pdrv;
    bool diskio_registered = false;
    esp_err_t ret = ESP_ERR_MSC_MOUNT_FAILED;
    msc_device_t *dev = (msc_device_t *)device;
    size_t block_size = dev->disk.block_size;
    size_t alloc_size = mount_config->allocation_unit_size;
    bool read_only = false;
#ifdef MSC_HOST_BDL_API_SUPPORTED
    MSC_RETURN_ON_FALSE(dev->bdl != NULL, ESP_ERR_INVALID_STATE);
    MSC_RETURN_ON_FALSE(!dev->bdl_vfs_registered, ESP_ERR_INVALID_STATE);
    read_only = dev->bdl->device_flags.read_only;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 2, 0)
    MSC_RETURN_ON_FALSE(!(mount_config->read_only && mount_config->format_if_mount_failed), ESP_ERR_INVALID_ARG);
    read_only |= mount_config->read_only;
#endif
#endif

    uint8_t candidates;
    MSC_RETURN_ON_ERROR(probe_partitions(dev, &candidates));

    msc_host_vfs_t *vfs = calloc(1, sizeof(msc_host_vfs_t));
    MSC_RETURN_ON_FALSE(vfs != NULL, ESP_ERR_NO_MEM);

    esp_err_t drive_err = ff_diskio_get_drive(&pdrv);
#ifdef MSC_HOST_BDL_API_SUPPORTED
    MSC_GOTO_ON_FALSE(drive_err == ESP_OK, ESP_ERR_NO_MEM);
#else
    MSC_GOTO_ON_ERROR(drive_err);
#endif

    vfs->pdrv = pdrv;
#ifdef MSC_HOST_BDL_API_SUPPORTED
    vfs->dev = dev;
    MSC_GOTO_ON_ERROR(ff_diskio_register_bdl(pdrv, dev->bdl));
#else
    ff_diskio_register_msc(pdrv, &dev->disk);
#endif
    char drive[DRIVE_STR_LEN] = {(char)('0' + pdrv), ':', 0};
    diskio_registered = true;

    strncpy(vfs->drive, drive, DRIVE_STR_LEN);
    MSC_GOTO_ON_FALSE(vfs->base_path = strdup(base_path), ESP_ERR_NO_MEM);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
    const esp_vfs_fat_conf_t conf = {
        .base_path = base_path,
        .fat_drive = drive,
        .max_files = mount_config->max_files,
    };
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    esp_err_t register_err = esp_vfs_fat_register(&conf, &fs);
#ifdef MSC_HOST_BDL_API_SUPPORTED
    // The SDK BDL helper allows mounting a previously registered FATFS.
    if (register_err == ESP_ERR_INVALID_STATE) {
        register_err = ESP_OK;
    }
#endif
    MSC_GOTO_ON_ERROR(register_err);
#else
    MSC_GOTO_ON_ERROR( esp_vfs_fat_register_cfg(&conf, &fs) );
#endif // ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#else
    MSC_GOTO_ON_ERROR( esp_vfs_fat_register(base_path, drive, mount_config->max_files, &fs) );
#endif // ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)

    FRESULT fresult = mount_volume(fs, vfs, candidates);

    if (fresult != FR_OK) {
        if (!read_only && mount_config->format_if_mount_failed &&
                (fresult == FR_NO_FILESYSTEM || fresult == FR_INT_ERR)) {
            BYTE num_fats = 0;
            UINT root_entries = 0;
#ifdef MSC_HOST_BDL_API_SUPPORTED
            num_fats = mount_config->use_one_fat ? 1 : 2;
            root_entries = dev->disk.block_count <= 128 ? (block_size == 512 ? 16 : 128) : 0;
#endif
            esp_err_t format_err = msc_format_storage(block_size, alloc_size, vfs, num_fats, root_entries);
#ifdef MSC_HOST_BDL_API_SUPPORTED
            // Preserve the SDK BDL helper's error code for automatic formatting.
            MSC_GOTO_ON_FALSE(format_err != ESP_ERR_MSC_FORMAT_FAILED, ESP_ERR_MSC_MOUNT_FAILED);
#endif
            MSC_GOTO_ON_ERROR(format_err);
#ifdef MSC_HOST_BDL_API_SUPPORTED
            MSC_GOTO_ON_FALSE(f_mount(fs, drive, 1) == FR_OK, ESP_ERR_MSC_MOUNT_FAILED);
#else
            MSC_GOTO_ON_FALSE(f_mount(fs, drive, 0) == FR_OK, ESP_ERR_MSC_MOUNT_FAILED);
#endif
        } else {
            goto fail;
        }
    }

#ifdef MSC_HOST_BDL_API_SUPPORTED
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 1, 0)
    if (read_only) {
        esp_vfs_set_readonly_flag(base_path);
    }
#endif
    dev->bdl_vfs_registered = true;
#endif
    *vfs_handle = vfs;
    return ESP_OK;

fail:
    if (diskio_registered) {
        unregister_disk(vfs);
    }
    if (fs) {
        f_mount(NULL, drive, 0);
    }
    esp_vfs_fat_unregister_path(base_path);
    dealloc_msc_vfs(vfs);
    return ret;
}

esp_err_t msc_host_vfs_unregister(msc_host_vfs_handle_t vfs_handle)
{
    MSC_RETURN_ON_INVALID_ARG(vfs_handle);
    msc_host_vfs_t *vfs = (msc_host_vfs_t *)vfs_handle;

    f_mount(NULL, vfs->drive, 0);
    unregister_disk(vfs);
    esp_vfs_fat_unregister_path(vfs->base_path);
#ifdef MSC_HOST_BDL_API_SUPPORTED
    vfs->dev->bdl_vfs_registered = false;
#endif
    dealloc_msc_vfs(vfs);
    return ESP_OK;
}
