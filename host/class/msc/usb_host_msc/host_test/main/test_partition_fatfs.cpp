/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <array>
#include <cstring>
#include <catch2/catch_test_macros.hpp>
#include "diskio_impl.h"
#include "msc_common.h"
#include "usb/msc_host_vfs.h"

namespace {

void put16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

void put32(uint8_t *data, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) {
        data[i] = (uint8_t)(value >> (8 * i));
    }
}

struct PartitionFixture;
PartitionFixture *active_disk;

// Sparse disk reproducing the reported image: an old BPB at LBA 0 and a
// current partition at LBA 32. Mounting uses real VFS, diskio and FatFs.
struct PartitionFixture {
    static constexpr uint32_t sector_count = 131072;
    std::array<uint8_t, 512> sector_zero = {};
    std::array<uint8_t, 512> partition_boot = {};
    std::array<uint8_t, 512> current_root = {};
    msc_device_t device = {};
    esp_vfs_fat_mount_config_t config = {};
    msc_host_vfs_handle_t vfs = nullptr;
    uint32_t root_lba = 2080;
    uint32_t failed_lba = UINT32_MAX;
    FRESULT format_result = FR_OK;
    unsigned format_calls = 0;
    unsigned write_calls = 0;
    BYTE drive_number;
    char path[4] = {'0', ':', '/', 0};

    static void boot_sector(uint8_t *sector, uint32_t start, uint32_t count, uint32_t fat_size)
    {
        sector[0] = 0xeb;
        sector[1] = 0x58;
        sector[2] = 0x90;
        std::memcpy(sector + 3, "MSCTEST ", 8);
        put16(sector + 11, 512);
        sector[13] = 1;
        put16(sector + 14, 32);
        sector[16] = 2;
        sector[21] = 0xf8;
        put32(sector + 28, start);
        put32(sector + 32, count);
        put32(sector + 36, fat_size);
        put32(sector + 44, 2);
        put16(sector + 48, 1);
        put16(sector + 50, 6);
        std::memcpy(sector + 82, "FAT32   ", 8);
        sector[510] = 0x55;
        sector[511] = 0xaa;
    }

    void partition(unsigned index, uint8_t type, uint32_t start, uint32_t count)
    {
        uint8_t *entry = sector_zero.data() + 446 + 16 * index;
        entry[4] = type;
        put32(entry + 8, start);
        put32(entry + 12, count);
    }

    PartitionFixture()
    {
        boot_sector(sector_zero.data(), 0, sector_count, 1009);
        boot_sector(partition_boot.data(), 32, sector_count - 32, 1008);
        partition(0, 0x0c, 32, sector_count - 32);
        std::memcpy(current_root.data(), "CURRENT TXT", 11);
        current_root[11] = AM_ARC;
        device.disk.block_size = 512;
        device.disk.block_count = sector_count;
        config.max_files = 4;
        REQUIRE(ff_diskio_get_drive(&drive_number) == ESP_OK);
        path[0] = (char)('0' + drive_number);
#ifdef MSC_HOST_BDL_API_SUPPORTED
        REQUIRE(msc_host_get_blockdev(&device, &device.bdl) == ESP_OK);
#endif
        active_disk = this;
    }

    ~PartitionFixture()
    {
        if (vfs) {
            CHECK(msc_host_vfs_unregister(vfs) == ESP_OK);
        }
        CHECK(VolToPart[drive_number].pt == 0);
        CHECK(write_calls == 0);
#ifdef MSC_HOST_BDL_API_SUPPORTED
        CHECK(msc_host_release_blockdev(device.bdl) == ESP_OK);
#endif
        active_disk = nullptr;
    }

    esp_err_t mount()
    {
        return msc_host_vfs_register(&device, "/msc-test", &config, &vfs);
    }

    void check_directory()
    {
        FF_DIR directory;
        FILINFO entry;
        REQUIRE(f_opendir(&directory, path) == FR_OK);
        REQUIRE(f_readdir(&directory, &entry) == FR_OK);
        CHECK(std::strcmp(entry.fname, "CURRENT.TXT") == 0);
        REQUIRE(f_readdir(&directory, &entry) == FR_OK);
        CHECK(entry.fname[0] == 0);
        CHECK(f_closedir(&directory) == FR_OK);
    }
};

} // namespace

extern "C" {

    esp_err_t __real_scsi_cmd_read10(msc_host_device_handle_t device, uint8_t *data,
                                     uint32_t sector, uint32_t count, uint32_t sector_size);
    esp_err_t __real_scsi_cmd_write10(msc_host_device_handle_t device, const uint8_t *data,
                                      uint32_t sector, uint32_t count, uint32_t sector_size);

    esp_err_t __wrap_scsi_cmd_read10(msc_host_device_handle_t device, uint8_t *data,
                                     uint32_t sector, uint32_t count, uint32_t sector_size)
    {
        if (!active_disk) {
            return __real_scsi_cmd_read10(device, data, sector, count, sector_size);
        }
        auto &disk = *active_disk;
        REQUIRE(device == &disk.device);
        REQUIRE(sector_size == 512);
        REQUIRE((uint64_t)sector + count <= PartitionFixture::sector_count);
        if (sector <= disk.failed_lba && disk.failed_lba < (uint64_t)sector + count) {
            return ESP_ERR_TIMEOUT;
        }
        for (uint32_t i = 0; i < count; ++i, data += 512) {
            std::memset(data, 0, 512);
            if (sector + i == 0) {
                std::memcpy(data, disk.sector_zero.data(), 512);
            } else if (sector + i == 32) {
                std::memcpy(data, disk.partition_boot.data(), 512);
            } else if (sector + i == disk.root_lba) {
                std::memcpy(data, disk.current_root.data(), 512);
            }
        }
        return ESP_OK;
    }

    esp_err_t __wrap_scsi_cmd_write10(msc_host_device_handle_t device, const uint8_t *data,
                                      uint32_t sector, uint32_t count, uint32_t sector_size)
    {
        if (!active_disk) {
            return __real_scsi_cmd_write10(device, data, sector, count, sector_size);
        }
        ++active_disk->write_calls;
        return ESP_FAIL;
    }

    FRESULT __real_f_mkfs(const TCHAR *drive, const MKFS_PARM *options, void *work, UINT size);
    FRESULT __wrap_f_mkfs(const TCHAR *drive, const MKFS_PARM *options, void *work, UINT size)
    {
        if (!active_disk) {
            return __real_f_mkfs(drive, options, work, size);
        }
        CHECK(options->fmt == (FM_ANY | FM_SFD));
        CHECK(VolToPart[active_disk->drive_number].pt == 0);
        ++active_disk->format_calls;
        return active_disk->format_result;
    }

} // extern "C"

TEST_CASE_METHOD(PartitionFixture, "MSC mounts the current MBR volume instead of the stale BPB", "[partition]")
{
    BYTE expected_partition = 1;
    SECTION("The first primary partition contains FAT") {}
    SECTION("A later primary partition contains FAT") {
        partition(0, 0x83, 16, 16);
        partition(1, 0x0c, 32, sector_count - 32);
        expected_partition = 2;
    }
#ifdef MSC_HOST_BDL_API_SUPPORTED
    SECTION("The application already registered the VFS path") {
        const char drive[] = {(char)('0' + drive_number), ':', 0};
        const esp_vfs_fat_conf_t conf = {
            .base_path = "/msc-test", .fat_drive = drive, .max_files = (size_t)config.max_files,
        };
        FATFS *fs;
        REQUIRE(esp_vfs_fat_register(&conf, &fs) == ESP_OK);
    }
#endif
    REQUIRE(mount() == ESP_OK);
    CHECK(VolToPart[drive_number].pt == expected_partition);
    check_directory();
    CHECK(format_calls == 0);
}

TEST_CASE_METHOD(PartitionFixture, "MSC still mounts an unpartitioned FAT volume", "[partition]")
{
    std::memset(sector_zero.data() + 446, 0, 64);
    SECTION("No partition entries") {}
    SECTION("Boot code occupies the partition table area") {
        sector_zero[446] = 0xeb;
        sector_zero[450] = 0x90;
    }
    root_lba = 2050;
    REQUIRE(mount() == ESP_OK);
    CHECK(VolToPart[drive_number].pt == 0);
    check_directory();
    CHECK(format_calls == 0);
}

TEST_CASE_METHOD(PartitionFixture, "MSC does not fall back to a stale BPB on partition failure", "[partition]")
{
    esp_err_t expected_error = ESP_ERR_MSC_MOUNT_FAILED;
    unsigned expected_format_calls = 0;
    SECTION("MBR read fails") {
        failed_lba = 0;
        config.format_if_mount_failed = true;
    }
    SECTION("Partition read fails") {
        failed_lba = 32;
        config.format_if_mount_failed = true;
    }
    SECTION("The partition contains no filesystem") {
        partition_boot.fill(0);
    }
    SECTION("Formatting after a missing filesystem fails") {
        partition_boot.fill(0);
        config.format_if_mount_failed = true;
        format_result = FR_DISK_ERR;
        expected_format_calls = 1;
#ifndef MSC_HOST_BDL_API_SUPPORTED
        expected_error = ESP_ERR_MSC_FORMAT_FAILED;
#endif
    }
    CHECK(mount() == expected_error);
    CHECK(vfs == nullptr);
    CHECK(format_calls == expected_format_calls);
}

TEST_CASE_METHOD(PartitionFixture, "MSC formatting keeps whole-device selection", "[partition]")
{
    REQUIRE(mount() == ESP_OK);
    REQUIRE(VolToPart[drive_number].pt == 1);
    SECTION("Successful formatting selects the whole device") {
        CHECK(msc_host_vfs_format(&device, &config, vfs) == ESP_OK);
        CHECK(VolToPart[drive_number].pt == 0);
    }
    SECTION("Failed formatting restores the partition selection") {
        format_result = FR_DISK_ERR;
        CHECK(msc_host_vfs_format(&device, &config, vfs) == ESP_ERR_MSC_FORMAT_FAILED);
        CHECK(VolToPart[drive_number].pt == 1);
    }
    CHECK(format_calls == 1);
}
