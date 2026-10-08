/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <algorithm>
#include <cstring>
#include <vector>
#include <catch2/catch_test_macros.hpp>
#include "usb/msc_host.h"
#include "esp_private/msc_scsi_bot.h"
#include "mock_add_usb_device.h"

extern "C" {
#include "Mockusb_host.h"
// The partition tests wrap these commands; exercise the real BOT path here.
    esp_err_t __real_scsi_cmd_read10(msc_host_device_handle_t, uint8_t *, uint32_t, uint32_t, uint32_t);
    esp_err_t __real_scsi_cmd_write10(msc_host_device_handle_t, const uint8_t *, uint32_t, uint32_t, uint32_t);
}

namespace {

enum : uint8_t { TEST_UNIT_READY = 0x00, REQUEST_SENSE = 0x03, INQUIRY = 0x12,
                 READ_CAPACITY = 0x25, READ10 = 0x28, WRITE10 = 0x2a
               };
enum class CswFault { NONE, SIGNATURE, TAG, SHORT, PHASE, RESERVED_STATUS, RESIDUE };

const usb_device_desc_t device_descriptor = {
    .bLength = sizeof(usb_device_desc_t), .bDescriptorType = USB_B_DESCRIPTOR_TYPE_DEVICE,
    .bcdUSB = 0x0200, .bDeviceClass = 0, .bDeviceSubClass = 0, .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64, .idVendor = 0, .idProduct = 0, .bcdDevice = 0,
    .iManufacturer = 0, .iProduct = 0, .iSerialNumber = 0, .bNumConfigurations = 1,
};

const struct __attribute__((packed))
{
    usb_config_desc_t config;
    usb_intf_desc_t interface;
    usb_ep_desc_t in, out;
} descriptors = {
    .config = {
        .bLength = 9, .bDescriptorType = 2, .wTotalLength = 32, .bNumInterfaces = 1,
        .bConfigurationValue = 1, .iConfiguration = 0, .bmAttributes = 0x80, .bMaxPower = 50,
    },
    .interface = {
        .bLength = 9, .bDescriptorType = 4, .bInterfaceNumber = 3, .bAlternateSetting = 0, .bNumEndpoints = 2,
        .bInterfaceClass = 8, .bInterfaceSubClass = 6, .bInterfaceProtocol = 0x50, .iInterface = 0,
    },
    .in = { .bLength = 7, .bDescriptorType = 5, .bEndpointAddress = 0x81, .bmAttributes = 2, .wMaxPacketSize = 64, .bInterval = 0 },
    .out = { .bLength = 7, .bDescriptorType = 5, .bEndpointAddress = 0x02, .bmAttributes = 2, .wMaxPacketSize = 64, .bInterval = 0 },
};

// Supply device responses at the USB boundary; initialization and SCSI commands
// run through the real driver. This fixture does not simulate USB data toggles.
class LunFallbackFixture {
public:
    LunFallbackFixture()
    {
        active = this;
        Mockusb_host_Init();
        usb_host_mock_dev_list_init();
        REQUIRE(ESP_OK == usb_host_mock_add_device(1, &device_descriptor, &descriptors.config, USB_SPEED_FULL));
        usb_host_client_register_Stub(usb_host_client_register_mock_callback);
        usb_host_client_deregister_Stub(usb_host_client_deregister_mock_callback);
        usb_host_device_open_Stub(open_callback);
        usb_host_device_close_Stub(close_callback);
        usb_host_get_active_config_descriptor_Stub(usb_host_get_active_config_descriptor_mock_callback);
        usb_host_get_device_descriptor_Stub(usb_host_get_device_descriptor_mock_callback);
        usb_host_device_info_Stub(usb_host_device_info_mock_callback);
        usb_host_interface_claim_Stub(claim_callback);
        usb_host_interface_release_Stub(release_callback);
        usb_host_transfer_alloc_Stub(alloc_callback);
        usb_host_transfer_free_Stub(free_callback);
        usb_host_endpoint_halt_IgnoreAndReturn(ESP_OK);
        usb_host_endpoint_flush_IgnoreAndReturn(ESP_OK);
        usb_host_endpoint_clear_IgnoreAndReturn(ESP_OK);
        usb_host_transfer_submit_Stub(bulk_callback);
        usb_host_transfer_submit_control_Stub(control_callback);
        msc_host_driver_config_t config = {};
        config.callback = [](const msc_host_event_t *, void *) {};
        REQUIRE(ESP_OK == msc_host_install(&config));
    }

    ~LunFallbackFixture()
    {
        if (device) {
            CHECK(ESP_OK == msc_host_uninstall_device(device));
        }
        CHECK(ESP_OK == msc_host_uninstall());
        CHECK(open_calls == close_calls);
        CHECK(claims == 0);
        CHECK(transfers == 0);
        CHECK(transfer_bytes == 0);
        Mockusb_host_Verify();
        Mockusb_host_Destroy();
        active = nullptr;
    }

    msc_host_device_handle_t device = nullptr;
    uint8_t max_lun = 1;
    uint16_t ready_luns = 1, inquiry_fail_luns = 0;
    scsi_sense_data_t inquiry_sense = {0x05, 0x25, 0x00}; // LOGICAL UNIT NOT SUPPORTED
    scsi_sense_data_t readiness_sense_once = {};
    bool fail_read = false, fail_sense = false, cancel_inquiry_data = false;
    int transport_error_opcode = -1, transport_error_lun = 0;
    CswFault csw_fault = CswFault::NONE;
    uint8_t fault_opcode = INQUIRY, fault_lun = 0;
    uint8_t sense_response_code = 0x70;
    int sense_length = 18;
    uint32_t last_lba = 63;
    usb_transfer_status_t get_max_status = USB_TRANSFER_STATUS_COMPLETED;
    int get_max_length = USB_SETUP_PACKET_SIZE + 1;
    unsigned open_calls = 0, close_calls = 0, claim_calls = 0, release_calls = 0, transfers = 0;
    unsigned get_max_count = 0, reset_count = 0, clear_count = 0;
    size_t transfer_bytes = 0, transfer_budget = SIZE_MAX;
    std::vector<uint16_t> commands; // High byte: LUN, low byte: opcode.

    unsigned count(uint8_t lun, uint8_t opcode) const
    {
        return std::count(commands.begin(), commands.end(), (lun << 8) | opcode);
    }

    void check_bound(uint8_t lun)
    {
        msc_host_device_info_t info = {};
        REQUIRE(ESP_OK == msc_host_get_device_info(device, &info));
        CHECK(info.lun == lun);
        CHECK(info.sector_size == 512);
        CHECK(info.sector_count == last_lba + 1);
        CHECK(open_calls == 1);
        CHECK(claim_calls == 1);
        CHECK(close_calls == 0);
        CHECK(release_calls == 0);
    }

private:
    static LunFallbackFixture *active;
    enum { CBW, DATA, CSW } phase = CBW;
    uint32_t tag = 0, data_length = 0;
    uint8_t opcode = 0, lun = 0, status = 0;
    scsi_sense_data_t sense = {};
    unsigned claims = 0;

    static esp_err_t open_callback(usb_host_client_handle_t client, uint8_t address,
                                   usb_device_handle_t *handle, int count)
    {
        ++active->open_calls;
        return usb_host_device_open_mock_callback(client, address, handle, count);
    }

    static esp_err_t close_callback(usb_host_client_handle_t client, usb_device_handle_t handle, int count)
    {
        ++active->close_calls;
        return usb_host_device_close_mock_callback(client, handle, count);
    }

    static esp_err_t claim_callback(usb_host_client_handle_t, usb_device_handle_t, uint8_t interface, uint8_t alternate, int)
    {
        CHECK(interface == 3);
        CHECK(alternate == 0);
        ++active->claim_calls;
        ++active->claims;
        return ESP_OK;
    }

    static esp_err_t release_callback(usb_host_client_handle_t, usb_device_handle_t, uint8_t interface, int)
    {
        CHECK(interface == 3);
        ++active->release_calls;
        REQUIRE(active->claims == 1);
        --active->claims;
        return ESP_OK;
    }

    static esp_err_t alloc_callback(size_t size, int packets, usb_transfer_t **transfer, int count)
    {
        if (size > active->transfer_budget || active->transfer_bytes > active->transfer_budget - size) {
            return ESP_ERR_NO_MEM;
        }
        esp_err_t err = usb_host_transfer_alloc_mock_callback(size, packets, transfer, count);
        if (err == ESP_OK) {
            ++active->transfers;
            active->transfer_bytes += (*transfer)->data_buffer_size;
        }
        return err;
    }

    static esp_err_t free_callback(usb_transfer_t *transfer, int count)
    {
        if (transfer) {
            REQUIRE(active->transfers > 0);
            --active->transfers;
            active->transfer_bytes -= transfer->data_buffer_size;
        }
        return usb_host_transfer_free_mock_callback(transfer, count);
    }

    static esp_err_t control_callback(usb_host_client_handle_t, usb_transfer_t *transfer, int)
    {
        auto &f = *active;
        usb_setup_packet_t setup;
        std::memcpy(&setup, transfer->data_buffer, sizeof(setup));
        transfer->status = USB_TRANSFER_STATUS_COMPLETED;
        transfer->actual_num_bytes = transfer->num_bytes;
        if (setup.bRequest == 0xfe) {
            CHECK(setup.bmRequestType == 0xa1);
            CHECK(setup.wValue == 0);
            CHECK(setup.wIndex == 3);
            CHECK(setup.wLength == 1);
            ++f.get_max_count;
            transfer->status = f.get_max_status;
            transfer->actual_num_bytes = f.get_max_length;
            transfer->data_buffer[USB_SETUP_PACKET_SIZE] = f.max_lun;
        } else if (setup.bRequest == 0xff) {
            CHECK(setup.wIndex == 3);
            ++f.reset_count;
            f.phase = CBW;
        } else {
            CHECK(setup.bRequest == USB_B_REQUEST_CLEAR_FEATURE);
            CHECK((setup.wIndex == 0x81 || setup.wIndex == 0x02));
            ++f.clear_count;
        }
        transfer->callback(transfer);
        return ESP_OK;
    }

    static esp_err_t bulk_callback(usb_transfer_t *transfer, int)
    {
        auto &f = *active;
        uint8_t *data = transfer->data_buffer;
        transfer->status = USB_TRANSFER_STATUS_COMPLETED;
        transfer->actual_num_bytes = transfer->num_bytes;
        if (f.phase == CBW) {
            REQUIRE(transfer->bEndpointAddress == 0x02);
            REQUIRE(transfer->num_bytes == 31);
            CHECK(std::memcmp(data, "USBC", 4) == 0);
            std::memcpy(&f.tag, data + 4, sizeof(f.tag));
            std::memcpy(&f.data_length, data + 8, sizeof(f.data_length));
            f.lun = data[13];
            f.opcode = data[15];
            REQUIRE(f.lun < 16);
            f.commands.push_back((f.lun << 8) | f.opcode);
            f.status = 0;
            if (f.lun == f.transport_error_lun && f.opcode == f.transport_error_opcode) {
                transfer->status = USB_TRANSFER_STATUS_ERROR;
                transfer->actual_num_bytes = 0;
                transfer->callback(transfer);
                return ESP_OK;
            }
            if (f.opcode == INQUIRY && (f.inquiry_fail_luns & (1 << f.lun))) {
                f.status = 1;
                f.sense = f.inquiry_sense;
            } else if (f.opcode == TEST_UNIT_READY) {
                if (f.lun == 0 && f.readiness_sense_once.key != 0) {
                    f.status = 1;
                    f.sense = f.readiness_sense_once;
                    f.readiness_sense_once = {};
                } else if (!(f.ready_luns & (1 << f.lun))) {
                    f.status = 1;
                    f.sense = {0x02, 0x3a, 0x00};
                }
            } else if (f.opcode == REQUEST_SENSE && f.fail_sense) {
                f.status = 1;
            } else if (f.opcode == READ10 && f.fail_read) {
                f.status = 1;
                f.sense = {0x03, 0x11, 0x00};
            }
            f.phase = f.data_length ? DATA : CSW;
        } else if (f.phase == DATA) {
            REQUIRE(f.data_length <= transfer->data_buffer_size);
            if (f.opcode == INQUIRY && f.cancel_inquiry_data) {
                f.phase = CBW;
                transfer->status = USB_TRANSFER_STATUS_CANCELED;
                transfer->actual_num_bytes = 0;
                transfer->callback(transfer);
                return ESP_OK;
            }
            if (f.opcode != WRITE10) {
                CHECK(transfer->bEndpointAddress == 0x81);
                std::fill_n(data, f.data_length, 0);
            }
            if (f.opcode == READ_CAPACITY) {
                REQUIRE(f.data_length == 8);
                for (unsigned i = 0; i < 4; ++i) {
                    data[i] = f.last_lba >> (8 * (3 - i));
                }
                data[6] = 2; // 512-byte sectors.
            } else if (f.opcode == REQUEST_SENSE) {
                REQUIRE(f.data_length == 18);
                data[0] = f.sense_response_code;
                data[2] = f.sense.key;
                data[7] = 10;
                data[12] = f.sense.code;
                data[13] = f.sense.code_q;
                // A second REQUEST SENSE must not erase the first diagnosis.
                f.sense = {};
            } else if (f.opcode == READ10) {
                std::fill_n(data, f.data_length, 0xa5);
            } else if (f.opcode == WRITE10) {
                CHECK(transfer->bEndpointAddress == 0x02);
            } else {
                CHECK(f.opcode == INQUIRY);
            }
            transfer->actual_num_bytes = f.opcode == REQUEST_SENSE ? f.sense_length : f.data_length;
            f.phase = CSW;
        } else {
            CHECK(transfer->bEndpointAddress == 0x81);
            std::memcpy(data, "USBS", 4);
            std::memcpy(data + 4, &f.tag, sizeof(f.tag));
            std::fill_n(data + 8, 4, 0);
            data[12] = f.status;
            transfer->actual_num_bytes = 13;
            if (f.lun == f.fault_lun && f.opcode == f.fault_opcode) {
                switch (f.csw_fault) {
                case CswFault::SIGNATURE: data[0] ^= 1; break;
                case CswFault::TAG: data[4] ^= 1; break;
                case CswFault::SHORT: transfer->actual_num_bytes = 12; break;
                case CswFault::PHASE: data[12] = 2; break;
                case CswFault::RESERVED_STATUS: data[12] = 3; break;
                case CswFault::RESIDUE: data[8] = 1; break;
                case CswFault::NONE: break;
                }
            }
            f.phase = CBW;
        }
        transfer->callback(transfer);
        return ESP_OK;
    }
};

LunFallbackFixture *LunFallbackFixture::active = nullptr;

} // namespace

TEST_CASE_METHOD(LunFallbackFixture, "MSC automatic installation preserves the LUN0 command sequence", "[msc][lun]")
{
    SECTION("Legacy installation") {
        REQUIRE(ESP_OK == msc_host_install_device(1, &device));
    }
    SECTION("Automatic installation") {
        REQUIRE(ESP_OK == msc_host_install_device_auto(1, &device));
    }
    const std::vector<uint16_t> expected = {INQUIRY, TEST_UNIT_READY, READ_CAPACITY};
    CHECK(commands == expected);
    CHECK(get_max_count == 0);
    CHECK(reset_count == 0);
    CHECK(clear_count == 0);
    check_bound(0);
}

TEST_CASE_METHOD(LunFallbackFixture, "MSC skips an unavailable LUN0 immediately in the same session", "[msc][lun]")
{
    ready_luns = 2;
    SECTION("No medium") {}
    SECTION("Unsupported LUN") {
        readiness_sense_once = {0x05, 0x25, 0x00};
    }
    REQUIRE(ESP_OK == msc_host_install_device_auto(1, &device));
    check_bound(1);
    CHECK(get_max_count == 1);
    CHECK(reset_count == 0);
    CHECK(clear_count == 0);
    const std::vector<uint16_t> initialization = {INQUIRY, TEST_UNIT_READY, REQUEST_SENSE,
                                                  0x100 | INQUIRY, 0x100 | TEST_UNIT_READY, 0x100 | READ_CAPACITY
                                                 };
    CHECK(commands == initialization);

    commands.clear();
    uint8_t sector[512] = {};
    REQUIRE(ESP_OK == __real_scsi_cmd_read10(device, sector, 0, 1, sizeof(sector)));
    CHECK(sector[0] == 0xa5);
    REQUIRE(ESP_OK == __real_scsi_cmd_write10(device, sector, 0, 1, sizeof(sector)));
    fail_read = true;
    CHECK(ESP_FAIL == __real_scsi_cmd_read10(device, sector, 0, 1, sizeof(sector)));
    REQUIRE(ESP_OK == msc_host_reset_recovery(device));
    const std::vector<uint16_t> expected = {0x100 | READ10, 0x100 | WRITE10,
                                            0x100 | READ10, 0x100 | REQUEST_SENSE, 0x100 | TEST_UNIT_READY
                                           };
    CHECK(commands == expected);
    CHECK(reset_count == 1);
    CHECK(clear_count == 2);
}

TEST_CASE_METHOD(LunFallbackFixture, "MSC preserves the legacy readiness wait for an empty LUN0", "[msc][lun]")
{
    ready_luns = 2;
    CHECK(ESP_OK != msc_host_install_device(1, &device));
    CHECK(device == nullptr);
    CHECK(count(0, TEST_UNIT_READY) == 51);
    CHECK(count(0, REQUEST_SENSE) == 102);
    CHECK(count(0, READ_CAPACITY) == 0);
    CHECK(get_max_count == 0);
    CHECK(count(1, INQUIRY) == 0);
}

TEST_CASE_METHOD(LunFallbackFixture, "MSC gives LUN0 its existing readiness retries for a recoverable condition", "[msc][lun]")
{
    SECTION("Unit attention") {
        readiness_sense_once = {0x06, 0x28, 0x00};
    }
    SECTION("Becoming ready") {
        readiness_sense_once = {0x02, 0x04, 0x01};
    }
    REQUIRE(ESP_OK == msc_host_install_device_auto(1, &device));
    const std::vector<uint16_t> expected = {INQUIRY, TEST_UNIT_READY, REQUEST_SENSE,
                                            REQUEST_SENSE, TEST_UNIT_READY, READ_CAPACITY
                                           };
    CHECK(commands == expected);
    CHECK(get_max_count == 0);
    check_bound(0);
}

TEST_CASE_METHOD(LunFallbackFixture, "MSC preserves LUN0 readiness retries after a transport error", "[msc][lun]")
{
    ready_luns = 2;
    fault_opcode = TEST_UNIT_READY;
    csw_fault = CswFault::SIGNATURE;
    CHECK(ESP_OK != msc_host_install_device_auto(1, &device));
    CHECK(device == nullptr);
    CHECK(count(0, TEST_UNIT_READY) == 51);
    CHECK(get_max_count == 0);
    CHECK(count(1, INQUIRY) == 0);
}

TEST_CASE_METHOD(LunFallbackFixture, "MSC tries advertised LUNs in order without reopening the interface", "[msc][lun]")
{
    inquiry_fail_luns = 1;
    ready_luns = 4;
    max_lun = 2;
    REQUIRE(ESP_OK == msc_host_install_device_auto(1, &device));
    check_bound(2);
    CHECK(count(0, INQUIRY) == 1);
    CHECK(count(0, REQUEST_SENSE) == 1);
    CHECK(count(1, TEST_UNIT_READY) == 1); // Later empty slots do not consume another five-second wait.
    CHECK(count(2, READ_CAPACITY) == 1);
    CHECK(get_max_count == 1);
    CHECK(reset_count == 0);
    CHECK(clear_count == 0);
}

TEST_CASE_METHOD(LunFallbackFixture, "MSC legacy installation never selects a different LUN", "[msc][lun]")
{
    inquiry_fail_luns = 1;
    ready_luns = 2;
    CHECK(ESP_OK != msc_host_install_device(1, &device));
    CHECK(device == nullptr);
    CHECK(get_max_count == 0);
    CHECK(count(1, INQUIRY) == 0);
}

TEST_CASE_METHOD(LunFallbackFixture, "MSC returns failure when no advertised candidate is available", "[msc][lun]")
{
    inquiry_fail_luns = 1;
    ready_luns = 0;
    SECTION("All advertised LUNs are empty") {}
    SECTION("GET MAX LUN stalls for a single-LUN device") {
        get_max_status = USB_TRANSFER_STATUS_STALL;
    }
    SECTION("Device advertises only LUN0") {
        max_lun = 0;
    }
    SECTION("Maximum LUN exceeds BOT's four-bit field") {
        max_lun = 16;
    }
    SECTION("GET MAX LUN has no response byte") {
        get_max_length = USB_SETUP_PACKET_SIZE;
    }
    SECTION("GET MAX LUN transport fails") {
        get_max_status = USB_TRANSFER_STATUS_ERROR;
    }
    CHECK(ESP_OK != msc_host_install_device_auto(1, &device));
    CHECK(device == nullptr);
    CHECK(get_max_count == 1);
    const bool can_scan = get_max_status == USB_TRANSFER_STATUS_COMPLETED && max_lun == 1 &&
                          get_max_length == USB_SETUP_PACKET_SIZE + 1;
    CHECK(count(1, INQUIRY) == (can_scan ? 1 : 0));
    CHECK(reset_count == 0);
    CHECK(clear_count == 0);
}

TEST_CASE_METHOD(LunFallbackFixture, "MSC does not infer an empty LUN from transport or unrelated SCSI errors", "[msc][lun]")
{
    inquiry_fail_luns = 1;
    ready_luns = 2;
    SECTION("INQUIRY transport failure") {
        transport_error_opcode = INQUIRY;
    }
    SECTION("INQUIRY data phase canceled") {
        cancel_inquiry_data = true;
    }
    SECTION("REQUEST SENSE transport failure") {
        transport_error_opcode = REQUEST_SENSE;
    }
    SECTION("REQUEST SENSE command failure") {
        fail_sense = true;
    }
    SECTION("Truncated REQUEST SENSE response") {
        sense_length = 8;
    }
    SECTION("Invalid REQUEST SENSE response format") {
        sense_response_code = 0;
    }
    SECTION("Unrelated ILLEGAL REQUEST") {
        inquiry_sense = {0x05, 0x24, 0x00};
    }
    SECTION("Unsupported-LUN ASC with a different qualifier") {
        inquiry_sense = {0x05, 0x25, 0x01};
    }
    SECTION("Medium error") {
        inquiry_sense = {0x03, 0x11, 0x00};
    }
    SECTION("Malformed CSW signature") {
        csw_fault = CswFault::SIGNATURE;
    }
    SECTION("Mismatched CSW tag") {
        csw_fault = CswFault::TAG;
    }
    SECTION("Short CSW") {
        csw_fault = CswFault::SHORT;
    }
    SECTION("BOT phase error") {
        csw_fault = CswFault::PHASE;
    }
    SECTION("Reserved CSW status") {
        csw_fault = CswFault::RESERVED_STATUS;
    }
    SECTION("Success CSW with residue") {
        inquiry_fail_luns = 0;
        csw_fault = CswFault::RESIDUE;
    }
    CHECK(ESP_OK != msc_host_install_device_auto(1, &device));
    CHECK(device == nullptr);
    CHECK(get_max_count == 0);
    CHECK(count(1, INQUIRY) == 0);
}

TEST_CASE_METHOD(LunFallbackFixture, "MSC stops searching when a later LUN has a transport failure", "[msc][lun]")
{
    inquiry_fail_luns = 1;
    ready_luns = 4;
    max_lun = 2;
    SECTION("USB transfer failure on LUN1") {
        transport_error_lun = 1;
        transport_error_opcode = INQUIRY;
    }
    SECTION("Invalid CSW on LUN1") {
        fault_lun = 1;
        csw_fault = CswFault::SIGNATURE;
    }
    CHECK(ESP_OK != msc_host_install_device_auto(1, &device));
    CHECK(device == nullptr);
    CHECK(get_max_count == 1);
    CHECK(count(1, INQUIRY) == 1);
    CHECK(count(2, INQUIRY) == 0);
}

TEST_CASE_METHOD(LunFallbackFixture, "MSC does not select another LUN for unsupported LUN0 capacity", "[msc][lun]")
{
    last_lba = UINT32_MAX; // Requires READ CAPACITY(16).
    CHECK(ESP_ERR_NOT_SUPPORTED == msc_host_install_device_auto(1, &device));
    CHECK(device == nullptr);
    CHECK(get_max_count == 0);
}

TEST_CASE_METHOD(LunFallbackFixture, "MSC grows its transfer buffer within the available memory", "[msc][lun][memory]")
{
    transfer_budget = 1024;
    SECTION("Legacy installation") {
        REQUIRE(ESP_OK == msc_host_install_device(1, &device));
    }
    SECTION("Automatic LUN selection") {
        REQUIRE(ESP_OK == msc_host_install_device_auto(1, &device));
    }
    uint8_t data[1024] = {};
    REQUIRE(ESP_OK == __real_scsi_cmd_read10(device, data, 0, 1, 512));
    REQUIRE(transfer_bytes == 512);
    REQUIRE(ESP_OK == __real_scsi_cmd_read10(device, data, 0, 2, 512));
    CHECK(transfer_bytes == 1024);
}
