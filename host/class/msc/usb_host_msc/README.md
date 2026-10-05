# USB Host MSC (Mass Storage Class) Driver

[![Component Registry](https://components.espressif.com/components/espressif/usb_host_msc/badge.svg)](https://components.espressif.com/components/espressif/usb_host_msc) ![maintenance-status](https://img.shields.io/badge/maintenance-passively--maintained-yellowgreen.svg) ![changelog](https://img.shields.io/badge/Keep_a_Changelog-blue?logo=keepachangelog&logoColor=E05735)

This directory contains an implementation of a USB Mass Storage Class Driver implemented on top of the [USB Host Library](https://components.espressif.com/components/espressif/usb).

MSC driver allows access to USB flash drivers using the BOT (Bulk-Only Transport) protocol and the Transparent SCSI command set.

## FatFS / VFS layering

On ESP-IDF 6.0+, MSC supplies an `esp_blockdev` handle. IDF FatFS owns diskio:

```
fopen / VFS -> FatFS -> diskio_bdl.c (IDF) -> msc_bdl read/write -> SCSI READ10/WRITE10 -> BOT/USB
```

`msc_host_install_device()` calls `msc_host_get_blockdev()`. `msc_host_vfs_register()` registers diskio with `ff_diskio_register_bdl()` and selects the volume before mounting FatFs. Apps can also call `msc_host_get_blockdev()` and mount with IDF FatFS BDL APIs themselves; release extra handles with `msc_host_release_blockdev()`. Only one `msc_host_vfs_register()` mount is allowed per device at a time; for multiple concurrent mounts of the same device, get separate handles via `msc_host_get_blockdev()`.

On older IDF, this component registers SCSI-backed FatFS callbacks itself:

```
fopen / VFS -> FatFS -> diskio_usb.c (ff_diskio_register_msc) -> SCSI READ10/WRITE10 -> BOT/USB
```

The public VFS helper (`msc_host_vfs_register`) is the same on both paths.

## Volume selection

`msc_host_vfs_register()` gives MBR partitions precedence over leftover FAT boot parameters in sector zero, which can otherwise cause FatFs to mount an obsolete volume. It tries primary partitions in table order, continuing only when a partition has no filesystem. GPT and unpartitioned volumes retain FatFs automatic discovery.

Volume selection does not write to the device. `msc_host_vfs_format()` and `format_if_mount_failed` still format the entire device as an unpartitioned volume.

## Usage

- First, usb host library has to be initialized by calling `usb_host_install`
- USB Host Library events have to be handled by invoking `usb_host_lib_handle_events` periodically. In general, an application should spawn a dedicated task handle USB Host Library events. However, in order to save RAM, an already existing task can also be used to call `usb_host_lib_handle_events`.
- Mass Storage Class driver is installed by calling `usb_msc_install` function along side with configuration.
- Supplied configuration contains user provided callback function invoked whenever MSC device is connected/disconnected and optional parameters for creating background task handling MSC related events. Alternatively, user can call `usb_msc_handle_events` function from already existing task.
- After receiving `MSC_DEVICE_CONNECTED` event, user has to install device with `usb_msc_install_device` function, obtaining MSC device handle.
- USB descriptors can be printed out with `usb_msc_print_descriptors` and general information about MSC device retrieved with `from usb_msc_get_device_info` function.
- Obtained device handle is then used in helper function `usb_msc_vfs_register` mounting USB Disk to Virtual filesystem.
- At this point, standard C functions for accessing storage (`fopen`, `fwrite`, `fread`, `mkdir` etc.) can be carried out.
- In order to uninstall the whole USB stack, deinitializing counterparts to functions above has to be called in reverse order.

## Performance tuning

The following performance tuning options have significant impact on data throughput in USB HighSpeed implementations. For original FullSpeed implementations, the effects are negligible.

- By default, Newlib (the implementation of C Standard Library) creates cache for each opened file
- The greater the cache, the better performance for the cost of RAM
- Size of the cache can be set with C STD library function `setvbuf()`
- Sizes over 16kB do not improve the performance any more

## Known issues

- Driver only supports flash drives using the BOT (Bulk-Only Transport) protocol and the Transparent SCSI command set

## Examples

- For an example, refer to [msc_host_example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/host/msc) in ESP-IDF
