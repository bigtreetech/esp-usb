idf_component_register(SRC_DIRS .
                        REQUIRES cmock
                        INCLUDE_DIRS .
                        PRIV_INCLUDE_DIRS "../../private_include"
                        WHOLE_ARCHIVE)

# Keep the real VFS, diskio and FatFs paths for partition regressions.
foreach(function scsi_cmd_read10 scsi_cmd_write10 f_mkfs)
    target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=${function}")
endforeach()
