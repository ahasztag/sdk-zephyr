# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: Apache-2.0

# Include MCUboot if enabled.
if(SB_CONFIG_BOOTLOADER_MCUBOOT)
  set(image mcuboot)
  set(configure_only FALSE)
  if (SB_CONFIG_BOOTLOADER_MCUBOOT_ONLY_IMAGES)
    set(configure_only TRUE)
  endif()
  ExternalZephyrProject_Add(
    APPLICATION ${image}
    SOURCE_DIR ${ZEPHYR_MCUBOOT_MODULE_DIR}/boot/zephyr/
    APP_TYPE BOOTLOADER
    CONFIGURE_ONLY ${configure_only}
  )
  set_target_properties(${image} PROPERTIES BUILD_COMMAND ${CMAKE_COMMAND} --build . --target config-twister)
  # Place MCUBoot first in list to ensure it is configured and flashed before other images.
  sysbuild_add_dependencies(CONFIGURE ${DEFAULT_IMAGE} ${image})
  sysbuild_add_dependencies(FLASH ${DEFAULT_IMAGE} ${image})

  set_config_string(${image} CONFIG_BOOT_SIGNATURE_KEY_FILE "${SB_CONFIG_BOOT_SIGNATURE_KEY_FILE}")
endif()
