# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(bma280.c)
zephyr_library_sources_ifdef(CONFIG_BMA280_TRIGGER bma280_trigger.c)
