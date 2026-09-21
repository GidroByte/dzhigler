cmake_minimum_required(VERSION 3.13)

if(NOT DEFINED PICO_SDK_PATH)
  set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()

if(NOT PICO_SDK_PATH OR NOT EXISTS "${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
  message(FATAL_ERROR
    "PICO_SDK_PATH не задан или указывает не туда: '${PICO_SDK_PATH}'.\n"
    "Передайте -DPICO_SDK_PATH=<путь к pico-sdk> или задайте переменную "
    "окружения PICO_SDK_PATH перед запуском cmake.")
endif()

set(PICO_BOARD pico CACHE STRING "Board type for pico-sdk (pico, pico_w, ...)")

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

if(NOT DEFINED PICO_PIO_USB_PATH)
  set(PICO_PIO_USB_PATH $ENV{PICO_PIO_USB_PATH})
endif()
if(NOT DEFINED PICO_TINYUSB_PATH)
  set(PICO_TINYUSB_PATH ${PICO_SDK_PATH}/lib/tinyusb)
endif()

project(dzhigler C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Если BUILD_TYPE не задан извне — Release (иначе CMake собирает без -O)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

if(NOT PICO_PIO_USB_PATH OR NOT EXISTS "${PICO_PIO_USB_PATH}/CMakeLists.txt")
  message(FATAL_ERROR
    "PICO_PIO_USB_PATH не задан или указывает не туда: '${PICO_PIO_USB_PATH}'.\n"
    "Передайте -DPICO_PIO_USB_PATH=<путь к Pico-PIO-USB> или задайте "
    "переменную окружения PICO_PIO_USB_PATH.")
endif()

add_subdirectory(${PICO_PIO_USB_PATH} pico_pio_usb)

set(target_name dzhigler)
add_executable(${target_name})

set(DZHIGLER_HCD_PIO_USB
    "${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c")

if(NOT EXISTS "${DZHIGLER_HCD_PIO_USB}")
  message(FATAL_ERROR
    "Не найден ${DZHIGLER_HCD_PIO_USB}.\n"
    "Драйвер hcd_pio_usb.c предоставляется TinyUSB (не Pico-PIO-USB), "
    "проверьте версию TinyUSB в PICO_TINYUSB_PATH (${PICO_TINYUSB_PATH}) — "
    "ожидается layout src/portable/raspberrypi/pio_usb/hcd_pio_usb.c.")
endif()

target_sources(${target_name} PRIVATE
  main.c
  usb_descriptors.c
  config_store.c

  # Host: PIO USB (роль хоста на core1)
  ${DZHIGLER_HCD_PIO_USB}

  # DCD для роли device через PIO USB.
  # Нам НЕ нужен: device работает по нативному USB (tinyusb_device
  # подключает dcd_rp2040/dcd_dwc2 сам). Оставлен закомментированным,
  # включать только если сознательно переводишь device на PIO USB.
  # ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/dcd_pio_usb.c
)

target_compile_definitions(${target_name} PRIVATE PIO_USB_USE_TINYUSB)

target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_compile_options(${target_name} PRIVATE -Wall -Wextra)
target_link_options(${target_name} PRIVATE -Xlinker --print-memory-usage)

target_link_libraries(${target_name} PRIVATE
  pico_stdlib
  pico_pio_usb
  tinyusb_device
  tinyusb_host
  hardware_flash
  pico_flash
  pico_multicore
)

# stdio выключен полностью:
#  - UART садится на GP0/GP1, которые заняты PIO USB (D+/D-);
#  - USB-stdio притащил бы свой CDC и сломал наш композитный дескриптор.
# Отладочный вывод идёт через наш собственный CDC (usb_descriptors.c).
pico_enable_stdio_uart(${target_name} 0)
pico_enable_stdio_usb(${target_name} 0)

# ---------------------------------------------------------------------
# Резерв места в конце flash под сектор конфига.
#
# DZHIGLER_FLASH_TOTAL_BYTES — CACHE STRING без FORCE: обычная сборка
# без -D получает дефолт 2097152 (реальный размер flash RP2040), а
# cmake -DDZHIGLER_FLASH_TOTAL_BYTES=<меньше> .. этот дефолт не тронет,
# позволяя намеренно проверить, что check_flash_size.cmake способен
# упасть (мета-тест самой проверки).
set(DZHIGLER_FLASH_TOTAL_BYTES 2097152 CACHE STRING
    "Total flash size in bytes (RP2040 flash chip size)")
set(DZHIGLER_CONFIG_SECTOR_BYTES 4096)    # FLASH_SECTOR_SIZE
set(DZHIGLER_XIP_BASE 0x10000000)         # начало flash-региона RP2040
math(EXPR DZHIGLER_FLASH_FOR_CODE
     "${DZHIGLER_FLASH_TOTAL_BYTES} - ${DZHIGLER_CONFIG_SECTOR_BYTES}")

target_compile_definitions(${target_name} PRIVATE
  DZHIGLER_CONFIG_FLASH_OFFSET=${DZHIGLER_FLASH_FOR_CODE}
)

add_custom_target(check_flash_size ALL
  COMMAND ${CMAKE_COMMAND}
    -DELF_FILE=$<TARGET_FILE:${target_name}>
    -DFLASH_LIMIT=${DZHIGLER_FLASH_FOR_CODE}
    -DXIP_BASE=${DZHIGLER_XIP_BASE}
    -DFLASH_TOTAL=${DZHIGLER_FLASH_TOTAL_BYTES}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_flash_size.cmake
  DEPENDS ${target_name} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_flash_size.cmake
  COMMENT "Проверка: прошивка не залезает в зарезервированный сектор конфига"
  VERBATIM
)

pico_add_extra_outputs(${target_name})
