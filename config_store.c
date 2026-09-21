/*
 * Хранилище конфигурации во flash.
 *
 * Использует последний физический сектор flash (4096 байт). Адрес
 * задаётся макросом DZHIGLER_CONFIG_FLASH_OFFSET, который передаёт
 * CMakeLists.txt — НЕ выводится из PICO_FLASH_SIZE_BYTES внутри этого
 * файла.
 *
 * Защита от коллизии кода с этим сектором реализована НЕ урезанием
 * линкерного региона FLASH, а отдельной проверкой на этапе сборки
 * (cmake/check_flash_size.cmake, таргет check_flash_size): она читает
 * LOAD-секции готового ELF по их LMA и останавливает сборку, если
 * код+данные выходят за пределы (DZHIGLER_FLASH_TOTAL_BYTES -
 * DZHIGLER_CONFIG_SECTOR_BYTES). Линкер про существование этого
 * сектора не знает вообще и в теории мог бы разместить туда код —
 * от этого защищает именно post-build проверка, а не сам линкер.
 *
 * Если собираешь без CMakeLists.txt из этого репозитория (руками,
 * через отдельные -D-флаги) — см. fallback чуть ниже, он подразумевает,
 * что PICO_FLASH_SIZE_BYTES отражает истинный физический размер чипа
 * (поведение pico-sdk по умолчанию, если никто его не трогал).
 *
 * Запись выполняется через flash_safe_execute, которая останавливает
 * core1 (через lockout) на время erase+program, чтобы избежать чтения
 * XIP-флеша во время записи с другого ядра — на RP2040 это гарантированно
 * приводит к зависанию/краху, если не синхронизировать ядра.
 */

#include "config_store.h"
#include <string.h>
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/flash.h"
#include "pico/stdlib.h"

#define CFG_MAGIC   0x445A4831u /* "DZH1" */
#define CFG_VERSION 1u

/* Размер структуры зафиксирован: он входит в формат записи во flash
 * и в бюджет сектора. Меняется — менять CFG_VERSION. */
_Static_assert(sizeof(dzh_config_t) == 56, "dzh_config_t must be 56 bytes");

#ifndef DZHIGLER_CONFIG_FLASH_OFFSET
/*
 * Fallback для сборки без нашего CMakeLists.txt (руками, через
 * отдельные -D-флаги). Предполагает, что PICO_FLASH_SIZE_BYTES
 * отражает истинный физический размер чипа — поведение pico-sdk
 * по умолчанию, если никто его не трогал.
 *
 * Если собираешь через CMakeLists.txt из этого репозитория как
 * есть — этот блок не задействуется: DZHIGLER_CONFIG_FLASH_OFFSET
 * передаётся явно отдельным -D-флагом (см. CMakeLists.txt).
 */
#define DZHIGLER_CONFIG_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#endif

/* Смещение внутри flash-чипа (не XIP-адрес!), где лежит сектор конфига.
 * Используется и flash_range_erase/flash_range_program (они принимают
 * именно offset от начала flash), и для построения указателя на чтение
 * через XIP_BASE ниже. */
#define CFG_FLASH_OFFSET DZHIGLER_CONFIG_FLASH_OFFSET

/*
 * Указатель для ЧТЕНИЯ конфига через XIP (memory-mapped flash).
 * В отличие от записи, читать через XIP можно в любой момент без
 * flash_safe_execute — это обычное чтение из адресного пространства,
 * а не операция с flash-контроллером.
 */
static const uint8_t *cfg_flash_ptr(void)
{
  return (const uint8_t *)(XIP_BASE + CFG_FLASH_OFFSET);
}

/* CRC32 (полином 0xEDB88320, стандартный для zlib/Ethernet), обычный
 * побитовый вариант без таблицы — размер кода важнее скорости здесь,
 * вызывается только при save/load, не в hot path. */
static uint32_t crc32_calc(const uint8_t *data, size_t len)
{
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (int b = 0; b < 8; b++)
      crc = (crc >> 1) ^ (0xEDB88320u & (~(crc & 1) + 1));
  }
  return ~crc;
}

void config_defaults(dzh_config_t *out)
{
  memset(out, 0, sizeof(*out));
  out->magic   = CFG_MAGIC;
  out->version = CFG_VERSION;

  out->idle_before_ms   = 30000;
  out->act_min_px       = 2;
  out->abort_return_ms  = 80;

  out->jiggle_radius_min  = 2;
  out->jiggle_radius_max  = 5;
  out->jiggle_dur_min_ms  = 120;
  out->jiggle_dur_max_ms  = 280;
  out->jiggle_hold_min_ms = 120;
  out->jiggle_hold_max_ms = 400;
  out->jiggle_gap_min_ms  = 30000;
  out->jiggle_gap_max_ms  = 60000;
}

/*
 * Читает конфиг из flash. Проверяет и magic/version, и CRC — magic
 * отсекает совсем чужой мусор (девственная flash после программирования
 * заполнена 0xFF, magic не совпадёт почти наверняка), CRC отсекает
 * частично повреждённую запись (например, если erase+program было
 * прервано питанием на середине — magic может случайно совпасть
 * с недописанными данными, CRC этот случай ловит).
 *
 * При любой ошибке валидации *out всё равно заполняется дефолтами
 * и функция возвращает false — вызывающий код может как проверять
 * результат, так и просто использовать *out не глядя на возврат.
 */
bool config_load(dzh_config_t *out)
{
  const dzh_config_t *stored = (const dzh_config_t *)cfg_flash_ptr();

  /* Копируем в локальную переменную одним memcpy, а не читаем поля
   * напрямую из XIP по одному — не по соображениям производительности
   * (незначимо для этого объёма), а чтобы работать с гарантированно
   * консистентным снепшотом структуры, а не с сырым мэппингом. */
  dzh_config_t tmp;
  memcpy(&tmp, stored, sizeof(tmp));

  if (tmp.magic != CFG_MAGIC || tmp.version != CFG_VERSION)
  {
    config_defaults(out);
    return false;
  }

  uint32_t expect_crc = tmp.crc32;
  uint32_t calc_crc = crc32_calc((const uint8_t *)&tmp,
                                  sizeof(tmp) - sizeof(tmp.crc32));

  if (expect_crc != calc_crc)
  {
    config_defaults(out);
    return false;
  }

  *out = tmp;
  return true;
}

/*
 * Контекст для flash_safe_execute: буфер размером ровно в один сектор,
 * выровненный по требованиям flash_range_program (страницы по
 * FLASH_PAGE_SIZE внутри сектора). static struct ctx в config_save
 * (см. ниже) — этот буфер не кладётся на стек, потому что
 * FLASH_SECTOR_SIZE (обычно 4096) на стеке core0 может быть избыточным
 * риском переполнения стека в момент, когда флеш и так уже
 * заблокирована lockout'ом на core1.
 */
struct save_ctx
{
  uint8_t page_buf[FLASH_PAGE_SIZE * (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE)];
};

/*
 * Вызывается уже ВНУТРИ flash_safe_execute — на этот момент core1
 * гарантированно остановлен (lockout), безопасно делать erase+program.
 * Никакого кода, читающего flash (включая косвенно, через вызов функций
 * не из RAM), здесь быть не должно — но обе используемые функции из
 * hardware_flash корректно работают в этом контексте по контракту SDK.
 */
static void save_callback(void *param)
{
  struct save_ctx *ctx = (struct save_ctx *)param;

  flash_range_erase(CFG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
  flash_range_program(CFG_FLASH_OFFSET, ctx->page_buf, FLASH_SECTOR_SIZE);
}

/*
 * Сохраняет конфиг во flash. Блокирующая операция (~50-100мс на erase+
 * program сектора), внутри останавливает core1 через lockout — не
 * вызывать из hot path или из колбэков, которые сами должны быть
 * быстрыми (например, из USB-колбэков).
 *
 * static struct save_ctx ctx — не на стеке (см. комментарий к структуре
 * выше), но и не переиспользуется конкурентно: config_save вызывается
 * только с core0, только синхронно из консольной команды "save",
 * никаких параллельных вызовов не бывает.
 */
bool config_save(const dzh_config_t *cfg)
{
  dzh_config_t tmp = *cfg;
  tmp.magic   = CFG_MAGIC;
  tmp.version = CFG_VERSION;
  tmp.crc32   = crc32_calc((const uint8_t *)&tmp, sizeof(tmp) - sizeof(tmp.crc32));

  static struct save_ctx ctx;
  memset(ctx.page_buf, 0xFF, sizeof(ctx.page_buf)); /* 0xFF = "стёртое" значение flash */
  memcpy(ctx.page_buf, &tmp, sizeof(tmp));

  int rc = flash_safe_execute(save_callback, &ctx, 1000);
  return rc == PICO_OK;
}
