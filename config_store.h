#ifndef CONFIG_STORE_H_
#define CONFIG_STORE_H_

#include <stdint.h>
#include <stdbool.h>

/*
 * Настраиваемые параметры джиглера, хранятся во flash последним сектором.
 * Все времена в миллисекундах, радиус в пикселях.
 */
typedef struct
{
  uint32_t magic;         /* сигнатура для проверки валидности записи */
  uint32_t version;       /* версия структуры, для будущей миграции */

  uint32_t idle_before_ms;      /* сколько ждать после последнего сигнала от мыши/клавы */
  uint32_t act_min_px;          /* порог движения, которое считается "активностью" */
  uint32_t abort_return_ms;     /* скорость возврата если джигл прервали */

  uint32_t jiggle_radius_min;
  uint32_t jiggle_radius_max;
  uint32_t jiggle_dur_min_ms;
  uint32_t jiggle_dur_max_ms;
  uint32_t jiggle_hold_min_ms;
  uint32_t jiggle_hold_max_ms;
  uint32_t jiggle_gap_min_ms;   /* как часто дёргать (минимум между джигл-циклами) */
  uint32_t jiggle_gap_max_ms;

  uint32_t crc32;         /* CRC всей структуры кроме этого поля, для целостности */
} dzh_config_t;

/* Загружает конфиг из flash. Если не найден/повреждён — заполняет defaults
 * и возвращает false (но *out всё равно валиден и пригоден к использованию). */
bool config_load(dzh_config_t *out);

/* Сохраняет конфиг во flash. Блокирующая операция ~50-100мс,
 * останавливает оба ядра (flash_safe_execute). Вызывать не в hot path. */
bool config_save(const dzh_config_t *cfg);

/* Заполняет структуру дефолтными значениями (текущие константы прошивки). */
void config_defaults(dzh_config_t *out);

#endif /* CONFIG_STORE_H_ */
