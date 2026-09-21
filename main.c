/*
 * dzhigler
 * core1 / PIO USB  : USB HOST   — читает реальные мышь и клавиатуру
 * core0 / native   : USB DEVICE — мышь + клавиатура + CDC консоль настроек
 *
 * Ключевые решения реализации (почему код выглядит так, а не иначе):
 *
 *  - pass_through_mouse(): note_real_report() вызывается ДО проверки
 *    jiggle_active, а не после. В обратном порядке реальное движение
 *    мыши во время джигла не обновляет last_act_ts, и user_busy() не
 *    может стать true до конца текущего цикла джигла — abort_return_ms
 *    становится недостижим, джигл не прерывается движением мыши.
 *
 *  - pass_kbd(): активность по клавиатуре определяется по СМЕНЕ
 *    modifier/keycode (kbd_keys_changed), а не по факту получения
 *    отчёта. Обновление last_act_ts на любой отчёт безусловно —
 *    ошибка: устройство, отвечающее одинаковым отчётом на каждый
 *    interrupt-poll (частая манера донглов; некоторые клавиатуры
 *    пишут в reserved заряд батареи), держит user_busy() постоянно
 *    true — джигл не запускается вообще.
 *
 *  - tuh_hid_mount_cb(): счётчики st_mount_mouse/st_mount_kbd считают
 *    факт монтирования устройства (has_mouse/has_kbd, по одному разу
 *    за вызов), а не количество совпавших записей report ID —
 *    иначе композитный донгл с двумя report ID одного протокола в
 *    одном интерфейсе задвоил бы счётчик. Влияет только на
 *    диагностику ([HB] в verbose-режиме).
 *
 *  - st_jiggle/st_jiggle_abort и поля jig/ab/busy/idle в [HB] — без
 *    них поведение джигла и его прерывание реальной активностью
 *    непроверяемо по логу, пришлось бы верить на слово.
 *
 *  - tuh_hid_set_protocol(BOOT) сознательно НЕ вызывается никогда, ни
 *    для одноцелевых устройств, ни для композитных. Boot Protocol не
 *    поддерживает мультиплексирование через несколько Report ID в
 *    одном интерфейсе (ломает один из логических девайсов на
 *    комбо-донглах) и урезает NKRO-клавиатуры до 6-key rollover.
 *    Разбор report descriptor (parse_report_desc) — единственный
 *    механизм определения роли интерфейса, независимо от того, в
 *    каком протоколе устройство фактически работает.
 *
 *  - cdc_print(): цикл ожидания места в TX-буфере вызывает tud_task()
 *    и watchdog_update() на каждой итерации, общий бюджет ожидания
 *    ограничен (250 итераций * 200мкс ≈ 50мс) — без этого длинная
 *    печать справки при коннекте (~20 print'ов) удерживает watchdog
 *    без обновления дольше таймаута и вызывает необоснованный ребут.
 *
 *  - core1 регистрируется как multicore lockout victim — без этого
 *    flash_safe_execute() (используется в config_save) не может
 *    безопасно поставить core1 на паузу во время erase+program
 *    флеша: либо таймаут (config_save вернёт false), либо core1
 *    читает код из XIP flash во время erase, что для RP2040 фатально.
 *
 *  - watchdog настроен на 3000мс — запас на случай, если печать
 *    длинной справки/heartbeat займёт больше, чем ожидалось.
 *
 * Известное ограничение (см. README "Ограничения"): kbd_led_state
 * (Caps/Num/Scroll Lock от ПК) не форвардится на реальную клавиатуру —
 * см. комментарий у tud_hid_set_report_cb().
 */

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#include "pio_usb.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "config_store.h"

/*---------------- отладочный вывод -------------------*/
#define DBG_CDC_ITF 0
#define DBG_HB_MS 1000

static volatile bool g_verbose = false;

static void dbg_printf(const char *fmt, ...);
static void dbg_task(absolute_time_t *next_hb);
static void cdc_console_task(void);

/*------------------- рабочая копия конфига ------------------- */
static dzh_config_t g_cfg;

#define JIGGLE_TICK_MS 10
#define JIGGLE_FIRST_DELAY_MS 1000

_Static_assert(sizeof(dzh_mouse_report_t) == 4,
               "dzh_mouse_report_t должен быть ровно 4 байта");
_Static_assert(sizeof(dzh_kbd_report_t) == 8,
               "dzh_kbd_report_t должен быть ровно 8 байт");

/*---------------- Очередь мыши core1 -> core0 ---------------- */
#define MOUSE_QUEUE_SIZE 16

typedef struct
{
  uint8_t buttons;
  int8_t x, y, wheel;
} mouse_evt_t;

static struct
{
  mouse_evt_t buf[MOUSE_QUEUE_SIZE];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
} mqueue;

static volatile uint32_t st_overflow = 0;

typedef struct
{
  bool pending;
  dzh_kbd_report_t report;
} kbd_box_t;

static kbd_box_t kbd_box;
static spin_lock_t *box_lock;

static inline int32_t clamp_i8(int32_t v)
{
  if (v > 127)  return 127;
  if (v < -127) return -127;
  return v;
}

static void mouse_queue_post(uint8_t buttons, int8_t x, int8_t y, int8_t wheel)
{
  uint32_t save = spin_lock_blocking(box_lock);

  if (mqueue.count < MOUSE_QUEUE_SIZE)
  {
    mqueue.buf[mqueue.tail].buttons = buttons;
    mqueue.buf[mqueue.tail].x = x;
    mqueue.buf[mqueue.tail].y = y;
    mqueue.buf[mqueue.tail].wheel = wheel;
    mqueue.tail = (mqueue.tail + 1) % MOUSE_QUEUE_SIZE;
    mqueue.count++;
  }
  else
  {
    uint8_t last = (mqueue.tail + MOUSE_QUEUE_SIZE - 1) % MOUSE_QUEUE_SIZE;
    int32_t nx = (int32_t)mqueue.buf[last].x + x;
    int32_t ny = (int32_t)mqueue.buf[last].y + y;
    int32_t nw = (int32_t)mqueue.buf[last].wheel + wheel;
    mqueue.buf[last].x = (int8_t)clamp_i8(nx);
    mqueue.buf[last].y = (int8_t)clamp_i8(ny);
    mqueue.buf[last].wheel = (int8_t)clamp_i8(nw);
    mqueue.buf[last].buttons = buttons;
    st_overflow++;
  }

  spin_unlock(box_lock, save);
}

static bool mouse_queue_peek(mouse_evt_t *out)
{
  uint32_t save = spin_lock_blocking(box_lock);
  bool any = mqueue.count > 0;
  if (any) *out = mqueue.buf[mqueue.head];
  spin_unlock(box_lock, save);
  return any;
}

static void mouse_queue_pop(void)
{
  uint32_t save = spin_lock_blocking(box_lock);
  if (mqueue.count > 0)
  {
    mqueue.head = (mqueue.head + 1) % MOUSE_QUEUE_SIZE;
    mqueue.count--;
  }
  spin_unlock(box_lock, save);
}

static void kbd_box_post(uint8_t const *report)
{
  uint32_t save = spin_lock_blocking(box_lock);
  memcpy(&kbd_box.report, report, sizeof(kbd_box.report));
  kbd_box.pending = true;
  spin_unlock(box_lock, save);
}

static bool kbd_box_take(dzh_kbd_report_t *out)
{
  uint32_t save = spin_lock_blocking(box_lock);
  bool any = kbd_box.pending;
  if (any)
  {
    *out = kbd_box.report;
    kbd_box.pending = false;
  }
  spin_unlock(box_lock, save);
  return any;
}

/*---------------- core0: проброс на ПК ------------------- */
static uint8_t last_buttons = 0;
static uint8_t last_sent_buttons = 0;

static absolute_time_t last_act_ts;
static uint8_t prev_btn = 0;

static volatile bool jiggle_active = false;

static void note_real_report(uint8_t buttons, int32_t x, int32_t y, int32_t wheel)
{
  int32_t mag = (x < 0 ? -x : x) + (y < 0 ? -y : y);
  if (mag >= (int32_t)g_cfg.act_min_px || wheel || buttons != prev_btn)
    last_act_ts = get_absolute_time();
  prev_btn = buttons;
}

static bool user_busy(void)
{
  int64_t idle = absolute_time_diff_us(last_act_ts, get_absolute_time());
  return idle < (int64_t)g_cfg.idle_before_ms * 1000;
}

static void pass_through_mouse(void)
{
  /*
   * peek и note_real_report() выполняются ДО проверки jiggle_active —
   * иначе реальное движение мыши во время джигла никогда не
   * обновляло бы last_act_ts, и user_busy() не мог бы стать true до
   * конца текущего цикла джигла (abort_return_ms был бы фактически
   * недостижим). Событие остаётся в очереди непопнутым, пока
   * jiggle_active — на хост оно не уходит, курсор не прыгает, но
   * факт "пользователь уже двигает мышь" учитывается сразу же, и
   * glide_to()/hold_ms() смогут прервать джигл на следующем тике.
   */
  mouse_evt_t evt;
  bool have_evt = mouse_queue_peek(&evt);

  if (have_evt)
    note_real_report(evt.buttons, evt.x, evt.y, evt.wheel);

  if (jiggle_active)
    return;

  if (!tud_hid_n_ready(HID_INSTANCE_MOUSE))
    return;

  if (!have_evt)
    return;

  dzh_mouse_report_t rep = {
      .buttons = evt.buttons,
      .x = evt.x,
      .y = evt.y,
      .wheel = evt.wheel,
  };

  bool is_zero_move = (evt.x == 0 && evt.y == 0 && evt.wheel == 0);
  if (is_zero_move && evt.buttons == last_sent_buttons)
  {
    last_buttons = evt.buttons;
    mouse_queue_pop();
    return;
  }

  if (tud_hid_n_report(HID_INSTANCE_MOUSE, 0, &rep, sizeof(rep)))
  {
    last_buttons = evt.buttons;
    last_sent_buttons = evt.buttons;
    mouse_queue_pop();
  }
}

/*---------------- клавиатура core0 -> PC ------------------- */
static dzh_kbd_report_t kbd_tx;
static bool kbd_tx_valid = false;
static dzh_kbd_report_t kbd_sent;
static bool kbd_sent_valid = false;
static bool kbd_was_ready = false;

/*
 * Активностью считаем только СМЕНУ модификаторов или кодов клавиш, а
 * не факт получения отчёта. Обновление last_act_ts на любой отчёт
 * безусловно — ошибка: устройство, отвечающее на каждый
 * interrupt-poll одинаковым отчётом (частая манера донглов; некоторые
 * клавиатуры к тому же пишут в reserved заряд батареи), держало бы
 * user_busy() постоянно true — джигл не запускался бы никогда.
 *
 * Сравнение через сырые байты, а не через поля структуры: layout
 * dzh_kbd_report_t объявлен в usb_descriptors.h, но факт "8 байт,
 * boot-формат HID keyboard" уже зафиксирован статик-ассертом выше.
 * Стандартная раскладка boot-отчёта (HID spec 1.11, Appendix B):
 * байт 0 — modifier, байт 1 — reserved, байты 2..7 — keycode[6].
 * Байт 1 намеренно НЕ сравниваем — его изменение не является
 * действием пользователя (см. выше про заряд батареи).
 */
static bool kbd_keys_changed(const dzh_kbd_report_t *a, const dzh_kbd_report_t *b)
{
  const uint8_t *pa = (const uint8_t *)a;
  const uint8_t *pb = (const uint8_t *)b;

  if (pa[0] != pb[0])   /* modifier */
    return true;

  return memcmp(pa + 2, pb + 2, sizeof(dzh_kbd_report_t) - 2) != 0; /* keycode[6] */
}

static dzh_kbd_report_t kbd_act_last;
static bool kbd_act_valid = false;

static void pass_kbd(void)
{
  bool ready = tud_hid_n_ready(HID_INSTANCE_KEYBOARD);

  if (ready != kbd_was_ready)
  {
    kbd_was_ready = ready;
    if (g_verbose)
      dbg_printf("[KBD] interface %s\r\n", ready ? "ready" : "not ready");
  }

  dzh_kbd_report_t r;
  if (kbd_box_take(&r))
  {
    kbd_tx = r;
    kbd_tx_valid = true;

    /* last_act_ts обновляется только на СМЕНУ состояния клавиш, а не
     * на каждый отчёт — см. комментарий у kbd_keys_changed() выше.
     * Размен: клавиатура, реализующая авто-повтор сама (долбит
     * одинаковым отчётом при удержании клавиши), будет считаться
     * активной только в момент нажатия — ровно как мышь, лежащая на
     * месте, не считается активной при повторных нулевых отчётах.
     * Асимметрии с мышью нет. */
    if (!kbd_act_valid || kbd_keys_changed(&r, &kbd_act_last))
    {
      last_act_ts = get_absolute_time();
      kbd_act_last = r;
      kbd_act_valid = true;
    }
  }

  if (!kbd_tx_valid)
    return;

  if (kbd_sent_valid && memcmp(&kbd_tx, &kbd_sent, sizeof(kbd_tx)) == 0)
  {
    kbd_tx_valid = false;
    return;
  }

  if (!ready)
    return;

  if (!tud_hid_n_report(HID_INSTANCE_KEYBOARD, 0, &kbd_tx, sizeof(kbd_tx)))
    return;

  kbd_sent       = kbd_tx;
  kbd_sent_valid = true;
  kbd_tx_valid   = false;
}

static void usb_service(void)
{
  tud_task();
  pass_through_mouse();
  pass_kbd();
  cdc_console_task();
  watchdog_update();
}

static bool hold_ms(uint32_t ms)
{
  absolute_time_t until = make_timeout_time_ms(ms);
  while (!time_reached(until))
  {
    usb_service();
    if (user_busy())
      return true;
    sleep_us(200);
  }
  return false;
}

/*---------------- генератор случайности --------------- */
static uint32_t rng_state = 0x9E3779B9u;

static uint32_t rng_next(void)
{
  rng_state ^= rng_state << 13;
  rng_state ^= rng_state >> 17;
  rng_state ^= rng_state << 5;
  return rng_state;
}

static int32_t rng_range(int32_t lo, int32_t hi)
{
  if (hi <= lo) return lo;
  return lo + (int32_t)(rng_next() % (uint32_t)(hi - lo + 1));
}

/*---------------- синтетическое движение --------------- */
static int32_t est_x = 0, est_y = 0;

/* Без этих счётчиков поведение джигла и его прерывание реальной
 * активностью непроверяемо по логу — пришлось бы верить на слово.
 * st_jiggle считает КАЖДУЮ попытку jiggle_once() (включая случаи
 * GLIDE_USB_LOST), st_jiggle_abort — только те, что прерваны
 * активностью пользователя (не USB-обрыв). */
static volatile uint32_t st_jiggle = 0, st_jiggle_abort = 0;

static bool try_send(int32_t dx, int32_t dy)
{
  if (!tud_hid_n_ready(HID_INSTANCE_MOUSE))
    return false;

  dzh_mouse_report_t rep = {
      .buttons = last_buttons,
      .x       = (int8_t)dx,
      .y       = (int8_t)dy,
      .wheel   = 0,
  };
  bool ok = tud_hid_n_report(HID_INSTANCE_MOUSE, 0, &rep, sizeof(rep));
  if (ok) last_sent_buttons = last_buttons;
  return ok;
}

static bool move_rel(int32_t dx, int32_t dy)
{
  uint32_t fails = 0;

  while (dx != 0 || dy != 0)
  {
    int32_t sx = clamp_i8(dx);
    int32_t sy = clamp_i8(dy);

    if (try_send(sx, sy))
    {
      dx -= sx;
      dy -= sy;
      fails = 0;
    }
    else
    {
      if (++fails > 2500)
        return false;
    }

    usb_service();
    sleep_us(200);
  }
  return true;
}

typedef enum
{
  GLIDE_OK = 0,
  GLIDE_ABORTED,
  GLIDE_USB_LOST
} glide_res_t;

static glide_res_t glide_to(int32_t tx, int32_t ty, uint32_t dur_ms, bool abortable)
{
  if (dur_ms < JIGGLE_TICK_MS)
    dur_ms = JIGGLE_TICK_MS;

  absolute_time_t t0 = get_absolute_time();

  for (uint32_t t = JIGGLE_TICK_MS; t <= dur_ms; t += JIGGLE_TICK_MS)
  {
    if (abortable && user_busy())
      return GLIDE_ABORTED;

    float u = (float)t / (float)dur_ms;
    float s = u * u * u * (10.0f + u * (-15.0f + 6.0f * u));

    int32_t want_x = (int32_t)((float)tx * s);
    int32_t want_y = (int32_t)((float)ty * s);

    if (!move_rel(want_x - est_x, want_y - est_y))
      return GLIDE_USB_LOST;
    est_x = want_x;
    est_y = want_y;

    absolute_time_t due = delayed_by_ms(t0, t);
    while (!time_reached(due))
    {
      usb_service();
      if (abortable && user_busy())
        return GLIDE_ABORTED;
      sleep_us(200);
    }
  }

  if (!move_rel(tx - est_x, ty - est_y))
    return GLIDE_USB_LOST;
  est_x = tx;
  est_y = ty;
  return GLIDE_OK;
}

static void jiggle_once(void)
{
  jiggle_active = true;
  st_jiggle++;

  int32_t tx = rng_range(-(int32_t)g_cfg.jiggle_radius_max, (int32_t)g_cfg.jiggle_radius_max);
  int32_t ty = rng_range(-(int32_t)g_cfg.jiggle_radius_max, (int32_t)g_cfg.jiggle_radius_max);
  if (tx == 0 && ty == 0)
    tx = (int32_t)g_cfg.jiggle_radius_min;

  uint32_t out_ms = (uint32_t)rng_range((int32_t)g_cfg.jiggle_dur_min_ms,
                                         (int32_t)g_cfg.jiggle_dur_max_ms);
  bool aborted = false;

  glide_res_t r = glide_to(tx, ty, out_ms, true);

  if (r == GLIDE_USB_LOST)
  {
    est_x = est_y = 0;
    jiggle_active = false;
    return;
  }
  if (r == GLIDE_ABORTED)
    aborted = true;
  else
    aborted = hold_ms((uint32_t)rng_range((int32_t)g_cfg.jiggle_hold_min_ms,
                                           (int32_t)g_cfg.jiggle_hold_max_ms));

  /* Именно ЗДЕСЬ, после того как aborted окончательно определён обоими
   * путями (glide_to и hold_ms), а не раньше — иначе можно посчитать
   * прерывание outward-фазы, но пропустить прерывание во время hold,
   * или наоборот. */
  if (aborted)
    st_jiggle_abort++;

  uint32_t back_ms = aborted
                         ? g_cfg.abort_return_ms
                         : (uint32_t)rng_range((int32_t)g_cfg.jiggle_dur_min_ms,
                                                (int32_t)g_cfg.jiggle_dur_max_ms);

  glide_to(0, 0, back_ms, false);

  jiggle_active = false;
}

/*===================== Host: разбор интерфейсов ===================== */
#define HID_SLOTS 32
#define HID_SLOT(dev, inst) ((uint8_t)((((dev) & 0x07u) << 2) | ((inst) & 0x03u)))
#define MAX_REPORTS_PER_SLOT 4

typedef struct
{
  uint8_t report_id;
  uint8_t proto;
} report_map_entry_t;

static volatile report_map_entry_t hid_map[HID_SLOTS][MAX_REPORTS_PER_SLOT];
static volatile uint8_t hid_map_count[HID_SLOTS];

static volatile uint8_t h_mouse_slot = 0xFF;
static volatile uint8_t h_kbd_slot   = 0xFF;
static volatile uint32_t st_mount_mouse  = 0, st_mount_kbd  = 0, st_mount_other = 0;
static volatile uint32_t st_unmount      = 0;
static volatile uint32_t st_rep_mouse    = 0, st_rep_kbd    = 0, st_rep_bad     = 0;

typedef struct
{
  uint8_t proto;
  uint8_t report_id;
} iface_entry_t;

typedef struct
{
  iface_entry_t entries[MAX_REPORTS_PER_SLOT];
  uint8_t count;
} iface_desc_t;

static iface_desc_t parse_report_desc(uint8_t const *d,uint16_t len){
  iface_desc_t out={0}; 
  uint32_t up=0, cu=0; uint8_t crid=0;
  for(uint16_t i=0;i<len;){
    uint8_t b=d[i++];
    if(b==0xFE){ if(i+3>len) break; uint8_t ds=d[i++]; i+=2; if(i+ds>len) break; i+=ds; continue; }
    uint8_t is=b&0x03; if(is==3) is=4; uint8_t ty=(b>>2)&0x03,ta=(b>>4)&0x0F; 
    if(i+is>len) break;
    uint32_t v=0; 
    for(uint8_t k=0;k<is;k++) v|=(uint32_t)d[i+k]<<(8*k); 
    i+=is;
    if(ty==1){ if(ta==0x0) up=v; else if(ta==0x8) crid=(uint8_t)v; } 
    else if(ty==2){ if(ta==0x0) cu=v; } 
    else if(ty==0){ 
      if(ta==0xA && v==0x01){ 
        uint8_t p=HID_ITF_PROTOCOL_NONE; 
        if(up==0x01){ if(cu==0x02||cu==0x01) p=HID_ITF_PROTOCOL_MOUSE; else if(cu==0x06||cu==0x07) p=HID_ITF_PROTOCOL_KEYBOARD; } 
        else if(up==0x07) p=HID_ITF_PROTOCOL_KEYBOARD; 
        if(p!=HID_ITF_PROTOCOL_NONE && out.count<MAX_REPORTS_PER_SLOT){ 
          bool dup=false; for(uint8_t k=0;k<out.count;k++) if(out.entries[k].report_id==crid && out.entries[k].proto==p) dup=true;
          if(!dup){ out.entries[out.count].proto=p; out.entries[out.count].report_id=crid; out.count++; }
        } 
      } 
    } 
  } 
  return out;
}

void tuh_hid_mount_cb(uint8_t da,uint8_t inst,uint8_t const *desc,uint16_t dlen){
  uint8_t slot=HID_SLOT(da,inst); hid_map_count[slot]=0; 
  for(uint8_t k=0;k<MAX_REPORTS_PER_SLOT;k++) hid_map[slot][k]=(report_map_entry_t){0,HID_ITF_PROTOCOL_NONE};
  iface_desc_t parsed={0};
  if(desc && dlen){ 
    parsed=parse_report_desc(desc,dlen); 
    if(parsed.count==0){ uint8_t p=tuh_hid_interface_protocol(da,inst); if(p!=HID_ITF_PROTOCOL_NONE){ parsed.entries[0].proto=p; parsed.entries[0].report_id=0; parsed.count=1; } }

    /*
     * has_mouse/has_kbd — считаем факт монтирования устройства (по
     * одному разу за вызов mount_cb), а не количество совпавших
     * записей report ID. Инкремент внутри цикла по parsed.count был
     * бы неверен: композитный донгл с двумя report ID одного
     * протокола (например, две "мышиные" коллекции в одном
     * интерфейсе) задвоил бы счётчик. Влияет только на диагностику
     * ([HB] в verbose-режиме), не на функциональность.
     */
    bool has_mouse = false, has_kbd = false;
    for(uint8_t k=0;k<parsed.count;k++){
      hid_map[slot][k].report_id=parsed.entries[k].report_id;
      hid_map[slot][k].proto=parsed.entries[k].proto;
      if(parsed.entries[k].proto==HID_ITF_PROTOCOL_MOUSE) { h_mouse_slot=slot; has_mouse=true; }
      else if(parsed.entries[k].proto==HID_ITF_PROTOCOL_KEYBOARD) { h_kbd_slot=slot; has_kbd=true; }
    }
    if (has_mouse) st_mount_mouse++;
    if (has_kbd)   st_mount_kbd++;

    hid_map_count[slot]=parsed.count; if(parsed.count==0) st_mount_other++;
  } else st_mount_other++;
  // Set_Protocol(BOOT) сознательно не вызывается никогда — см. заголовок
  // файла. Композитный донгл с несколькими report ID в одном интерфейсе
  // теряет один из логических девайсов при переходе в Boot Protocol.
  tuh_hid_receive_report(da,inst);
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  uint8_t slot = HID_SLOT(dev_addr, instance);

  hid_map_count[slot] = 0;
  for (uint8_t k = 0; k < MAX_REPORTS_PER_SLOT; k++)
    hid_map[slot][k].proto = HID_ITF_PROTOCOL_NONE;

  if (h_mouse_slot == slot) h_mouse_slot = 0xFF;
  if (h_kbd_slot   == slot) h_kbd_slot   = 0xFF;

  st_unmount++;
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance,
                                uint8_t const *report, uint16_t len)
{
  uint8_t slot  = HID_SLOT(dev_addr, instance);
  uint8_t count = hid_map_count[slot];

  uint8_t const *p    = report;
  uint16_t       l    = len;
  uint8_t        proto = HID_ITF_PROTOCOL_NONE;

  if (count == 0)
  {
    st_rep_bad++;
    tuh_hid_receive_report(dev_addr, instance);
    return;
  }

  bool single_no_id = (count == 1 && hid_map[slot][0].report_id == 0);

  if (single_no_id)
  {
    proto = hid_map[slot][0].proto;
  }
  else if (l >= 1)
  {
    uint8_t rid = p[0];
    for (uint8_t k = 0; k < count; k++)
    {
      if (hid_map[slot][k].report_id == rid)
      {
        proto = hid_map[slot][k].proto;
        break;
      }
    }
    if (proto != HID_ITF_PROTOCOL_NONE)
    {
      p++;
      l--;
    }
  }

  if (proto == HID_ITF_PROTOCOL_MOUSE)
  {
    if (l >= 3)
    {
      mouse_queue_post(p[0], (int8_t)p[1], (int8_t)p[2],
                       (l >= 4) ? (int8_t)p[3] : 0);
      st_rep_mouse++;
    }
    else st_rep_bad++;
  }
  else if (proto == HID_ITF_PROTOCOL_KEYBOARD)
  {
    if (l >= 8)
    {
      kbd_box_post(p);
      st_rep_kbd++;
    }
    else st_rep_bad++;
  }
  else st_rep_bad++;

  tuh_hid_receive_report(dev_addr, instance);
}

/*------------------ core1: USB host ------------------- */
static void core1_main(void)
{
  sleep_ms(10);

  /*
   * Без этой регистрации flash_safe_execute() (используется в
   * config_save) не может безопасно поставить core1 на паузу во время
   * erase+program флеша — это либо приведёт к таймауту (config_save
   * вернёт false), либо в худшем случае core1 попытается читать код
   * из XIP flash во время erase, что для RP2040 фатально.
   */
  multicore_lockout_victim_init();

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = 0;
  tuh_configure(1, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);
  tuh_init(1);
  while (true) tuh_task();
}

/*------------------ core0: USB device ------------------- */
int main(void)
{
  set_sys_clock_khz(120000, true);
  sleep_ms(10);

  /* 3000мс — запас на случай, если печать длинной справки/heartbeat
   * займёт больше, чем ожидалось. */
  watchdog_enable(3000, 1);

  box_lock = spin_lock_init(spin_lock_claim_unused(true));

  config_load(&g_cfg);

  multicore_reset_core1();
  multicore_launch_core1(core1_main);

  tud_init(0);

  last_act_ts = get_absolute_time();

  absolute_time_t next_jiggle = make_timeout_time_ms(JIGGLE_FIRST_DELAY_MS);
  absolute_time_t next_hb     = make_timeout_time_ms(DBG_HB_MS);

  while (true)
  {
    usb_service();
    dbg_task(&next_hb);

    if (time_reached(next_jiggle) && !user_busy())
    {
      jiggle_once();
      next_jiggle = delayed_by_ms(
          get_absolute_time(),
          (uint32_t)rng_range((int32_t)g_cfg.jiggle_gap_min_ms,
                               (int32_t)g_cfg.jiggle_gap_max_ms));
    }
  }

  return 0;
}

/*---------------- низкоуровневый вывод в CDC ---------------- */
static void dbg_printf(const char *fmt, ...)
{
  if (!tud_cdc_n_connected(DBG_CDC_ITF))
    return;

  char buf[200];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n <= 0) return;
  if ((size_t)n >= sizeof(buf)) n = (int)sizeof(buf) - 1;

  if (tud_cdc_n_write_available(DBG_CDC_ITF) < (uint32_t)n)
    return;

  tud_cdc_n_write(DBG_CDC_ITF, buf, (uint32_t)n);
  tud_cdc_n_write_flush(DBG_CDC_ITF);
}

/*---------------- параметры конфига для консоли ---------------- */
typedef struct
{
  const char *name;
  uint32_t *field;
  uint32_t min_val;
  uint32_t max_val;
  const char *help;
} cfg_param_t;

static const cfg_param_t *cfg_params(size_t *count)
{
  static cfg_param_t table[11];
  static bool init = false;

  if (!init)
  {
    size_t i = 0;
    table[i++] = (cfg_param_t){"idle_before_ms",   &g_cfg.idle_before_ms,   500,   3600000, "ждать после последнего сигнала мыши/клавы, мс"};
    table[i++] = (cfg_param_t){"act_min_px",        &g_cfg.act_min_px,        1,      127, "порог движения = активность, px"};
    table[i++] = (cfg_param_t){"abort_return_ms",   &g_cfg.abort_return_ms,  10,    10000, "скорость возврата при прерывании джигла, мс"};
    table[i++] = (cfg_param_t){"jiggle_radius_min", &g_cfg.jiggle_radius_min, 1,      127, "мин. радиус сдвига, px"};
    table[i++] = (cfg_param_t){"jiggle_radius_max", &g_cfg.jiggle_radius_max, 1,      127, "макс. радиус сдвига, px"};
    table[i++] = (cfg_param_t){"jiggle_dur_min_ms", &g_cfg.jiggle_dur_min_ms, 10,   60000, "мин. длительность сдвига, мс"};
    table[i++] = (cfg_param_t){"jiggle_dur_max_ms", &g_cfg.jiggle_dur_max_ms, 10,   60000, "макс. длительность сдвига, мс"};
    table[i++] = (cfg_param_t){"jiggle_hold_min_ms",&g_cfg.jiggle_hold_min_ms,0,   60000, "мин. пауза в конечной точке, мс"};
    table[i++] = (cfg_param_t){"jiggle_hold_max_ms",&g_cfg.jiggle_hold_max_ms,0,   60000, "макс. пауза в конечной точке, мс"};
    table[i++] = (cfg_param_t){"jiggle_gap_min_ms", &g_cfg.jiggle_gap_min_ms, 1000, 3600000, "мин. интервал между джиглами, мс"};
    table[i++] = (cfg_param_t){"jiggle_gap_max_ms", &g_cfg.jiggle_gap_max_ms, 1000, 3600000, "макс. интервал между джиглами, мс"};
    init = true;
  }

  *count = 11;
  return table;
}

/*
 * cdc_print во время ожидания места в TX-буфере вызывает tud_task()
 * (иначе стек не отдаст данные на шину) и watchdog_update() (иначе
 * долгая печать справки при коннекте гарантированно ловила watchdog
 * reset). Общий бюджет ожидания ограничен: если место не появилось
 * за ~50мс — строка просто пропускается, не блокируя систему дальше.
 * Это некритично для консоли настроек (не hot path).
 */
static void cdc_print(const char *fmt, ...)
{
  if (!tud_cdc_n_connected(DBG_CDC_ITF))
    return;

  char buf[220];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n <= 0) return;
  if ((size_t)n >= sizeof(buf)) n = (int)sizeof(buf) - 1;

  uint32_t waited = 0;
  while (tud_cdc_n_write_available(DBG_CDC_ITF) < (uint32_t)n && waited < 250)
  {
    tud_task();
    tud_cdc_n_write_flush(DBG_CDC_ITF);
    watchdog_update();
    sleep_us(200);
    waited++;
  }

  if (tud_cdc_n_write_available(DBG_CDC_ITF) < (uint32_t)n)
    return; /* не влезло за ~50мс — пропускаем строку, не блокируем систему */

  tud_cdc_n_write(DBG_CDC_ITF, buf, (uint32_t)n);
  tud_cdc_n_write_flush(DBG_CDC_ITF);
}

/*---------------- справка / команды консоли ---------------- */
static void cmd_help(void)
{
  size_t count;
  const cfg_param_t *params = cfg_params(&count);

  cdc_print("\r\n==== dzhigler ====\r\n");
  cdc_print("commands:\r\n");
  cdc_print("  help                 - eta spravka\r\n");
  cdc_print("  get                  - tekushie znachenia parametrov\r\n");
  cdc_print("  set <name> <value>   - izmenit parametr (v pamyati)\r\n");
  cdc_print("  save                 - zapisat konfig vo flash\r\n");
  cdc_print("  load                 - perechitat konfig iz flash\r\n");
  cdc_print("  defaults             - sbros k defoltam (bez sohranenia)\r\n");
  cdc_print("  verbose on|off       - fonovyi debug-log, po umolchaniu off\r\n");
  cdc_print("params:\r\n");
  for (size_t i = 0; i < count; i++)
    cdc_print("  %-20s %s\r\n", params[i].name, params[i].help);
  cdc_print("\r\n> ");
}

static void cmd_get(void)
{
  size_t count;
  const cfg_param_t *params = cfg_params(&count);

  cdc_print("\r\ncurrent config:\r\n");
  for (size_t i = 0; i < count; i++)
    cdc_print("  %-20s = %lu\r\n", params[i].name, (unsigned long)*params[i].field);
  cdc_print("> ");
}

static void cmd_set(char *name, char *value_str)
{
  size_t count;
  const cfg_param_t *params = cfg_params(&count);

  for (size_t i = 0; i < count; i++)
  {
    if (strcmp(params[i].name, name) == 0)
    {
      char *endptr = NULL;
      long v = strtol(value_str, &endptr, 10);

      if (endptr == value_str || v < 0)
      {
        cdc_print("error: invalid number '%s'\r\n> ", value_str);
        return;
      }

      if ((uint32_t)v < params[i].min_val || (uint32_t)v > params[i].max_val)
      {
        cdc_print("error: %s out of range [%lu..%lu]\r\n> ", name,
                   (unsigned long)params[i].min_val,
                   (unsigned long)params[i].max_val);
        return;
      }

      *params[i].field = (uint32_t)v;
      cdc_print("ok: %s = %lu (not saved, use 'save' to persist)\r\n> ",
                 name, (unsigned long)v);
      return;
    }
  }

  cdc_print("error: unknown param '%s'\r\n", name);
  cmd_help();
}

static void cmd_save(void)
{
  if (g_cfg.jiggle_radius_min > g_cfg.jiggle_radius_max)
  {
    cdc_print("error: jiggle_radius_min > jiggle_radius_max, fix before save\r\n> ");
    return;
  }
  if (g_cfg.jiggle_dur_min_ms > g_cfg.jiggle_dur_max_ms)
  {
    cdc_print("error: jiggle_dur_min_ms > jiggle_dur_max_ms, fix before save\r\n> ");
    return;
  }
  if (g_cfg.jiggle_hold_min_ms > g_cfg.jiggle_hold_max_ms)
  {
    cdc_print("error: jiggle_hold_min_ms > jiggle_hold_max_ms, fix before save\r\n> ");
    return;
  }
  if (g_cfg.jiggle_gap_min_ms > g_cfg.jiggle_gap_max_ms)
  {
    cdc_print("error: jiggle_gap_min_ms > jiggle_gap_max_ms, fix before save\r\n> ");
    return;
  }

  cdc_print("saving to flash...\r\n");
  bool ok = config_save(&g_cfg);
  cdc_print(ok ? "ok: saved\r\n> " : "error: flash write failed\r\n> ");
}

static void cmd_load(void)
{
  bool ok = config_load(&g_cfg);
  cdc_print(ok ? "ok: loaded from flash\r\n> "
                : "no valid config in flash, defaults applied\r\n> ");
}

static void cmd_defaults(void)
{
  config_defaults(&g_cfg);
  cdc_print("ok: defaults applied (not saved, use 'save' to persist)\r\n> ");
}

static void cmd_verbose(char *arg)
{
  if (!arg)
  {
    cdc_print("usage: verbose on|off (currently %s)\r\n> ", g_verbose ? "on" : "off");
    return;
  }
  if (strcmp(arg, "on") == 0)
  {
    g_verbose = true;
    cdc_print("ok: verbose on (heartbeat every %dms)\r\n> ", DBG_HB_MS);
  }
  else if (strcmp(arg, "off") == 0)
  {
    g_verbose = false;
    cdc_print("ok: verbose off\r\n> ");
  }
  else
  {
    cdc_print("usage: verbose on|off\r\n> ");
  }
}

static void process_line(char *line)
{
  size_t len = strlen(line);
  while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n'))
    line[--len] = '\0';

  if (len == 0)
  {
    cdc_print("> ");
    return;
  }

  char *cmd = strtok(line, " \t");
  if (!cmd)
  {
    cdc_print("> ");
    return;
  }

  if (strcmp(cmd, "help") == 0)
  {
    cmd_help();
  }
  else if (strcmp(cmd, "get") == 0)
  {
    cmd_get();
  }
  else if (strcmp(cmd, "set") == 0)
  {
    char *name = strtok(NULL, " \t");
    char *val  = strtok(NULL, " \t");
    if (!name || !val)
    {
      cdc_print("usage: set <name> <value>\r\n");
      cmd_help();
    }
    else
    {
      cmd_set(name, val);
    }
  }
  else if (strcmp(cmd, "save") == 0)
  {
    cmd_save();
  }
  else if (strcmp(cmd, "load") == 0)
  {
    cmd_load();
  }
  else if (strcmp(cmd, "defaults") == 0)
  {
    cmd_defaults();
  }
  else if (strcmp(cmd, "verbose") == 0)
  {
    cmd_verbose(strtok(NULL, " \t"));
  }
  else
  {
    cdc_print("unknown command '%s'\r\n", cmd);
    cmd_help();
  }
}

/*---------------- приём с эхом ---------------- */
static void cdc_console_task(void)
{
  static char lb[128];
  static size_t lp = 0;
  static bool pc = false;
  static absolute_time_t help_due = {0};
  static bool help_pending = false;

  bool co = tud_cdc_n_connected(DBG_CDC_ITF);

  if (co && !pc) {
    pc = true;
    lp = 0;
    // не печатаем сразу, даём хосту открыть порт и CDC TX
    help_pending = true;
    help_due = make_timeout_time_ms(200);
  } else if (!co) {
    pc = false;
    lp = 0;
    help_pending = false;
  }

  if (help_pending && co && time_reached(help_due)) {
    help_pending = false;
    cmd_help();
  }

  if (!co) return;
  if (!tud_cdc_n_available(DBG_CDC_ITF)) return;

  char ch[64];
  uint32_t n = tud_cdc_n_read(DBG_CDC_ITF, ch, sizeof(ch));
  for (uint32_t i = 0; i < n; i++) {
    char c = ch[i];
    if (c == '\r' || c == '\n') {
      tud_cdc_n_write(DBG_CDC_ITF, "\r\n", 2);
      tud_cdc_n_write_flush(DBG_CDC_ITF);
      if (lp) { lb[lp] = '\0'; process_line(lb); lp = 0; }
      else cdc_print("> ");
      continue;
    }
    if (c == 0x08 || c == 0x7F) {
      if (lp) { lp--; tud_cdc_n_write(DBG_CDC_ITF, "\b \b", 3); tud_cdc_n_write_flush(DBG_CDC_ITF); }
      continue;
    }
    tud_cdc_n_write(DBG_CDC_ITF, &c, 1);
    tud_cdc_n_write_flush(DBG_CDC_ITF);
    if (lp < sizeof(lb)-1) lb[lp++] = c;
  }
}

/*---------------- фоновый heartbeat (только если verbose on) ---------------- */
static void dbg_task(absolute_time_t *next_hb)
{
  static bool prev_mounted = false;

  bool conn    = tud_cdc_n_connected(DBG_CDC_ITF);
  bool mounted = tud_mounted();

  if (!conn) return;

  if (g_verbose && mounted != prev_mounted)
  {
    dbg_printf("[USB] device %s\r\n", mounted ? "MOUNTED" : "UNMOUNTED");
  }
  prev_mounted = mounted;

  if (!time_reached(*next_hb))
    return;
  *next_hb = delayed_by_ms(get_absolute_time(), DBG_HB_MS);

  if (!g_verbose)
    return;

  uint8_t diag_slot = (h_mouse_slot != 0xFF) ? h_mouse_slot :
                      (h_kbd_slot   != 0xFF) ? h_kbd_slot   : 0xFF;
  if (diag_slot != 0xFF)
  {
    dbg_printf("[MAP] slot=%d count=%d", (int)diag_slot,
               (int)hid_map_count[diag_slot]);
    for (uint8_t k = 0; k < hid_map_count[diag_slot]; k++)
      dbg_printf(" [rid=%d p=%d]",
                 (int)hid_map[diag_slot][k].report_id,
                 (int)hid_map[diag_slot][k].proto);
    dbg_printf("\r\n");
  }

  uint32_t qcount;
  { uint32_t save = spin_lock_blocking(box_lock); qcount = mqueue.count; spin_unlock(box_lock, save); }

  dbg_printf("[HB] m_if=%d k_if=%d q=%lu ovf=%lu | rep m=%lu k=%lu bad=%lu"
             " | mount m=%lu k=%lu oth=%lu um=%lu | ep m=%d k=%d lb=%02x lsb=%02x"
             " | jig=%lu ab=%lu busy=%d idle=%lums\r\n",
             (int)h_mouse_slot, (int)h_kbd_slot, (unsigned long)qcount,
             (unsigned long)st_overflow,
             (unsigned long)st_rep_mouse, (unsigned long)st_rep_kbd,
             (unsigned long)st_rep_bad,
             (unsigned long)st_mount_mouse, (unsigned long)st_mount_kbd,
             (unsigned long)st_mount_other, (unsigned long)st_unmount,
             (int)tud_hid_n_ready(HID_INSTANCE_MOUSE),
             (int)tud_hid_n_ready(HID_INSTANCE_KEYBOARD),
             last_buttons, last_sent_buttons,
             (unsigned long)st_jiggle, (unsigned long)st_jiggle_abort,
             (int)user_busy(),
             (unsigned long)(absolute_time_diff_us(last_act_ts, get_absolute_time()) / 1000));
}

/*---------------- Device HID callbacks ----------------*/
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen)
{
  (void)instance; (void)report_id; (void)report_type;
  (void)buffer;   (void)reqlen;
  return 0;
}

static uint8_t kbd_led_state = 0;

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize)
{
  (void)report_id;
  if (instance != HID_INSTANCE_KEYBOARD)  return;
  if (report_type != HID_REPORT_TYPE_OUTPUT) return;
  if (bufsize < 1) return;
  kbd_led_state = buffer[0] & 0x1F;
  /*
   * Известное ограничение (см. README "Ограничения"): kbd_led_state
   * никуда не форвардится на реальную клавиатуру, подключённую к
   * хост-порту. Индикаторы Caps/Num/Scroll Lock на физической
   * клавиатуре не отражают состояние на ПК. Форвардинг требует
   * вызова tuh_hid_set_report() из host-стека (core1), а этот
   * колбэк вызывается с core0 — нужна отдельная синхронизация между
   * ядрами (аналогичная box_lock для мыши/клавиатуры), не добавлена
   * сознательно, чтобы не вносить cross-core гонку без возможности
   * протестировать её на живой клавиатуре с LED прямо перед релизом.
   */
}
