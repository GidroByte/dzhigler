# Считает суммарный физический размер прошивки во flash: сумму Size
# секций, которые (а) имеют флаг LOAD (реально содержат байты в
# бинарном образе) и (б) чей LMA попадает в диапазон flash-региона.
#
# Почему нужен флаг LOAD, а не только диапазон адресов: у секции .bss
# нет физического содержимого во flash (она просто зануляется в RAM
# при старте), поэтому у неё нет флага LOAD — но линковщик всё равно
# печатает для неё какое-то значение LMA, которое в этом линкер-скрипте
# оказывается "унаследованным" от предыдущей секции (.data) и
# случайно попадает в диапазон flash. Первая версия проверки без
# фильтра по LOAD посчитала .bss как физически занимающую flash и
# выдала 91112 B вместо реальных 73044 B — расхождение (18068 B)
# в точности равно размеру .bss (0x4694), что и вскрыло проблему.

cmake_policy(SET CMP0007 NEW)

if(NOT DEFINED ELF_FILE OR NOT DEFINED FLASH_LIMIT OR NOT DEFINED XIP_BASE OR NOT DEFINED FLASH_TOTAL)
  message(FATAL_ERROR "check_flash_size.cmake: не все параметры переданы (ELF_FILE/FLASH_LIMIT/XIP_BASE/FLASH_TOTAL)")
endif()

find_program(DZHIGLER_OBJDUMP_TOOL arm-none-eabi-objdump)

# Fix #2: раньше отсутствие инструмента давало WARNING + return() —
# для скрипта, запущенного через "cmake -P" отдельным процессом,
# return() — это штатное завершение с кодом 0. add_custom_target видит
# успешный exit code и считает шаг сборки пройденным, хотя проверка
# фактически не выполнилась. Это опаснее отсутствия проверки: создаёт
# ложную уверенность в CI. Теперь — жёсткий отказ.
if(NOT DZHIGLER_OBJDUMP_TOOL)
  message(FATAL_ERROR
    "arm-none-eabi-objdump не найден в PATH — проверка размера flash "
    "не может быть выполнена. Установите arm-none-eabi-binutils или "
    "добавьте его в PATH.")
endif()

execute_process(
  COMMAND ${DZHIGLER_OBJDUMP_TOOL} -h ${ELF_FILE}
  OUTPUT_VARIABLE OBJDUMP_OUTPUT
  ERROR_VARIABLE OBJDUMP_ERROR
  RESULT_VARIABLE OBJDUMP_RESULT
)

# Fix #2 (продолжение): та же логика — WARNING+return() маскировал
# сбой инструмента как успех. ERROR_VARIABLE добавлен, чтобы при
# падении в лог попадал реальный stderr объдампа, а не только код
# возврата — раньше stderr никуда не захватывался и терялся молча.
if(NOT OBJDUMP_RESULT EQUAL 0)
  message(FATAL_ERROR
    "arm-none-eabi-objdump завершился с ошибкой (код ${OBJDUMP_RESULT}): "
    "${OBJDUMP_ERROR}")
endif()

math(EXPR XIP_BASE_DEC "${XIP_BASE}")
math(EXPR XIP_END_DEC "${XIP_BASE_DEC} + ${FLASH_TOTAL}")

string(REPLACE "\n" ";" OBJDUMP_LINES "${OBJDUMP_OUTPUT}")
list(LENGTH OBJDUMP_LINES LINE_COUNT)
math(EXPR LAST_IDX "${LINE_COUNT} - 1")

# Fix #3: раньше здесь была накопительная сумма Size подходящих секций.
# Между секциями линковщик вставляет выравнивающий паддинг, который
# физически присутствует в образе, но не входит ни в один Size —
# сумма систематически недооценивала реальную занятую границу flash.
# Теперь считаем FLASH_END_DEC = max(LMA + Size) по всем подходящим
# секциям — это истинный "самый дальний занятый байт", независимо от
# того, сколько пустот между секциями.
set(FLASH_END_DEC ${XIP_BASE_DEC})

# Fix #1: раньше при несовпадении regex со всеми строками (например,
# другая версия binutils изменила формат "objdump -h") FLASH_USED
# оставался 0, сравнение "0 > FLASH_LIMIT" было ложным, и проверка
# тихо проходила, ничего не проверив. Считаем количество совпавших
# секций отдельно от фильтра LOAD/диапазон — если совпадений НОЛЬ,
# значит сам парсинг сломан, а не "просто нет подходящих секций".
set(DZHIGLER_MATCHED_SECTIONS 0)

foreach(i RANGE 0 ${LAST_IDX})
  list(GET OBJDUMP_LINES ${i} line)
  # Строка секции: "  N  Name  Size  VMA  LMA  FileOff  Algn"
  string(REGEX MATCH "^[ \t]*[0-9]+[ \t]+[^ \t]+[ \t]+([0-9a-fA-F]+)[ \t]+[0-9a-fA-F]+[ \t]+([0-9a-fA-F]+)[ \t]+[0-9a-fA-F]+[ \t]+" m "${line}")
  if(m)
    math(EXPR DZHIGLER_MATCHED_SECTIONS "${DZHIGLER_MATCHED_SECTIONS} + 1")

    set(sec_size_hex "${CMAKE_MATCH_1}")
    set(sec_lma_hex "${CMAKE_MATCH_2}")

    # Флаги секции — на следующей строке
    math(EXPR next_i "${i} + 1")
    set(flags_line "")
    if(next_i LESS_EQUAL LAST_IDX)
      list(GET OBJDUMP_LINES ${next_i} flags_line)
    endif()

    string(FIND "${flags_line}" "LOAD" has_load)

    if(NOT has_load EQUAL -1)
      math(EXPR sec_size_dec "0x${sec_size_hex}")
      math(EXPR sec_lma_dec "0x${sec_lma_hex}")
      if(sec_lma_dec GREATER_EQUAL XIP_BASE_DEC AND sec_lma_dec LESS XIP_END_DEC AND sec_size_dec GREATER 0)
        math(EXPR sec_end_dec "${sec_lma_dec} + ${sec_size_dec}")
        if(sec_end_dec GREATER FLASH_END_DEC)
          set(FLASH_END_DEC ${sec_end_dec})
        endif()
      endif()
    endif()
  endif()
endforeach()

# Fix #1 (продолжение): если ни одна строка не распознана как секция —
# формат вывода objdump не совпал с ожидаемым, доверять результату
# нельзя. Раньше это молча давало FLASH_USED=0 и PASS.
if(DZHIGLER_MATCHED_SECTIONS EQUAL 0)
  message(FATAL_ERROR
    "check_flash_size.cmake: регулярное выражение не совпало ни с одной "
    "строкой вывода '${DZHIGLER_OBJDUMP_TOOL} -h' — вероятно другая "
    "версия binutils изменила формат вывода. Проверке нельзя доверять, "
    "сборка остановлена намеренно вместо ложного успеха.")
endif()

math(EXPR FLASH_USED "${FLASH_END_DEC} - ${XIP_BASE_DEC}")

message(STATUS "dzhigler: занято во flash ~${FLASH_USED} B (макс. LMA+Size среди LOAD-секций, с учётом паддинга между секциями), бюджет ${FLASH_LIMIT} B")

if(FLASH_USED GREATER FLASH_LIMIT)
  math(EXPR OVER "${FLASH_USED} - ${FLASH_LIMIT}")
  message(FATAL_ERROR
    "Размер прошивки (${FLASH_USED} B) превышает выделенный бюджет "
    "(${FLASH_LIMIT} B) на ${OVER} B. Последний сектор flash "
    "зарезервирован под config_store.c и не должен быть перекрыт "
    "кодом/данными.")
endif()
