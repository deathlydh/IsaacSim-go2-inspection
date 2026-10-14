# Isaac Sim 6.1.0 — заметки по установке

Проект: симуляция инспекционного робота Unitree Go2 (дальше ROS 2, Nav2, CV для чтения приборов).
Дата установки: 2026-10-13 / 2026-10-14.

## Система

| Параметр | Значение |
|---|---|
| ОС | Windows 10 Корпоративная 22H2, сборка 19045.6456 (официально 6.x требует Win11) |
| CPU | Intel Core i5-14400F, 10 ядер / 16 потоков |
| RAM | 32 ГБ DDR5 |
| GPU | NVIDIA GeForce RTX 4060 Ti, 8 ГБ VRAM |
| Драйвер | 595.79 → обновлён до **617.42** (требуется 595.97+) |
| Диски | C: 1 ТБ (145 ГБ свободно), D: Samsung 870 QVO 2 ТБ (~1.8 ТБ свободно) |

## Установка

- Архив: `isaac-sim-standalone-6.1.0-windows-x86_64.zip` (9.66 ГБ) с
  https://downloads.isaacsim.nvidia.com/ — MD5 `a07968e980072c9ca27b2166443e2d89` совпал.
- Путь установки: **`D:\isaacsim`** (17.6 ГБ). Версия: `6.1.0-rc.26+release.49347.2d230af4`.
- Архив оставлен в `D:\isaacsim-download\` (можно удалить).
- Asset packs не скачивались (ассеты грузятся из облака NVIDIA).

## Compatibility check

| Пункт | Результат |
|---|---|
| Driver 617.42 | зелёный (минимум чекера 537.58) |
| GPU RTX 4060 Ti | зелёный |
| VRAM 8.59 ГБ | **красный** (минимум 10 ГБ) — не блокирует запуск |
| CPU / ядра | зелёный / excellent |
| RAM 34 ГБ | оранжевый (enough, more recommended) |
| Storage | excellent |
| ОС Windows 10 22H2 | **зелёный** — чекер 6.1 считает Win10 поддерживаемой |
| Итог | `FAILED` только из-за VRAM |
| **Test Kit** | **прошёл** (app ready ~80 с при первом запуске, 100 кадров, 0 ошибок) |

## Тест: Unitree Go2 (Robotics Examples → POLICY → Quadruped)

- Запускается и управляется стрелками.
- **FPS: 20–30.**
- **VRAM: 3.5 / 8 ГБ** выделенной, 1.2 ГБ общей. GPU 3D ~7%, температура 43 °C.
- **CPU 60%+** — симуляция упирается в процессор (физика PhysX + инференс политики + Python-цикл), а не в видеокарту.
  Запас по VRAM ~4 ГБ — на камеры/лидар для ROS 2 должно хватить при умеренных разрешениях.
- Замер через python_server: кадр 35 мс, из них физика ~19 мс (4 шага × ~4.8 мс, GPU dynamics).
  RTX Minimal FPS почти не меняет — рендер не узкое место.
- **Физика на CPU** (`PhysxSceneAPI.enableGPUDynamics=False`, broadphase `MBP`): **~39–40 FPS**, кадр 25.5 мс,
  физика ~12 мс. Для одного робота CPU-физика быстрее GPU. Настройка не сохраняется при повторном Load примера.
- Потолок 50 FPS задан примером (`rateLimitFrequency=50`, physics 200 Гц, 4 шага на кадр).
- Низкая «жучиная» походка — свойство демо-политики Isaac Lab (flat terrain), а не ошибка установки:
  при обучении стойка задана с сильно согнутыми ногами (thigh 0.8/1.0, calf −1.5 рад), мягкий PD
  (stiffness 25, damping 0.5), награды за высоту корпуса нет. На GPU и CPU физике выглядит одинаково.

## Проблемы и решения

1. **Драйвер ниже требуемого** (595.79) — обновлён пользователем до 617.42.
2. **`post_install.bat`: "You do not have sufficient privilege"** — `mklink /D` требует админа.
   Решение: создан junction без прав админа (вариант, указанный в самом скрипте):
   `New-Item -ItemType Junction -Path D:\isaacsim\extension_examples -Target D:\isaacsim\exts\isaacsim.examples.interactive\isaacsim\examples\interactive`
3. **Test Kit: ошибка 55, "dependency solver failure", "extension.toml doesn't exist"**.
   - Первая попытка: сервер реестра расширений NVIDIA был недоступен.
   - Дальше: **настоящая причина** — Isaac Sim запускался из Claude Desktop, а он установлен как MSIX-приложение.
     Дочерние процессы получают виртуализацию `C:\Users\Admin\AppData\Local` → запись уходит в
     `AppData\Local\Packages\Claude_pzs8sxrjxfjjc\LocalCache\Local\ov`, а junction-ссылки, которые Kit создаёт в кэше
     `AppData\Local\ov\data\...`, ядро Windows разрешает по реальному пути, где файлов нет.
   - Решение: **запускать Isaac Sim через Проводник (двойной клик), а не из Claude.** Битый кэш удалён.
   - ⚠️ Это касается и `isaac-sim.bat` — иначе кэш шейдеров/расширений сломается так же.

## Управление из Claude (на следующие шаги)

- В дистрибутиве есть официальный навык `D:\isaacsim\skills\isaac-sim-remote` и расширение
  `isaacsim.code_editor.python_server` (TCP-сервер, выполняет Python внутри Isaac Sim, только `127.0.0.1:8226`).
- Ярлык запуска с включённым сервером: **`D:\isaacsim\isaac-sim.claude.bat`** (запускать через Проводник).
- Claude подключается к уже запущенному Isaac Sim через скрипты `skills\isaac-sim-remote\scripts\*.py`
  (health_check, stage_info, simulation_control, viewport_screenshot и т.д.).
- Безопасность: сервер без токена выполняет любой Python от имени пользователя, но слушает только localhost.
  Не пробрасывать порт 8226 наружу.
- `omni.cip.mega.mcp` (порт 8227) — внутренний MCP для конвейера публикации контента, для проекта не нужен.

## Полезное

- Логи Kit: `C:\Users\Admin\.nvidia-omniverse\logs\Kit\<app>\<версия>\kit_*.log`
- Первый запуск `isaac-sim.bat` — 5–10+ минут (компиляция шейдеров), дальше быстрее.
- Очистка кэшей: `D:\isaacsim\clear_caches.bat`.
