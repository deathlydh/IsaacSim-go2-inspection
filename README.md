# Go2 Inspection

Симуляция инспекционного робота Unitree Go2 в NVIDIA Isaac Sim 6.1: контролируемый
обход и чтение стрелочных приборов. ROS 2/Nav2 и реальный робот — будущие этапы.


## Последний результат: R0 + A1/A3/A4 (6 ноября 2026)

[CV HTML: метрики, разметка, галерея ошибок](reports/cv_eval_20261106T115635Z_a1b1fc71/evaluation.html) ·
[200 кадров и manifest](data/gauges/20261106T115331Z_merged_34d957a3/manifest.jsonl) ·
[состояние проекта](docs/PROGRESS_REPORT.md).

На test (100 кадров, 95 видимых): **known ROI 98.95% чтений, nMAE 0.319%**;
**полный поиск 67.37%, nMAE 3.941%, 7 уверенных ошибок/100**. ROI взят из разметки
только для оценки; это не готовая автономная детекция. Полный поиск пока ненадёжен.
Модель после раскрытия test не настраивалась. Все отказы сохранены.

```powershell
cd D:\go2-inspection
# Независимое CV: JSON и размеченный PNG в новом reports/cv_read_...
python -m cv.read_image shots/camera_Gauge_01.png --site sites/factory_hall/site.json --gauge Gauge_01
# Проверка/оценка сохранённого набора, без Isaac
python tools/validate_dataset.py data/gauges/20261106T115331Z_merged_34d957a3
python -m cv.eval data/gauges/20261106T115331Z_merged_34d957a3 --workers 4
# Повторный подсчёт из сохранённых предсказаний (тоже новый каталог)
python -m cv.eval data/gauges/20261106T115331Z_merged_34d957a3 --predictions reports/cv_eval_20261106T115635Z_a1b1fc71/predictions.jsonl
# Тесты и другой профиль (офлайн: 7 целей, проход 90°, без движения)
python -m unittest discover -s tests -v
python tools/check_site.py --site sites/rotated_demo/site.json --start 10 19 0
# Живой маршрут, только в уже запущенном пользователем Isaac
python tools/run_route_cli.py --site sites/factory_hall/site.json
# Новый сбор: сначала 10 одного прибора, затем 5×40, результаты всегда уникальны
python tools/collect_dataset_cli.py --gauges Gauge_01 --per-gauge 10 --seed 1701
python tools/collect_dataset_cli.py --per-gauge 40 --seed 5701
```

Для нового сбора пользователь запускает start_factory.bat; агент Kit не запускает.
ESC или cancel_route.bat отменяет сбор/маршрут. Сборщик меняет значения/свет и
виртуальные extrinsics камеры, восстанавливает их в finally. Робот доезжает до
позиций физически; виртуальная съёмка не является доказательством реального крепления.
Исходный набор пришлось продолжить после таймаута стабилизации: 160+40 кадров,
все строки включены, provenance и исходные каталоги сохранены.

Настройки площадки — sites/<id>/site.json: сцена, карта/frame, станции, приборы,
route и inspection_pose. Относительные пути считаются от config/site-файла.
Общий runner — inspection/mission.py, контроллер — inspection/actions.py;
Isaac и known_aisle — отдельные адаптеры. Подробнее: docs/SIM_TO_REAL_ARCHITECTURE.md.
Rotated demo — только офлайн, отдельная USD-сцена не построена.

## Структура

| Папка | Что там |
|---|---|
| `docs/` | Заметки: установка, решения, результаты замеров (`SETUP_NOTES.md`) |
| `scenes/` | Наши USD-сцены (сохраняются на диск, не теряются при перезапуске) |
| `tools/isaac/` | Скрипты, которые Claude отправляет в запущенный Isaac Sim |
| `shots/` | Скриншоты вьюпорта |

## Окружение

- Isaac Sim 6.1.0: `D:\isaacsim` (установка NVIDIA, свой код туда не кладём).
- Запуск с управлением из Claude: двойной клик по `D:\isaacsim\isaac-sim.claude.bat` **в Проводнике**.
- Сервер управления: `127.0.0.1:8226` (только localhost).

## Запуск Go2 в нашей сцене

**Цех с манометрами, камерой и походкой robot_lab:** двойной клик `D:\go2-inspection\start_factory.bat`.
Что загружать, задаётся строкой параметров в конце этого файла (открыть в Блокноте):
`scene_path=... policy=robot_lab|nvidia_rough|bundled camera=True|False camera_window=True|False speed=0.5`.

**Простая сцена-сетка:** двойной клик `D:\go2-inspection\start_go2.bat` в Проводнике
(те же параметры можно дописать: `start_go2.bat policy=robot_lab`).
Isaac Sim откроется, через несколько секунд после появления интерфейса сам загрузит сцену и робота.
Кликнуть во вьюпорт (окно с 3D-сценой). Стрелки — вперёд/назад/вбок, N/M — поворот.

**Вместе с Claude:** двойной клик `D:\isaacsim\isaac-sim.claude.bat`, затем Claude отправляет
`tools\isaac\start_go2.py` через python_server. (`start_go2.bat` тоже подходит — сервер в нём включён.)

**Перезапустить робота без перезапуска Isaac Sim:** Window → Script Editor → File → Open
`tools\isaac\run_in_kit.py` → Run (Ctrl+Enter).

**Только посмотреть/поправить сцену:** File → Open → `scenes\inspection.usd` (робота там нет — он
добавляется скриптом). Изменения — File → Save.

| Скрипт | Что делает |
|---|---|
| `build_scene.py` | Пересоздаёт `scenes/inspection.usd` (пол, свет, физика CPU 200 Гц) |
| `start_go2.py` | Открывает сцену, добавляет Go2 + нейросеть походки + клавиатуру, Play |
| `measure_fps.py` | Замер FPS и времени физики (6 с) |
| `check_scene.py` | Проверка содержимого файла сцены без его открытия |
| `build_factory.py` | Пересоздаёт цех `scenes/factory_hall.usda` + манометры + `factory_hall_meta.json` |
| `make_gauge.py` | Процедурный манометр: циферблат PNG, стрелка, `set_gauge_value()`, эталон для CV |
| `go2_camera.py` | Передняя камера Go2 (720p, 10 Гц) и окно 640×360 |
| `policies/` | Альтернативные походки: `robot_lab_go2` (по умолчанию), `nvidia_rough_go2` |
| `goto_gauge.py` | Подвести робота к точке осмотра манометра и сохранить кадр камеры |
| `probe_drift.py`, `measure_go2_gait.py` | Замеры дрейфа и походки (результаты: `docs/gait_ab/RESULTS.md`) |
| `session_status.py` | Состояние живой сессии робота |

Перед File → New / Open в интерфейсе сначала **Stop** — иначе Isaac Sim 6.1 может упасть.

## Проверенный обход пяти приборов (5 ноября 2026)

### Быстро посмотреть проход

1. Двойной клик `start_factory.bat` и дождаться загрузки робота.
2. Двойной клик **`run_inspection_route.bat`** — запускает обход пяти приборов
   в уже открытом Isaac, включает Play при необходимости и показывает прогресс.
3. Отмена: **Esc** во вьюпорте или **`cancel_route.bat`**. После завершения в консоли
   указан новый `reports/<run_id>/route.html`.

`start_factory.bat` сам маршрут не запускает. Во время обхода не нажимать Stop/Play
для управления миссией: использовать отмену. Для проверки одного прибора:
`python tools/run_route_cli.py --limit 1`.

Старт из остановленного состояния может восстановить исходную позу физической
сессии; этот reset выполняется ДО маршрута. Во время маршрута телепортаций нет.

Выполнены A0, A2 и N0 из `docs/TODO.md`: 5/5 точек, 126.36 с, пять новых
кадров. Максимальная ошибка позиции 10.01 см, курса 2.44° (допуски 15 см / 3°).
Управление использует истинную позу симулятора и известную статическую сцену.
Маршрут задаёт скорости существующей robot_lab; новую сеть не обучали.

[Итоговый HTML](reports/route_summary_20261106T061601Z_a7449ac8/evaluation.html),
[исходный журнал](reports/20261106T061252Z_91a1e749/route.json),
[состояние и ограничения](docs/PROGRESS_REPORT.md).
HTML открывается как локальный файл; исходные изображения лежат рядом в reports,
поэтому при переносе сохраняйте оба каталога и относительные пути.

Пользователь запускает `start_factory.bat` двойным кликом. Пока маршрут активен,
стрелки/N/M игнорируются; **ESC** во вьюпорте отменяет маршрут. По завершении или
ошибке команда обнуляется и управление возвращается в manual. Вторая миссия
одновременно не допускается. Робот может начинать с текущей проверенной позы;
неизвестное/занятое начало даст отказ. Телепортации для маршрута нет.

Команды PowerShell из `D:\go2-inspection`, без установки пакетов в Kit:

```powershell
# Только проверка траектории/коллайдеров, без движения
python -I D:/isaacsim/skills/isaac-sim-remote/scripts/isaacsim_send.py --file tools/isaac/run_route.py --arg dry_run=True --arg limit=5 --timeout 30

# Один прибор; затем limit=2 и limit=5 для полного обхода
python -I D:/isaacsim/skills/isaac-sim-remote/scripts/isaacsim_send.py --file tools/isaac/goto_gauge.py --timeout 300
python -I D:/isaacsim/skills/isaac-sim-remote/scripts/isaacsim_send.py --file tools/isaac/run_route.py --arg limit=5 --timeout 1200

# Для длинного запуска — немедленный ACK, без ожидания одного TCP-соединения
python -I D:/isaacsim/skills/isaac-sim-remote/scripts/isaacsim_send.py --file tools/isaac/run_route.py --arg limit=5 --fire-and-forget --execution-timeout 1200 --timeout 30

# Отмена отдельным вызовом; сцену и timeline не меняет
python -I D:/isaacsim/skills/isaac-sim-remote/scripts/isaacsim_send.py --file tools/isaac/cancel_route.py --timeout 30
python -I D:/isaacsim/skills/isaac-sim-remote/scripts/isaacsim_send.py --file tools/isaac/session_status.py --timeout 30

# Офлайн-проверки и повторная сборка HTML из сохранённого лога
python -m unittest discover -s tests -v
python tools/route_report.py --route reports/20261106T061252Z_91a1e749/route.json
```

У длинного TCP-вызова возможен WinError 121 после выполнения в Kit. Это не
подтверждение остановки: проверьте `reports/<новый run_id>/route.json` и состояние
сессии; при необходимости используйте cancel_route. Каждый запуск создаёт новый
каталог. Пороги/скорость/таймауты — `config/inspection_route.json`, максимум 0.5 м/с.

Офлайн-окружение: Python 3.12.8, OpenCV 4.10.0, NumPy 2.5.2, Pillow 12.2.0;
зависимости отдельно в `requirements-cv.txt`, контракт в `docs/CV_CONTRACT.md`.
A1/A3/A4 выполнены; свежие CV-метрики и команды приведены в начале README. Следующий этап A5.
