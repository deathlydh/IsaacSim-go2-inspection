# Go2 Inspection

Симуляция инспекционного робота Unitree Go2 в NVIDIA Isaac Sim 6.1: сцена объекта, ROS 2, Nav2,
компьютерное зрение для чтения приборов.

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
