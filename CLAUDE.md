# Go2 Inspection — правила для Claude

Пользователь новичок в Isaac Sim: объяснять коротко, по-русски. Перед изменениями системы
(драйверы, переменные окружения, удаление файлов, настройки Claude/MCP) — спрашивать.

## Isaac Sim

- Установка: `D:\isaacsim` (6.1.0). Официальные навыки NVIDIA: `D:\isaacsim\skills\` (читать по необходимости,
  особенно `isaac-sim-remote`).
- **Никогда не запускать Kit/Isaac Sim из Claude.** Claude Desktop — MSIX-приложение: дочерние процессы получают
  виртуализацию `AppData\Local`, и junction-ссылки в кэше Kit ломаются. Пользователь запускает
  `D:\isaacsim\isaac-sim.claude.bat` двойным кликом в Проводнике.
- Управление идёт через python_server на `127.0.0.1:8226`:
  `python -I D:\isaacsim\skills\isaac-sim-remote\scripts\isaacsim_send.py --file <script.py> [--arg k=v]`
  - Код передавать файлом (`--file`): PowerShell 5.1 ломает кавычки в аргументах.
  - Файлы писать без BOM (инструмент Write, не `Set-Content`).
  - Top-level `await` в скриптах работает. Скриншот: `scripts\viewport_screenshot.py --arg "output_path='D:/go2-inspection/shots/x.png'"`.
- **Перед new/open stage — `timeline.stop()` и ~10 кадров ожидания.** Иначе Isaac Sim 6.1 падает
  (access violation): Stop приходит во время закрытия сцены, `virtual_gantry._on_stop` чистит debug_draw.
  Пользователю: перед File → New/Open сначала жать Stop.
- **Камера (go2_camera.py): никогда не ставить рендер/вьюпорт камеры на паузу и не закрывать render
  product, пока к нему подключены аннотаторы** — это сломало CUDA (error 700, нужен перезапуск Kit).
  Сессия закрывает камеру на событии stage CLOSING; при смене сцены сначала `session.close()`.
- Субагентам, работающим параллельно с одним Kit: запрещать смену сцены/таймлайна и эксперименты
  с жизненным циклом рендера.
- Походка по умолчанию — `robot_lab` + удержание курса (`docs/gait_ab/RESULTS.md`). Без удержания
  курса политика крутится ~10°/с при нулевой команде.
- Скрипты длиннее ~60 с: ставить `--timeout` с запасом (симуляция идёт ~30 FPS, медленнее реального времени).
- Наш робот — `/World/Robot` (не `/World/Go2`, чтобы не пересекаться с загруженным примером NVIDIA).
- Для одного Go2 физика на CPU быстрее GPU (~40 vs ~28 FPS): `enableGPUDynamics=False`, broadphase `MBP`.

## Проект

- Свои скрипты — в `tools/isaac/`, сцены — в `scenes/`, заметки — в `docs/`.
- Логи Kit: `C:\Users\Admin\.nvidia-omniverse\logs\Kit\`.
