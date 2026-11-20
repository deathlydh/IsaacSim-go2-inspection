# Go2 · картирование и одна станция с Nav2

Для сохранённой карты откройте `start_factory.bat`, дождитесь Go2 у
стартовой метки и запустите `run_demo.bat nav` из PowerShell или выберите
пункт 1 в меню. Пункты `cancel`, `obstacle` и `mapping` запускают
отдельные проверки. Навигационные режимы стартуют каждый раз из исходной
позы Go2. После завершения запустите Stop → Play для следующего опыта.
При ошибке DDS смотрите `reports/demo/<id>/navigation.log` и повторите
из нового эпизода. Раздельные команды ниже пригодятся для диагностики,
калибровки новой карты и измерений.

Isaac Sim запускается пользователем через `start_factory.bat`. В Ubuntu-22.04
WSL2 используются ROS 2 Humble и существующий relay из
[передачи моста](GO2_ROS2_HANDOFF.md). Домен 42 обслуживает Kit, домен 43
предназначен для воспроизведения. Команды ниже выполняются из
`/mnt/d/go2-inspection` в WSL.

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
dpkg-query -W ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-rclpy ros-humble-rosbag2
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
```

В проверенном Ubuntu-22.04 установлены Navigation2 и Nav2 bringup 1.1.20,
SLAM Toolbox 2.6.10, rclpy 3.3.20, rosbag2 0.15.16, BehaviorTree.CPP 3.8.7.
Запуск использует NavFn, DWB с боковой скоростью, AMCL с OmniMotionModel,
map_server, BT Navigator и lifecycle manager.
Скрипт задаёт UDPv4-профиль Fast DDS для своих процессов WSL. Он применён
после таймаутов action на штатном транспорте в домене 42.
[Параметры транспорта Fast DDS](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/rtps_layer/rtps_layer.html)

## Картирование

1. Откройте цех через `start_factory.bat`. Подключите мост и relay по
   [инструкции](GO2_ROS2_HANDOFF.md). Снимите активный маршрут и оставьте Go2
   в manual. Нажмите Play в Kit и щёлкните основной viewport.
2. В отдельном терминале WSL запустите SLAM. Во втором терминале начните
   отдельную запись ручного прохода.

   ```bash
   bash tools/ros/go2_nav2.sh mapping
   # Во втором терминале с ROS_DOMAIN_ID=42
   ros2 bag record --use-sim-time -o reports/go2_ros2/manual_factory_new \
       /scan /odom /tf /tf_static /clock
   ```

3. Со старта около Isaac (−8, 0), носом вдоль +X, удерживайте ↑ небольшими
   отрезками и двигайтесь посередине длинного прохода. Отпускайте клавиши,
   чтобы остановиться, ←/→ используют боковой шаг. У дальнего конца,
   около X=+7 м, остановитесь на свободном месте, развернитесь N/M и
   вернитесь к западному концу. Для обзора станции подъедьте к Gauge_01
   на южной стороне, разверните нос к прибору и дайте LiDAR снять зону
   осмотра. Отъедьте в свободную середину прохода ближе к стартовому торцу,
   развернитесь примерно вдоль +X, остановитесь и оставьте Play включённым.
4. Проверьте покрытие в RViz с Fixed Frame `map`, дисплеями Map (`/map`)
   и LaserScan (`/scan`). По карте должны быть видны оба конца прохода и
   свободное пространство у точки. Проверьте TF, сохраните карту под новым
   именем и запишите начальную позу и Gauge_01 в профиль. Калибровка
   использует пару поз в Isaac world и `map` на общем timestamp скана.
   Команда заменяет развёрнутую конфигурацию Nav2 в профиле площадки.

   ```bash
   bash tools/ros/go2_nav2.sh check --seconds 5
   bash tools/ros/go2_nav2.sh save sites/factory_hall/nav2/factory_hall_new
   bash tools/ros/go2_nav2.sh calibrate --map sites/factory_hall/nav2/factory_hall_new.yaml
   ```

   Для PGM в режиме trinary используется `free_thresh: 0.196`. При таком
   пороге серый пиксель `205` загружается в map_server как неизвестная клетка.

5. Остановите запись и SLAM через Ctrl+C в их терминалах. Запись live
   включает SLAM TF `map → odom`. Для повторного запуска SLAM на этой
   записи оставьте в `/tf` только исходный `odom → base_link`. Короткая
   запись из передачи уже содержит только TF моста.

## Навигация и кадр

Сохранённая начальная поза соответствует месту остановки при калибровке.
При другой стартовой позиции до передачи управления задайте
локализацию через **2D Pose Estimate** в RViz. Профиль также содержит
вычисленную позу для штатного spawn, используйте `navigation initial_pose:=spawn`
после Stop → Play. Перед запуском оставьте Kit в Play
и уберите ранее запущенные издатели команд скорости.

```bash
# Терминал Nav2
bash tools/ros/go2_nav2.sh navigation
# Терминал демонстрации
bash tools/ros/go2_nav2.sh check --seconds 5
ros2 topic info /cmd_vel -v
bash tools/ros/go2_nav2.sh demo --output reports/go2_ros2/gauge_01_run
```

Команда демонстрации передаёт владение ROS через штатный Kit endpoint.
По завершении возвращает manual и обнуляет команду, ждёт стабилизацию
и сохраняет `result.json` и свежий `front.png` из FrontCamera. В логе
есть UUID цели, исход action, траектория и ошибка остановки в `map`.
На отмене или отказе записывается исход без кадра. Для проверки отмены
используйте `--cancel-after 2` и новый каталог вывода. Цель вне карты
можно проверить флагом `--goal 100 100 0` на свободном участке.

Для отмены action нажмите Ctrl+C в терминале демо. Esc в Kit немедленно
возвращает ручной режим. Убедитесь, что владелец manual, а команда
нулевая. Если WSL потерял связь с Kit, используйте Esc. Удерживайте терминал
Nav2 доступным для диагностики, по окончании остановите его через Ctrl+C.

При перезапуске моста, Stop или сбросе часов завершите текущий action и
запуски ROS, затем поднимите новый процесс SLAM или Nav2 с чистыми
буферами. При новом старте Nav2 снова задайте начальную позу. Сторож запуска
завершает эпизод при обратном скачке `/clock` или двух настенных секундах
без продвижения live-часов.
Скрипт запуска блокирует вторую копию эпизода в том же домене. Если lifecycle
manager не дошёл до `Managed nodes are active`, остановите этот запуск и
поднимите чистый эпизод. В испытаниях встречались таймауты обнаружения DDS.

### Интерфейс движения

DWB публикует Humble `Twist` в `/go2/nav2_cmd_vel`. Адаптер ограничивает
вектор XY и yaw по профилю моста. Каждое полученное значение расходуется
один раз, на следующей строго возрастающей метке `/clock`. Команда,
пролежавшая 150 мс, или ожидающая после остановки часов отбрасывается.
Один издатель `/cmd_vel` передаёт `TwistStamped` в `base_link`.
При тишине продолжают действовать лимиты, проверка возраста и настенный
watchdog 0,4 с существующего моста. Pause, Stop и Esc снимают владение ROS.

### Испытания препятствий

Для статического испытания сохраните baseline, остановите Go2 в manual и
поставьте куб 0,45 × 0,45 × 0,80 м на его прежней линии движения. Выберите
свободный участок с местом для объезда и путь с запасом до старта и цели.
Куб создаётся в session layer, после теста удаляется.

```powershell
$kitSend = 'D:/isaacsim/skills/isaac-sim-remote/scripts/isaacsim_send.py'
python -I $kitSend --file tools/isaac/nav2_inspection.py --arg "action='obstacle_add'" --arg 'obstacle_xy=(-6.3,-0.5)'
# Повторите demo из терминала WSL с новым каталогом и сравните траектории.
python -I $kitSend --file tools/isaac/nav2_inspection.py --arg "action='obstacle_remove'"
```

Предстоящее динамическое испытание по ТЗ. Внести движущееся препятствие
в LiDAR-плоскость на проверенном участке. Измерить минимальный зазор между
контурами, требование ≥1 м. При перекрытии прохода измерить ожидание 10 с
и зарегистрировать попытку перепланирования. Текущее дерево выполняет
перепланирование при движении и завершает action при отказе планировщика.
Ожидание по ТЗ и контроль метрового зазора требуют отдельной реализации
и живого испытания.

## Офлайн smoke в домене 43

```bash
export ROS_DOMAIN_ID=43
bash tools/ros/go2_nav2.sh mapping
# Второй терминал, тот же домен
ros2 bag play --delay 2 --rate 0.5 reports/go2_ros2/manual_pass_simtime
bash tools/ros/go2_nav2.sh save reports/go2_ros2/smoke_replay_map
# Остановите SLAM, затем запустите Nav2, после чего начните replay заново
bash tools/ros/go2_nav2.sh navigation map:=reports/go2_ros2/smoke_replay_map.yaml \
  'initial_pose:={"x":0,"y":0,"yaw_rad":0}'
```

Используйте записанный `/clock` без опции `ros2 bag play --clock`.
В домене 43 отсутствует издатель выходной команды адаптера. Короткая
запись отражает 1,332 м между крайними позами.

## Файлы

- `tools/ros/go2_nav2.sh` и `go2_nav2.launch.py` запускают режимы и инструменты.
- `go2_slam.yaml`, `go2_nav2.yaml` и `go2_navigate*.xml` задают параметры и деревья.
- `nav2_episode.py` ставит метки команд и следит за часами.
- `go2_nav2_run.py` проверяет TF, калибрует станцию и ведёт журнал action.
- `tools/isaac/nav2_inspection.py` связывает демонстрацию со штатным
  управлением и камерой Kit.
- `sites/factory_hall/site.json` содержит профиль станции и Nav2.
- `sites/factory_hall/nav2/factory_hall.yaml` и `factory_hall.pgm` содержат LiDAR-карту.

## Измерения 20 ноября 2026

**Rosbag, домен 43.** SLAM сохранил карту 397 × 605 с разрешением 0,05 м,
map_server открыл сохранённый PGM, AMCL прошёл 7 проверок полной цепочки
TF по меткам сканов без ошибок. Цель (100, 100) вернула `ABORTED`.
В отдельном синтетическом тесте адаптер выдал две команды на отметках
1,020 и 1,120 с. При пропаже Twist и при остановке и возобновлении
часов старая команда повторно не вышла. Обратный скачок завершил эпизод.
[Карта smoke](../reports/go2_ros2/nav2_smoke/map.yaml) ·
[TF](../reports/go2_ros2/nav2_smoke/navigation_tf.json) ·
[Отказ](../reports/go2_ros2/nav2_smoke/no_path.log) ·
[Адаптер](../reports/go2_ros2/nav2_smoke/adapter.json)

**Живой Isaac Sim, домен 42.** Оператор провёл Go2 туда и обратно по
основному проходу. По 2349 отсчётам odom пройдено 31,53 м, размах
продольной координаты 13,97 м. Сохранённая LiDAR-карта 396 × 605
содержит 192 371 свободную, 3823 занятые и 43 386 неизвестных клеток.
Полную TF-цепочку проверили у 12 сканов в SLAM и 11 сканов в AMCL,
без ошибок. Цель вне карты (100, 100) также вернула `ABORTED` в live.
[Ручная запись](../reports/go2_ros2/nav2_live/manual_factory/metadata.yaml) ·
[Покрытие](../reports/go2_ros2/nav2_live/manual_factory_coverage.json) ·
[TF AMCL](../reports/go2_ros2/nav2_live/amcl_tf.json) ·
[Отказ](../reports/go2_ros2/nav2_live/no_path_live.log)

**Gauge_01.** Цель в `map` (3,425 м; −1,096 м; −1,572 рад). Живой
`NavigateToPose` завершился `SUCCEEDED`. Последняя оценка Nav2 до
завершения отклонялась на 0,114 м и 0,146 рад. Независимый отсчёт Isaac
при переходе в manual, пересчитанный в `map` по калибровке, дал 0,178 м
и 0,246 рад. Go2 остановлен в manual с командой `[0, 0, 0]`. После
стабилизации сохранён кадр с видимым прибором.
[Исход и координаты](../reports/go2_ros2/nav2_live/gauge_01_cli/result.json) ·
[Журнал Nav2](../reports/go2_ros2/nav2_live/gauge_01_cli/navigation.log) ·
[Передача CV](GO2_NAV2_CV_HANDOFF.md)

**Статический куб.** Куб со стороной 0,45 м, центр Isaac (−6,25; −0,20),
перекрыл линию базового прохода. После включения отметок LiDAR выше пола
и настройки допуска курса 0,26 рад `NavigateToPose` завершился `SUCCEEDED`.
Траектория прошла севернее куба, минимум от центра Go2 до его границы
по локализации составил 0,682 м. Ошибка по последней оценке Nav2
составила 0,180 м и 0,227 рад. После возврата в manual получен свежий кадр.
[Проверка TF и /cmd_vel](../reports/go2_ros2/nav2_live/gauge_01_obstacle_verified/preflight.json) ·
[Траектория](../reports/go2_ros2/nav2_live/gauge_01_obstacle_verified/trajectory.json) ·
[Исход](../reports/go2_ros2/nav2_live/gauge_01_obstacle_verified/result.json) ·
[Журнал](../reports/go2_ros2/nav2_live/obstacle_verified_navigation.log)

**Потеря Twist в живом эпизоде.** Приостановка controller_server на 1,2 с
оборвала поток. Команда в Kit изменилась с приблизительно
`[0,33, −0,11, 0,043]` на `[0, 0, 0]` в режиме ROS, счётчик watchdog
вырос с 2 до 3. Действие отозвалось `ABORTED` по таймауту контроллера.
После теста управление переключено в manual с нулевой командой.
[До потери](../reports/go2_ros2/nav2_live/stream_loss/before_loss.json) ·
[При пропаже](../reports/go2_ros2/nav2_live/stream_loss/during_loss.json) ·
[Manual](../reports/go2_ros2/nav2_live/stream_loss/final.json)

**Отмена в живом эпизоде.** Запуск `demo --cancel-after 2` отменил
принятую цель. NavigateToPose вернул `CANCELED`, контроллер подтвердил
остановку, Kit остался в manual с `[0, 0, 0]`. В журнале BT появился
таймаут ожидания результата FollowPath при остановке дерева, после
которого контроллер обработал отмену.
[Исход](../reports/go2_ros2/nav2_live/cancel_verified/result.json) ·
[Журнал action](../reports/go2_ros2/nav2_live/cancel_verified/action.log)
