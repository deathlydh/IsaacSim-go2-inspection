# Go2 в Isaac Sim 6.1 · интерфейс Nav2

Проверено 2026-11-20 в открытом цехе `factory_hall.usda` на `robot_lab`.
Isaac Sim `6.1.0-rc.26+release.49347.2d230af4.gl`, ROS Bridge 5.1.4,
ROS Core 1.11.0, встроенный ROS Jazzy rclpy 7.1.11. Внешняя сторона — WSL2
Ubuntu 22.04.5 LTS, Humble rclpy 3.3.20, rosbag2 0.15.16, RViz 11.2.25.
Также установлен WSL2 Ubuntu 24.04.3 LTS без дистрибутива в `/opt/ros`.
На ПК Windows 10 Enterprise 22H2 (19045), RTX 4060 Ti 8 GiB, драйвер 617.42.

NVIDIA документирует Windows 11 для 6.1.0. На этом Windows 10 прямое
обнаружение Fast DDS из WSL не дало сообщений `/clock`, включая пробу с
явным адресом Windows-шлюза. Рабочий путь использует DDS в каждом окружении
и WSL interop pipes между двумя ROS-узлами. Типы, CDR, timestamps и QoS
сохраняются. Домены 0 (Kit/Windows) и 42 (WSL) разделяют графы.
Системные переменные, firewall и portproxy для проверки не менялись.
[Документация ROS 2 для Isaac Sim 6.1.0](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_ros_other_platforms.html)

## Запуск и проверка

Пользователь открывает Isaac. Для чистого запуска цеха используется
`D:\go2-inspection\start_factory.bat` из Проводника. Если Go2 уже открыт
в Kit, оставьте текущую сцену и включите Play. Команды attach/status сцену
не перезагружают. В совместной сессии используйте живой Kit по очереди.

Выполните команды в PowerShell из `D:\go2-inspection`.

```powershell
$kitSend = 'D:/isaacsim/skills/isaac-sim-remote/scripts/isaacsim_send.py'
python -I $kitSend --file tools/isaac/probe_navigation_environment.py
python -I $kitSend --file tools/isaac/navigation_session.py --arg "rmw='rmw_fastrtps_cpp'" --timeout 30
python -I $kitSend --file tools/isaac/navigation_session.py --arg "action='status'"
```

`rmw` задаёт Fast DDS только внутри Kit. Если уже загружен ROS Core с другим
RMW, нужен новый запуск Kit пользователем. Для LiDAR без ROS вместо `rmw`
укажите `--arg bridge=False`.

Запустите relay в первом терминале Ubuntu-22.04 в WSL и оставьте его работать.

```bash
cd /mnt/d/go2-inspection
source /opt/ros/humble/setup.bash
python3 tools/ros/wsl_go2_relay.py --windows-python /mnt/c/Users/Admin/AppData/Local/Programs/Python/Python312/python.exe
```

Для Windows-узла нужен Python 3.12 с NumPy. Путь к интерпретатору можно
заменить. Проверка и RViz запускаются во втором терминале WSL.

```bash
cd /mnt/d/go2-inspection
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic info /scan --verbose
ros2 topic echo --once --qos-reliability best_effort /clock
python3 tools/ros/verify_go2_navigation.py
rviz2 -d tools/ros/go2_navigation.rviz --ros-args -p use_sim_time:=true
```

Режим после подключения — `manual`. Для теста движения на свободном месте
вызовите в PowerShell `python -I $kitSend --file tools/isaac/navigation_session.py --arg "action='ros'"`,
затем в подготовленном WSL выполните `python3 tools/ros/verify_go2_navigation.py --motion`.
Возврат клавиатуры — `--arg "action='manual'"` или Esc в Kit.
`--arg "action='close'"` отключает узел и убирает mount из session layer.
Перезапуск моста начинает новую временную шкалу. После Stop начинайте новый
эпизод SLAM/Nav2 с пустыми буферами TF и начальным состоянием локализации.

## Топики

Частоты даны на секунду физического времени. QoS Keep Last, в скобках глубина.
Расстояния в метрах, углы в радианах, скорости в соответствующих единицах в секунду.

| Топик | Тип | Частота и QoS | Кадры / источник |
| --- | --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | 10 Гц · best effort (5), volatile | `lidar_link`, снимок PhysX |
| `/odom` | `nav_msgs/msg/Odometry` | 20 Гц · reliable (10), volatile | `odom` → `base_link`, pose/twist из articulation |
| `/tf` | `tf2_msgs/msg/TFMessage` | 20 Гц · reliable (100), volatile | `odom` → `base_link`, поза и stamp как в `/odom` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | При подключении · reliable (1), transient local | `base_link` → `lidar_link`, крепление из профиля |
| `/clock` | `rosgraph_msgs/msg/Clock` | 50 Гц · best effort (1), volatile | Сумма physics dt в наносекундах с момента подключения |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Вход · reliable (1), volatile | `base_link` или пустой frame, +X вперёд, +Y влево, +wz против часовой стрелки |

- `use_sim_time:=true` для Nav2, SLAM, RViz и узла, который ставит stamp на
  команду. Physics dt не накапливается при Pause. Для каждого скана отправляются
  `/odom` и `/tf` с тем же stamp. При некратных частотах профиля возможны
  дополнительные отсчёты одометрии на timestamps сканов.
- Одометрия — **симуляторная ground truth в локальном odom**. Начало X/Y
  и yaw — первый отсчёт эпизода. Z считается от Z=0 сцены. Roll/pitch
  сохранены, twist в `base_link`. Ковариации нулевые, модель ошибки отсутствует.
  `map → odom` публикует SLAM или локализация. Глобальная поза Isaac не
  входит в локализацию Nav2. Отдельного `base_footprint` нет.
- Финальный `/cmd_vel` публикует одно звено после всех фильтров скорости.
  В Nav2 Jazzy задайте `enable_stamped_cmd_vel: true` на каждом звене, где
  доступен параметр. Для Nav2 Humble с выходом `Twist` нужен конечный
  адаптер в `TwistStamped` со свежим stamp из sim time. При потере входа
  адаптер прекращает публикацию.
  [Nav2 · переход с Iron на Jazzy](https://docs.nav2.org/jazzy/configuration_and_development/migration_guides/iron/Iron/)
- Isaac проверяет строго растущую метку после входа в режим ROS, возраст
  до 0,35 с физического времени, конечность и планарность. Лимиты по
  умолчанию 0,5 м/с по вектору XY и 0,8 рад/с по yaw. Через 0,4 с тишины
  по монотонным часам команда обнуляется до следующего шага исполнения.
  Свежий поток может возобновить движение в режиме ROS. Pause, Stop и Esc
  снимают владение ROS. Route и ROS не могут захватить управление друг
  у друга, движение с клавиатуры доступно в manual. Штатный heading hold
  действует при `|wz| ≤ 0,001`, в том числе после обнуления команды.

## LiDAR

Профиль `robots/go2_edu.json` задаёт крепление в метрах и yaw в градусах,
дальность, FOV, число лучей и частоты. Временный mount —
`/World/Robot/base/lidar_link`, на 0,20 м выше базы. По умолчанию 720 лучей,
углы от −180° до 179,5° с шагом 0,5°, дальность 0,12–18 м. Угол 0
соответствует +X LiDAR. Лучи синхронны. `time_increment=0`, `scan_time=0,1`,
интенсивности пустые. Промах — `+inf`. Ближние, собственные и почти
горизонтальные попадания в пол до 0,035 м от Z=0 помечаются `NaN`:
за перекрытием луча область не считается свободной.

Выбран планарный PhysX для 360° по коллайдерам и общего timestamp без
дополнительного RTX render product. Лучи лежат в плоскости корпуса,
которая наклоняется вместе с роботом. 3D-проекции и полосы высот нет.
Визуальная геометрия без коллайдера и препятствия ниже плоскости скана
могут пропускаться. При наклоне часть лучей отсекается полом.

## Видеоконтракт

Камера инспекции работает через `FrontCamera`. Отдельный контракт видео
предусматривает `/camera/front/image_raw` типа `sensor_msgs/msg/Image`,
RGB8, 1280×720, и `/camera/front/camera_info` типа
`sensor_msgs/msg/CameraInfo`. Оба используют `camera_front_optical_frame`,
одинаковый stamp события получения кадра в sim time, best effort/volatile,
глубину 5. Калибровка из `FrontCamera.intrinsics()`. Оптические оси ROS:
+X вправо, +Y вниз, +Z вперёд. Для публикации нужен статический TF из
`base_link` с преобразованием осей USD camera в ROS optical axes.
RGB и CameraInfo в этом мосте пока не публикуются.

## Фактические результаты

Три окна A/B/A измерены при включённой штатной камере, неизменных шаге
физики и походке.

| LiDAR | FPS обновлений Kit | RTF | Scan на настенную секунду |
| --- | ---: | ---: | ---: |
| Выкл. | 18,34 | 0,367 | 0 |
| Вкл. | 19,80 | 0,396 | 4,11 |
| Выкл. | 20,34 | 0,407 | 0 |

За окно 8 с запросы PhysX занимали в среднем 6,62 мс на скан. Разброс
базовых замеров не позволяет приписать разницу FPS сенсору. Со стороны
WSL в покое измерены 4,00 Гц scan, 8,00 Гц odom и TF, 19,59 Гц clock.
Стоя — 606 попаданий, 114 промахов, 0 отброшенных лучей из 720.
Луч 0° до известной стены — 17,489668 м, геометрический расчёт 17,489669 м.
Ещё четыре луча при 6°, −15,5°, −48°, −53° совпали с расчётом по граням
стен с ошибкой до 3 мкм. Это проверка симуляционных коллайдеров.

На тесте движения у всех 111 сканов совпали метки `/odom` и `/tf`.
`odom → lidar_link` разрешался на timestamp в пределах кэша tf2.
556 дистанций изменились более чем на 3 см. Команды дали +0,327 м
вперёд, +0,198 м влево и +0,909 рад по yaw. При прямом ходе с включённым
heading hold yaw изменился на −0,112 рад. Тест с заведомо старым stamp
повысил счётчик входящих с 146 до 147, счётчик принятых остался равен 124.
После исчезновения издателя
watchdog обнулил команду, остаточный дрейф в последующем окне покоя —
0,021 м. Проверены конфликты route/ROS и возврат клавиатуры. RViz запущен
в WSLg, подписки на `/scan`, `/tf` и `/clock` видны в ROS-графе.
Захват окна WSLg завершился таймаутом, поэтому визуальное отображение не подтверждено.

Ручной проход записан через события обычных клавиш UP/N/UP.
Готовая к воспроизведению копия — `reports/go2_ros2/manual_pass_simtime/`.
В исходнике `manual_pass/` у часов приёма было четыре обратных скачка.
Копия упорядочена по исходным меткам симуляции, CDR каждого сообщения сохранён.
SQLite3/CDR, 120 сканов, 240 odom, 240 TF,
1 статический TF, 600 clock. В записи 11,98 с физического времени и
1,332 м между первой и последней позой. Все сканы имеют записанные
`/odom` и `/tf` с теми же метками времени. При воспроизведении в домене
43 все 120 сканов прошли проверку `tf2` на своих timestamps. Результат
в `reports/go2_ros2/replay.json`.

```bash
ros2 bag info reports/go2_ros2/manual_pass_simtime
# Домен воспроизведения отделён от live-симуляции.
ROS_DOMAIN_ID=43 ros2 bag play --delay 1 reports/go2_ros2/manual_pass_simtime
ROS_DOMAIN_ID=43 rviz2 -d tools/ros/go2_navigation.rviz --ros-args -p use_sim_time:=true
# Новая запись в live-домене.
ROS_DOMAIN_ID=42 ros2 bag record --use-sim-time -o reports/go2_ros2/manual_pass_new /scan /odom /tf /tf_static /clock
```

Исходные измерения — `reports/go2_ros2/performance.json`, `passive.json`
и `motion.json`. A/B/A повторяется через
`python -I $kitSend --file tools/isaac/measure_navigation.py --timeout 80`.
