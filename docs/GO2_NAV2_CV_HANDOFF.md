# Передача CV · Gauge_01

В живом Isaac Sim действие NavigateToPose до `station_Gauge_01`
завершилось `SUCCEEDED`. Кадр получен штатной FrontCamera после возврата
Go2 в manual, обнуления команды и стабилизации. Циферблат виден.

| Поле | Значение |
| --- | --- |
| RGB PNG, 1280 × 720 | [front.png](../reports/go2_ros2/nav2_live/gauge_01_cli/front.png) |
| Путь на Windows | `D:/go2-inspection/reports/go2_ros2/nav2_live/gauge_01_cli/front.png` |
| capture_id | `ac57d90aeebc47d7954c9f449b3e7d47` |
| Timestamp получения | `2026-11-20T11:22:33.504959+00:00` |
| Время моста | `492.50` с |
| Render count | `85494 → 85496` |
| Станция и прибор | `station_Gauge_01`, `Gauge_01`, PI-101 |
| Профиль и статическая калибровка | [site.json](../sites/factory_hall/site.json) |
| Метаданные камеры и action | [result.json](../reports/go2_ros2/nav2_live/gauge_01_cli/result.json) |

Точка подключения читателя в проекте — `cv.gauge_reader.read_gauge(rgb,
calibration, gauge_id, roi=None)`. CLI загружает PNG и преобразует BGR в
RGB перед вызовом читателя.

```powershell
cd D:/go2-inspection
python -m cv.read_image reports/go2_ros2/nav2_live/gauge_01_cli/front.png --site sites/factory_hall/site.json --gauge Gauge_01
```

Подробности полей и формата результата есть в [контракте CV](CV_CONTRACT.md).
В этом запуске обработка изображения закончилась сохранением RGB и метаданных.

Дополнительный кадр снят после успешного статического объезда.
`capture_id` — `8e35ec2f241a4baf9e413bad3995db61`, timestamp —
`2026-11-20T11:35:45.085459+00:00`.
[Кадр после объезда](../reports/go2_ros2/nav2_live/gauge_01_obstacle_verified/front.png) ·
[Метаданные](../reports/go2_ros2/nav2_live/gauge_01_obstacle_verified/result.json)
