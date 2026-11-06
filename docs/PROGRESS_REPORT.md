# Состояние проекта Go2 Inspection

Дата: 6 ноября 2026 (America/Los_Angeles). Каталоги результатов именуются по UTC.

## Выполнено

Закрыты R0, A1, A3, A4; A0/A2/N0 сохраняются выполненными. Подготовлены границы
переносимости, независимое CV и измеренная оценка на 200 кадрах Isaac. Полный поиск
прибора пока ненадёжен: успех чтения заданной области не выдаётся за автономную детекцию.

R0: inspection/mission.py не импортирует Kit, не содержит абсолютного корня,
жёсткой сцены или лимита пяти. Площадки — sites/<id>/site.json (сцена, карта/frame,
станции, приборы, route, inspection_pose, статическая calibration). Общие действия
в inspection/actions.py; отдельны Isaac IO/Camera и known_aisle. Протоколы Pose,
Motion/TTL, Navigation, Camera и Clock зафиксированы. В симуляции управление и
стабилизация используют physics time, watchdog — monotonic wall time; capture_id,
frame и время получения сохраняются. Физику/FPS/походку и установку не меняли.

Офлайн: rotated_demo, семь целей, иной origin и проход 90°, тот же runner:
[site_check.json](../reports/20261106T120001Z_5094523b/site_check.json).
Профиль offline_only, живая отправка запрещена; отдельной второй USD-сцены нет.
Неidentity world_from_map пока отвергается: нужен localization adapter.

Живой R0 через совместимый CLI: [маршрут 5/5](../reports/20261106T072605Z_manual_3c732ed9/route.json),
180.781 с wall / 73.2 с control, максимум 10.70 см / 2.217°.
Это новый прогон после [ревью](REVIEW_AND_PERFORMANCE_2026-11-05.md), не новый
замер FPS. После добавления TTL исправлена ошибка округления лимита скорости;
этот вариант использован при физическом перемещении между точками сбора A3.
Телепортаций для маршрута или сбора не применяли. Холодный startup не проверяли.

## Данные и качество CV

[Итоговый HTML с галереей ошибок](../reports/cv_eval_20261106T115635Z_a1b1fc71/evaluation.html)
· [metrics.json](../reports/cv_eval_20261106T115635Z_a1b1fc71/metrics.json)
· [predictions.jsonl](../reports/cv_eval_20261106T115635Z_a1b1fc71/predictions.jsonl)
· [фиксированный протокол](../reports/cv_eval_20261106T115635Z_a1b1fc71/protocol.json).

[Набор: manifest и 200 RGB](../data/gauges/20261106T115331Z_merged_34d957a3/manifest.jsonl),
пять приборов ×40; 100 dev / 100 test по разным seed и дистанциям. 190 visible и
10 hidden. Значения включают края шкал; два множителя света 0.8 и 1.15,
дистанция запроса 0.809–1.598 м, yaw −14.50..14.95°.
[QA пяти приборов](../data/gauges/20261106T115331Z_merged_34d957a3/qa_five_gauges_dev.png).

На test 100 кадров: 95 visible, 5 hidden. Visible означает сценарий видимого
прибора, а не независимую экспертную оценку читаемости каждого кадра.

| Режим | Числовые чтения visible | Отказы всего / visible | nMAE по числовым visible | Уверенные ошибки / все | Числовой ответ hidden |
|---|---:|---:|---:|---:|---:|
| Известная ROI (из разметки) | 94/95 = 98.95% | 6/100 / 1/95 | 0.3192% | 0/100 | 0/5 |
| Полный поиск | 64/95 = 67.37% | 35/100 / 31/95 | 3.9407% | 7/100 | 1/5 |

Confidence — эвристика. Уверенная ошибка: confidence≥0.8 и ошибка >2% диапазона,
IoU области <0.5 либо числовой ответ при hidden. У full это также 7/64 уверенных
числовых ответов (10.94%). Visible detection recall full — 73.68%.

MAE по test, отдельно по приборам; усреднять bar/psi/°C в одно raw число нельзя:

| Прибор | MAE known ROI | MAE full | Visible отказы ROI / full |
|---|---:|---:|---:|
| Gauge_01 | 0.0279085 bar | 0.0199232 bar | 1 / 4 |
| Gauge_02 | 0.0551156 bar | 0.0583184 bar | 0 / 4 |
| Gauge_03 | 0.415588 psi | 5.31401 psi | 0 / 5 |
| Gauge_04 | 0.00380759 bar | 0.321011 bar | 0 / 13 |
| Gauge_05 | 0.824632 °C | 0.829944 °C | 0 / 5 |

Предварительная цель 2% / 95% достигнута только для чтения известной области.
Полный поиск её не достиг. ID приходит из задания, визуальное распознавание ID
не реализовано. Все 400 результатов сохранены до сравнения с truth.value;
worker получает RGB/калибровку и optional ROI, без истинных чисел/стрелки.
Код/настройки reader после A1 не менялись; SHA256 совпадает с состоянием до сбора.
После просмотра test настройки не изменялись.

Реальные ошибки:

- [Ограждение IBC ошибочно принято за прибор](../reports/cv_eval_20261106T115635Z_a1b1fc71/annotated/full_Gauge_04_0038.png):
  −0.7963 вместо −0.1162 bar, IoU=0. Нужен более строгий детектор кандидатов.
- [Отказ на дальней тонкой стрелке](../reports/cv_eval_20261106T115635Z_a1b1fc71/annotated/known_roi_Gauge_01_0036.png):
  ambiguous_or_occluded_needle, значение null.

## Сбор и воспроизведение

[Пробная серия из 10](../data/gauges/20261106T114124Z_14302a32/smoke_contact_sheet.png)
проверена до основного сбора. Ранние серии с самоперекрытием виртуальной камеры
отвергнуты визуальной QA и сохранены с validation.json=False. Это ошибки генерации,
а не фильтрация сложных предсказаний. Основные 160 кадров сохранились перед
stabilization_timeout на пятом приборе; ограниченный повтор помог, досняты 40.
Итоговый merge сохранил все строки, изображения и provenance SHA256, исходные
каталоги не менялись. В HTML это не называется одним безошибочным сбором.

Робот физически идёт на проверенную позицию 2.2 м от прибора, затем виртуальная
камера варьирует позу перед корпусом. Поза робота не подменяется. Этот rig расширяет
изображения датасета; он не доказывает работоспособность физического крепления.
Значения/свет/видимость/локальные transforms камеры восстановлены в finally;
после сбора live ready=True, failed=False, manual, команда [0,0,0].

Команды PowerShell (дополнительно в [README](../README.md)):

```powershell
cd D:\go2-inspection
python -m cv.read_image shots/camera_Gauge_01.png
python tools/validate_dataset.py data/gauges/20261106T115331Z_merged_34d957a3
python -m cv.eval data/gauges/20261106T115331Z_merged_34d957a3 --workers 4
python -m cv.eval data/gauges/20261106T115331Z_merged_34d957a3 --predictions reports/cv_eval_20261106T115635Z_a1b1fc71/predictions.jsonl
python tools/check_site.py --site sites/rotated_demo/site.json --start 10 19 0
python tools/run_route_cli.py --site sites/factory_hall/site.json
python tools/collect_dataset_cli.py --per-gauge 40 --seed 5701
python -m unittest discover -s tests -v
```

Live команды требуют Isaac, запущенного пользователем через start_factory.bat.
Отмена — ESC во вьюпорте или cancel_route.bat. Каждый результат имеет новый каталог.
Python 3.12.8/OpenCV 4.10.0/NumPy 2.5.2/Pillow 12.2.0; установок не было.

## Итоговая проверка кода

35/35 unittest прошли. Проверены синтаксис 63 Python-файлов, отсутствие BOM,
отсутствие omni/pxr imports в общих inspection-модулях, git diff --check.
[Протокол тестов](../reports/verification_20261106T120516Z_382e4337/unittest.txt),
[checks.json](../reports/verification_20261106T120516Z_382e4337/checks.json).
Финальный live status: robot_lab ready=True, failed=False, valid tensors,
manual, route active=False, команда [0,0,0].

## Граница sim-to-real и оставшаяся привязка

| Привязка | Текущее состояние | Что потребуется для реального стенда |
|---|---|---|
| Поза и карта | Oracle-поза из PhysX, world=map, USD AABB | Локализация/возраст данных/TF, сенсорная карта |
| Движение | Go2Session + robot_lab, heading hold, TTL в physics callback | Проверенный high-level Go2 SDK и независимый watchdog |
| Планировщик | known_aisle: прямой прямоугольный проход | Адаптер навигации площадки; новые препятствия сейчас не обнаруживаются |
| Камера | FrontCamera, pinhole/extrinsics Go2, render timestamps | Реальная калибровка/искажения, аппаратные timestamps и синхронизация |
| Генератор | make_gauge, USD truth, виртуальный rig | Независимая ручная разметка реальных изображений |
| Запуск | Батники и bootstrap текущей установки Isaac | Настройка подключения/установки на новом хосте |

Известная ROI в оценке берётся из разметки; передавать такую динамическую truth ROI
в рабочий CV недопустимо. Проверены 294 локальные ссылки HTML и все 400 PNG;
браузерная визуальная проверка file:// недоступна из-за политики инструмента.
Отдельные разметки, ошибки и QA-сетки осмотрены. Видео не записывалось.

Не проверены реальный Go2/SDK, реальные фото, вторая USD-сцена, cold startup,
живые искусственные падения/сбои. ROS/Nav2, пульт, 64 прибора и полноценный A5 не
реализованы. Следующий шаг A5: интеграция чтения в контролируемый обход с явными
отказами; улучшения full-детектора делать на dev и проверять на новом test.
