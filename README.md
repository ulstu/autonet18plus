# Робототехническая платформа для участников соревнований "Autonet 18+"

## Важные ссылки

* [Описание платформы робота, порядка доступа команд к нему](https://docs.google.com/document/d/1wRQyuIwhhfYffbE6CxJ1mTlKcZbFuP9l5_G1zhkKjcg)
* [Официальные регламенты соревнований](https://www.russianrobotics.ru/competition/competition/autonet/autonet-18/)

## Скрипты для установки решения
* install.sh - прописываются символические ссылки из папки проекта в папку с исхдоными кодами решений ROS (обычно catkin_ws)
* run.sh - запуск решения

## Публикация сообщений для начала движения робота 
rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'