#!/bin/bash

# Проверка DISPLAY
if [ -z "$DISPLAY" ]; then
    echo "ОШИБКА: DISPLAY не установлен!"
    exit 1
fi

# Проверка X11 авторизации (не критично, если используется xhost)
if [ ! -f /tmp/.docker.xauth ] || [ ! -s /tmp/.docker.xauth ]; then
    echo "ℹ X11 auth файл не найден или пуст. Если используется xhost, это нормально."
fi

# Выполнение переданной команды
exec "$@"

