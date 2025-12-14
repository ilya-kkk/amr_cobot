#!/bin/bash

# Скрипт для запуска ROS2 модели AMR_COBOT в Docker

# Проверка DISPLAY
if [ -z "$DISPLAY" ]; then
    echo "ОШИБКА: DISPLAY не установлен!"
    echo "Установите DISPLAY переменную: export DISPLAY=:0"
    exit 1
fi

# Создание X11 auth файла для GUI приложений
XAUTH="$HOME/.docker.xauth"

echo "Настройка X11 авторизации..."
if command -v xauth >/dev/null 2>&1; then
    if [ ! -f "$XAUTH" ]; then
        echo "Создание X11 auth файла..."
        touch "$XAUTH"
        if xauth nlist "$DISPLAY" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - 2>/dev/null; then
            chmod 644 "$XAUTH"
            echo "✓ X11 auth файл создан: $XAUTH"
        else
            echo "⚠ Не удалось создать xauth файл через xauth, используем xhost..."
            rm -f "$XAUTH"
            xhost +local:docker 2>/dev/null || xhost +SI:localuser:root 2>/dev/null || true
            # Создаем пустой файл для монтирования
            touch "$XAUTH"
            chmod 644 "$XAUTH"
        fi
    else
        echo "X11 auth файл уже существует: $XAUTH"
        # Обновляем существующий файл
        xauth nlist "$DISPLAY" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - 2>/dev/null || true
    fi
else
    echo "⚠ xauth не найден, используем xhost для разрешения доступа..."
    xhost +local:docker 2>/dev/null || xhost +SI:localuser:root 2>/dev/null || true
    # Создаем пустой файл для монтирования
    touch "$XAUTH" 2>/dev/null || true
    chmod 644 "$XAUTH" 2>/dev/null || true
fi

# Проверка существования xauth файла перед запуском
if [ ! -f "$XAUTH" ] && ! xhost 2>/dev/null | grep -q "LOCAL:"; then
    echo "⚠ ПРЕДУПРЕЖДЕНИЕ: X11 авторизация может не работать!"
    echo "Попробуйте выполнить вручную:"
    echo "  xhost +local:docker"
    echo ""
fi

# Экспорт переменных для docker-compose
export HOME
export XAUTH_FILE="$XAUTH"

# Проверка что файл существует перед запуском
if [ ! -f "$XAUTH" ]; then
    echo "Создание пустого xauth файла для монтирования..."
    touch "$XAUTH"
    chmod 644 "$XAUTH"
fi

echo "Используется XAUTH файл: $XAUTH_FILE"

# Запуск контейнера
echo "Запуск Docker контейнера..."
docker compose up --build

echo "=========================================="
echo "ROS2 контейнер остановлен"
echo "=========================================="

