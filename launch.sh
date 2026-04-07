#!/bin/bash
# launch.sh — запуск агентов команды bayesian_agent
#
# Использование:
#   ./launch.sh -all                   все агенты, localhost:27931
#   ./launch.sh -all -h localhost      явно указать хост
#   ./launch.sh -at 3 -fb 2 -pf 2     выбрать количество агентов
#   ./launch.sh -all -pre true         режим precompute
#
# Первым аргументом в adf.core.Main передаётся имя Loader-класса,
# затем остальные параметры. Алиасы -all, -local и т.д. обрабатываются
# внутри adf.core.Main до передачи в AgentLauncher.

LOADER="bayesian_agent.Loader"

cd "$(dirname "$0")"

if [ -z "$1" ]; then
    echo "Использование: ./launch.sh [опции]"
    echo ""
    echo "Примеры:"
    echo "  ./launch.sh -all"
    echo "  ./launch.sh -all -h localhost"
    echo "  ./launch.sh -at 3 -fb 2 -pf 2 -h localhost"
    echo "  ./launch.sh -all -pre true"
    echo ""
    echo "Опции:"
    echo "  -tn [NAME]    Название команды"
    echo "  -t F,S,P,O,A,C  Количество агентов каждого типа"
    echo "  -fb [N]       FireBrigade"
    echo "  -fs [N]       FireStation"
    echo "  -pf [N]       PoliceForce"
    echo "  -po [N]       PoliceOffice"
    echo "  -at [N]       AmbulanceTeam"
    echo "  -ac [N]       AmbulanceCentre"
    echo "  -h [HOST]     Хост сервера (порт 27931)"
    echo "  -s [HOST]:[PORT]  Сервер"
    echo "  -pre [true|false]  Precompute"
    echo "  -d [true|false]    Debug"
    echo "  -dev [true|false]  Development"
    echo "  -mc [FILE]    module.cfg"
    echo "  -all          все агенты (-t -1,-1,-1,-1,-1,-1)"
    echo "  -allp         по одному  (-t 1,0,1,0,1,0)"
    echo "  -local        -h localhost"
    echo "  -precompute   -pre true"
    echo "  -debug        -d true"
    echo "  -develop      -dev true"
    exit 0
fi

./gradlew launch --args="${LOADER} $*"