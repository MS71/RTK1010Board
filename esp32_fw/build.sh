#!/bin/bash
IDFPATH=~/workspace/esp32/esp-idf-v4.4
case "$1" in
    "env")
    . $IDFPATH/export.sh
    ;;
    "clean")
    $0 env
    idf.py clean-microros
    idf.py clean
    rm -rf build
    ;;
    "mower_s2")
    $0 env
    rm sdkconfig
    ln -s sdkconfig.mower_s2 sdkconfig
    $0 clean
    idf.py set-target esp32s2
    idf.py build
    ;;
    "mower_s2_4")
    $0 env
    rm sdkconfig
    ln -s sdkconfig.mower_s2_4 sdkconfig
    $0 clean
    idf.py set-target esp32s2
    idf.py build
    ;;
    "mower_s2_16")
    $0 env
    rm sdkconfig
    ln -s sdkconfig.mower_s2_16 sdkconfig
    $0 clean
    idf.py set-target esp32s2
    idf.py build
    ;;
    "s2")
    $0 env
    rm sdkconfig
    ln -s sdkconfig.$1 sdkconfig
    $0 clean
    idf.py set-target esp32s2
    idf.py build
    ;;
    "lilygot8")
    $0 env
    rm sdkconfig
    ln -s sdkconfig.lilygot8 sdkconfig
    $0 clean
    idf.py set-target esp32s2
    idf.py build
    ;;
    "zumo")
    $0 env
    rm sdkconfig
    ln -s sdkconfig.zumo sdkconfig
    $0 clean
    idf.py set-target esp32s2
    idf.py build
    ;;
    "flashmon")
    idf.py flash monitor
    ;;
    "m")
    idf.py menuconfig
    ;;
    *)
    $0 env
    echo "./build.sh"
    echo "       env"
    echo "       clean"
    echo "       mowser_s2_4"
    echo "       mowser_s2_16"
    echo "       lilygot8"
    echo "       s2             # generic S2 board with 4MB PSRAM"    
    echo "       zumo"
    echo "       update         # update workspace and sub components"
    echo "       flashmon       # flash & monitor"
    echo "       m              # menuconfig"
    ;;
esac
