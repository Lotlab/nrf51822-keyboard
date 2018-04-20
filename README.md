# nrf51822-keyboard

## 概述

这是一个基于nrf51822蓝牙键盘的固件，使用了nRF SDK 10.0作为底层硬件驱动，并使用TMK键盘库作为键盘功能的上部实现。

## 软件

目录说明：

- bootloader：用于DFU更新固件的Bootloader
- main：主程序
- sdk：nRF SDK 10.0 的用到的源码文件
- tmk_core：TMK源码

## 硬件

这个固件是为BLE4100设计的，关于BLE4100可以参见[这个页面](https://wiki.lotlab.org/page/ble4100/advanced/)

## 编译

打开`main/project/arm5_no_packs/`目录下的keil工程，点击编译即可。

