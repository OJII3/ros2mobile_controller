# ROS2 to Mobile Controller

## Overview

ROS2 でないスマホコントローラーとローカルで UDP 通信を行い、マイコンにデータを送信する別のノードのトピックに publish するノードです.

- `udp_broadcaster`: 一定時間ごとにブロードキャストします。
- `udp_listerner`:受信: 受信後コールバック関数を呼び、再び受信待機状態に戻ります。
- `watchdog`: 最終更新時刻を記録し、更新タイムアウトを検知するとコールバック関数を呼びます。

上記クラスのインスタンスのシェアードポインタ作成し、各ループを別々のスレッドで非同期に実行します。

## Requirements

- OS: Ubuntu 22.04, Arch Linux
- ROS2 Humble

## Usage

## Clone

`ros2のワークスペース/src` にクローンします。

```bash
git clone <this repository>
```

### Build

ROS2 のワークスペースでビルドします。

```bash
colcon build --packages-select ros2mobile_controller
```

### Run

```bash
ros2 run ros2mobile_controller ros2mobile_controller_bin
```
