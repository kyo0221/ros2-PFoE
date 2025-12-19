# PFoE - Particle Filter on Episode for ROS2

視覚ベースの教示再生システム。カメラ画像から抽出した特徴量を用いたParticle Filter on Episode実装。

## 使用環境

- ROS2 Humble
- PyTorch

## インストール

```bash
cd ~/pfoe_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select pfoe_msg pfoe
source install/setup.bash
```

## 使い方

### 起動

```bash
ros2 launch pfoe teach_and_replay.launch.py
```

### 教示・リプレイ

1. **Button 6** を押下 → 教示モード開始
2. joyコントローラで操作:
   - **左スティック縦 (axis 1)**: 前進・後退
   - **右スティック横 (axis 3)**: 左右旋回
3. **Button 6** を再度押下 → リプレイモード開始（自動走行）

教示データは `~/.ros/pfoe_bags/` に自動保存されます。

## パラメータ

設定ファイル: `config/params.yaml`

### joy_controller
```yaml
linear_vel: 0.8          # 並進速度の最大値 [m/s]
angular_vel: 1.0         # 角速度の最大値 [rad/s]
button_mode_toggle: 6    # モード切り替えボタン
axis_linear: 1           # 並進速度軸
axis_angular: 3          # 角速度軸
```

### logger
```yaml
bag_directory: "~/.ros/pfoe_bags"  # bag保存先
```

### replay
```yaml
num_particles: 1000     # パーティクル数
loop_rate: 10.0         # 実行周期 [Hz]
bag_file: ""            # 指定bagファイル (省略可)
```

## 主要トピック

| トピック | 型 | 説明 |
|---------|---|------|
| `/zed/zed_node/rgb/image_rect_color` | sensor_msgs/Image | カメラ画像入力 |
| `/joy` | sensor_msgs/Joy | ゲームパッド入力 |
| `/cmd_vel` | geometry_msgs/Twist | 速度指令出力 |
| `/image_feature` | std_msgs/Float32MultiArray | 画像特徴ベクトル |
| `/teaching_mode` | std_msgs/Bool | 教示モード状態 |

## 参考

- [raspimouse_gamepad_teach_and_replay](https://github.com/rt-net/raspimouse_gamepad_teach_and_replay)

## ライセンス

BSD 3-Clause License

Copyright (c) 2025, Kyo Yamashita
Copyright (c) 2017, Ryuichi Ueda

詳細は [LICENSE](LICENSE) を参照してください。
