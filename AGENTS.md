# Project

STM32CubeMX で生成された STM32F767ZIT6 / NUCLEO-F767ZI 向けファームウェア。

主な構成:

* STM32 HAL / LL
* FreeRTOS
* LwIP
* CAN1 / CAN2 / CAN3
* USART3
* USB OTG FS
* Ethernet
* CMake + Ninja
* ARM GNU Toolchain

CubeMX 設定元は `26catch_f7.ioc`。

# Build

Debug:

```sh
cmake --preset Debug
cmake --build --preset Debug
```

Release:

```sh
cmake --preset Release
cmake --build --preset Release
```

通常のコード変更では、可能なら Debug / Release の両方をビルドする。

# STM32 / CubeMX rules

* CubeMX 生成コードは、可能な限り `USER CODE BEGIN/END` 内で変更する。
* コメントは原則日本語で書く。自動生成されたコメントは削除しない。
* CubeMX 再生成時に消える可能性のある変更を生成領域へ直接追加しない。
* `.ioc` と生成コードの設定を矛盾させない。
* CubeMX 再生成後は `ioc` を確認し、意図しない変更や削除がないことを確認する。
* `Drivers/`、`Middlewares/` の外部・生成コードは、必要性が明確でない限り変更しない。
* `micro_ros_stm32cubemx_utils/` は micro-ROS 関連作業の場合のみ変更する。

# CMake rules

* 新しいソースやライブラリを追加した場合は、必要な CMake 定義も更新する。
* MCU、FPU、ABI、リンカ設定を不用意に変更しない。
* Cortex-M7 / `fpv5-d16` / hard-float の設定を維持する。
* リンカスクリプトやメモリ設定を変更した場合は、ビルド結果とメモリ使用量を確認する。

# Hardware safety

* 実機接続が必要な操作を、成功したと仮定しない。
* ST-LINK、CubeProgrammer、その他の書き込みコマンドを推測して実行しない。
* ファームウェアのビルド成功と実機動作確認を区別する。
* CAN、Ethernet、USB、モータ制御など実機依存の変更について、未確認の場合はその旨を明示する。
