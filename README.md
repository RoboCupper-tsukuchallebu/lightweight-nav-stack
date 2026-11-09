# lightweight-nav-stack

Nav2 パッケージ（`nav2_*`）に依存しない、差動 2 輪ロボット向けの軽量ウェイポイント走行スタック `lightnav`（ROS 2 Jazzy / RoboStack、osx-arm64 と linux-64）。

仕様: [Issue #1](https://github.com/RoboCupper-tsukuchallebu/lightweight-nav-stack/issues/1)、自己位置推定: [Issue #2](https://github.com/RoboCupper-tsukuchallebu/lightweight-nav-stack/issues/2)

```
pixi run build
pixi run test
pixi run lint
```

ロボット（X13）は Docker で動かす（Issue #35、詳細は CLAUDE.md の「Docker」）:

```
docker build -f docker/Dockerfile -t lightnav:local .                                  # build（ビルド機で）
docker build -f docker/Dockerfile --target test -t lightnav:test . && docker run --rm --shm-size=1g lightnav:test   # test
LIGHTNAV_BUILD_HOST=... LIGHTNAV_ROBOT_HOST=... scripts/deploy.sh [git-ref]          # deploy（ssh の宛先。Mac で）
```

ライセンス: Apache-2.0

障害物監視の死角と記憶の設定は [lightnav_obstacle_monitor の README](src/lightnav_obstacle_monitor/README.md) を参照。LiDAR 高さ 0.66 m、最下段 −30.67° の地面の死角は約 1.12 m、ドライバの min_range は 0.9 m。
