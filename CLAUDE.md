# CLAUDE.md

軽量ウェイポイント走行スタック `lightnav`。仕様の正は [Issue #1](https://github.com/RoboCupper-tsukuchallebu/lightweight-nav-stack/issues/1)（契約節）と [Issue #2](https://github.com/RoboCupper-tsukuchallebu/lightweight-nav-stack/issues/2)。契約を変えたい場合は実装せず Issue にコメントして止まる。

## ビルドと試験

```
pixi run build   # colcon build（--symlink-install は新しい setuptools で失敗するため使わない）
pixi run test    # colcon test（build に依存）
pixi run lint    # ruff check src
```

- 実機・bag 再生用: `pixi run -e hardware ...`、評価用: `pixi run -e eval ...`
- 通常は `ROS_DOMAIN_ID=42`。`test` タスクだけ別ドメイン（43）と localhost 限定に上書きする。
- pixi.lock は linux-64 と osx-arm64 でバージョンが異なる（例: Python 3.14 と 3.12）。Mac の試験結果は本番（Linux）の代わりにならない。
- 初回ビルド前は `install/local_setup.sh` が無いため pixi が activation の警告を出すが、無視してよい。

## パッケージ配置

すべて `src/` 直下に `lightnav_*`。PR ごとに自分のパッケージのディレクトリだけを触る。`pixi.toml` と CI は基盤 PR で管理し、後続 PR は触らない。

## 禁止事項

- 次の操作は人の承認なしに行わない: 駆動系トピックの publish（`/cmd_vel` など）、駆動系サービスの call（モーター有効化など）、ファーム書き込み、リポジトリ外への破壊的操作。
- 実機を接続している PC（ドライバ起動中を含む）で試験する場合は、必ず `pixi run test`（`ROS_DOMAIN_ID=43`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`）を使う。実機のドメイン 42 で試験用ノードを動かさない。
- `nav2_*` への依存を入れない（CI が package.xml / CMakeLists.txt / pixi.toml を検査する）。
- 変数名を省略しない（`pos` ではなく `position` など）。
- マージは人間が行う。force push・履歴書き換えをしない。
