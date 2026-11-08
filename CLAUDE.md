# CLAUDE.md

軽量ウェイポイント走行スタック `lightnav`。仕様の正は [Issue #1](https://github.com/RoboCupper-tsukuchallebu/lightweight-nav-stack/issues/1)（契約節）と [Issue #2](https://github.com/RoboCupper-tsukuchallebu/lightweight-nav-stack/issues/2)。契約を変えたい場合は実装せず Issue にコメントして止まる。

## ビルドと試験

```
pixi run build   # colcon build（--symlink-install は新しい setuptools で失敗するため使わない）
pixi run test    # colcon test（build に依存）
pixi run lint    # ruff check src
```

- 実機・bag 再生用: `pixi run -e hardware ...`、評価用: `pixi run -e eval ...`
- 試験（`pixi run test`）は `ROS_DOMAIN_ID=44`、`LOCALHOST` で走る（タスクの env が activation の env に勝つ。pixi 0.59 で実測）。GUI（43）と実機（42）に混ざらない。`tools/pytest_plugins/lightnav_test_environment.py` が pytest 起動時に 44 以外はすべて中止し、`lightnav_msgs` の試験が 44 と中止の配線を検証する。この中止は `pixi run test` の中だけで効く。`pixi shell` から直接 `colcon test` を実行すると task env もプラグインも無く、既定の 43 で走り中止もされないので、試験は必ず `pixi run test` で実行する。パッケージは 1 つずつ順に走る（`--executor sequential`）。試験は `faulthandler_timeout=120`（`PYTEST_ADDOPTS`）で走り、1 つの試験が 120 秒を超えると全スレッドのスタックをログに出す（止めはしない。#17）。全パッケージの試験が同じドメイン 44 で同じトピック名・サービス名（`/lightnav/planner/plan_path`、`/lightnav/path`、`/lightnav/pose` など）を使うので、並列にすると互いの通信が見えて試験が壊れる。1 つのパッケージの試験を直しても並列には戻さない。
- `pixi run test` は常にドメイン 44 で走るので、複数の worktree で同時に実行しない（トピックが混ざるおそれがある）。
- 既定（activation）は `ROS_DOMAIN_ID=43`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`。`pixi shell` / `pixi run ...` すべてこの既定で、外から ROS_DOMAIN_ID を渡しても上書きされない（確認済み）。
- pixi.lock は linux-64 と osx-arm64 でバージョンが異なる（例: Python 3.14 と 3.12）。Mac の試験結果は本番（Linux）の代わりにならない。
- 初回ビルド前は `install/local_setup.sh` が無いため pixi が activation の警告を出すが、無視してよい。

## パッケージ配置

すべて `src/` 直下に `lightnav_*`。PR ごとに自分のパッケージのディレクトリだけを触る。`pixi.toml` と CI は基盤 PR で管理し、後続 PR は触らない。

## 禁止事項

- 次の操作は人の承認なしに行わない: 駆動系トピックの publish（`/cmd_vel` など）、駆動系サービスの call（モーター有効化など）、ファーム書き込み、リポジトリ外への破壊的操作。
- 実機（`ROS_DOMAIN_ID=42`）に届くのは X13 の `docker compose` だけ。`nav`（`/cmd_vel` を含む）の起動とパッドの Start は毎回人の承認が必須で、エージェントは行わない。`pixi run ros2-robot` が実機に届くのは tsukuchalle_drivers #19 の Mac モード（`TSUKUCHALLE_DDS_MODE=lan`）中だけで、その間は best-effort の欠けが戻る。これも起動・publish・call は毎回人の承認が必須で、エージェントは使わない。試験は既定のまま `pixi run test` を使い、実機のドメイン 42 で試験用ノードを動かさない。
- `nav2_*` への依存を入れない（CI が package.xml / CMakeLists.txt / pixi.toml を検査する）。
- 変数名を省略しない（`pos` ではなく `position` など）。
- マージの条件とマージする人は AGENTS.md に従う。force push・履歴書き換えをしない。

## Docker（X13 の実行環境）

X13 では nav-stack を pixi ではなく Docker で走らせる（Issue #35）。pixi は Mac の開発と CI に残す。ビルドと試験の環境は apt（Python 3.12、pytest 7.4）で、pixi の linux-64 とは版が違う。

```
docker build -f docker/Dockerfile -t lightnav:local .                  # runtime（最後の段）
docker build -f docker/Dockerfile --target test -t lightnav:test . && docker run --rm --shm-size=1g lightnav:test   # 試験（CI と同じ。ビルド機で行う）
LIGHTNAV_BUILD_HOST=<ssh 宛先> LIGHTNAV_ROBOT_HOST=<ssh 宛先> scripts/deploy.sh [git-ref]   # Mac で実行。ビルド機でビルドして X13 に入れる
```

- Mac では `docker build` をしない。イメージはビルド機で作り、deploy.sh が Mac 経由で X13 に送る。ホスト名・IP はスクリプトにもリポジトリにも書かない（環境変数で渡す）。
- X13 の `~/lightnav/` に `compose.yaml`・`config/robot.yaml`・`.env`・`data/`。`.env` に `TSUKUCHALLE_UID` と `TSUKUCHALLE_GID`（他リポのコンテナが root の間は 0）が無いと compose は止まる。deploy.sh は `.env` が無いときだけ `docker/.env.example` を置く。
- `config/robot.yaml` を現地で変えない（次の deploy が差分を見せて止まる）。値の変更は `docker/config/robot.yaml` を PR で直す。
- 起動（毎回人の承認）: `docker compose up -d nav`。`safety_gate_node` が起動と同時に active になり、Start の前に mission が `IDLE` であることを確かめる。ティーチ走行（パッドで `record_waypoints`）だけ `LIGHTNAV_SAFETY_GATE=false docker compose up -d nav`。その状態で `send_waypoints` をしない。gate の稼働中はパッドで操縦しない（止める手段はパッドの Back と 24 V）。
- 停止: `docker compose exec nav bash -lc 'ros2 lifecycle set /safety_gate_node deactivate'`（Twist 0）→ `docker compose stop nav`。`stop` だけだと 0 は出ず、ドライバの 0.5 s のタイムアウトで止まる。
- コンテナ内の確認用 CLI は `docker compose exec nav bash -lc '…'` で、`ros2 topic hz` などに `--no-daemon` を付ける。
- DDS の `docker/dds/` は tsukuchalle_drivers `ros2/dds/` の複製（正は drivers 側）。

## 合否の基準と Mac の差

- 合否の基準は ubuntu の CI。Mac では Python とコンパイラが異なる（`-Werror` は gcc で落ちることがある）。

## 独自 msg を使うノードの実行（macOS）

- macOS の SIP は /bin/sh・/usr/bin/env などを通ると `DYLD_*` を落とす。`pixi run test` では colcon が試験を `/bin/sh -c ". hook && env -0"` で作った環境で起動するため、`DYLD_LIBRARY_PATH` が pytest に届かない（実測）。
- 対策（パッケージ側の conftest や回避は不要）: dlopen は起動時の `DYLD_LIBRARY_PATH` が必要なので、test タスクが `LIGHTNAV_DYLD_LIBRARY_PATH` で渡し、pytest プラグインが macOS でだけ DYLD 付きで pytest を再起動する。
- ビルドは `--merge-install`（全ライブラリが `install/lib`）。`ros2` / `ros2-robot` タスクは従来どおり env で `DYLD_LIBRARY_PATH` を渡す。
- `pixi run ros2 run ...` / `pixi run ros2 launch ...` と `pixi run test` を使う。素の `pixi run python ノード.py` は Mac では失敗する。

## テストの書き方

- ament_python: `setup.py` に `extras_require={"test": ["pytest"]}` を書き、`test/` に pytest 形式で置く（colcon が unittest 扱いにして 0 件で落ちるのを防ぐ）。launch_testing は `@pytest.mark.launch_test` で動く。pytest は 9 で launch_testing plugin が壊れるため `>=8,<9` にピン留め済み。
- C++ の gtest: `ament_add_gtest` は ROS の gtest ソースと conda の gtest 1.18 ヘッダが食い違いビルド不能。`find_package(GTest REQUIRED)` + `add_executable` + `GTest::gtest_main` + `ament_add_test(... --gtest_output=xml:${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/<name>.gtest.xml GENERATE_RESULT_FOR_RETURN_CODE_ZERO)` を使う。
- C++ ノードが Python に依存する場合は `find_package(Python3 REQUIRED COMPONENTS Development)` + `Python3::Python` と書くだけでよい（`pixi run build` が `Python3_FIND_STRATEGY=LOCATION` / `Python3_ROOT_DIR` / `Python3_FIND_FRAMEWORK=NEVER` を渡し、Homebrew の Python に繋がるのを防ぐ。パッケージ側で指定しない）。
