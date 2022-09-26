# Learning-pick-and-place-sim
## セットアップ方法
- レポジトリをクローン
- [ここ](https://www.coppeliarobotics.com/previousVersions)から**CoppeliaSim Edu, Ubuntu 16.04**をダウンロード
- ダウンロードしたCoppeliaSim Edu, Ubuntu 16.04を解凍して，このディレクトリに展開
- [ここ](https://github.com/stepjam/PyRep)にあるPyRepをセットアップ．セットアップ方法は[Install](https://github.com/stepjam/PyRep#install)を参考に
- quaternion用のパッケージをインストール
```shell
$ pip install numpy-quaternion
```
- セットアップできたら以下のコマンドを実行
```shell
$ python PyrepArrangementEnv.py
```
- 白いロボットが表示されて，黒い円柱上に箱を置いていたらOK
- ロボット環境のファイルは`PyrepEnv.py`なので，これとSACを組み合わせてください．

## すべての準備が完了した状態のディレクトリ構成test
- ./
  - CoppeliaSim_Edu_V4_3_0_Ubuntu20_04/
    - ...
  - PyRep
    - ...
  - README.md (this file)
  - test_env_v1.ttt
  - run_test.py
  - PyrepEnv.py

## タスクについて
- エピソードの長さは箱の数 (==4)
- 行動の次元数は4
  - 物体を持つ位置 (x) (1次元) 
  - 物体を置く位置 (x, y)とz軸周りの回転 (3次元)
- 状態の次元数は3 x 4 = 12
  - (x, y, z) x 箱の数 = 12
