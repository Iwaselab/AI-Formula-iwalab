# ROS2 trxセットアップ

## 参考
- [ROS2 JazzyJalisco : Installation Ubuntu (Debian packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

## 1.事前準備
1. udev設定ファイル作成<br>
    /etc/udev/90-aero.rulesを作成し、下記のとおり記入する。
    ```
    $ sudo nano /etc/udev/rules.d/90-aero.rules 

    【記入内容】
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="a1e8", SYMLINK+="trx_s", MODE="666"
    ```
    記入後、udev設定の適応
    ```
    $ sudo udevadm control --reload-rules
    $ sudo udevadm trigger
    ```

1. 全てのパッケージをupgradeする。
    ```
    $ sudo apt update
    $ sudo apt upgrade
    ```
2. pip3インストール
    ```terminal
    $ sudo apt install python3-pip
    ```

## 2. ROS2 jazzyインストール
公式サイトの手順に従いROS2 jazzyをインストールする。
1. インストール  
[ROS2 jazzy : Installation Ubuntu (Debian packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

1. 環境設定  
[Configuring environment](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)  
※「3.1 The ROS_DOMAIN_ID variable」の設定は不要。

1. ビルドの確認  
[Using colcon to build packages](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)  
1. colcon cleanをインストール
    ```terminal
    $ sudo apt install python3-colcon-clean 
    ```
1. rosdep初期化
    ```terminal
    $ sudo rosdep init
    $ rosdep update
    ```
    ※colcon cleanをインストールすると以下のコマンドでビルドによる出力内容を削除することが出来る。<br> 
    ワークスペース以外で誤ってビルドを行った場合などビルド結果を削除するのに便利。
    ```
    $ colcon clean workspace
    ```

## 3.trxインストール
1. ワークスペース作成 
    (ワークスペース名は任意に設定することが可能だが、ここでは"ros2_ws"とする。)
    ```
    $ mkdir -p ~/ros2_ws/src
    ```
1. trxパッケージインストール
    ```terminal
    $ cd ~/ros2_ws/src
    $ git clone git@github.com:thkrrc1/trx.git
    ```
1. その他パッケージインストール
    ```
    $ sudo apt install ros-jazzy-ros2-control ros-jazzy-controller-manager
    $ sudo apt install ros-jazzy-joint-state-broadcaster 
    $ sudo apt install ros-jazzy-joint-trajectory-controller
    ```
1. rqt本体とjoint_trajectory_controllerプラグインのインストール
    ```
    $ sudo apt install ros-jazzy-rqt
    $ sudo apt install ros-jazzy-rqt-joint-trajectory-controller
    ```
1. ビルド
    ```
    $ cd ~/ros2_ws
    $ colcon build --symlink-install
    ```

## 4.実行
1. ロボットモデル起動
    ```terminal
    $ cd ~/ros2_ws
    $ source install/setup.bash
    $ ros2 launch trx bringup.launch.py
    ```
    Rviz2に下記画像のようなtrxのモデルが表示されればok.
    ![trx_model](https://github.com/user-attachments/assets/9c5e7f8b-ead5-4308-aa24-848e36354fdc)
<br><br>
1. コントローラ起動

    別ターミナルを開き、
    ```terminal
    $ ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller 
    ```
    下記画像のようにrqtのGUIが起動し、jointsのスライダーを動かして動作することが確認できればok.
    ![rqt＿GUI](https://github.com/user-attachments/assets/eb2d4a19-3fc8-461d-a316-50dc0c0b4c59)

1. スクリプト実行

    新規ターミナルを開いて以下のサービスを呼び出すことで、trxのモータードライバー(SEED MC)にあらかじめ書き込まれている動作スクリプトをROS2側から番号指定で実行できる。
    ```terminal
    $ ros2 service call /trx/run_script trx/srv/RunScript "{id: 1, script_no: 1}"
    ```
    - `id` : 実行対象のドライバのID（`urdf`の`can_id1`/`can_id2`のいずれか）
    - `script_no` : 実行したいスクリプト番号（1〜14。ドライバ側に書き込み済みのスクリプトのみ実行可能）

    ※スクリプト実行中〜完了検知までの間（最大10秒程度）、`read()`/`write()`による通常の位置制御・Rvizへの姿勢の反映は一時停止する。スクリプト終了後は、その時点のtrxの姿勢を保持するようにコントローラの目標値も自動的に更新される。

## ※諸注意
1. ID変更対応<br>
   制御するtrxのモータードライバー(SEED MC)のIDを把握した上で、ソースコード内の下記パラメータの記述を適宜修正してください。
   ```
   /trx/urdf/trx_s.urdf 内の記述
              ︙
   <ros2_control name="SeedHW" type="system">
    <hardware>
      <plugin>trx/SeedHardwareInterface</plugin>
      <param name="serial_port">/dev/trx_s</param>
      <param name="can_id">1</param>   ＜ーーーーーーーー制御するIDに合わせる（この場合は ID 1）
      <param name="controller_rate">20.0</param>
    </hardware>
              ︙
2. 依存パッケージ不具合（26/5/15 時点）<br>
   ros-jazzy-joint-trajectory-controller、ros-jazzy-joint-state-broadcasterのパッケージバージョン v4.39において、Nodeが正常に起動しない不具合を確認しております。
   構築環境にてこれらに起因する不具合を確認した場合、各パッケージ 推奨バージョン v4.36にダウングレードした上で、本パッケージを再度ビルドして動作をお試しください。

3. ros2_controlのバージョン不整合によるクラッシュ（26/7/16 時点）<br>
   apt upgrade後、ros2 launch trx bringup.launch.py 実行時にros2_control_node がクラッシュする不具合を確認いたしました。

   **対処方法**<br>
   上記のクラッシュが発生した場合は、以下の手順でワークスペース内の `hardware_interface`/`controller_manager`/`joint_limits` を、apt版と同じバージョンに更新してください。

   1. apt版 `hardware_interface` のバージョンを確認する
      ```bash
      apt-cache policy ros-jazzy-hardware-interface | head -n2
      ```
      表示された `4.45.2-1noble...` のようなバージョン番号のうち、先頭の `4.45.2` の部分を控える

   2. 本家 `ros-controls/ros2_control` から、同じバージョンのタグを取得する
      ```bash
      git clone --depth 1 --branch <控えたバージョン番号> https://github.com/ros-controls/ros2_control.git ros2_control_latest
      ```

   3. ワークスペースの3パッケージを、取得した内容で丸ごと置き換える
      ```bash
      cd ~/ros2_ws/src/seed_robot_ros2_pkg/controller/ros2_control
      for pkg in hardware_interface controller_manager joint_limits; do
        rm -rf "$pkg"
        cp -r "$HOME/ros2_control_latest/$pkg" "$pkg"
      done
      ```

   4. `delegate()` を新バージョンに手動で再度追加する
      - `hardware_interface/include/hardware_interface/handle.hpp` の `StateInterface` クラス内に追記
        ```cpp
        template<class T>
        void delegate(T func) const{
          func(get_prefix_name(), value_ptr_);
        }
        ```
      - `hardware_interface/include/hardware_interface/loaned_state_interface.hpp` の `LoanedStateInterface` クラス内に追記
        ```cpp
        template<class T>
        void delegate(T func) {
          state_interface_.delegate(func);
        }
        ```
   5. クリーンビルドする
      ```bash
      colcon clean workspace
      source /opt/ros/jazzy/setup.bash
      cd ~/ros2_ws
      colcon build --symlink-install
      source install/setup.bash
      ```
   6. 起動確認する
      ```bash
      ros2 launch trx bringup.launch.py
      ```
      <br><br>


以上
