# formation_offboard

PX4 ROS2 Leader-Follower 編隊 offboard control package。
每架機一台 RPi + 一顆飛控 (UART 連接)，透過 uXRCE-DDS 橋接。

## 目錄結構
```
formation_offboard/
├── CMakeLists.txt
├── package.xml
├── src/formation_offboard_control.cpp   # node 原始碼
├── launch/single_node.launch.py         # 實機部署用，每台 RPi 各跑一份
├── params/
│   ├── leader.yaml
│   ├── follower_left.yaml
│   └── follower_right.yaml
└── systemd_examples/                    # 開機自動啟動範例
    ├── micro-xrce-agent.service
    └── formation-offboard.service
```

## 1. 在 RPi 上準備 workspace

假設 ROS2 (Humble/Jazzy，依你的版本) 已裝好：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 1.1 取得 px4_msgs（版本要對應你的 PX4 韌體！）

```bash
git clone https://github.com/PX4/px4_msgs.git -b <對應你 PX4 韌體版本的 branch/tag>
```

> 這一步最容易出錯：如果 px4_msgs 版本跟飛控韌體版本不一致，
> uORB 訊息欄位對不上，會出現 subscribe 收不到資料、或欄位值錯亂。
> 版本對照請看 px4_msgs repo 的 README 或直接用跟你飛控 release 相同 tag。

### 1.2 放入這個 package

把 `formation_offboard/` 整個資料夾複製到 `~/ros2_ws/src/` 底下。

## 2. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

（可以先 `colcon build --packages-select px4_msgs` 讓它獨立建好，再建 `formation_offboard`，
   RPi 效能有限時建議加 `--parallel-workers 1` 避免記憶體不足被 OOM kill。）

建議把 `source ~/ros2_ws/install/setup.bash` 加進 `~/.bashrc`。

## 3. 啟動 uXRCE-DDS Agent（橋接飛控 <-> ROS2）

先確認 PX4 這邊的 `UXRCE_DDS_CFG` 參數指到你接線的那個 serial port
（例如 TELEM2），並確認 baudrate 跟下面指令一致：

```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```

看到持續有 "session established" / topic 訂閱建立的訊息，代表橋接成功。
之後可以用 `ros2 topic list` 確認有沒有看到 `/MAV1/fmu/out/vehicle_local_position` 之類的 topic。

> 注意：這個 agent 目前是「全域」的 —— 沒有加 MAV 前綴。
> 如果每台 RPi 只接一顆飛控，這樣沒問題；但如果你未來想在一台 RPi 上同時接多顆飛控，
> 就要另外用 `-n`/namespace 參數幫 agent 分流，這裡先不展開。

## 4. 啟動 formation node

在 **Leader 的 RPi** 上：
```bash
ros2 launch formation_offboard single_node.launch.py \
    params_file:=$HOME/ros2_ws/src/formation_offboard/params/leader.yaml
```

在 **Follower 的 RPi** 上（各自帶自己的 yaml）：
```bash
ros2 launch formation_offboard single_node.launch.py \
    params_file:=$HOME/ros2_ws/src/formation_offboard/params/follower_left.yaml
```

## 5. 讓多台 RPi 互相看到彼此的 topic

Follower 需要訂閱到 Leader RPi 發布的 `/MAV1/fmu/out/vehicle_local_position`，
所以幾台 RPi 必須在同一個 ROS2 DDS 網域裡：

- 全部設定同一個 `ROS_DOMAIN_ID`（例如都設 0，前提是同網路沒有別的 ROS2 系統在用這個 ID）。
- Wi-Fi 路由器如果會擋 multicast（很多消費級路由器預設會擋），預設的 Fast DDS/CycloneDDS
  探索機制可能連不起來。這種情況建議：
  - 換一台支援 multicast 的路由器/AP，或
  - 改用 **Fast DDS Discovery Server** 模式（單一固定 IP 當探索伺服器），避免依賴 multicast。
- 先用簡單方式驗證：兩台 RPi 都 `source install/setup.bash` 後，
  在其中一台跑 `ros2 topic list`，看能不能看到另一台發布的 topic。

## 6. 開機自動啟動（選用）

`systemd_examples/` 裡有兩個範例：
- `micro-xrce-agent.service`：開機自動啟動 agent。
- `formation-offboard.service`：開機自動啟動 node（**每台 RPi 記得改 ExecStart 裡的 params_file 路徑**，
  leader 的機器指到 `leader.yaml`，follower 的機器指到對應的 yaml）。

安裝方式：
```bash
sudo cp systemd_examples/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now micro-xrce-agent.service
sudo systemctl enable --now formation-offboard.service
```

## 建議的測試順序

1. **先只開 Leader**,確認它自己能不能單機 arm + offboard + 飛到目標高度(先在 SITL 或繫留狀態測試,不要一開始就用實機自由飛)。
2. **再開一台 Follower**，確認它能收到 leader 的 `vehicle_local_position`（用 `ros2 topic echo` 檢查），且會照 offset 算出正確目標點。
3. 都正常後，才把整個機隊一起開起來實際試飛，並且**務必先在有安全繩/低高度/開闊場地**測試，
   再逐步拉高高度跟複雜度。

## 分階段測試流程（懸停 → 編隊，MAV1+2 → 加入 MAV3）

新版程式碼支援「先待命、統一 start、原地懸停、之後才切編隊模式」的流程，對應你的測試步驟：

### 階段 A：MAV1 + MAV2，純懸停測試（不追蹤 leader）

1. 三份 params 都已經把 `follow_leader` 設好（follower 預設 `false`）。
2. 在 MAV1 的 RPi 上：
   ```bash
   ros2 launch formation_offboard single_node.launch.py params_file:=$HOME/ros2_ws/src/formation_offboard/params/leader.yaml
   ```
3. 在 MAV2 的 RPi 上：
   ```bash
   ros2 launch formation_offboard single_node.launch.py params_file:=$HOME/ros2_ws/src/formation_offboard/params/follower_left.yaml
   ```
4. 兩邊都會印出「waiting for start signal on /formation/start」，此時完全不會發送任何 offboard 指令，飛機不會動。
5. **確認兩台的地面站/log都已經 ready** 後，在任一台（或你的地面電腦，只要跟它們同一個 ROS2 網域）發 start：
   ```bash
   ros2 topic pub /formation/start std_msgs/msg/Bool "data: true" --once
   ```
6. 兩架機會**各自**用「收到 start 當下的位置」當 home，原地起飛爬升到 0.5m 並懸停（此階段 MAV2 不會管 MAV1 在哪裡，兩台各自獨立懸停）。
7. 驗證：兩台高度、位置是否穩定，log 裡有沒有出現「Offboard engaged and armed. Will NOT force mode again」——出現這行代表之後可以安全交給遙控器。

### 階段 B：MAV1 + MAV2，切換成編隊模式

懸停確認沒問題後，把 MAV2 切換成追蹤 leader，**不需要重開 node**：
```bash
ros2 param set /formation_offboard_control follow_leader true
```
（如果 ROS2 節點名稱前面有 namespace，記得補上實際的節點路徑，可以先用 `ros2 node list` 確認。）

之後 MAV2 就會即時用 `leader 位置 + offset` 計算目標點，此時如果遙控器切 MAV1 的模式離開 Offboard 手動飛，MAV2 依然會照 MAV1 的實際位置跟隨（因為追蹤的是 telemetry，不是 offboard 指令）。

### 階段 C：加入 MAV3

確認 MAV1+MAV2 的懸停與編隊都正常後，比照上面步驟在 MAV3 的 RPi 上啟動（`follower_right.yaml`），一樣先用 `follow_leader: false` 做懸停測試，確認沒問題後再 `ros2 param set ... follow_leader true` 切進編隊。

### 交給遙控器控制 MAV1

MAV1 一旦 log 出現「Offboard engaged and armed. Will NOT force mode again」，代表我們的 node 不會再主動搶 Offboard 模式，這時候你可以直接用遙控器切換飛行模式手動接管 MAV1，程式不會跟遙控器打架。Follower 的追蹤邏輯不受影響，因為它讀的是 MAV1 的實際飛行 telemetry，不管 MAV1 當下是 Offboard 還是被遙控器手動飛行都一樣有效。

> ⚠️ 注意：node 仍會持續發布 `TrajectorySetpoint`（PX4 offboard 模式需要這個持續的 stream，不然逾時會自動跳出 offboard），但只要 MAV1 不在 Offboard nav_state，PX4 會忽略這些 setpoint，不影響遙控器的手動控制權。
