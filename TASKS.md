# 任務清單

ROS 2 + Python 機器人開發基礎的學習任務，依執行順序排列。

> **給 Agent**：開始任何任務前先讀這份清單，找出第一個未打勾的任務，一次只推進一個。
> 協作規則、環境規格（Docker、ROS 2 Jazzy）與官方文件索引寫在 `plan.md`。

## 完成定義

每個任務都要符合以下四項才算完成：

1. 程式可在容器內 `colcon build` 成功並執行（Phase 0 為純 Python，以 pytest 通過為準）。
2. 附上驗收指令與預期輸出。
3. `notes/` 下有一份簡短筆記：學到的概念、卡關點、與 Unity 的類比（若有）。
4. 本清單對應的勾選框已打勾，並在「進度紀錄」新增一行。

---

## Phase 0：機器人數學（純 Python，約 1–2 週）

不需要 ROS，只需 Python + NumPy + matplotlib。程式放在 `phase0_math/`。

- [x] **P0-T1 齊次轉換矩陣**
  - 內容：實作 2D / 3D 旋轉矩陣與 4x4 齊次轉換矩陣的建構函式。
  - 驗收：以 pytest 驗證 `T_ab @ T_bc == T_ac`、`T @ inv(T) == I`。
- [x] **P0-T2 兩軸平面手臂正向運動學（FK）**
  - 內容：輸入兩關節角與連桿長度，輸出末端座標。
  - 驗收：數個已知角度（0、π/2、π）的手算結果與程式一致。
- [ ] **P0-T3 兩軸手臂逆向運動學（IK，解析解）**
  - 內容：處理 elbow-up / elbow-down 兩組解，以及目標超出工作範圍的情況。
  - 驗收：`fk(ik(p)) ≈ p`；超出範圍時回傳明確錯誤。
- [ ] **P0-T4 四元數與尤拉角**
  - 內容：實作四元數 ↔ 尤拉角互轉、四元數乘法，並寫一段說明萬向鎖的筆記。
  - 驗收：與 `scipy.spatial.transform.Rotation` 結果比對一致。
- [ ] **P0-T5 視覺化**
  - 內容：用 matplotlib 畫出手臂姿態，以 slider 拖動關節角即時更新。
  - 驗收：拖動 slider 時手臂姿態即時更新。

## Phase 1：ROS 2 核心概念（約 2–3 週）

- [x] **P1-T1 環境建置**
  - 內容：啟動 Docker 容器，於 noVNC 桌面執行 turtlesim 與 teleop。
  - 驗收：可用鍵盤控制烏龜。
- [ ] **P1-T2 Beginner: CLI tools（官方教學）**
  - 內容：完成全部 CLI 教學：Node、Topic、Service、Parameter、Action、`rqt`、`ros2 launch`、`ros2 bag`。
  - 驗收：在 `notes/` 留下各概念的一句話定義，以及最常用的 CLI 指令速查表。
- [ ] **P1-T3 Beginner: Client libraries（官方教學，Python 部分）**
  - 內容：建立 workspace、Python 套件、Publisher/Subscriber、Service/Client、自訂 msg/srv、Parameter。
  - 驗收：各教學的範例 Node 可在容器內 build 並執行。
- [ ] **P1-T4 自訂練習：two_link_fk 套件**
  - 內容：把 P0-T2 的 FK 包成 ROS 2 Node：訂閱 `/joint_states`（`sensor_msgs/JointState`），
    發布 `/end_effector`（`geometry_msgs/Point`）。連桿長度以 Parameter 宣告，可用 `ros2 param set` 動態修改。
  - 建立套件：
    ```bash
    ros2 pkg create --build-type ament_python --license Apache-2.0 \
      two_link_fk --dependencies rclpy sensor_msgs geometry_msgs
    ```
  - 驗收：以下指令的輸出與 Phase 0 純 Python 版本一致。
    ```bash
    ros2 topic echo /end_effector
    ros2 topic pub --once /joint_states sensor_msgs/msg/JointState \
      "{name: ['joint1', 'joint2'], position: [0.5, 0.3]}"
    ```
- [ ] **P1-T5 自訂練習：IK Service**
  - 內容：把 P0-T3 包成 Service：輸入目標點，回傳關節角（需自訂 srv）。
  - 驗收：`ros2 service call` 能取得正確解；超出範圍時回傳失敗狀態。

## Phase 2：中階實務（約 3–4 週）

- [ ] **P2-T1 Launch files（官方 Intermediate → Launch）**
  - 內容：用 Python launch 一次啟動 P1 的 FK Node 與 IK Service，並透過 launch 傳入 Parameter。
  - 驗收：單一 `ros2 launch` 指令啟動後，`ros2 node list` 可看到兩個 Node，Parameter 值與 launch 傳入的一致。
- [ ] **P2-T2 TF2（官方 TF2 tutorials）**
  - 內容：完成 static broadcaster、broadcaster、listener 教學。
    自訂練習：FK Node 改為發布 TF（`base_link → link1 → link2 → end_effector`）。
  - 驗收：RViz2 中看到座標系隨關節角移動。
- [ ] **P2-T3 URDF（官方 URDF tutorials）**
  - 內容：建立 `arm_description` 套件，撰寫兩軸手臂 URDF（可再擴充為 xacro）。
    用 `robot_state_publisher` + `joint_state_publisher_gui` 在 RViz2 拖動關節。
  - 驗收：URDF 顯示的末端位置與 FK Node 計算結果一致。
- [ ] **P2-T4 Actions**
  - 內容：實作「移動到目標點」Action：以 IK 求解後，逐步插值發布 `/joint_states`，回報 feedback（進度）並可取消。
  - 驗收：`ros2 action send_goal --feedback` 可看到進度回報，取消後停止發布。
- [ ] **P2-T5 Executors 與 Callback groups**
  - 內容：閱讀官方 Executors 說明，實驗 SingleThreadedExecutor 與 MultiThreadedExecutor 在長時間 callback 下的行為差異。
  - 驗收：`notes/` 下有記錄實驗做法與行為差異的筆記。
- [ ] **P2-T6 QoS**
  - 內容：實驗 Reliable vs Best Effort、Durability（Transient Local）對訂閱者的影響。
  - 驗收：`notes/` 下有記錄實驗做法與觀察結果的筆記。

## Phase 3：模擬與整合（約 4 週以上）

- [ ] **P3-T1 Gazebo（Jazzy 搭配 Gazebo Harmonic）**
  - 內容：把 P2-T3 的 URDF 加入物理與碰撞設定，放進 Gazebo。
    用 `ros_gz_bridge` 把關節狀態與控制指令接上 ROS 2。
  - 驗收：從 ROS 2 發出控制指令可驅動 Gazebo 內的手臂，並能在 ROS 2 端讀到關節狀態。
- [ ] **P3-T2 Nav2 入門**
  - 內容：依官方 Getting Started 跑 TurtleBot3 導航範例，理解 map / odom / base_link 的 TF 關係。
  - 驗收：範例可完成一次導航，`notes/` 下有說明三個座標系關係的筆記。
- [ ] **P3-T3 MoveIt 2 入門**
  - 內容：依官方教學完成手臂運動規劃範例。
  - 驗收：範例可規劃並執行一段手臂運動。
- [ ] **P3-T4 Isaac Sim 整合（作品集收尾，需在可用 Omniverse 的環境進行）**
  - 內容：將同一份 URDF 匯入 Isaac Sim，透過 ROS 2 Bridge 接上 Phase 1–2 的 Node。
  - 驗收：完成「同一套 ROS 2 控制程式，在 Gazebo 與 Isaac Sim 都能運作」的展示影片與 README。

---

## 進度紀錄

| 日期 | 任務 | 備註 |
|---|---|---|
| 2026-10-08 | P1-T1 | 容器與 noVNC 可用，turtlesim 可用鍵盤控制；筆記見 `notes/p1-t1-environment-setup.md` |
| 2026-10-08 | P0-T1 | 旋轉矩陣與 4x4 齊次轉換完成，pytest 19 個測試通過；筆記見 `notes/p0-t1-homogeneous-transforms.md` |
| 2026-10-08 | P0-T2 | 兩軸平面手臂 FK 完成，pytest 30 個測試通過；筆記見 `notes/p0-t2-two-link-fk.md` |
