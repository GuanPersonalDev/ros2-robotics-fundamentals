# ROS 2 + Python 機器人開發基礎學習計畫

> 本文件是給「學習者本人」與「協助實作的 AI Agent」共同使用的計畫書。
> Agent 在開始任何任務前，請先完整閱讀「給 Agent 的協作規則」與「環境規格」兩節。

---

## 1. 背景與目標

- 學習者：Unity 工程師（約 6 年經驗，熟悉 shader/HLSL、效能最佳化、ECS），機械工程背景。
- 長期目標：轉職數位孿生／Omniverse 相關職位，已有 Isaac Sim 數位孿生作品。
- 本計畫目標：在**不安裝 Omniverse** 的環境下，系統性補齊 ROS 2 + Python 的機器人開發基礎，最終能把同一套 ROS 2 控制程式接到 Gazebo 與 Isaac Sim。
- 學習主軸：以 **docs.ros.org 官方教學** 為骨幹，每個階段搭配自訂練習與可展示的成果。

---

## 2. 給 Agent 的協作規則

1. **以學習為優先**：這是學習計畫，不是交付專案。預設先說明概念、給提示或骨架，讓學習者自己實作；學習者明確要求時才給完整解答。
2. **官方文件優先**：解釋概念或做法時，優先引用 docs.ros.org（Jazzy 版）對應頁面，其次才是社群做法。
3. **語言**：說明一律使用繁體中文；程式碼註解使用繁體中文。
4. **Python 命名**：遵循 PEP 8（模組／函式／變數 `snake_case`，類別 `PascalCase`，常數 `UPPER_SNAKE_CASE`，內部成員以 `_` 開頭）。
5. **一次一個任務**：依照 `TASKS.md` 的任務編號（如 `P1-T3`）推進，完成後更新 `TASKS.md` 的勾選狀態與「進度紀錄」。
6. **驗收要可重現**：每個任務都要提供可在容器內執行的驗證指令（`ros2 topic echo`、`ros2 node list` 等）。
7. **不要擅自升級版本**：ROS 2 發行版固定為 Jazzy，除非學習者要求變更。

---

## 3. 環境規格

| 項目 | 規格 |
|---|---|
| ROS 2 發行版 | Jazzy Jalisco（LTS）。概念與最新 LTS Lyrical Luth 幾乎相同，但周邊生態（Nav2、MoveIt 2、Gazebo、Isaac Sim ROS 2 Bridge）較成熟 |
| 執行方式 | Docker + noVNC，瀏覽器連線操作 GUI |
| 映像 | `tiryoh/ros2-desktop-vnc:jazzy`（支援 amd64 / arm64） |
| GUI 存取 | http://localhost:6080 |
| 程式碼位置 | 主機 `./ros2_ws/src` 掛載至容器內 workspace 的 `src` |
| Python | 使用映像內建版本（Ubuntu 24.04 對應版本） |

### 3.1 docker-compose.yml

```yaml
services:
  ros2:
    image: tiryoh/ros2-desktop-vnc:jazzy
    container_name: ros2_jazzy
    ports:
      - "6080:80"          # 瀏覽器開 http://localhost:6080
    shm_size: "512m"       # RViz2 等 GUI 需要較大的共享記憶體
    security_opt:
      - seccomp:unconfined
    volumes:
      - ./ros2_ws/src:/home/ubuntu/ros2_ws/src   # 主機上編輯，容器內編譯
    restart: unless-stopped
```

> 若容器內出現權限問題，先執行 `whoami`、`echo $HOME` 確認預設使用者與家目錄，再調整掛載路徑。

### 3.2 常用指令

```bash
docker compose up -d                 # 啟動
docker compose down                  # 停止
docker exec -it ros2_jazzy bash      # 從主機進入容器終端機

cd ~/ros2_ws
colcon build --symlink-install       # Python 套件修改後免重新 build（新增檔案或改 setup.py 時仍需 build）
source install/setup.bash            # 每個新終端機都要執行
```

### 3.3 已知限制

- 容器內沒有 GPU 加速：Phase 0–2 完全足夠；Phase 3 的 Gazebo 可能偏慢，屆時再評估改用 Linux + GPU 機器或 Isaac Sim。

---

## 4. 建議的 Repository 結構

```
ros2-robotics-fundamentals/
├── README.md                  # 學習歷程總覽（作品集用）
├── docker-compose.yml
├── plan.md                    # 本文件
├── TASKS.md                   # 任務清單、完成定義、進度紀錄
├── phase0_math/               # 純 Python，不依賴 ROS
│   ├── transforms.py
│   ├── two_link_arm.py
│   ├── quaternion_utils.py
│   └── tests/
├── ros2_ws/
│   └── src/
│       ├── two_link_fk/       # Phase 1 套件
│       ├── arm_description/   # Phase 2 URDF 套件
│       └── ...
└── notes/                     # 每個任務的學習筆記
```

---

## 5. 學習階段

各階段的任務、驗收條件、完成定義與進度紀錄都寫在 `TASKS.md`，勾選狀態只在那裡更新。
這一節只留各階段的範圍與參考資料。

### Phase 0：機器人數學（純 Python，約 1–2 週）

不需要 ROS，只需 Python + NumPy + matplotlib（可在容器內或主機執行）。

參考資料：Robotics Toolbox for Python（`roboticstoolbox-python`，對答案用）、Kevin Lynch《Modern Robotics》。

### Phase 1：ROS 2 核心概念（約 2–3 週）

官方教學入口：https://docs.ros.org/en/jazzy/Tutorials.html

Unity 工程師的心智模型提醒：ROS 是「多個獨立行程透過訊息溝通」，Node ≈ 獨立小程式，Topic ≈ 跨行程 event bus，`rclpy.spin()` 是事件驅動而非每幀呼叫。

### Phase 2：中階實務（約 3–4 週）

Launch、TF2、URDF、Actions、Executors 與 QoS。官方教學連結見第 6 節。

### Phase 3：模擬與整合（約 4 週以上）

| 任務 | 參考 |
|---|---|
| P3-T1 Gazebo | https://gazebosim.org/docs/harmonic/ros2_integration |
| P3-T2 Nav2 | https://docs.nav2.org/getting_started/index.html |
| P3-T3 MoveIt 2 | https://moveit.picknik.ai/main/doc/tutorials/tutorials.html |

P3-T4 的 Isaac Sim 整合需在可用 Omniverse 的環境進行。

---

## 6. 官方文件索引

| 主題 | 連結 |
|---|---|
| Jazzy 教學總覽 | https://docs.ros.org/en/jazzy/Tutorials.html |
| Beginner: CLI tools | https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html |
| Beginner: Client libraries | https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries.html |
| Intermediate | https://docs.ros.org/en/jazzy/Tutorials/Intermediate.html |
| TF2 | https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html |
| URDF | https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html |
| ROS 2 + VSCode + Docker | https://docs.ros.org/en/jazzy/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html |
| noVNC Docker 映像 | https://github.com/Tiryoh/docker-ros2-desktop-vnc |

> 若連結失效，請 Agent 先到 docs.ros.org 首頁切換至 Jazzy 版本搜尋同名頁面。
