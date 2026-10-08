# P1-T1 環境建置

日期：2026-10-08

## 做了什麼

- 以 `docker-compose.yml` 啟動 `tiryoh/ros2-desktop-vnc:jazzy`，主機的 `ros2_ws/src` 掛載到容器的 `/home/ubuntu/ros2_ws/src`。
- 瀏覽器開 http://localhost:6080 進入 noVNC 桌面，執行 turtlesim 與 teleop。

## 驗收

在 noVNC 桌面開兩個終端機：

```bash
ros2 run turtlesim turtlesim_node      # 終端機 1：出現烏龜視窗
ros2 run turtlesim turtle_teleop_key   # 終端機 2：焦點放在這裡，用方向鍵控制
```

預期：烏龜隨方向鍵移動。

## 學到的概念

- **workspace 的建置**：`colcon build` 要在 workspace 根目錄執行，會產生 `build/`、`install/`、`log/`。
  `--symlink-install` 讓 `install/` 以符號連結指回 `src/`，改 Python 檔不必重新 build；
  新增檔案或改 `setup.py`、`package.xml` 時仍要 build。
- **underlay 與 overlay**：`/opt/ros/jazzy/setup.bash` 載入 ROS 本體（映像的 `.bashrc` 已自動執行），
  `install/setup.bash` 把自己的 workspace 疊上去。後者每個新終端機都要 `source` 一次。
- **`source` 與直接執行的差別**：直接執行會開子 shell，環境變數設完就消失；`source` 才會留在目前的 shell。

## 卡關點

| 現象 | 原因 | 解法 |
|---|---|---|
| `docker compose up -d` 回報 `empty compose file` | `docker-compose.yml` 是空檔案，內容沒存進去 | 補上內容；可先用 `docker compose config` 檢查 |
| `echo $ROS_DISTRO` 是空值 | 指令打在容器外的 WSL 終端機 | 看提示字元：`ubuntu@<容器 ID>` 才是在容器內 |
| `docker exec -it ros2_jazzy bash` 進去是 `root` | 映像的 `docker exec` 預設使用者是 `root` | 加 `-u ubuntu`，否則找不到 `~/ros2_ws`，build 出來的檔案擁有者也會是 root |
| `qt.qpa.xcb: could not connect to display` | `docker exec` 的終端機沒有 `DISPLAY` | 在 noVNC 桌面的終端機執行，或先 `export DISPLAY=:1` |
| noVNC 要求密碼 | 映像預設帳號與密碼都是 `ubuntu` | 輸入 `ubuntu`；容器內 `sudo` 也是這組 |

## 與 Unity 的類比

- `colcon build` 接近編譯 assembly；`--symlink-install` 像是腳本改了不用重新編譯。
- `source install/setup.bash` 沒有直接對應，作用是告訴這個終端機去哪裡找剛 build 好的套件。

## 參考

- https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
- https://github.com/Tiryoh/docker-ros2-desktop-vnc
