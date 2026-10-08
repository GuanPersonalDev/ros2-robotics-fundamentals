# P0-T2 兩軸平面手臂正向運動學（FK）

日期：2026-10-08

## 做了什麼

- `phase0_math/two_link_arm.py`：`forward_kinematics(theta1, theta2, l1, l2)`，
  輸入兩個關節角與兩根連桿長度，回傳末端在基座座標系的 `[x, y]`。
- `phase0_math/tests/test_two_link_arm.py`：11 個 pytest 測試。

## 驗收

在專案根目錄執行（WSL 用 `python3`，Windows 用 `python`）：

```bash
python3 -m pytest phase0_math -v
```

預期：最後一行是 `30 passed`（P0-T1 的 19 個加上這次的 11 個）。對應任務驗收條件的測試是
`TestForwardKinematics::test_known_angles_match_hand_calculation`，以 `l1 = 1.0`、`l2 = 0.5`
比對下表七組手算結果：

| θ1 | θ2 | 末端 (x, y) | 代入 l1 = 1.0、l2 = 0.5 |
|---|---|---|---|
| 0 | 0 | (l1 + l2, 0) | (1.5, 0) |
| π/2 | 0 | (0, l1 + l2) | (0, 1.5) |
| π | 0 | (−(l1 + l2), 0) | (−1.5, 0) |
| 0 | π/2 | (l1, l2) | (1.0, 0.5) |
| 0 | π | (l1 − l2, 0) | (0.5, 0) |
| π/2 | π/2 | (−l2, l1) | (−0.5, 1.0) |
| π/2 | −π/2 | (l2, l1) | (0.5, 1.0) |

## 學到的概念

- **正向運動學（FK）**：已知關節角，求末端位置。答案唯一，直接代公式就有；
  反過來由末端位置求關節角是逆向運動學（IK），可能有多組解或無解。
- **θ2 是相對角**：θ2 量的是連桿 2 相對於連桿 1 的夾角，不是相對於基座 x 軸。
  所以連桿 2 在基座座標系的方向是 `θ1 + θ2`。ROS 的 `/joint_states` 與 URDF 的關節角也是相對角。
- **封閉解**：先算肘部，再從肘部沿連桿 2 的方向走 `l2`。
  - 肘部：`(l1·cos θ1, l1·sin θ1)`
  - 末端：`x = l1·cos θ1 + l2·cos(θ1 + θ2)`、`y = l1·sin θ1 + l2·sin(θ1 + θ2)`
- **同一件事可以用轉換鏈表示**：`T_0e = T_01 @ T_12 @ T_2e`，其中 `T_01` 是繞 z 轉 θ1，
  `T_12` 是沿連桿 1 平移 `l1` 後繞 z 轉 θ2，`T_2e` 是沿連桿 2 平移 `l2`。
  `T_0e` 的平移欄就是末端位置。兩軸時封閉解比較短；關節變多時轉換鏈只是多乘幾個矩陣，比較好擴充。
- **末端到基座的距離只跟 θ2 有關**：`|p|² = l1² + l2² + 2·l1·l2·cos θ2`（餘弦定理）。
  θ1 只會讓整隻手臂繞基座轉，不改變這個距離。θ2 = 0 時手臂伸直，距離最大為 `l1 + l2`；
  θ2 = π 時完全折回，距離最小為 `|l1 − l2|`。這兩個值就是工作範圍的外圈與內圈，P0-T3 的 IK 會用到。
- **測試的連桿長度要取不相等的值**：`l1 == l2` 時，把兩個長度寫反的錯誤測不出來。
- **測試要有獨立的對照來源**：實作用封閉解，測試就用 P0-T1 的轉換鏈算一次來比，
  再加上不經過程式的手算值。拿同一條公式比自己，永遠會過。

## 卡關點

| 現象 | 原因 | 解法 |
|---|---|---|
| pytest 回報 `ModuleNotFoundError: No module named 'phase0_math.two_link_arm'` | 檔名打成 `two_link.arm.py`；句點在 `import` 裡是套件分隔符號，這個檔案無法被當成模組匯入 | 檔名改為 `two_link_arm.py`，模組檔名只用小寫字母、數字與底線 |

## 與 Unity 的類比

- 兩軸手臂就是兩層父子 `Transform`：基座 → 連桿 1 → 連桿 2，末端是連桿 2 底下的一個子物件。
  FK 做的事等於讀那個子物件的 `transform.position`，Unity 是沿階層把 `localToWorldMatrix` 逐層乘起來。
- 關節角對應 `transform.localEulerAngles.z`（相對於父物件），不是 `transform.eulerAngles.z`（相對於世界）。
  θ2 是相對角，和 local 與 world 的區別是同一件事。
- 連桿長度對應子物件的 `localPosition.x`：子物件放在父物件的 x 軸上、距離為連桿長度。
- 差異：Unity 用角度，這裡用弧度。

## 參考

- Kevin Lynch《Modern Robotics》第 4 章
- `notes/p0-t1-homogeneous-transforms.md`
