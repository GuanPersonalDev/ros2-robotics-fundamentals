# P0-T4 四元數與尤拉角

日期：2026-10-12

## 做了什麼

- `phase0_math/quaternion_utils.py`：
  - `quat_multiply(q1, q2)`：四元數乘法（Hamilton 乘積）。
  - `euler_to_quat(roll, pitch, yaw)`：尤拉角轉四元數。
  - `quat_to_euler(quat)`：四元數轉尤拉角，回傳 `(roll, pitch, yaw)`；pitch = ±π/2（萬向鎖）時另外處理。
- `phase0_math/tests/test_quaternion_utils.py`：33 個 pytest 測試案例（12 個測試函式，參數化展開），
  以 `scipy.spatial.transform.Rotation` 為主要對照來源，另外用 P0-T1 的旋轉矩陣做獨立對照。

採用的慣例（與 ROS 一致）：

- 四元數順序是 `(x, y, z, w)`，實部在最後，與 `geometry_msgs/Quaternion`、scipy 的 `as_quat()` 相同。
- 尤拉角是 REP 103 的固定軸 roll-pitch-yaw：依序繞固定的 X、Y、Z 軸旋轉，
  等於 `R = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)`。
- 輸入的四元數假設是單位四元數。

## 驗收

在專案根目錄執行：

```bash
python3 -m pytest phase0_math -v
```

預期：最後一行是 `73 passed`（P0-T1 ～ P0-T3 的 40 個 + 這次的 33 個）。
需要 `scipy`；缺的話在容器內用 `sudo apt install python3-scipy` 安裝。

對應驗收條件「與 scipy 結果比對一致」的測試：

| 函式 | 測試 | scipy 的對照 |
|---|---|---|
| `euler_to_quat` | `TestEulerToQuat::test_general_angles_match_scipy` | `Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat()` |
| `quat_to_euler` | `TestQuatToEuler::test_general_angles_match_scipy` | `as_euler('xyz')` |
| `quat_multiply` | `TestQuatMultiply::test_different_axes_match_scipy` | `(r1 * r2).as_quat()` |

## 學到的概念

- **四元數用四個數表示一個旋轉**：繞單位軸 `n` 轉 θ 的四元數是 `(sin(θ/2)·n, cos(θ/2))`。
  角度是半角，所以 `q` 和 `−q` 是同一個旋轉——比對兩個四元數是否代表同一個旋轉時，
  不能直接比四個數，要比內積的絕對值是否為 1。
- **四元數乘法就是旋轉的合成，而且不可交換**：把四元數拆成實部 `w` 和向量部 `v`，
  乘積是 `(w1·w2 − v1·v2, w1·v2 + w2·v1 + v1×v2)`。`q1 ⊗ q2` 表示先做 `q2` 再做 `q1`，
  順序和矩陣相乘一樣。裡面有外積，所以交換順序結果不同。
- **尤拉角轉四元數是三個單軸四元數相乘**：`q = q_z(yaw) ⊗ q_y(pitch) ⊗ q_x(roll)`，
  順序和 `rot_z @ rot_y @ rot_x` 相同。程式裡的四條式子就是這個乘積的展開。
- **尤拉角一定要講清楚軸序和固定軸／自身軸**：同樣三個數字，慣例不同就是不同的旋轉。
  scipy 用大小寫區分：小寫 `'xyz'` 是固定軸（REP 103 用這個），大寫 `'XYZ'` 是繞自身軸，兩者結果不同。
- **四元數轉尤拉角是從旋轉矩陣的元素反推**：把 `rot_z @ rot_y @ rot_x` 展開後，
  `R[2,0] = −sin(pitch)` 只跟 pitch 有關，roll 和 yaw 各由一組 `atan2` 決定；
  再把這些矩陣元素換成四元數的表示式。
- **測試要有獨立的對照來源**（延續 P0-T2、P0-T3）：除了 scipy，還拿 P0-T1 自己寫的
  `rot_z @ rot_y @ rot_x` 比對，等於同時驗證了這次採用的軸序慣例和 P0-T1 一致。
- **乘法測試要取不同軸的兩個旋轉**：同軸旋轉可交換，順序寫反測不出來，
  道理和 P0-T2 的連桿長度要取不相等的值一樣。

## 萬向鎖

**現象**：pitch = ±π/2 時，三個尤拉角只剩兩個自由度。

**原因**：固定軸 roll-pitch-yaw 是先繞 X、再繞 Y、最後繞 Z。pitch 轉了 90° 之後，
原本沿 X 軸的方向被轉到 Z 軸上（pitch = +π/2 時是 −Z，−π/2 時是 +Z），
所以第一步「繞 X 轉 roll」和第三步「繞 Z 轉 yaw」變成繞同一條軸轉，效果可以互相抵換。

**從公式看**：把 pitch = π/2 代進 `euler_to_quat`，四個分量變成

```
x =  (1/√2)·sin((roll − yaw)/2)
y =  (1/√2)·cos((roll − yaw)/2)
z = −x
w =  y
```

四元數裡只出現 `roll − yaw`。也就是說 `(0.4, π/2, 0.9)` 和 `(0.0, π/2, 0.5)` 是同一個旋轉，
`roll` 和 `yaw` 各自是多少已經無法還原。pitch = −π/2 時同理，只是出現的是 `roll + yaw`。

**程式怎麼處理**：`quat_to_euler` 偵測到 `|sin(pitch)|` 接近 1 時，把 `yaw` 定成 0，
剩下的轉角全部放進 `roll`。這只是一種選法（scipy 也是把其中一個角設為 0 並發出警告），
回傳的尤拉角不一定等於當初輸入的那一組，但代表的旋轉相同。

**為什麼機器人系統用四元數**：萬向鎖是尤拉角這種表示法的問題，不是旋轉本身的問題。
四元數沒有這個奇異點，內插也比較平順，所以 ROS 的 `geometry_msgs` 和 tf2 都用四元數傳遞姿態；
尤拉角只留給人看和人輸入。

## 卡關點

| 現象 | 原因 | 解法 |
|---|---|---|
| 萬向鎖的兩個測試失敗：`(0.4, π/2, 0.9)` 轉成四元數再轉回來變成 `roll = 0, yaw = 0`，轉回去已經不是同一個旋轉 | pitch = ±π/2 時，一般公式裡 roll 和 yaw 兩個 `atan2` 的分子與分母同時為 0，`atan2(0, 0)` 回傳 0，原本的轉角資訊消失 | `quat_to_euler` 加上萬向鎖分支：`yaw` 定為 0，`roll` 改用另一組不會同時為 0 的分量算出 `roll − yaw`（或 `roll + yaw`） |
| 萬向鎖的測試不能直接比三個角度 | 這個位置的尤拉角本來就不唯一，實作和 scipy 可能各選一組 | 改成比「轉回四元數後是否為同一個旋轉」 |

預防性處理：`asin` 的輸入先夾在 `[-1, 1]`，避免浮點誤差讓它略微超出範圍而丟 `ValueError`
（和 P0-T3 的 `cos_theta2` 同一類問題）。

## 與 Unity 的類比

- Unity 的 `Quaternion` 也是 `(x, y, z, w)` 四個分量，`Transform.rotation` 內部存的就是四元數；
  Inspector 顯示的 Rotation 三個數字是尤拉角，只是給人看的介面。這和 ROS「訊息用四元數、人用 RPY」是同一種分工。
- `Quaternion.Euler(x, y, z)` 對應這裡的 `euler_to_quat`，`transform.eulerAngles` 對應 `quat_to_euler`，
  `q1 * q2` 對應 `quat_multiply`。
- **慣例不同，數字不能直接搬**：Unity 是左手座標系、Y 軸朝上，尤拉角的套用順序是 Z → X → Y；
  ROS 是右手座標系、Z 軸朝上，順序是 X → Y → Z。
- 在 Unity 把 Inspector 的 X 旋轉設成 90° 後，調 Y 和調 Z 看起來是同一種轉動，就是萬向鎖。
  設定 `eulerAngles` 之後再讀回來，數字有時和設定的不一樣，也是因為同一個旋轉有不只一組尤拉角。

## 參考

- [REP 103](https://www.ros.org/reps/rep-0103.html)：座標系與旋轉表示慣例
- [Quaternion fundamentals（Jazzy）](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html)
- [scipy `Rotation.from_euler`](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_euler.html)：大小寫軸序的定義
- `notes/p0-t1-homogeneous-transforms.md`（`rot_x` / `rot_y` / `rot_z` 與座標系慣例）
