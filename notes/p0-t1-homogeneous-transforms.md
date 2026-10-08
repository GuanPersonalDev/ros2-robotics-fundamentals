# P0-T1 齊次轉換矩陣

日期：2026-10-08

## 做了什麼

- `phase0_math/transforms.py`：2D 旋轉 `rot2d`、3D 繞軸旋轉 `rot_x` / `rot_y` / `rot_z`、
  組出 4x4 齊次轉換的 `make_transform`、求反矩陣的 `invert_transform`。
- `phase0_math/tests/test_transforms.py`：19 個 pytest 測試。

## 驗收

在專案根目錄執行（WSL 用 `python3`，Windows 用 `python`）：

```bash
python3 -m pytest phase0_math -v
```

預期：最後一行是 `19 passed`。其中對應任務驗收條件的兩個測試：

| 驗收條件 | 測試 |
|---|---|
| `T_ab @ T_bc == T_ac` | `TestComposition::test_chained_transforms_equal_directly_built_transform` |
| `T @ inv(T) == I` | `TestInvertTransform::test_transform_times_inverse_returns_identity` |

## 學到的概念

- **慣例要先定**：向量是 column vector，轉換寫成 `p_a = T_ab @ p_b`；右手座標系、角度用弧度、
  逆時針為正（與 ROS REP 103 一致）。這些寫在 `transforms.py` 的模組 docstring。
- **`T_ab` 的讀法**：把 b 座標系下的點換算到 a 座標系，同時也是 b 座標系在 a 中的姿態。
  下標相鄰相消，所以 `T_ab @ T_bc = T_ac`。
- **齊次轉換的區塊結構**：左上 3x3 是旋轉 `R`、右上 3x1 是平移 `p`、最底列固定 `[0, 0, 0, 1]`。
  把旋轉和平移收進同一個矩陣後，連續的座標轉換就只是矩陣相乘。
- **反矩陣有封閉解**：`R` 是正交矩陣，`R⁻¹ = Rᵀ`，所以 `T⁻¹` 的旋轉是 `Rᵀ`、平移是 `-Rᵀ p`，
  不需要呼叫 `np.linalg.inv`。
- **`rot_y` 的負號位置和另外兩個相反**：`rot_x`、`rot_z` 的 `-sin` 在右上，`rot_y` 的在左下。
  原因是軸的循環順序 x→y→z→x：繞 y 軸轉時是 z 轉向 x。
- **旋轉矩陣的檢查方式**：`R @ R.T == I` 且 `det(R) == 1`。只有前者成立而行列式是 -1，代表混進了鏡射。
- **浮點數不能用 `==` 比**：用 `np.testing.assert_allclose`；預期值含 0 時相對誤差無法比較，要給 `atol`。
- **驗證 `T_ab @ T_bc == T_ac` 時，`T_ac` 要另外建**：用 `R_ac = R_ab @ R_bc`、
  `p_ac = R_ab @ p_bc + p_ab` 組出來再比，否則是拿自己比自己，永遠會過。
- **模組 docstring**：檔案最頂端、`import` 之前的三引號字串，說明整個檔案；前面有其他敘述就只是普通字串。

## 卡關點

| 現象 | 原因 | 解法 |
|---|---|---|
| WSL 執行 `python3 -m pytest` 回報 `No module named pytest` | WSL 的系統 Python 沒裝 NumPy 與 pytest | `sudo apt install python3-numpy python3-pytest`；Ubuntu 24.04 不允許直接 `pip install` 到系統 Python |
| 不確定慣例要寫在哪裡 | 沒分清楚模組 docstring 與函式 docstring | 對整個檔案都成立的約定寫在模組 docstring |

## 與 Unity 的類比

- 4x4 齊次轉換就是 Unity 的 `Matrix4x4`；`make_transform` 相當於不含縮放的 `Matrix4x4.TRS`。
- `transform.localToWorldMatrix` 相當於 `T_world_local`，父子階層逐層相乘就是 `T_ab @ T_bc`。
- `transform.worldToLocalMatrix` 相當於 `invert_transform` 的結果。
- 差異：Unity 是左手座標系、Y 軸朝上；ROS 是右手座標系、Z 軸朝上。同一個角度繞同一個軸，
  兩邊看到的旋轉方向相反。Unity 的 Inspector 與 `Quaternion.Euler` 用角度，這裡用弧度。

## 參考

- https://www.ros.org/reps/rep-0103.html
- Kevin Lynch《Modern Robotics》第 3 章
