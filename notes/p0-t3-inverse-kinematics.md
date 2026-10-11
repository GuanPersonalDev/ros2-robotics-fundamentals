# P0-T3 兩軸手臂逆向運動學（IK，解析解）

日期：2026-10-11

## 做了什麼

- `phase0_math/ik.py`：`ik(x, y, l1, l2)`，輸入末端目標座標與兩根連桿長度，
  回傳兩組解 `(sol_1, sol_2)`，每組是 `(theta1, theta2)`，分別對應 elbow-up / elbow-down。
  目標點超出工作範圍（太遠或太近）時 `raise ValueError`。
- `phase0_math/tests/test_ik.py`：10 個 pytest 測試（兩組解的 FK 回代驗證、兩組解正負號相反、
  超出最大/最小可達範圍各一個）。

## 驗收

在專案根目錄執行：

```bash
python3 -m pytest phase0_math -v
```

預期：最後一行是 `40 passed`（P0-T1 的 19 個 + P0-T2 的 11 個 + 這次的 10 個）。
對應驗收條件的測試是 `TestInverseKinematics::test_both_solutions_round_trip_through_forward_kinematics`：
先用 `forward_kinematics` 由已知角度算出目標點，再丟進 `ik` 反解，確認兩組解各自回代
`forward_kinematics` 都近似等於原始目標點（也就是 `fk(ik(p)) ≈ p`）。

## 學到的概念

- **逆向運動學（IK）可能有多組解**：跟 FK 不一樣，同一個末端位置可以用兩種手肘彎法到達
  （elbow-up / elbow-down），對應 `theta2` 正負號相反的兩組解。
- **解法是餘弦定理的反推**：P0-T2 已經推過 `|p|² = l1² + l2² + 2·l1·l2·cos θ2`，IK 第一步就是
  把這條式子反過來解 `cos θ2`：
  `cos θ2 = (x² + y² − l1² − l2²) / (2·l1·l2)`。
  `sin θ2 = ±√(1 − cos²θ2)`，正負號就是兩組解的分岔點。
- **解出 θ2 後再解 θ1**：把 FK 的式子整理成 `x = k1·cosθ1 − k2·sinθ1`、`y = k1·sinθ1 + k2·cosθ1`
  （`k1 = l1 + l2·cosθ2`、`k2 = l2·sinθ2`），兩邊取 `atan2` 相減就能解出 `theta1 = atan2(y,x) − atan2(k2,k1)`。
  這一步也印證了 θ2 是相對角（P0-T2 筆記）：如果 θ2 是絕對角，這條式子會不一樣。
- **工作範圍的邊界就是 P0-T2 算過的最大/最小伸展距離**：`cos θ2` 要落在 `[-1, 1]` 才有解，
  對應 `|p|` 落在 `[|l1−l2|, l1+l2]`——這兩個邊界值在 P0-T2 筆記最後一段已經推過，這裡直接拿來當
  超出範圍時 `raise ValueError` 的判斷條件，不用重新推。
- **驗證 IK 不能自己手算三角函數比對自己**：跟 P0-T2 的教訓一樣，測試用 `forward_kinematics`
  產生目標點、再丟進 `ik` 反解比對，是獨立於實作的驗證方式，比手算一組組角度更不容易出錯，
  也同時測到 FK 與 IK 兩個方向的一致性。

## 卡關點

| 現象 | 原因 | 解法 |
|---|---|---|
| 呼叫 `ik()` 直接丟 `NameError: name 'math' is not defined` | `ik.py` 完全沒有 `import math`，但用了 `math.sqrt`、`math.atan2` | 檔案開頭加 `import math` |
| 補上 `import math` 後，`_solution()` 內部還是丟 `NameError`（`x`/`y`/`l1`/`l2` 未定義） | `_solution` 是模組層級的獨立函式，不是 `ik()` 內的巢狀函式；Python 的模組層級函式不會自動拿到呼叫端的區域變數，跟巢狀函式的 closure 是兩件事 | 把 `x`、`y`、`l1`、`l2` 加進 `_solution` 的參數列表，呼叫時明確傳入 |
| 一開始 `cos_theta2` 只用 `min(max(cos_theta2, -1), 1)` 夾緊，沒有任何超出範圍的錯誤，不符合驗收條件「超出範圍時回傳明確錯誤」 | 夾緊只是避免 `arccos`/`sqrt` 吃到域外值產生 `NaN`，不等於「判斷範圍」——這兩件事要分開處理 | 在算 `cos_theta2` 之前先用 `l = x²+y²` 跟 `(l1+l2)²`、`(l1-l2)²` 比較，超出就先 `raise ValueError`，夾緊留給邊界附近的浮點誤差 |

## 與 Unity 的類比

- 這等同於 Unity 裡的 Two Bone IK（`Animation Rigging` 的 Two Bone IK Constraint，或早期的 Final IK）：
  給一個目標位置，解算器要反推兩節骨骼的旋轉角。
- elbow-up / elbow-down 兩組解，對應 Two Bone IK 裡的 **pole target / hint**：單靠目標位置無法決定手肘
  要往哪邊彎，需要額外資訊消除歧義。這裡沒有 pole target，所以兩組解都算出來，由呼叫端決定要用哪一組。
- 工作範圍檢查則對應 Unity IK 解算器內部也會做的事：目標點超出骨骼總長或小於差值時，解算器通常會讓骨骼
  伸到極限方向，而不是算出無意義的角度——這裡選擇直接 `raise`，等於把這個邊界情況明確丟給呼叫端處理。

## 參考

- Kevin Lynch《Modern Robotics》第 4 章
- `notes/p0-t2-two-link-fk.md`（餘弦定理推導、相對角慣例、工作範圍的最大/最小值）
