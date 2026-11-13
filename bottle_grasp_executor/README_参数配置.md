# bottle_grasp_executor 参数配置速查

对着 [`config/bottle_grasp_executor.yaml`](config/bottle_grasp_executor.yaml) 写的**改参数**说明。
yaml 里每个参数都有一段长注释讲「为什么」；这份文档讲的是「什么时候要改、改成什么、
改错了会看到什么」，以及几条**改不对就起不来或跑不动**的硬规矩。

运行步骤、状态机、踩坑记录见 [`README_H3001_grasp.md`](README_H3001_grasp.md)。

---

> **改参数有两条路（改的是同一个文件，选一条）**
>
> | 路子 | 什么时候用 |
> | --- | --- |
> | **图形界面 `h3001_tuner.py`**（工作区根目录，控制台的「调参」按钮也能开） | 位姿、抓取姿态、抓取高度、dwell、跟随参数、开关、夹爪力这些**常动的**（共 50 项，分 6 组） |
> | 手改 `config/bottle_grasp_executor.yaml` | 界面没收进来的（话题名、IK 阈值、工作空间边界、夹爪串口），或者你想连注释一起改 |
>
> ```bash
> /usr/bin/python3 h3001_tuner.py            # 不需要 source ROS
> /usr/bin/python3 h3001_tuner.py --dry-run  # 只载入+校验+打印将要写入的改动, 一个字节都不写盘
> ```
>
> 界面**只改值**: 逐行替换 `key:` 后面那一段, 注释/空行/顺序/行数一个字节都不动 ——
> 这个 yaml 的注释比数值值钱, 所以它不用 `yaml.safe_load` + `dump` 重写(那会把注释全抹掉)。
> 保存前把 `键: 旧值 -> 新值` 列出来让你确认; 写之前先存一份
> `bottle_grasp_executor.yaml.bak-<时间戳>`; 写完回读校验(解析得开 + 每个改动键的值对 +
> **行数不变 + 未改动的行逐字节相同**), 任一条不满足就从备份回滚。
>
> 三件要知道的事:
> - **界面不下发**: 保存只改文件, 生效要重启执行器(控制台点「起/停 执行器」)。带
>   【改后要重启执行器才生效】提示的, 就是下面 §0.1 那张表里的参数。控制台会盯着这个
>   文件的 mtime, 改过就在日志区提醒一句(但**绝不自动重启** —— 机械臂可能正夹着瓶子)。
> - **它只写源码树那份**: `install/` 那份是指向它的符号链接(见 §0.3), 所以不用 `colcon build`。
>   两边不是同一个文件时它顶上会红字警告。
> - **校验不过就拒绝保存**: 跨字段不变量(`max_result_age > 跟随周期`、死区 < 落后上限、
>   自动下降容差落在 `[0.005, 死区]`、`保持力矩 < 力矩硬上限` 等, 与测试里那几条同源)、
>   位姿的 6 个数/单位(mm 还是 m)/工作空间、以及行程与 `descend_dwell` 的匹配度。
>   warn 只提示, error 拦住。
>
> 唯一需要 ROS 的功能是「抄当前末端」(读 `/feedback/tcp_pose` 直接填位姿那 6 个数):
> 没 source 过 ROS 时它会说明原因, 手填一直可用。

---

## 0. 先看三条规矩

### 0.1 有一批参数**只在节点启动时读一次**，改了必须重启

`ros2 param set` 对它们**没有任何效果**（不会报错，就是不起作用）：

| 参数 | 为什么是一次性的 |
| --- | --- |
| `hand_eye_yaml_path` | 启动时读进 `_T_base_camera` |
| `gripper_offset_tool_xyz` | 启动时读进 `_gripper_offset`，后面所有坐标换算都用它 |
| `standby_pose_reported_mm_deg` | 启动时做 mm→m、RPY→四元数、工作空间校验 |
| `place_enabled` / `place_pose_reported_mm_deg` | 启动时校验并算出放置位姿（位置 + 姿态） |
| `gripper_enabled` / `gripper_port` | 启动时查串口在不在、脚本找不找得到（`dry_run` 下不查串口） |
| `candidates_source` / `candidates_topic` / `tcp_pose_topic` / `arm_status_topic` / `move_p_topic` / `move_l_topic` / `grasp_confirm_topic` | 订阅和发布是在启动时建立的 |
| `use_sim_time` | rclpy 内置 |

> **⚠️ 最坑的一个**：`ros2 param set /bottle_grasp_executor place_enabled true` 之后，
> 参数值是 true 了，但 `_place_ready` 还是 false —— 于是**每一次 `execute_grasp` 都被拒**：
>
> ```text
> success=False, message='place_enabled=true 但 place_pose_reported_mm_deg 无效 (见启动日志), 拒绝启动; ...'
> ```
>
> 看着像"参数已经改对了却被拒"。**开放置必须改 yaml 再重启节点。**

> **`gripper_enabled` 是同一个坑的第二个实例**：`ros2 param set` 之后参数值是 `true` 了，
> 但 `_gripper_ready` 还是 `false`（脚本路径与串口的校验都在加载期做一次），于是每一次
> `execute_grasp` 都被拒：
>
> ```text
> success=False, message='gripper_enabled=true 但夹爪不可用 (见启动日志), 拒绝启动; ...'
> ```
>
> **开夹爪也必须改 yaml 再重启节点。**

其余参数都是**逐次读取**的，`ros2 param set` 立刻生效（下文标了「可在线调」的可以直接调）。

### 0.2 序列里每个数都必须写成浮点，且类型一致

```yaml
standby_pose_reported_mm_deg: [100.0, 0.0, 290.0, 84.4, 0.2, 90.0]   # ✅
standby_pose_reported_mm_deg: [100, 0.0, 290.0, 84.4, 0.2, 90.0]     # ❌ 混合 -> 节点起不来
standby_pose_reported_mm_deg: [100, 0, 290, 84, 0, 90]               # ❌ 全整数 -> 类型冲突
```

rcl 按序列元素推断数组类型：混写报 `Sequence should be of same type`，全整数被推断成
integer array 与声明的 double array 冲突，报 `InvalidParameterTypeException`。
**这两种写法都是节点直接起不来**，不是静默出错。

`place_pose_reported_mm_deg` 同理，而且它**和待机位一样是 6 个数**（2026-09-18 之前是
3 个数、夹爪系、米 —— 那个写法已经废弃）：

```yaml
place_pose_reported_mm_deg: [130.3, -100.0, 194.1, 84.4, 0.2, 90.0]  # ✅
place_pose_reported_mm_deg: [130.3, -100.0, 194.1]                   # ❌ 旧约定, 加载期判无效
place_pose_reported_mm_deg: [0.1303, -0.1, 0.1941, 84.4, 0.2, 90.0]  # ❌ 按米填 -> 判无效
```

### 0.3 改 yaml 之后不用 `colcon build`

本工作空间是 `--symlink-install` 建的，`install/` 里那份 yaml 是指向 `src/` 的软链接，
改完直接重新 `ros2 launch` 就生效。（如果是普通 `colcon build`，就得重新 build 一次，
否则 `ros2 launch` 加载的还是 `install/` 里的旧副本。）

---

## 1. 部署前必须按现场改的（6 个）

| 参数 | 现在的值 | 怎么定 | 改错/没改的症状 |
| --- | --- | --- | --- |
| `dry_run` | `true` | 无机械臂验证全流程时保持 `true`；上真机改 `false` | 保持 `true` 上真机 = 机械臂**一动不动**，日志里全是 `[dry_run] move_p 目标:`（下降/抬升那两段现在打的是 `[dry_run] move_l 目标:`） |
| `hand_eye_yaml_path` | `.../H3003/hand_eye_extrinsics.yaml` | 必须与终端 B（demo）的 `--hand-eye` 指向**同一次标定** | 抓取点整体偏移；两边不一致时 yaml 与 demo 算出的基座系坐标不是一个东西 |
| `side_orientation_xyzw` | `[0.5837, 0.5150, 0.4875, 0.3955]` | 夹爪摆到合适抓取姿态、静止读若干帧取一致值 | 抓取时姿态差十几度；夹爪戳不到瓶身 |
| `gripper_offset_tool_xyz` | `[0.0, 0.0, 0.08]`（2026-09-21 起 8cm） | 上报末端原点 → 夹爪在 tool 系的平移，现场实测 | 整体差 8cm 量级；置 `[0,0,0]` 退化为恒等（换新机械臂时用）。**注意**：历史记录里有一次 60 mm 的实测，与现在 yaml 的 0.08 不一致，尚未查清（见 §10 第 1 条） |
| `standby_pose_reported_mm_deg` | `[100.0, 0.0, 290.0, 84.4, 0.2, 90.0]` | 从厂商 GUI 直接抄那 6 个数（**上报末端原点系、mm、度**），不要自己换算 | 见 §5 的三条自检 |
| `place_enabled` + `place_pose_reported_mm_deg` | `true` + `[214.0, -306.0, 154.1, 95.4, 0.2, 26.0]`（yaml 现值；**尚未在真机上确认过**，见 §2） | 见 §2，**两个必须一起改** | 只开开关不填位姿 → 所有任务被拒；只填位姿不开开关 → 放这段被跳过 |
| `grasp_confirm_required` / `place_confirm_required` | `false` / `false` | 见 §2b，**现场默认关掉后两个确认点** | 关着时夹爪按 dwell 定时动作，不看机械臂走到哪（可能夹空） |

**上真机前先用 `dry_run` 核对启动日志**（详见 `README_H3001_grasp.md` §3.3）：

```text
已加载手眼矩阵: <路径>                                  ← 是不是这次标定的那份
待机位: 上报原点=[...] m (与 GUI/参数一致才是对的), 夹爪=[...] m, 偏移=[...] m
放置位姿: 上报原点=[...] m (与 GUI/参数一致才是对的), 夹爪=[...] m (base_link), 姿态=[...]
                                                        ← 只有 place_enabled=true 才有这行
夹爪脚本: <路径>                                        ← 只有 gripper_enabled=true 才有这行
夹爪: 开 (... 端口=... 脚本超时 25s / 节点超时 30s ...)   ← 同上; 这两行都在才说明加载期过了
```

`待机位` / `放置位姿` 那两行的**「上报原点」必须等于参数里 X/Y/Z 换成米**。对不上就是参数
或换算错了。

夹爪那两行：只有 `夹爪脚本:` **和**「`夹爪: 开`」**都出现**才算校验通过 —— 只出现
「`夹爪: 关 (夹爪端口/脚本不可用 …)`」就说明脚本没找到或串口不在，这时任务会被拒。

---

## 2. 放置（2026-09-17；2026-09-18 改成完整位姿）

```yaml
place_enabled: true                                                 # 总开关 (yaml 现状; 代码默认 false)
place_pose_reported_mm_deg: [214.0, -306.0, 154.1, 95.4, 0.2, 26.0] # 放置位姿: 上报末端原点系 (yaml 现值)
place_descend_dwell: 0.5                                            # 判到位之后再停多久才张开夹爪 (2026-09-21 起 0.5)
place_confirm_required: false                                       # 要不要等人工按 Enter 才张开 (2026-09-18)
```

**怎么填 `place_pose_reported_mm_deg`**（这是最容易填错的一个）：

- 它是**上报末端原点系**的 6 个数 `[X,Y,Z(mm), RX,RY,RZ(度)]`，**与
  `standby_pose_reported_mm_deg` 同一个约定、同一种填法** —— 就是 `/feedback/tcp_pose`
  和厂商 GUI 上显示的那 6 个数，**直接抄，不要自己换算**。
- 2026-09-18 之前这里是 **3 个数**、约定是"夹爪中心 / `base_link` 系 / 米"。两套约定
  在同一个 yaml 里共存过，抄错一次就是机械臂往自己底座里扎 —— 现在统一了，那个旧写法
  已废弃，写 3 个数会在加载期判无效。
- **最省事的办法**：把机械臂手动挪到想放的位置，直接抄 GUI 上那 6 个数。位置随便设，
  姿态也随便设 —— 放置段**不再沿用抓取时的姿态**，`PLACE_MOVE` 这一段全程用这里的
  RX/RY/RZ（2026-09-18 起 `PLACE_DESCEND` 那一段已经删掉，状态只剩
  `LIFT -> PLACE_MOVE -> PLACE_WAIT -> HOME`）。
- **yaml 里现在这组还没在真机上确认过**：yaml 注释写明它被固件判过一次 IK 拒收
  （`status=2`），同一组数也出现过 `status=0` —— 只能上机试，指令发出去那一拍看
  `h3001_bridge` 回的是 0 还是 2。**上真机第一件事就是这一条**，别的都排在它后面。
- **释放点不需要在台面上**：节点做的就是"把夹爪送到这个位姿、然后松手"，不假设下面有
  什么。所以 z 比抬升高度还高（整段运动成了**上升**）是合法的 —— 往货架/高台上放、或者
  只是要求"送到这个位置释放"都算。那种情况会打一行 warn，那是**提醒**不是判错：它防的
  是另一个可能 —— 从 GUI 抄 z 时抄错一位，"本该往下放"悄悄变成往上抬。
- 默认全零是**占位值**，加载期会因为"离基座原点太近"判无效 —— 这是故意的，防的是
  "忘了填"变成"机械臂往自己底座里扎"。位置按米填（`0.1303` 而不是 `130.3`）也会被这条
  拦下。

**放置姿态换了一组之后怎么看对不对**（2026-09-18 新增两条日志）：

- 启动时那行 `放置位姿: 上报原点=[...] 夹爪=[...] 姿态=[...]`，上报原点那栏必须等于
  参数里 X/Y/Z 换成米、姿态那栏必须等于 RX/RY/RZ 转出来的四元数。
- 进 `PLACE_MOVE` 时那行末尾会带 **`全程同时转腕 N°`** —— 抓取姿态到放置姿态要转多少。
  超过 90° 会另打一条 `WARN`（多半是 RX/RY/RZ 抄错了一行）。
- 若 `PLACE_MOVE` **原地不动**、最后 `运动超时`：优先怀疑这组姿态机械臂解不出来
  （IK 无解 / 关节限位），**换一组 RPY 再试**，不要去动 `motion_timeout` 或容差。
  这正是 2026-09-18 把放置姿态从"沿用抓取姿态"解耦出来的原因。

**上真机的推荐顺序**：先 `place_enabled: false` 跑一遍确认老流程没变 → 填真实坐标、z 先填得
**比目标高 5cm**（要放到台面上时就是比台面高 5cm）→ **把 `place_confirm_required` 设成
`true`**，确认各确认点的顺序和提示都对、机械臂停在放置点上方 → 再逐步往下调 z → 位置定
下来之后才把 `place_confirm_required` 关回 `false`。关掉之后"放得对不对"就没有停下来的
时机可看了。

**`place_descend_dwell` 与 `descend_dwell` 是分开的两个参数**，别改错：
抓取下降是 12cm（2026-09-21 起，此前 9cm —— 悬停 0.12 减去抓取点的高度），放置下降是
"从抬升高度到放置高度"，两者距离不同。放得太早会在机械臂还在
往下走时就往下走。判据是日志里那行"实际"是否还在明显移动。

**`place_confirm_required` 关掉（现场默认）之后，这个 dwell 就是唯一的把关**：
放置段没有盲降那一截（`PLACE_MOVE` 是直线直达 + 判到位，2026-09-18 起
`PLACE_DESCEND` 已经删掉），张开时刻完全由它决定 —— 短了就是在机械臂还在晃的时候
松手。"放得太早/放歪"**先调大 `place_descend_dwell`**，而不是把开关改回 `true`
（那会把整条链变回逐步确认）。

---

## 2b. 夹爪（2026-09-17；2026-09-18 改成一次性命令）

```yaml
gripper_enabled: true           # 总开关: 节点是否接管夹爪 (yaml 现状; 代码默认 false)
gripper_port: "/dev/ttyACM0"    # 夹爪的串口 CAN 适配器 (不是机械臂的 can0)
gripper_velocity: 2.0           # 开/闭共用的速度
gripper_resistance_torque: 0.3  # 阻力阈值: 超过「空载 + 这个值」就停下
gripper_hard_torque_limit: 2.0  # 硬上限: 碰到就停下 (不算失败)
gripper_script_timeout: 25.0    # 传给脚本的 --timeout
gripper_timeout: 30.0           # 节点侧上限, 必须 > 上面那个
```

**节点只发命令，不看夹爪状态**（2026-09-18 用户明确要求）：每次动作就是「起一个进程
跑一条命令，等它退出」，**判成败只看退出码**。脚本 stdout 上写什么都不参与判断 ——
包括 `[warn] 扭矩冲到硬上限` 这种。`gripper_resistance_torque` 只决定「闭合到哪里
停」，一路合到底也照样往下走抬升。夹到什么、夹没夹住由人在确认点看 ——
**而现场默认两个确认点都关掉了**，所以这时连"人在看"也没有，见下。

**夹持力不由节点维持**：闭合命令跑完脚本就失能退出，抬升期间靠机构自锁。这一条
**没有在本机实测过**（SDK 文档说失能后机构 goes limp），掉瓶就是它挡不住 ——
真掉了要把「保持 enabled + 持续发帧」那套加回来，不要靠调参数。详见
[`README_H3001_grasp.md`](README_H3001_grasp.md) §5.13。

**打开之后语义会变**（这是最容易踩的一条）：三个确认点的**后两个含义变了**。

| 确认点 | `gripper_enabled: false` | `true` |
| --- | --- | --- |
| `TRACK_WAIT` | 位置对了，现在下降 | 同左（不变） |
| `GRASP_WAIT` | **我已经手动夹好瓶子了**，现在抬 | **现在夹** —— 节点发一条闭合命令，**命令跑完才**抬 |
| `PLACE_WAIT` | **我已经把物体放下了**，现在回位 | **现在放** —— 节点发一条张开命令，**命令跑完才**回位 |

另外 `true` 时**每轮开始前会先张开一次**，保证从"确定张开"这个状态出发。

**后两个确认点各有开关，现场默认都是关的**（2026-09-18）：

```yaml
grasp_confirm_required: false   # GRASP_WAIT 不等 Enter, descend_dwell 一到就闭合
place_confirm_required: false   # PLACE_WAIT 不等 Enter, place_descend_dwell 一到就张开
```

两个都关掉之后，**整条链只剩 `TRACK_WAIT` 一次确认**：按一次 Enter 确认下降，之后
夹、抬、搬、放、回位全自动。上表那两行的"现在夹/现在放"仍然成立，只是触发者从
人的 Enter 换成了 dwell 到点。

**2026-09-22 起，那最后一次也能自动化**：`auto_grasp_stable_time`（现场 `3.0`）——
机械臂到物体上方、物体坐标连续 3s 不变就自己下降。现场 yaml 下三个开关合起来的效果是
**整条链一次 Enter 都不用按**；想把人放回链上，把 `auto_grasp_stable_time` 设成 `0`
（见 `README_H3001_grasp.md` §5.10）。

**代价**：`DESCEND` 是盲降、不判是否到位，所以**闭合**时刻完全由 `descend_dwell` 决定
（放置侧不一样：`PLACE_MOVE` 是判到位的，`place_descend_dwell` 管的是"停稳之后再松手"）
—— 机械臂还没走完就夹，夹爪会在半空合上（夹个空）。
**真机上夹空了先调大 `descend_dwell`，不要改回 `true`**（改回 true 会把整条链变回
逐步确认）。另外"手还伸在夹爪里也来得及退开"这条只在开关打开时成立。

**怎么定 `gripper_resistance_torque`**：它决定"夹到什么程度算夹住了"。太小会在没夹到
东西时就停（瓶子其实没夹稳），太大则可能把东西夹变形。判据是台架实测（脚本 stdout 上
会说明这次是"遇到阻力停下"还是"一路合到底"）：夹一个瓶子应该在遇到阻力时停；
**空夹爪**跑一次应该说"一路合到底"。空夹爪也说遇到阻力，就说明阈值太钝，往下调。

**`gripper_hard_torque_limit` 不要为了"夹得更紧"调大** —— 它是防夹坏东西/防堵转的
那一道。要更紧应该调 `gripper_resistance_torque`。撞上它**不判失败**（2026-09-18
起）：真机上那次扭矩 0.43s 就冲到 -2.160 N·m、消抖窗口还没成立，夹爪其实已经夹住了，
老代码判失败让整趟任务作废。现在按"已夹住"停下，只在日志里打一行 `[warn]`。

**`gripper_timeout` 必须大于 `gripper_script_timeout`**（`test_gripper.py` 盯着这条）。
脚本先自己判超时、干净退出并给出原因，节点才有东西可读；反过来的话日志里只剩一句
"超时"，看不到为什么失败。

> **夹爪命令跑不成（退出码非 0）一律判任务失败并回待机位**，与其它失败走同一条路。
> 失败时**不再**补发一条"松手"指令 —— 进程要么已经退出，要么自己会撞上脚本内部的
> `--timeout` 退掉，失败路径上少一个可能失败的分支。

**上真机前先在台架上单独跑脚本**（不起 ROS），命令见
[`README_H3001_grasp.md`](README_H3001_grasp.md) §5.13。那里也写了**为什么不能直接用
厂商的 04/05**（它们在 `finally` 里无条件 `disable()`，电一断夹爪就松，抬升时手里是空的）。

---

## 3. 抓取几何（决定抓哪、下降多少）

| 参数 | 现在的值 | 说明 |
| --- | --- | --- |
| `pre_grasp_offset_z` | `0.12` | 悬停点 = 抓取中心 + 这个值。**它同时就是下降行程**（下降距离 = `pre_grasp_offset_z - grasp_offset_z`），所以改它等于改悬停高度**和**下降距离两件事。**下降变长了记得同步调大 `descend_dwell`** |
| `grasp_offset_z` | `0.0` | 最终抓取点相对抓取中心的高度。**只想调"抓高一点/低一点"就改这个**，不动悬停高度 |
| `lift_distance` | `0.10` | `GRASP_WAIT` 之后抬升多少（用户口径 9cm≈10cm，按 10cm 配） |
| `min_confidence` / `min_quality` | `0.30` / `0.05` | 候选筛选门限。抓不到目标时先调这两个，再看别的 |

> 想「悬停高度仍是 12cm，但只下降 11cm（落点比抓取中心高 1cm）」：**不要改 `pre_grasp_offset_z`**
> （它就是行程），改成 `grasp_offset_z: 0.01` —— 下降距离 = `pre_grasp_offset_z - grasp_offset_z`
> = 0.12 - 0.01 = 0.11 m。反过来让落点比抓取中心低 1cm 就填 `-0.01`（行程变 13cm）。
> （2026-09-21 起悬停值是 0.12；上面这个例子是按现在的值重算的。）

---

## 4. 时序与等待（真机调试最常动的）

全部**可在线调**（`ros2 param set` 立即生效，不用重启）。

| 参数 | 现在 | 调大的效果 | 调小的效果 |
| --- | --- | --- | --- |
| `track_min_command_interval` | `1.0` | 跟随更迟钝、但更不容易抖 | 跟得更勤，启停频繁易抖 |
| `max_result_age` | `6.0` | 容忍感知更长时间的停顿（2026-09-21 起由 `5.0` 调到 6.0：代码默认值向 yaml 现场值对齐） | 更早判 `TRACK_TARGET_LOST` |
| `track_deadband` | `0.010` | 目标小动不跟，机械臂更安静 | 更灵敏，但容易追着噪声动 |
| `track_settle_time` / `track_stable_epsilon` | `0.30` / `0.0005` | 更保守地等上一条走完 | 更容易叠指令（= 抖动） |
| `track_max_lag` | `0.050` | 停稳门更硬（跟丢风险↑） | 更容易"不等停稳直接追" |
| `auto_grasp_stable_time` | `3.0` | 悬停后要等物体**更久**不动才自动下降（等到不耐烦就是它太大） | 更早下降。**调到 1s 以下要小心**：人手把瓶子放到位的过程里本来就有短暂的停顿，窗口太短会把这种"放一半的停顿"当成"已经放好了"而下降。`0` = 关掉，退回等人按 Enter（2026-09-22 新增） |
| `auto_grasp_stable_epsilon` | `0.010` | 判"位置没变"更松，更容易触发 | **低于 0.005 就基本不会触发**：静止物体的 `position_base` 峰峰值实测 3~4 mm，容差比噪声还细等于永远判"还在变"（2026-09-22 新增） |
| `descend_dwell` | `2.0` | 抓取下降指令**发出后**多等一会再闭合夹爪（2026-09-21 起由 `6.0` 调到 2.0） | **可能还没走完就闭合（夹空）** |
| `place_descend_dwell` | `0.5` | 抵达放置点（**判到位**）之后再停多久才张开夹爪 —— 名字是旧流程留下的，不再是"盲降发出后"（2026-09-21 起由 `6.0` 调到 0.5） | 同上（放得太早/放歪） |
| `settle_time` / `position_tolerance` | `0.30` / `0.005` | `settle_time` 调大 = 到位后要多停一会才算数；`position_tolerance` 调大 = 容差更松 | 反向：更容易放行 / 更容易 `motion_timeout` |
| `standby_tolerance` / `standby_settle_time` | `0.015` / `0.30` | 回位更容易判定成功 | 回位频繁报"只差几厘米" |
| `motion_timeout` | `30.0` | 更容忍慢。**HOME 段真正用的不是这个数，是它 ×N**（N ≤ 4，由 `max_command_distance` 和 `gripper_offset_tool_xyz` 算出来，见 §7） | 更容易提前判失败 |
| `input_timeout` | `2.0` | 容忍反馈更长的断续 | 更早判定"反馈断了" |
| `blind_home_dwell` | `8.0` | 盲走等反馈恢复更久 | 更早放弃确认 |
| `stall_detect_time` / `stall_detect_epsilon` | `1.0` / `0.0005` | 停稳判定更保守 | **低于 `settle_time` 就会在机械臂还在走时误判停稳**，别调小 |
| `ik_verdict_grace` | `0.8` | 更晚才认固件的 IK 判决（更保守，拒收也晚报 0.8s） | **别低于 0.5**：MCU 要收齐 `send_target_pose` 的 3 帧位姿（帧间 20ms）才重新判决，等不够会读到**上一条**指令的判决 |

> 时序参数的取值只有一条通用判据：**看日志里带回来的"实际位置"**。还在明显移动就是给少了。

---

## 5. 开关：真机出问题先关谁

| 参数 | 现在 | 关掉的后果 |
| --- | --- | --- |
| `track_enabled` | `true` | 跳过整个 `TRACK_WAIT` —— 不跟随、也不要那唯一一次确认，悬停到位直接下降。**整条链变成零确认**（后两个确认现场本来就是关的），新环节有问题先关它验证其余部分，不用回滚代码 |
| `top_skip_track` | `true`（yaml 现状） | 关掉的正是它 → **倒瓶（`top` 模式）也进 `TRACK_WAIT`**。那一模式从正上方抓，机械臂一到瓶子正上方相机就看不着它（现场表现：候选停更 → 判 `TRACK_TARGET_LOST` → 回待机位，一轮都抓不成，2026-09-29），所以现场开着它：`top` 悬停到位直接下降，下降点用锁定目标那一刻的坐标。**`side` 不受这个开关影响**。代码默认 `false` = 改动前的行为 |
| `auto_grasp_stable_time` | `3.0` | 这个开关的形状是"设成 `0`"：`TRACK_WAIT` 那次确认退回人工，悬停后停下等人按 Enter。**现场 yaml 下它是关着一半的** —— 后两个确认（`grasp_confirm_required` / `place_confirm_required`）本来就是 `false`，所以把它设成 0 是**唯一**能把"人"放回整条链上的办法（2026-09-22 新增，见 `README_H3001_grasp.md` §5.10） |
| `grasp_confirm_required` | `false` | 关掉的正是它 → `GRASP_WAIT` 不等 Enter，`descend_dwell` 一到就闭合。开着它才能"看一眼再夹" |
| `place_confirm_required` | `false` | 同上，对应 `PLACE_WAIT` / `place_descend_dwell` |
| `ik_verdict_enabled` | `true` | 退回旧行为：固件 IK 拒收（`ik_status=2`，机械臂一步不走）不再当场判失败，要等满 `motion_timeout` 才报一个 `运动超时, 偏差 N mm` —— 从日志里分不出这是「被挡住推不动」还是「IK 无解」。**判据是「固件报非 0」+「机械臂一步没动」两条同时成立**，不看 `ik_status_valid`（真机上它恒为 0，SDK 漏拷字段，见 `h3001_msgs/msg/H3001ArmStatus.msg` 里的注释）；没什么理由关，除非现场就想让它等满超时 |
| `place_enabled` | `true`（yaml 现状） | 跳过整个放置（`LIFT` 后直接回待机位）；**同时是循环模式唯一的硬闸** —— 关着它循环开不起来（同一个瓶子会无限循环），见本表下方的提示 |
| `gripper_enabled` | `true`（yaml 现状） | 节点不碰夹爪，`GRASP_WAIT`/`PLACE_WAIT` 的语义退回旧版（"我已经夹好了" / "我已经放好了"）。**注意此时若两个确认开关仍是 `false`，节点会不等人直接抬升/回位** —— 要人工夹就得自己算准 dwell 的时间，或者临时把开关打开，见 §2b |
| `return_to_standby_on_finish` | `true` | 任务结束后不回位，机械臂留在原地 |
| `return_to_standby_on_error` | `true` | 机械臂报故障/反馈断了时**不**回位（旧行为）。**不建议关** —— 2026-09-16 真机上就是这条把机械臂丢在抓取点上方了 |
| `endpoint_correction_enabled` | `true` | 不补重力下垂，末端固定差 3~55mm，到位判定会一直够不到 |
| `use_motion_status` | `false` | 打开则额外要求固件自报 `motion_status == 0` 才算到位（该字段稳定性未验证） |

> **循环模式（2026-09-21）没有新增参数** —— 它是 `~/execute_grasp` 自己的语义：调一次
> = 启动抓取 + 开启循环，再调一次 = 关闭（**跑完当前这一次再停**，不打断机械臂）。
> 唯一的硬闸就是上面的 `place_enabled=true`，其余只打 WARN 不拦。自动停止条件只有
> 「这一轮**真失败**」（IK 拒收 / 无反馈 / 夹爪失败 / 回位不到位）和人工再调一次服务：
> 跟丢目标、以及回到待机位后视野里一直没有新目标，都是**在待机位接着等**，不自动收工
> （判据与响应 `message` 对照表见 `README_H3001_grasp.md` §3.4，设计取舍见 §5.14）。

---

## 6. 安全边界（一般不改，但要知道它们在哪）

| 参数 | 现在 | 作用 |
| --- | --- | --- |
| `workspace_x/y/z_min/max` | `±0.90` / `±0.90` / `-0.10~0.60` | 每个目标点（含它对应的上报原点）都要在盒子里，越界报 `WORKSPACE_REJECTED` 并拒绝/失败 |
| `max_command_distance` | `0.50` | 单条指令的位移上限，**只管感知驱动的运动**（`PRE_GRASP` / `LIFT` / 跟随时每一跳），防的是「感知给的坐标突然跳一大截」。**`PLACE_MOVE` 与 HOME 豁免这个上限**（放置 2026-09-20、回位 2026-09-21）：这两段的目标是 yaml 常量、不是感知，距离远是正常的 —— 各自一条**绝对** `move_p` 直接发出去、路径交给固件规划，不再由节点在直线上插中间点。豁免给在 `_tick_motion` 的**调用点**上（按状态判断），`_check_distance` 自己照拦；回位超限时会多打一行 WARN。**调得太小会让悬停那一条指令就过不了**，而且它同时决定 HOME 允许走多久（见 §7）。**这个上限与走 `move_p` 还是 `move_l` 无关**：`LIFT` 现在走 `move_l`，但照样受它管（上行段），下降段 `DESCEND` 也照管 —— 改的只是路径形状，`_check_distance` 看的是两端距离。`README_H3001_grasp.md` §5.12 是放置那一段的来龙去脉 |
| `max_correction_distance` | `0.08` | 迭代修正量的上限。病根若是"被挡住/IK 无解"，超过它就停手并报出来，不会越推越远 |
| `max_correction_retries` | `4` | 每个路点最多迭代几次 |
| `orientation_source` | `fixed` | `fixed` 用配置里的固定姿态（可复现）；`current_tcp` 保持触发瞬间姿态（要求触发前先摆好手腕） |

---

## 7. 参数之间的硬约束

改一个必须同时看另一个，否则会出现"看着改了却更糟"：

| 约束 | 为什么 |
| --- | --- |
| `max_result_age` **>** `track_min_command_interval` | 相等或更小时，跟随还没读到下一次坐标就先判 `TRACK_TARGET_LOST`。`test_track_wait.py` 的 `ConfigConsistencyTest` 盯着这条 |
| `stall_detect_time` **明显长于** `settle_time` / `track_settle_time` | 否则会在机械臂还在走时就判"停在新稳态上"，量到的残差是假的，迭代修正会乱推 |
| `pre_grasp_offset_z - grasp_offset_z` = 下降行程 | 改悬停高度会顺带改下降距离 |
| `place_enabled: true` **⟺** `place_pose_reported_mm_deg` 是真实位姿 | 两者必须一起改，否则所有任务被拒 |
| `place_descend_dwell` ≥ 该段下降的实际走完时间 | 短了会在机械臂还在走时就往下走。**两个确认开关关掉后这条是硬要求**：张开时刻完全由它定 |
| `descend_dwell` ≥ 抓取段下降的实际走完时间 | 同上，闭合时刻。夹空了先调它，别改 `grasp_confirm_required` |
| `auto_grasp_stable_epsilon` **≥** 感知噪声的实际量级 | 静止物体的 `position_base` 峰峰值实测 3~4 mm（112 帧 / 13.9 s）。拿 `track_stable_epsilon`（0.0005）来判「物体没动」**一次都不会通过**（3s 窗口 0/73）；0.010 有约 2 倍裕量。`ConfigConsistencyTest` 钉 `0.005 ≤ eps ≤ track_deadband`（2026-09-22） |
| `auto_grasp_stable_time` **明显大于** `track_min_command_interval` | 自动下降自带一道新鲜度门：候选超过**一个跟随周期**没更新就作废计时窗口（这是为了拦住「上游整个不发了、坐标冻在最后一个值上」—— 冻住的值恰好满足「位置没变」）。窗口如果短到和一个周期同量级，这道门就形同虚设。现场 `3.0` vs `1.0`，比值 3（2026-09-22） |
| 所有序列参数：元素类型一致 + 全浮点 | 见 §0.2 |
| HOME 的到位超时 = `motion_timeout` **× `N`**，且 `N` **不超过 4** | 回位现在只发**一条**绝对待机位，而老代码把返程拆成 `N` 段、**每段各给一个完整的 `motion_timeout`**；不放大就等于把这笔预算砍掉（对角 1.3 m 的回位原来是 4×30 s，合并成一条后只剩 30 s，远距离回位会从「走得慢」变成「没收敛」）。`N` 用**老拆段算式**算：`budget = max(max_command_distance − 2 × gripper_offset_tool_xyz 的模长, 0.02)`、`N = ceil(起点到待机位的直线距离 / budget)`，**超过 4 就截断到 4 并打一行 WARN**（`max_command_distance` 被调到 0.05 m 时这算式会给出 65 倍 = 32 分钟，那不是「给足时间」而是把卡住伪装成正常等待）。**所以改 `max_command_distance` 或 `gripper_offset_tool_xyz` 会顺带改 HOME 允许走多久** —— 这两个参数不只是「上限/偏移」，还是回位时间预算的输入。离开 HOME 时倍数清回 1，不漏到后面的运动阶段 |
| `max_command_distance` 对 `PLACE_MOVE` / HOME **不生效** | 这两段的目标是 yaml 里的常量（放置位姿、待机位），不是感知；那个上限防的是「感知坐标突然跳变」，套在这里只会把一段正常的远距离横移判成失败。所以两段都是**一条绝对指令、路径交给固件规划**，不再插中间点。**别再照着拆回去**：拆出来的中间点一律带上待机/放置姿态，而固件 IK 在固定姿态下的可达域**不是凸的** —— 2026-09-21 真机上回位的中间点 `[0.1348, -0.1889, 0.2147]`（上报系）被固件拒收（`ik_status=2 超出工作空间`），而两个端点各自都够得到（节点自己的 `workspace_*` 凸盒子检查一路放行，证明不了固件的可达域）。当天单发一条 353 mm 的绝对待机位被固件收下，到位差 0.5 mm。别的状态（`PRE_GRASP` / `LIFT` / 跟随）**照旧受它约束** —— 那才是「感知给的坐标突然跳一大截」的兜底；拿掉回位那层兜底的代价是回位超限时只剩一行 WARN |

---

## 8. `ros2 run` 和 `ros2 launch` 起出来**不一样**

`ros2 launch` 会加载这份 yaml；`ros2 run` **不加载**，用的是代码里的默认值。
2026-09-21 把 `_declare_parameters` 里那个 `defaults` 与 yaml 的
`bottle_grasp_executor: ros__parameters` **逐字对过一遍**：两边有 **15 个键**不一致。
其中 13 个是刻意的（安全默认值 / 现场标定值 / 部署开关，代码里给保守值，部署时才在
yaml 里打开）；另外 2 个（`candidates_topic`、`hand_eye_yaml_path`）**不是「保守值 vs
现场值」，是真的不一样**，表后各单列一段：

| 参数 | `ros2 run`（代码默认） | `ros2 launch`（yaml，现场值） |
| --- | --- | --- |
| `candidates_topic` | `/bottle_grasp_perception/selected_candidate`（已选好的单个候选） | `/bottle_grasp_perception/candidates`（整组候选） |
| `orientation_source` | `current_tcp` | `fixed` |
| `return_to_standby_on_finish` | `false` | `true` |
| `place_enabled` | `false` | `true` |
| `gripper_enabled` | `false` | `true` |
| `grasp_confirm_required` | `true` | `false` |
| `place_confirm_required` | `true` | `false` |
| `hand_eye_yaml_path` | `~/hand_eye_calibrate/H3003/hand_eye_extrinsics.yaml`（`~` 会展开成家目录） | `hand_eye_calibrate/H3003/hand_eye_extrinsics.yaml`（相对路径） |
| `side_orientation_xyzw` | `[0.0549, 0.0302, -0.9943, 0.0868]`（09-14 那次实机读数） | `[0.5837, 0.5150, 0.4875, 0.3955]` |
| `standby_pose_reported_mm_deg` | `[78.0, 0.0, 290.0, 2.4, 4.2, -180.0]` | `[100.0, 0.0, 290.0, 84.4, 0.2, 90.0]` |
| `place_pose_reported_mm_deg` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`（占位值 = 关闭） | `[214.0, -306.0, 154.1, 95.4, 0.2, 26.0]`（yaml 现值，**尚未在真机上确认过**） |
| `workspace_x_min/max`、`workspace_y_min/max` | `±0.60` | `±0.90` |

2026-09-22 新增的 `auto_grasp_stable_time` / `auto_grasp_stable_epsilon` **两边同值**
（`3.0` / `0.010`），不在上表里 —— 它们不是部署开关，是判据本身的量级，跟
`track_deadband` 一样两边给同一个数；`test_track_wait.py` 的 `NEW_TRACK_KEYS` 钉住
代码默认值 == `config/*.yaml`。

**`hand_eye_yaml_path` 那两行不是同一个文件**（旧表里写的 `.../my_calibration/...` 早就
不对了）：代码默认走
`os.path.expanduser("~/hand_eye_calibrate/H3003/hand_eye_extrinsics.yaml")`，`~` 展开成
**家目录**，也就是 `$HOME/hand_eye_calibrate/...`；yaml 里是**相对路径**
`hand_eye_calibrate/H3003/hand_eye_extrinsics.yaml`，只有**从工作空间根目录启动**才解析
得到。本机上 `$HOME/hand_eye_calibrate/H3003/` 这个目录**不存在**（这份标定实际在
`/home/zdl/H3001_grasp/hand_eye_calibrate/H3003/hand_eye_extrinsics.yaml`）—— 所以
`ros2 run` 不带 `--ros-args -p hand_eye_yaml_path:=...` 时，加载期 `_load_hand_eye_matrix`
根本打不开文件，节点直接起不来。这比「值不一样」更硬。哪套路径在生产机器上成立，以
**现场实际的启动方式**为准，别照这张表猜。

**`candidates_topic` 是语义上的不同，不只是话题名**：代码默认
`/bottle_grasp_perception/selected_candidate` 收的是 `SelectedCandidate`（**已经选好的
单个候选** —— 感知节点自己挑目标、自己切换）；yaml 的
`/bottle_grasp_perception/candidates` 是 `GraspCandidateArray`（**整组候选**，设计上是执行
节点自己挑，`_on_candidates` -> `_select_and_store`）。所以 `ros2 run` 不带 params file
起出来的节点订阅的是**另一个话题、另一种消息**，走的是**另一条挑选路径**。两个补充事实，
免得读拧：① 当前
代码里只挂了 `SelectedCandidate` 那一条订阅，处理整组候选的 `_on_candidates` **定义了但
没有任何地方订阅它**；② 两边 `candidates_source` 默认都是 `file`，所以这处分歧只有把
`candidates_source` 改成 `topic` 时才真的用得上。**不要为了「对齐」去改代码或 yaml 里的
这个话题名** —— 它是接口约定，改要四处一起改（见 §9）。

`gripper_offset_tool_xyz` **2026-09-18 起不在这个表里了**（原来是 `[0.0, 0.0, 0.0]` vs
`[0.0, 0.0, 0.08]`）：按"**夹爪偏移以 yaml 为准**"把代码默认值也对齐了过去，两边同值。
它不是部署开关、也不是保守/激进的选择，而是一个**物理事实**（上报原点离夹爪
有多远，实测值）—— `ros2 run` 少带这 8cm，机械臂就整体偏 8cm，没有"保守一点"的中间态，
所以不该两边不同。**这一条前后对过两次，别被历史版本误导**：2026-09-18 那次只把代码
默认值改到了 `[0.0, 0.0, 0.06]`（yaml 一直是 `[0.0, 0.0, 0.08]`），当时写的"两边同值"
其实是错的；**2026-09-21 才把代码默认值从 `0.06` 改成 `0.08`**，现在两边才真的同值，
这一档的模长是 8cm。§10 第 1 条还留着一次 60 mm 的实测，与 0.08 对不上，尚未查清。

`dry_run` 两边都是 `true`（launch 另有 `dry_run:=false` 参数可覆盖，命令行给的值排在 yaml
之后，会盖掉 yaml）。

**夹爪那一组除开关外两边都同值**（`gripper_port` / `gripper_velocity` /
`gripper_resistance_torque` / `gripper_hard_torque_limit` / `gripper_script_timeout` /
`gripper_timeout` 六个），`gripper_enabled` 是那个"代码默认 `false`、
部署时在 yaml 里开"的键 —— **yaml 现已改成 `true`，代码默认值仍然是 `false`，不要去
"同步"代码默认值**：与 `place_enabled` / `orientation_source` 一样，它就是该两边不同的部署
开关，所以它**不在**任何一致性名单里（守着它的是 `test_gripper.py` 里那条
「开着就必须真找得到脚本」的不变量）。

**2026-09-18 起又多了一对这样的一边倒参数**：`grasp_confirm_required` /
`place_confirm_required` 代码默认都是 `true`，yaml 现场设成 `false`。这是**故意**的
（默认值保的是"逐步确认"这个安全行为，现场用法由 yaml 显式关掉），所以它们也**不在**
任何"默认值必须与 yaml 一致"的名单里 —— 但**逐次读取**，`ros2 param set` 立即生效。

**所以真机一律用 `ros2 launch`**（或至少确认自己清楚哪套值在生效）。
`README_H3001_grasp.md` 里 §3.3 用的是 launch；若要 `ros2 run` 上真机，必须把上面这些
用 `--ros-args -p` 逐个补上。

`test_return_to_standby.py` / `test_place.py` / `test_gripper.py` 的
`ConfigConsistencyTest` 会盯着**新增**参数在两边保持一致（现场标定值与部署开关天然不同，
故意不在名单里）。

---

## 9. 一般来说**不要动**的

| 参数 | 原因 |
| --- | --- |
| `candidates_json_path` / `candidates_topic` / `tcp_pose_topic` / `arm_status_topic` / `move_p_topic` / `move_l_topic` / `grasp_confirm_topic` / `robot_base_frame` | 与 bridge、感知、demo 的接口约定，改了要四处一起改 |
| `track_stable_epsilon` / `stall_detect_epsilon` / `target_stability_tolerance` / `auto_grasp_stable_epsilon` | 噪声量级实测出来的，改大等于把"停稳"判成"一直在动"（`auto_grasp_stable_epsilon` 反过来：改**小**到 5 mm 以下就永远判"物体还在动"，自动下降静默失效） |
| `min_stable_samples` / `stable_duration` | 稳定窗口的样本数与时长，配合使用 |
| `use_sim_time` | 本链路用真实时钟 |

---

## 10. 已知的不一致（待确认）

1. **`standby_pose_reported_mm_deg` 下面的注释是旧值。** 注释里写的是
   `X=78, Y=0, Z=290, RX=2.4, RY=4.2, RZ=-180`（以及换算结果 `0.1977, 0, 0.2988`、
   四元数 `[-0.0366, 0.0209, 0.9991, 0.0008]`），而参数**实际是**
   `[100.0, 0.0, 290.0, 84.4, 0.2, 90.0]`。按后者启动日志打出来的是：

   ```text
   待机位: 上报原点=[0.1  0.   0.29] m, 夹爪=[0.1597 0. 0.2959] m,
           偏移=[0.0597 0. 0.0059] m (模长 60.0 mm),
           姿态=[+0.47406321, +0.47589171, +0.52299816, +0.52465615]
   ```

   **参数值是准的，注释那段是过期的** —— 别照注释去"修"参数。下次改这个参数时顺手把
   注释一起更新。

   > ⚠️ 夹爪/偏移/模长 这三栏按 **`gripper_offset_tool_xyz` 的现值 `[0.0, 0.0, 0.08]`**
   > 算（模长 80mm；2026-09-21 起，此前是 0.06 / 60mm）。yaml 注释与本节旧文里写的
   > `0.1796 / 0.0796 / 80mm` 也正对应 8cm 这一档 —— 换算公式没变，变的是那个参数。
   > 但真机日志里那行是 `夹爪偏移=[ 0.0586  0.0024 -0.0127] m`（抓取姿态下），**模长
   > 60.0 mm**：**注意**，历史记录里有一次 60 mm 的实测，与现在 yaml 的 0.08 不一致，
   > 尚未查清（就是上面那条真机日志）。所以别照那行日志去"修"参数，先把这条查清。

2. `side_orientation_xyzw` 的 yaml 注释说它是"2026-09-14 由实机直接读出"，但
   `orientation_source` 已经是 `fixed` 且姿态在 09-15 的偏移改动后可能又动过 ——
   **上真机遇到姿态不对时，重新读一次比查注释快**。

3. ~~**放置点会让每个任务在加载期就被拒**（2026-09-17 发现）。~~
   **已修复（2026-09-18）**。当时的坑是那条参数写成了毫米量级：

   ```yaml
   place_enabled: true
   place_pose_base_m: [280.0, -240.0, 150.0]   # 旧值, 错的
   ```

   参数名是 `_m`（米）、yaml 注释也写"单位: 米"，但这三个数**是毫米的量级**。
   加载期 `_load_place_pose()` 按米做工作空间检查（`x`/`y` ±0.90、`z` -0.10~0.60、
   模长也在界内），三个轴全部越界 → `WORKSPACE_REJECTED` → `_place_ready = False`
   → **`execute_grasp` 每次都被拒**。

   **留这一段是因为它是个典型错误**：任何一次改放置位姿，改完都要用 `dry_run: true`
   起一次节点，看启动日志里的放置位姿是否真的落在工作空间内 —— 越界不会崩，只会让
   之后每一次 `execute_grasp` 都被拒。

   **2026-09-18 之后这个坑换了形态，但没消失**：参数改成上报末端原点系 + mm（与待机位
   统一），所以"按米填"（`0.1303` 而不是 `130.3`）现在会被"离基座原点太近"这条拦下 ——
   症状一样（所有任务被拒），日志是 `放置点的上报原点离基座原点只有 …mm`。
   判据用**上报原点**而不是换算后的夹爪点：后者 = 上报 + `R @ offset`，即使上报归零也还
   有 60mm 的偏移，永远够不到那个阈值。两侧都另做一次工作空间检查（差一个偏移量，
   一个在界内另一个仍可能越界）。
