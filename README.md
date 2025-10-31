# MB-TerrainSim: 多波束声呐海底地形采集仿真工具

> Multibeam Bathymetric Terrain Simulation Toolkit

本工具用于实现多波束声呐对海底地形的模拟采集。通过仿真AUV运动并模拟多波束声呐扫测过程，生成适用于水下SLAM研究的仿真数据集。

[![DeepWiki Documentation](https://img.shields.io/badge/docs-DeepWiki-blue?logo=readthedocs&logoColor=white)](https://deepwiki.com/Chi-hong22/MB-TerrainSim) [![ZRead Documentation](https://img.shields.io/badge/docs-ZRead-green?logo=readthedocs&logoColor=white)](https://zread.ai/Chi-hong22/MB-TerrainSim)

## 核心架构与设计

### 统一配置文件系统

项目采用统一的配置文件 `src/config.m` 管理所有可变参数，确保**一体化处理**（`main.m`）和**分步处理**模式的参数一致性。

**主要配置类别：**
- **通用参数**：数据路径、文件名等
- **预处理参数**：降采样、坐标缩放、数据裁剪等
- **INS误差模拟**：各类误差参数、启用开关
- **多波束声呐参数**：深度、探测距离、扇面角度、波束数量
- **后处理参数**：可视化开关、采样率等

### 数据处理管道

项目的数据流采用三阶段管道设计：

1. **预处理阶段**：路径降采样、坐标变换、INS误差注入
2. **仿真阶段**：多波束声呐扫测、子地图生成
3. **后处理阶段**：坐标系转换、PCD文件生成、全局可视化

### 坐标系统约定

- **世界坐标系 (NED)**：北-东-下全局参考系，用于子地图拼接
- **体坐标系 (FRD)**：前-右-下AUV本体系，所有局部点云的参考系
- **旋转约定**：艏向角逆时针为正（数学正向）
- **四元数格式**：统一采用 `[qw, qx, qy, qz]` PCD标准格式

## 原理简介

本工具的主要工作原理包括：

1. **路径处理**：对AUV轨迹进行降采样和坐标变换，并模拟INS误差
2. **多波束仿真**：基于声呐参数（扇面角度、测距等）进行扫测仿真
3. **子图生成**：将采集数据分割为多个子图，并进行坐标系转换
4. **数据导出**：生成TXT和PCD格式的点云数据，便于后续处理

## 运行方法

### 方法一：一体化处理（推荐）

使用`main.m`进行一站式处理：

```matlab
% 1. 配置参数（可选）
params.target_points = 70000;      % 降采样目标点数
params.scale_factor = 6;           % 坐标缩放系数
params.enable_ins_error = true;    % INS误差开关（新增功能）
params.error = struct(...          % INS误差参数（当enable_ins_error=true时生效）
    'line_std', [0.03, 0.05],     % 直线段误差
    'turn_std', [0.0005, 0.02],   % 转弯段误差
    'cumulative', [0.019, 0.0025];     % 累积误差因子 [x, y]
    'turn_factor', [0.01, 0.01];       % 转弯误差系数 [x, y]
    'no_error_fraction', 0.03;         % 无误差路径比例
    'window_size', 40                 % 滑动窗口大小
);

% 2. 运行主程序
main()
```

**INS误差控制说明：**

- `params.enable_ins_error = true`：启用INS误差模拟，生成含误差的仿真路径
- `params.enable_ins_error = false`：关闭INS误差模拟，使用理想路径进行仿真

### 方法二：分步处理

1. 数据预处理：
```matlab
% 在脚本开头设置 INS 误差开关
enable_ins_error = true;  % 或 false
run('dataPreproccess.m')
```

2. 多波束仿真：
```matlab
run('main_multibeamSimulink.m')
```

3. 后处理生成子图：
```matlab
run('dataPostproccess_createSubmap.m')
```

**分步处理模式说明：**

- 支持独立的INS误差开关控制，在`dataPreproccess.m`中修改`enable_ins_error`变量
- `main_multibeamSimulink.m`和`dataPostproccess_createSubmap.m`会自动检测INS文件存在性
- 当INS误差关闭时，后续脚本自动回退到使用理想路径

> 需要注意传递数据的名称(时间戳前缀)

## 输入数据要求

### 地形数据：`MapPoint_900_900.mat`

包含三个 double 矩阵变量：X、Y、Z
- 实际尺寸：900×900
- 扩展尺寸：1100×1100 double（边缘缓冲区）
- 数据说明：
  - X、Y：采样点平面坐标
  - Z：对应点的地形高程值
- 注意：边缘各扩展100个采样点作为缓冲区，用于确保多波束模拟过程中的边界安全
- 可使用[MB-SeabedSim](https://github.com/Chi-hong22/MB-SeabedSim)仓库代码生成符合要求的地形数据

### 路径数据：`PathFollowing.mat`

包含变量：PathFollowing
- 数据结构：n×3 double矩阵
- 列说明：[x, y, theta]
  - x：AUV在X轴位置
  - y：AUV在Y轴位置
  - theta：AUV艏向角（角度制）
- n为路径采样点数量

### 数据格式要求

1. 所有数据必须采用MATLAB .mat格式保存
2. 变量名必须与上述说明保持一致
3. 坐标系统应统一为右手系
4. 角度使用角度制（0-360度）

## 输出说明

程序运行后在`Data`目录下生成以下文件：

### 基础输出文件
1. `YYMMDD_Processed_path_data.mat`：处理后的AUV路径（总是生成）
2. `YYMMDD_recoder.mat`：多波束采集记录（总是生成）
3. `YYMMDD_sub_maps/`：子图数据（总是生成）
   - `TXT_sub_maps/`：TXT格式子图
   - `PCD_sub_maps/`：PCD格式子图

### 条件输出文件
- `YYMMDD_Ins_path_simulated_data.mat`：模拟的INS路径
  - **仅在 `enable_ins_error = true` 时生成**
  - 包含 `ins_path_simulated` 和 `ins_simulated_error` 变量
  - 用于误差分析和后续处理

### 输出模式说明
| INS误差开关 | 生成文件 | 仿真模式 |
|------------|----------|----------|
| `enable_ins_error = true` | 所有文件 | 含误差仿真 |
| `enable_ins_error = false` | 基础文件 | 理想路径仿真 |

## 依赖项

- MATLAB R2020a或更高版本
- Computer Vision Toolbox（用于点云处理）

## 参数配置说明

### INS误差控制参数（新增）
```matlab
params.enable_ins_error = true;    % INS误差开关，true启用/false关闭
```

### 路径处理参数
```matlab
params.target_points = 70000;      % 降采样后的目标点数，影响采样精度
params.scale_factor = 6;           % 坐标缩放系数，用于调整地图尺度
```

### INS误差模拟参数（当enable_ins_error=true时生效）
```matlab
params.error.line_std = [0.03, 0.05];          % 直线段x/y方向标准差
params.error.turn_std = [0.0005, 0.02];        % 转弯段x/y方向标准差
params.error.cumulative = [0.019, 0.0025];     % x/y方向累积误差因子
params.error.turn_factor = [0.01, 0.01];       % 转弯误差x/y方向系数
params.error.no_error_fraction = 0.03;         % 起始无误差段比例
params.error.window_size = 40;                 % 平滑窗口大小
```

**INS误差开关工作模式：**

| 模式 | 设置 | 行为描述 |
|------|------|----------|
| 启用误差 | `enable_ins_error = true` | 生成INS误差文件，使用含误差路径仿真 |
| 关闭误差 | `enable_ins_error = false` | 跳过误差生成，使用理想路径仿真 |

### 多波束声呐参数
```matlab
SONAR_DEPTH = 0;        % 声呐深度（米）
SONAR_RANGE = 100;      % 最大探测距离（米）
SONAR_ANGLE = 60;       % 单侧扇面角度（度）
SONAR_BEAM_NUM = 256;   % 波束数量
```

## 后处理数据管道

### 数据流概述

后处理阶段将多个AUV本体坐标系下的子地图转换为统一世界坐标系下的点云数据，并提供可视化功能。

### 处理流程

1. **TXT到PCD转换** (`submap2PCD`)
   - 读取 `sub_map_*.txt` 文件（包含位姿和局部点云）
   - 将艏向角 `phi` 转换为PCD标准四元数格式 `[qw, qx, qy, qz]`
   - 生成包含 `VIEWPOINT` 位姿信息的PCD文件头
   - 输出：`submap_*.pdc` 文件集合

2. **全局坐标转换** (`loadAllSubmaps`)
   - 解析PCD文件的 `VIEWPOINT` 字段获取位姿信息
   - 使用 `body2World` 函数将局部点云转换到世界坐标系
   - 支持并行处理和可选的坐标变换
   - 输出：世界坐标系下的点云元胞数组

3. **点云可视化** (`visualizeSubmaps`)
   - 聚合所有子地图点云
   - 支持高程着色、子地图索引着色、随机着色
   - 可选降采样以提升渲染性能
   - 输出：全局地形点云可视化图

### 坐标系变换详解

**核心变换公式：**
- 世界→体坐标：`points_local = R * (points_world - t)`
- 体→世界坐标：`points_world = R' * points_local + t`

其中 `R` 为旋转矩阵，`t` 为平移向量，`R'` 为 `R` 的转置（逆矩阵）。

**四元数约定：**
- 统一采用PCD标准格式：`[qw, qx, qy, qz]`（实部在前）
- 艏向角逆时针为正（数学正向）
- 自动归一化处理，确保数值稳定性

### PCD文件格式

**头文件关键字段：**
```
VIEWPOINT tx ty tz qw qx qy qz
```
- `tx ty tz`：关键帧在世界坐标系中的位置
- `qw qx qy qz`：体坐标系到世界坐标系的旋转四元数

**数据格式：**
```
DATA ascii
x1 y1 z1
x2 y2 z2
...
```
每行为体坐标系下的一个三维点。

### 核心函数依赖关系

**主要驱动脚本：**
- `main.m` - 一体化处理入口
- `dataPostproccess_createSubmap.m` - 后处理主控脚本

**核心处理模块：**
- `submap2PCD.m` - 子地图格式转换
- `loadAllSubmaps.m` - 全局坐标聚合
- `visualizeSubmaps.m` - 点云可视化

**坐标变换工具集 (`src/utils/transform/`)：**
- `angle2Quaternion.m` - 角度到四元数转换
- `quaternion2RotationMatrix.m` - 四元数到旋转矩阵
- `body2World.m` - 体坐标到世界坐标
- `world2Body.m` - 世界坐标到体坐标
- `angle2RotationMatrix.m` - 角度到旋转矩阵

**配置管理：**
- `src/config.m` - 统一配置文件

## 版本管理

### v1.4.0 (250919) - 子地图可视化与坐标系统统一化

- ✨ 四元数格式统一为PCD标准 `[qw, qx, qy, qz]`
- 🔧 简化数据流，移除重排序操作
- 📚 完善后处理数据管道文档
- 🔍 新增子地图可视化检查功能，支持全局点云拼接验证
- 🎯 提高代码可维护性和一致性

### v1.3.0 (250826) - INS误差开关功能

- ✨ 核心功能：
  - **新增INS误差开关控制**：支持 `enable_ins_error` 参数控制误差模拟
  - **双模式运行支持**：函数式（main.m）+ 脚本式（dataPreproccess.m）
  - **智能文件检测**：自动检测INS文件存在性，支持优雅回退
  - **空数组语义**：关闭误差时返回空数组，下游自动跳过处理

### v1.2.2 (250111)

- ✨ 新增 angle2Quaternion 函数和坐标转换优化
- 🔧 更新文档说明和注释完善
- 🐛 修复子地图四元数异常问题

### 历史版本

<details>
<summary>点击展开查看历史版本详情</summary>

### v1.2.1 (250104)
- ✨ 完整INS误差模拟模块、PCD格式支持、数据可视化
- 🔧 路径处理重构、子图算法优化、配置接口改进
- 🐛 坐标转换精度、内存泄漏、点云索引问题修复

### v1.2.0 (241230)
- ✨ 基于实验数据的INS误差模拟、数据预处理模块、路径可视化
- 🔧 降采样算法改进、数据存储结构优化

### v1.1.0 (241219)
- ✨ 子图自动生成、数据后处理模块、TXT格式导出
- 🔧 多波束算法优化、数据输出格式改进

### v1.0.0 (241216)
- 🎉 初始版本：基础多波束仿真、简单路径处理、基本可视化

</details>

### 开发计划

- [ ] 优化INS路径噪声模拟算法

## 文件结构

```shell
240409_多波束模拟地形采集/
├── README.md                           # 项目说明文档
├── .gitignore                          # Git忽略文件配置
├── src/                               # 源代码目录
│   ├── main.m                         # 一体化处理主程序入口
│   ├── main_multibeamSimulink.m       # 多波束仿真主程序
│   ├── dataPreproccess.m              # 数据预处理脚本
│   ├── dataPostproccess_createSubmap.m # 后处理主控脚本
│   ├── config.m                       # 统一配置文件
│   └── utils/                         # 工具函数库
│       ├── *.m                        # 各种处理工具函数
│       └── transform/                 # 坐标变换工具集
│           └── *.m                    # 坐标变换相关函数
├── Data/                              # 数据目录 (不上传GitHub)
│   ├── MapPoint_900_900.mat          # 地形数据
│   ├── *_PathFollowing.mat            # AUV路径数据
│   └── YYMMDD_*/                      # 各种输出数据
├── Backup/                            # 备份文件目录 (不上传GitHub)
├── Picture/                           # 图片资源目录 (不上传GitHub)
└── Docs/                             # 详细技术文档 (不上传GitHub)
```

### 目录说明

- **`src/`**: 核心源代码，包含主程序和工具函数
- **`Data/`**: 输入数据和输出结果存储
- **`Backup/`**: 历史版本和备份文件
- **`Picture/`**: 项目相关图片和可视化结果
- **`Docs/`**: 详细技术文档（本地开发使用）

### Git忽略配置

根据 `.gitignore` 设置，以下目录不会上传到GitHub：
- `Data/` - 数据文件通常较大，本地存储
- `Backup/` - 备份文件，避免仓库冗余
- `Picture/` - 图片资源，本地管理
- `Docs/` - 技术文档，内部使用

## 使用建议

1. **INS误差开关选择**
   - **研究含误差SLAM算法**：设置 `enable_ins_error = true`，获得更真实的仿真数据
   - **算法基础验证**：设置 `enable_ins_error = false`，使用理想数据验证算法正确性
   - **性能对比研究**：交替使用两种模式，对比算法在不同条件下的表现

2. **参数调优**
   - 根据实际需求调整`target_points`以平衡精度和计算效率
   - 通过`error`参数模拟不同精度等级的INS系统
   - 调整`SONAR_ANGLE`和`SONAR_BEAM_NUM`以匹配实际声呐设备

3. **数据预处理**
   - 确保输入地形数据范围合适
   - 检查AUV路径是否覆盖目标区域
   - 路径降采样率会影响仿真精度

4. **性能优化**
   - 对于大规模数据，建议增加内存分配
   - 可通过并行计算加速处理
   - 适当调整子图大小以优化后续SLAM处理

## 常见问题

1. **INS误差开关相关**
   - **问题**：设置 `enable_ins_error = false` 后仍然使用了误差路径
   - **解决**：检查是否有旧的INS文件残留，或确认参数设置正确
   - **问题**：分步处理模式下开关不生效
   - **解决**：确保在 `dataPreproccess.m` 中正确设置了 `enable_ins_error` 变量

2. **内存不足**
   - 减小`target_points`
   - 降低地形数据分辨率
   - 分批处理大型数据集

3. **精度问题**
   - 检查坐标系统是否统一
   - 确认INS误差参数合理性
   - 验证声呐参数设置

4. **数据导出**
   - 确保输出目录具有写入权限
   - 检查磁盘空间是否充足
   - 验证PCD文件格式兼容性

5. **文件检测问题**
   - **问题**：后续脚本无法正确检测INS文件
   - **解决**：确认文件命名格式正确，检查Data目录路径设置

## 联系方式

- 作者：[Chihong（游子昂）](https://github.com/Chi-hong22)
- 邮箱：`you.ziang@hrbeu.edu.cn`
- 项目地址：[MB-TerrainSim](https://github.com/Chi-hong22/MB-TerrainSim)

