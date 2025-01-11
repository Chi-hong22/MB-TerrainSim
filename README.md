# MB-TerrainSim: 多波束声呐海底地形采集仿真工具

> Multibeam Bathymetric Terrain Simulation Toolkit

本工具用于实现多波束声呐对海底地形的模拟采集。通过仿真AUV运动并模拟多波束声呐扫测过程，生成适用于水下SLAM研究的仿真数据集。

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
params.error = struct(...          % INS误差参数
    'line_std', [0.03, 0.05],     % 直线段误差
    'turn_std', [0.0005, 0.02],   % 转弯段误差
    'cumulative', [0.019, 0.0025];     % 累积误差因子 [x, y]
    'turn_factor', [0.01, 0.01];       % 转弯误差系数 [x, y]
    'o_error_fraction', 0.03;         % 无误差路径比例
    'window_size', 40                 % 滑动窗口大小
);

% 2. 运行主程序
main()
```

### 方法二：分步处理

1. 数据预处理：
```matlab
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

> 需要注意传递数据的名称(时间戳前缀)

## 输入数据要求

- 地形数据：`MapPoint_900_900.mat`（包含X、Y、Z三个矩阵）
- 路径数据：`PathFollowing_1.mat`（包含AUV轨迹点）

## 输出说明

程序运行后在`Data`目录下生成以下文件：

1. `YYMMDD_Processed_path_data.mat`：处理后的AUV路径
2. `YYMMDD_Ins_path_simulated_data.mat`：模拟的INS路径
3. `YYMMDD_recoder.mat`：多波束采集记录
4. `YYMMDD_sub_maps/`：子图数据
   - `TXT_sub_maps/`：TXT格式子图
   - `PCD_sub_maps/`：PCD格式子图

## 依赖项

- MATLAB R2020a或更高版本
- Computer Vision Toolbox（用于点云处理）

## 参数配置说明

### 路径处理参数
```matlab
params.target_points = 70000;      % 降采样后的目标点数，影响采样精度
params.scale_factor = 6;           % 坐标缩放系数，用于调整地图尺度
```

### INS误差模拟参数
```matlab
params.error.line_std = [0.03, 0.05];          % 直线段x/y方向标准差
params.error.turn_std = [0.0005, 0.02];        % 转弯段x/y方向标准差
params.error.cumulative = [0.019, 0.0025];     % x/y方向累积误差因子
params.error.turn_factor = [0.01, 0.01];       % 转弯误差x/y方向系数
params.error.no_error_fraction = 0.03;         % 起始无误差段比例
params.error.window_size = 40;                 % 平滑窗口大小
```

### 多波束声呐参数
```matlab
SONAR_DEPTH = 0;        % 声呐深度（米）
SONAR_RANGE = 100;      % 最大探测距离（米）
SONAR_ANGLE = 60;       % 单侧扇面角度（度）
SONAR_BEAM_NUM = 256;   % 波束数量
```

## 版本管理

### v1.2.2 (250111)

- ✨ 新增功能：
  - 新增 angle2Quaternion 函数用于角度到四元数的转换
  - 优化 coordinateTransform 函数，增加数据验证和错误处理
- 🔧 优化改进：
  - 更新 README.md 中的作者信息、邮箱和项目地址
  - 移除 README.md 中的许可证信息
  - 更新数据预处理、多波束仿真、后处理等脚本的注释和使用说明
  - 完善多个函数的误差生成算法和使用说明
- 🐛 问题修复：
  - 修复子地图四元数异常的问题

### v1.2.1 (250104)

- ✨ 新增功能：
  - 添加了完整的INS误差模拟模块
  - 支持PCD格式点云数据输出
  - 新增多种数据可视化选项
- 🔧 优化改进：
  - 重构了路径处理模块，提高计算效率
  - 优化了子图生成算法
  - 改进了参数配置接口
- 🐛 问题修复：
  - 修复了坐标转换中的精度损失问题
  - 解决了大规模数据处理时的内存泄漏
  - 修复了点云数据索引错误

### v1.2.0 (241230)

- ✨ 新增功能：
  - 实现基于实验数据的INS误差模拟
  - 添加数据预处理模块
  - 新增路径可视化功能
- 🔧 优化改进：
  - 改进降采样算法
  - 优化数据存储结构

### v1.1.0 (241219)

- ✨ 新增功能：
  - 实现子图自动生成
  - 添加数据后处理模块
  - 支持TXT格式数据导出
- 🔧 优化改进：
  - 优化多波束模拟算法
  - 改进数据输出格式

### v1.0.0 (241216)

- 🎉 初始版本发布：
  - 基础多波束声呐仿真功能
  - 简单路径处理
  - 基本数据可视化

### 开发计划

- [ ] 优化INS路径噪声模拟算法

## 文件结构

```shell
├── main.m                             # 主程序入口
├── simulateMultibeam.m               # 多波束仿真核心函数
├── preprocessData.m                  # 数据预处理函数
├── postprocessData.m                 # 数据后处理函数
├── utils/                           # 工具函数目录
│   ├── multibeam.m                  # 多波束模型实现
│   ├── generateSimulatedInsPath.m   # INS误差生成
│   ├── createSubmap.m               # 子图生成算法
│   └── ...
└── Data/                           # 数据目录
    ├── MapPoint_900_900.mat        # 地形数据
    └── PathFollowing_1.mat         # 路径数据
```

## 使用建议

1. **参数调优**
   - 根据实际需求调整`target_points`以平衡精度和计算效率
   - 通过`error`参数模拟不同精度等级的INS系统
   - 调整`SONAR_ANGLE`和`SONAR_BEAM_NUM`以匹配实际声呐设备

2. **数据预处理**
   - 确保输入地形数据范围合适
   - 检查AUV路径是否覆盖目标区域
   - 路径降采样率会影响仿真精度

3. **性能优化**
   - 对于大规模数据，建议增加内存分配
   - 可通过并行计算加速处理
   - 适当调整子图大小以优化后续SLAM处理

## 常见问题

1. **内存不足**
   - 减小`target_points`
   - 降低地形数据分辨率
   - 分批处理大型数据集

2. **精度问题**
   - 检查坐标系统是否统一
   - 确认INS误差参数合理性
   - 验证声呐参数设置

3. **数据导出**
   - 确保输出目录具有写入权限
   - 检查磁盘空间是否充足
   - 验证PCD文件格式兼容性

## 联系方式

- 作者：[Chihong（游子昂）](https://github.com/Chi-hong22)
- 邮箱：[you.ziang@hrbeu.edu.cn]
- 项目地址：[GitHub-Repository-URL]

## 许可证

本项目基于 MIT 许可证开源。
