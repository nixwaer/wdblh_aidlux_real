# WDBLH 机器人描述

本目录包含 `wdblh_v5` 的 URDF（统一机器人描述格式）和引用的 STL 网格。
资产用于说明机器人连杆、关节和几何结构，当前串口主控不读取这些文件。

## 文件结构

```text
wdblh_v5/
├── urdf/wdblh_v5.urdf    # 机器人描述
├── meshes/              # URDF 引用的 17 个 STL 网格
├── source-manifest.json # 来源路径与文件校验值
├── SOURCE_LICENSE       # 来源工程的许可证原文
└── README.md            # 本说明
```

URDF 使用 `../meshes/` 相对路径，无需修改原机绝对路径。
移动资源时应一起保留 `urdf/` 和 `meshes/` 的相对位置。
模型的单位、坐标、关节轴、限位和惯性参数均沿用源文件，未重新缩放或标定。

## 来源

文件取自本机 `HTDW4438_HIMloco` 工程的 `resources/robots/wdblh_v5/`。
URDF 和全部网格均逐字节复制，来源及 SHA-256 见 [source-manifest.json](source-manifest.json)。
来源工程许可证按原文保留为 [SOURCE_LICENSE](SOURCE_LICENSE)。

本次仅导入 URDF 和它直接引用的资源。
源目录中的网格压缩包、导出表格、MuJoCo XML 和场景文件未纳入本工程。

## 使用范围

URDF 定义 12 个转动关节和 4 个固定足端关节，关节树共有 17 个连杆。
转动关节的名称和顺序已与当前基础配置核对。
几何尺寸、惯性和零位与实物的对应关系仍需由操作者核对。

本次导入不改变控制配置或启动流程，也不自动增加仿真支持。
