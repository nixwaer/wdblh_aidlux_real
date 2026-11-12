# 第三方声明

本页说明工程来源、上游署名、外部依赖和模型的许可范围。

## 上游代码

本工程提取自用户提供的 `rl_sar_v2` 部署副本，基础框架为
[fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar)。

相关原始声明予以保留，包括：

```text
Copyright (c) 2024-2025 Ziqi Fan
SPDX-License-Identifier: Apache-2.0
```

许可证全文见 [LICENSE](LICENSE)。
必要公共代码包括状态机、循环、日志、观测缓存、数学工具、控制 SDK 和推理接口。
修改后的代码文件保留 2026 年 11 月 11 日的提取与配置修改说明。

## 贡献者署名

[CONTRIBUTORS.md](CONTRIBUTORS.md) 保留上游贡献者名单。
该名单不代表上游贡献者验证或维护本次独立工程。

## 外部依赖

源码包未捆绑以下运行依赖：

- yaml-cpp、oneTBB 和 Python。
- aiohttp、cryptography 和 qrcode。
- Pillow 和 PyYAML。
- 可选的 ONNX Runtime。

使用者应按各依赖的许可证安装和分发对应版本。
本工程的 Apache-2.0 声明不改变这些依赖的许可。

## 供应商组件与模型

使用者应按供应商条款获取 AidLite SDK、QNN248 DSP 插件和设备授权。
工程只保留适配调用代码。

当前 AIDEM 模型从原工程逐字节保留，用于本地迁移。
模型来源权利和公开再分发许可尚未确认，详见[模型说明](policy/wdblh/himloco/model-card.md)。
公开发布前，应确认模型、固件资料和自有修改的发布权限。

## 机器人描述资源
原始来源与文件校验值见[资源说明](resources/robots/wdblh_v5/README.md)。
来源工程的许可证原文单独保留，不由本工程的 Apache-2.0 声明替代。

## 排除的资产

交付未包含原网页证书或私钥、其他机器人 SDK、仿真第三方代码、训练数据或旧模型。
本机凭据和设备授权应保留在私有部署环境。
