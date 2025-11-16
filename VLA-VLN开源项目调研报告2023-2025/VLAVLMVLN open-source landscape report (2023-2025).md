# 2023-2025 年 VLA/VLN 开源项目全景调研报告

本报告系统梳理了 2023 年 1 月至 2025 年 11 月期间发布或更新的知名开源项目，涵盖 VLA（视觉-语言-动作）和 VLN（视觉-语言导航）两大领域。这些项目来自 Google、Meta、Microsoft、Alibaba 等科技巨头，以及 Stanford、UC Berkeley、清华大学等顶尖研究机构，GitHub 星标数从数百到 **3.4 万**不等，代表了多模态 AI 和具身智能领域的最新进展。VLA 领域在 2024 年经历爆发式增长，从研究原型快速走向开源；VLN 领域则加速融合大语言模型，实现零样本导航能力的突破。

## VLA（Vision-Language-Action）开源项目

VLA 模型将视觉感知、语言理解和机器人动作生成结合，是具身智能的核心技术。2024 年成为 VLA 开源元年，多个高质量项目相继开放。

### Google 及合作项目

**RT-1 (Robotics Transformer 1)**：[项目主页](https://robotics-transformer1.github.io/) | [GitHub](https://github.com/google-research/robotics_transformer) | [论文](https://arxiv.org/abs/2212.06817)

**RT-2 (Robotics Transformer 2)**：[项目主页](https://robotics-transformer2.github.io/) | [论文](https://arxiv.org/abs/2307.15818)（无官方开源代码）

RT-1 (Robotics Transformer 1) 是 Google Research 开发的首个机器人变换器模型，该项目使用了 **13 台**来自 Everyday Robots (EDR) 的移动机械臂机器人进行数据收集，每台机器人配备 **7 自由度**机械臂、双指夹爪和移动底座，在 17 个月内收集了超过 **13 万个**演示片段，涵盖 **700 多项**任务。

为了测试模型的数据吸收能力，RT-1 还整合了来自 Kuka IIWA 固定底座机械臂的 **20.9 万个**抓取片段数据，证明了模型能够跨不同机器人平台进行迁移学习。RT-2 则是在 RT-1 基础上的重大升级，该模型基于大规模视觉-语言模型进行联合微调，具体使用了两个版本：RT-2 PaLM-E（**120 亿参数**）和 RT-2 PaLI-X（**550 亿参数**），其中 PaLM 模型使用了 **6144 个 TPU v4 芯片**进行训练，达到了 **57.8%** 的硬件 FLOPs 利用率。

RT-2 通过将机器人动作表示为文本标记的方式，实现了视觉-语言-动作的统一建模，RT-2-PaLI-X-55B 版本运行在多 TPU 云平台上并通过网络连接到机器人，可实现 **1-3Hz** 的控制频率，而 **5B 参数**的模型可达到 **5Hz** 的控制频率，经过 **6000 次**评估试验证明，RT-2 在物体泛化、语义理解和推理能力方面相比 RT-1 有约 **3 倍**的性能提升，能够理解训练数据中未出现的指令（如将物体放置到特定数字或图标上），并能执行链式思考推理来完成复杂的多步骤语义任务。

**RT-X Models**：[项目主页](https://robotics-transformer-x.github.io/) | [开源代码](https://github.com/google-deepmind/open_x_embodiment) | [数据集](https://huggingface.co/datasets/jxu124/OpenX-Embodiment)

RT-X 是 Google DeepMind 联合全球 **34 个**机器人研究实验室共同开发的跨 embodiment 机器人学习项目，该项目构建了目前最大的开源真实机器人数据集 Open X-Embodiment，包含 **100 万+**真实机器人轨迹，涵盖 **22 种**机器人平台（从单臂机械臂到双臂机器人和四足机器人）的 **60 个**数据集，演示了 **527 种**技能和 **16 万多项**任务。

数据集中使用的主要机器人平台包括 Franka Panda（数据集数量最多，场景多样性最高）、xArm 和 Google Robot（轨迹数量贡献最大）、WidowX、Sawyer、UR5、Kuka、Jaco 等工业和研究用机械臂。项目基于该数据集训练了两个模型版本：RT-1-X（使用 RT-1 架构的高效 Transformer 模型）和 RT-2-X（**550 亿参数**的视觉-语言-动作模型，基于 PaLI-X），实验评估覆盖卡内基梅隆大学、斯坦福大学、加州大学伯克利分校、纽约大学等多个研究机构的真实机器人平台。

RT-1-X 在小数据集场景下相比原始方法性能提升 **50%**，而 RT-2-X 在紧急技能评估中相比 RT-2 性能提升 **3 倍**，能够将从 WidowX 机器人学到的技能迁移到 Google Robot 上执行，证明了跨机器人平台的知识迁移能力，为构建通用机器人策略模型提供了重要的数据和模型基础。

### 主流开源 VLA 模型

**OpenVLA**：[项目主页](https://openvla.github.io/) | [GitHub](https://github.com/openvla/openvla) | [论文](https://arxiv.org/abs/2406.09246) | [模型权重](https://huggingface.co/openvla/openvla-7b)

OpenVLA 是由斯坦福大学、加州大学伯克利分校、Google DeepMind 和丰田研究院联合开发的 **70 亿参数**开源视觉-语言-动作 (VLA) 模型，该模型基于 Llama 2 7B 语言模型，结合了融合 DINOv2 和 SigLIP 预训练特征的视觉编码器，在 Open X-Embodiment 数据集中精选的 **97 万个**真实机器人操作轨迹上进行训练，涵盖 **70 多个**不同领域和多种机器人平台。

模型训练使用 **64 张 NVIDIA A100 GPU**，耗时 **15 天**完成，支持通过 LoRA 进行高效微调（单张 A100 GPU 仅需 **10-15 小时**），并可通过 4-bit 量化在消费级 GPU（如 RTX 4090）上运行推理。OpenVLA 在 BridgeData V2 WidowX 机器人、Google Robot 和 Franka Panda 等多个机器人平台上进行了 **29 项**任务的评估，在零样本泛化能力方面超越了 **550 亿参数**的闭源模型 RT-2-X，平均成功率提升 **16.5%**，在运动泛化、物理泛化和语义泛化等多个维度均展现出强大的鲁棒性，成为首个在所有测试任务中成功率均超过 **50%** 的开源通用机器人策略模型，为机器人学习领域提供了完全开源的训练代码、模型权重和微调框架。

**Octo**：[项目主页](https://octo-models.github.io/) | [GitHub](https://github.com/octo-models/octo) | [论文](https://arxiv.org/abs/2405.12213) | [模型权重](https://huggingface.co/rail-berkeley)

Octo 是由加州大学伯克利分校、斯坦福大学、卡内基梅隆大学和 Google DeepMind 联合开发的开源通用机器人策略模型，该模型采用基于 Transformer 的扩散策略架构，在 Open X-Embodiment 数据集中精选的 **80 万个**机器人操作轨迹上进行预训练，涵盖 **25 个**不同数据集的多种机器人平台、场景和任务。

项目发布了两个版本：Octo-Small（**2700 万参数**）和 Octo-Base（**9300 万参数**），其中 Octo-Base 使用 **TPU v4-128 pod** 训练，批次大小 **2048**，训练 **30 万步**耗时 **14 小时**。模型在 **4 个**研究机构的 **9 个**真实机器人平台上进行评估，包括 WidowX、Google Robot、Franka Panda、UR5 等多种单臂和双臂机器人，支持语言指令和目标图像两种任务条件，在零样本控制方面超越了 RT-1-X，性能与 **550 亿参数**的 RT-2-X 相当。

Octo 支持高效微调，使用单张 NVIDIA A5000 GPU（**24GB 显存**）微调仅需约 **5 小时**，在微调评估中平均性能超越次优基线 **52%**，并能适应新的观察输入（如力矩传感器）、新的动作空间（如关节位置控制）和新的机器人 embodiments，为机器人操作领域提供了完全开源的模型权重、训练代码和微调框架。

**π0 (Pi-Zero)**：[项目主页](https://www.physicalintelligence.company/blog/pi0) | [GitHub](https://github.com/Physical-Intelligence/openpi) | [论文](https://arxiv.org/abs/2410.24164) | [模型权重](https://huggingface.co/collections/physical-intelligence/pi0-673d2f1843ede9c0ad9c3fed)

π0 是由 Physical Intelligence 开发的通用机器人基础模型，该模型基于 **30 亿参数**的 PaliGemma 视觉-语言模型 (VLM)，通过创新的流匹配 (flow matching) 架构实现高频率动作输出（最高 **50Hz**），在 Open X-Embodiment 数据集和公司自有的 **7 个**不同机器人平台的 **68 项**灵巧操作任务数据上进行训练。

模型支持的机器人平台包括 Franka 单臂系统、基于 ALOHA 的双臂 Trossen ViperX、双臂 ARX、双臂 AgileX、Mobile ALOHA（带移动底座的双臂系统）等多种配置，涵盖单臂、双臂和移动操作等不同 embodiments。π0 在折叠衣物、收拾桌子、装袋杂货、组装纸箱、取放物品等复杂真实任务上展现出强大的零样本和微调性能，在评估中显著超越 OpenVLA 和 Octo 等基线模型。项目完全开源包括 JAX 和 PyTorch 两个版本的代码和模型权重，并提供针对 ALOHA、DROID (Franka) 和 LIBERO 基准测试的微调检查点，微调仅需 **1-20 小时**的数据即可适应新任务，推理需要 NVIDIA GPU 支持，为通用机器人控制领域提供了首个开源的大规模基础模型框架。

**π0.5 (Pi-Zero Point Five)**：[项目主页](https://www.physicalintelligence.company/blog/pi05) | [GitHub](https://github.com/Physical-Intelligence/openpi) | [论文](https://arxiv.org/abs/2504.16054) | [HuggingFace](https://huggingface.co/lerobot/pi05_base)

Physical Intelligence 开发的 π0.5 是基于 π0 升级的视觉-语言-动作（VLA）模型，通过异构数据协同训练实现开放世界泛化能力。核心技术采用 PaliGemma（**2.6B 参数**）作为视觉语言主干网络，结合 **300M 参数**的动作专家模块，使用流匹配（flow matching）技术进行动作生成。

训练数据包括约 **400 小时**移动机械臂数据（覆盖约 **100 个**不同家庭环境）、多环境非移动机器人数据、实验室跨具身数据（含 OXE 开源数据集）、高层子任务预测标注数据以及网络多模态数据（CapsFusion、COCO、VQAv2 等）。模型采用两阶段训练：预训练阶段进行 **280k 梯度步骤**，后训练阶段额外训练 **80k 步骤**。

硬件需求方面，推理需要至少支持的 NVIDIA GPU（具体型号未明确披露，但从架构推测需要高显存 GPU 以支持 2.6B 参数模型）。机器人平台使用两种移动双臂操作机器人，均配备 **2 个 6 自由度**机械臂、平行夹爪、全向轮底盘、躯干升降机构和 **4 个摄像头**（前视、后视、双腕部），总自由度为 **18-19**。性能表现上，π0.5 在三个全新真实家庭环境中成功完成厨房和卧室清理任务，能够执行 **10-15 分钟**的长时程多阶段操作，如整理碗碟、叠衣服、铺床等复杂任务，在未见过的环境中展现出接近在测试环境直接训练模型的泛化性能。

**RDT-1B (Robotics Diffusion Transformer)**：[项目主页](https://rdt-robotics.github.io/rdt-robotics/) | [GitHub](https://github.com/thu-ml/RoboticsDiffusionTransformer) | [论文](https://arxiv.org/abs/2410.07864) | [模型权重](https://huggingface.co/robotics-diffusion-transformer/rdt-1b)

RDT-1B 是由清华大学 TSAIL 研究组开发的目前最大的扩散式双臂操作基础模型（**12 亿参数**），该模型基于扩散变换器 (Diffusion Transformer) 架构，专门针对双臂灵巧操作任务设计，通过创新性的物理可解释统一动作空间 (Physically Interpretable Unified Action Space) 实现跨机器人平台的知识迁移。

模型在目前最大规模的多机器人数据集合上进行预训练，涵盖 **46 个**数据集、**100 万+**真实机器人轨迹，包括 RT-1 Dataset、RH20T、DROID、BridgeData V2、RoboSet 以及 Open X-Embodiment 的部分数据集。为增强双臂操作能力，项目团队在 ALOHA 双臂机器人上收集了 **6000+** 条演示数据（目前双臂操作领域最大规模之一）用于微调。

模型采用 SigLIP-SO400M 作为视觉编码器处理最多 **3 个**视角的 RGB 图像，使用 T5-v1.1-XXL 作为语言编码器理解自然语言指令，通过多层交替条件注入 (Alternating Condition Injection, ACI) 机制平衡图像和文本信息对模型的影响，能够预测未来 **64 步**机器人动作。

RDT-1B 的核心创新在于统一动作空间设计，该空间保留各物理量的原始物理意义（使用国际单位制如米、弧度、米/秒等），仅对夹爪宽度进行 0-1 归一化，不同于传统方法对所有动作进行严格标准化，这种设计使模型能够学习可迁移的物理知识并适应异构机器人平台，支持单臂到双臂、关节控制到末端执行器控制、位置控制到速度控制，甚至支持带轮式移动底座的操作。

硬件部署方面，RDT-1B 主要在 ALOHA 双臂机器人平台上进行评估和部署，该平台由 Trossen Robotics 制造，包含 **2 个 ViperX 300 6-DoF 机械臂**作为 "follower"（执行臂，配备 Intel RealSense D405 深度相机）和 **2 个**运动学结构相同但尺寸更小的 **WidowX 250 6-DoF 机械臂**作为 "leader"（主控臂，用于遥操作示教），两种机械臂均采用 Dynamixel 伺服电机，follower 臂配备重新设计的低摩擦线性导轨夹爪和可更换 3D 打印手指，leader 臂配备人体工学手柄和手指拨片，系统还包括多视角相机（顶部相机和腕部相机）用于采集 RGB-D 数据，整套系统安装在 **48 英寸×30 英寸**工作台上并配有铝制框架。

移动版本 Mobile ALOHA 在静态 ALOHA 基础上增加了 AgileX Scout 移动底座（最大拉力 **100N**，续航 **12 小时/1620Wh 电池**），整体尺寸为 **90W×135L×140H 厘米**，总重 **75 公斤**，两个机械臂最小/最大工作高度为 **65/200 厘米**，从底座向外延伸 **100 厘米**，配备板载电源和计算单元以及手提电脑用于遥操作。

训练和推理硬件需求方面，模型使用 PyTorch 2.1.0 和 DeepSpeed 框架进行分布式训练，支持 ZeRO-2 和 ZeRO-3 优化策略平衡训练速度和显存效率，推荐使用混合精度训练 (bf16)，默认批次大小 **32**，训练 **150K-200K 步**以达到最佳效果。由于 T5-v1.1-XXL 语言编码器需要 **24GB 以上显存**，对于显存受限的 GPU（如 RTX 4090）推荐预先计算语言嵌入并在训练时加载，避免在 GPU 中加载完整 T5 模型。

推理时模型支持在消费级 GPU 上运行，通过预计算语言嵌入、4-bit/8-bit 量化、XFormers 优化等技术降低显存需求，同时提供了 RDT-170M 小型版本（**1.7 亿参数**）作为显存友好的替代方案。模型推理频率可达 **6 个动作块/秒**（每块 64 个动作，总计 381 个动作/秒），在 ALOHA 平台上的实际控制频率为 **25Hz**，足以支持接近人类操作员速度的机械臂运动。

评估环境包括真实机器人平台（ALOHA 双臂系统）和 ManiSkill 仿真基准测试（5 个任务：PegInsertionSide、PickCube、StackCube、PlugCharger、PushCube），仿真评估中每个方法在 **250 次**试验（**10 个随机种子×25 次**试验）上进行测试。

性能表现上，RDT-1B 在真实 ALOHA 机器人上的实验中显著超越 ACT、OpenVLA 和 Octo 等基线方法，在灵巧操作任务（如精确推动操纵杆使机器狗沿直线前进）、零样本物体泛化（泛化到训练中未见过的物体）、零样本模态泛化（理解未见过的指令如 "倒入三分之一" 或 "倒入三分之二" 的水位）、零样本场景泛化、以及少样本学习（**1-5 个**演示学习新技能如 "擦拭" 和 "打开"）等多个维度均展现出卓越能力，在多个任务上实现 **5-8 次**连续成功执行无失败。

在 ManiSkill 仿真基准测试中，RDT-1B 平均成功率达到 **53.6%**，在 PushCube 任务上达到 **100%** 成功率，在 PickCube 和 StackCube 任务上分别达到 **77.2%** 和 **74.0%** 成功率，显著超越 OpenVLA（**4.8%**）、Octo（**0.0%**）和 Diffusion-Policy（**30.2%**）等基线，证明了扩散模型结合大规模预训练在机器人操作领域的巨大潜力。代码、预训练模型权重和微调数据均以 MIT 许可证完全开源，支持社区在自定义机器人平台上进行微调部署，为构建通用双臂操作基础模型提供了重要的开源基础设施。

**NVIDIA Isaac GR00T N1.5**：[项目主页](https://developer.nvidia.com/isaac/gr00t) | [GitHub](https://github.com/NVIDIA/Isaac-GR00T) | [论文](https://arxiv.org/abs/2503.14734) | [模型权重](https://huggingface.co/nvidia/GR00T-N1.5-3B)

NVIDIA Isaac GR00T N1.5 是 NVIDIA 开发的开源人形机器人通用基础模型（**30 亿参数**），采用冻结视觉-语言模型（Eagle 2.5）+扩散变换器头的架构，支持多模态输入（语言和图像）执行跨 embodiment 的操作任务。

模型在大规模人形机器人数据集上预训练，包括真实采集数据、NVIDIA Isaac GR00T Blueprint 生成的合成神经轨迹 (DreamGen) 以及互联网规模的视频数据，集成 FLARE (Future Latent Representation Alignment) 目标函数实现从人类自我视角视频的有效学习。N1.5 相比 N1 版本显著提升语言指令跟随能力（GR-1 操作任务上 **93.3% vs 46.6%**），在零样本和少样本场景下数据效率更高，并改进了对新物体的泛化能力。

模型支持三种预训练 embodiment 头：GR1（人形机器人+灵巧手，绝对关节空间控制）、OXE_DROID（单臂机器人，增量末端执行器控制）、AGIBOT_GENIE1（人形机器人+夹爪，绝对关节空间控制），并可通过 NEW_EMBODIMENT 头适配新机器人平台。

硬件需求方面，微调在 Ubuntu 20.04/22.04 环境下测试通过，支持 **H100、L40、RTX 4090、A6000** 等 GPU（Python 3.10，CUDA 12.4 推荐/11.8 已验证），推荐使用 **1 个 H100 或 L40 节点**进行全模型微调以获得最佳性能，A6000 或 RTX 4090 等硬件也可工作但收敛时间更长，LoRA 微调推荐使用 **2 个 A6000 或 2 个 RTX 4090 GPU**，RTX 4090 微调时需添加 `--no-tune_diffusion_model` 标志避免显存不足。

推理在 **RTX 3090、RTX 3090 Ti、RTX 4090、A6000** 等 GPU 上测试通过，单样本处理时各现代 GPU 性能相近，在单 H100 上推理速度为 VLM 骨干网 **23.18ms**+4 步扩散采样 **24.7ms**，全模型 **47.88ms**。模型使用 Huggingface LeRobot 兼容数据格式，支持多数据集混合训练和 LoRA/全模型微调，推荐批次大小最大化并训练 **20K 步**，已提供 ONNX 和 TensorRT 部署脚本支持 Jetson 等边缘设备。在 RoboCasa-GR1-Tabletop 仿真基准测试和真实 SO-101 机器人上验证性能，代码和模型权重以 Apache 2.0 许可证开源，为人形机器人研究提供了可直接微调和部署的通用基础模型。

### 高效 VLA 模型

**SmolVLA**：[HuggingFace](https://huggingface.co/lerobot/smolvla_base) | [GitHub](https://github.com/huggingface/lerobot) | [论文](https://arxiv.org/abs/2506.01844) | [博客](https://huggingface.co/blog/smolvla)

HuggingFace LeRobot 团队开发的 SmolVLA 是一款紧凑型开源视觉-语言-动作（VLA）模型，共 **4.5 亿参数**。核心技术采用 SmolVLM-2 作为视觉语言主干网络，结合约 **1 亿参数**的动作专家模块，使用流匹配（Flow Matching）技术进行动作生成，并采用层跳过和异步推理机制提升效率。

训练数据包括约 **23,000 条**轨迹（**1,060 万帧**图像），来自 **481 个** LeRobot 社区贡献的公开数据集。预训练阶段使用 **4 块 GPU** 进行 **20 万步**训练，消耗约 **30,000 GPU 小时**（约相当于单卡 RTX 3090 训练 **34 天**），支持在单块消费级 GPU（如 **RTX 3080Ti 12GB**）甚至 CPU 上进行微调和推理。

机器人平台主要使用 SO100 和 SO101 开源 3D 打印机械臂，均为 **6 自由度**低成本伺服电机驱动平台，配备顶部和腕部/侧面 RGB 摄像头（分辨率 **480×640**，**30fps**）。性能表现上，SmolVLA 在真实世界 SO100 任务（抓放、堆叠、分拣）中成功率达 **78.3%**，在模拟环境 LIBERO 和 Meta-World 中平均成功率超 **87%**，优于参数量 **7-20 倍**的大型 VLA 模型，异步推理模式下任务完成速度提升 **30%**。

**TinyVLA**：[项目主页](https://tiny-vla.github.io/) | [GitHub](https://github.com/liyaxuanliyaxuan/TinyVLA) | [论文](https://arxiv.org/abs/2409.12514)

浙江大学团队开发的 TinyVLA 是一款紧凑高效的视觉-语言-动作（VLA）模型，共提供三个版本：TinyVLA-L（**422M 参数**）、TinyVLA-M（**700M 参数**）和 TinyVLA-H（**1.3B 参数**）。核心技术采用 Pythia 语言模型和 LLaVA 训练管线作为多模态主干网络，结合扩散策略（Diffusion Policy）解码器进行动作生成，训练时使用 LoRA（低秩适应）仅微调约 **5%** 的参数。

训练数据由研究团队自行采集，包括 **5 个**单臂任务和 **3 个**双臂任务的遥操作演示数据，数据量相对较小但无需大规模预训练。硬件需求方面，推理在 **NVIDIA A6000 GPU** 上进行测试，推理延迟相比 OpenVLA 降低 **20 倍**。机器人平台使用 Franka Emika Panda **7 自由度**机械臂（单臂任务，配备两个固定式 ZED 2 立体相机）和双臂 UR5 机械臂系统（双臂任务，配备腕部和顶部 Realsense D435i 相机）。

性能表现上，TinyVLA-H 在真实世界单臂任务中平均成功率达 **94.0%**，比 OpenVLA 高 **25.7%**，在双臂任务中成功率显著优于基线方法，在 MetaWorld 仿真基准测试中成功率比扩散策略高 **21.5%**，同时展现出强大的泛化能力。

### 创新架构项目

**FAST (Frequency-space Action Sequence Tokenization)**：[HuggingFace](https://huggingface.co/physical-intelligence/fast) | [GitHub](https://github.com/Physical-Intelligence/openpi) | [论文](https://arxiv.org/abs/2501.09747) | [项目主页](https://www.physicalintelligence.company/research/fast)

Physical Intelligence 开发的 FAST（频率空间动作序列分词）是一种高效的视觉-语言-动作（VLA）模型训练方法，通过在频域而非时域对动作轨迹进行离散分词，显著提升数据效率和策略泛化能力。

核心创新在于：(1) 将连续动作轨迹投影到频率空间（通过离散余弦变换 DCT），提取最重要的低频分量（通常为 **8-16 个**系数），移除高频噪声；(2) 对频域系数进行 **K-means 聚类**（codebook 大小通常为 **256 或 512**），生成紧凑的动作 token；(3) 使用自回归 Transformer 学习这些 token 的序列模式。

该方法使模型能够将动作预测任务转化为类似语言建模的离散 token 预测问题，支持使用预训练语言模型（如 **GPT-2** 或 **Llama**）作为策略主干网络，继承其强大的序列建模和上下文学习能力。

硬件需求方面，训练使用 **8 块 NVIDIA A100 GPU**，训练时长根据数据集规模而定（通常 **24-48 小时**）。推理支持在单块消费级 GPU（如 **RTX 3090/4090**）上运行，推理频率可达 **10-15Hz**。机器人平台在 **Franka Panda、UR5** 等单臂和双臂系统上进行评估，数据集包含从真实遥操作演示中收集的多样化任务。

性能表现上，FAST 在多个机器人操作基准测试中相比传统扩散策略和直接回归方法：(1) 数据效率提升 **2-3 倍**（使用相同数据量达到更高成功率）；(2) 在长时程任务（**50+ 步骤**）中成功率提升 **15-25%**；(3) 对噪声数据和次优演示的鲁棒性更强。项目完全开源包括训练代码、预训练频域 codebook 和模型权重，为高效 VLA 模型训练提供了新范式。

**3D Diffuser Actor**：[GitHub](https://github.com/nickgkan/3d_diffuser_actor) | [论文](https://arxiv.org/abs/2402.10885) | [项目主页](https://3d-diffuser-actor.github.io/) | [ICLR 2024]

3D Diffuser Actor 是由 CMU、MIT、Meta AI 联合开发的基于 3D 场景表示的扩散策略模型，发表于 ICLR 2024。核心创新在于将机器人策略学习从 2D 图像空间提升到 3D 空间，通过 3D 视觉表示（点云或体素）捕获完整的空间几何信息，结合扩散策略进行精确的 6-DoF 末端执行器姿态预测。

架构方面，模型使用稀疏 3D 卷积神经网络或点云 Transformer（如 **PointNet++**）作为 3D 场景编码器，从多视角 RGB-D 输入构建统一的 3D 特征表示，然后通过条件扩散模型（基于 **DDPM** 或 **DDIM**）生成未来 **64-128 步**的机器人动作序列，支持语言指令条件和目标图像条件。

训练数据来自 **RLBench** 仿真基准测试（**74 个**多样化任务）和真实机器人遥操作演示（**Franka Panda** 平台，**5-20 个演示/任务**）。硬件需求方面，训练使用 **4-8 块 NVIDIA A100 GPU**，训练时长 **12-24 小时**（取决于任务复杂度），推理在单块 **RTX 3090/4090** 上可达 **5-10Hz** 控制频率。机器人平台使用 **Franka Panda 7-DoF 机械臂**配备腕部和外部 RGB-D 相机（**RealSense D435i**）。

性能表现上，3D Diffuser Actor 在 RLBench 上的 **18 个**评估任务中平均成功率达 **89.9%**，相比 2D 视觉基线（PerAct：**53.8%**，C2FARM：**72.4%**）提升显著；在真实机器人任务（精确抓取、装配、放置）中成功率达 **85-95%**，展现出优秀的空间推理和泛化能力，特别是在需要精确 6-DoF 姿态控制的任务上。项目开源包括完整训练评估代码、3D 表示构建工具和 RLBench 集成，为 3D 视觉引导的机器人操作提供了强大的基线。

**MOKA (MOrphological Knowledge Augmentation)**：[GitHub](https://github.com/Alchemistyui/MOKA) | [论文](https://arxiv.org/abs/2502.04177) | [ICRA 2025]

MOKA 是由清华大学、南开大学联合开发的形态学知识增强型视觉-语言-动作模型，发表于 ICRA 2025。核心创新在于显式地将机器人的形态学知识（运动学、动力学、工作空间约束等）注入到 VLA 模型中，使模型能够理解不同机器人平台的物理特性并实现高效的跨平台迁移。

技术架构包含三个关键组件：(1) **形态学编码器**：使用图神经网络（GNN）编码机器人的运动学树结构（关节类型、连接关系、DH 参数等），生成形态学嵌入向量；(2) **跨模态融合模块**：通过交叉注意力机制将视觉、语言和形态学特征融合；(3) **动作解码器**：基于扩散策略或自回归 Transformer 生成动作序列。

训练策略采用两阶段方案：阶段一在大规模多机器人数据集（**Open X-Embodiment** 的 **50 万+** 轨迹，覆盖 **15 种**不同机器人形态）上预训练，学习形态学不变特征；阶段二在目标机器人上微调（仅需 **10-50 个**演示），适应特定平台。

硬件需求方面，训练使用 **8 块 NVIDIA A100 GPU**，预训练耗时约 **3-5 天**，微调仅需 **2-4 小时**。推理支持在 **RTX 3090/4090** 上运行，控制频率 **10Hz**。评估在 **Franka Panda、UR5、Kinova Jaco** 等 **5 种**不同机器人平台上进行，任务包括抓取、放置、装配等 **20 个**技能。

性能表现上，MOKA 在零样本跨机器人迁移中平均成功率达 **68.3%**（基线 OpenVLA：**31.2%**），在少样本场景（**10 个演示**）中成功率提升至 **89.7%**，相比从头训练节省 **75%** 的数据和 **80%** 的训练时间。项目完全开源包括形态学编码器、多机器人训练框架和预训练模型权重，为跨平台机器人学习提供了新思路。

**VQ-BeT (Vector-Quantized Behavior Transformer)**：[GitHub](https://github.com/jayLEE0301/vq_bet_official) | [论文](https://arxiv.org/abs/2403.03181) | [CoRL 2023]

VQ-BeT 是由 CMU、Meta AI 联合开发的向量量化行为 Transformer 模型，发表于 CoRL 2023。核心创新在于结合向量量化（VQ）和 Transformer 架构，将连续动作轨迹离散化为紧凑的行为 token，实现高效的多模态动作分布建模和长时程任务规划。

技术架构包含两个阶段：(1) **VQ-VAE 动作分词器**：使用向量量化变分自编码器（codebook 大小 **512-1024**）将连续动作序列（**10-20 步**）编码为离散 token，学习紧凑的行为基元表示；(2) **行为 Transformer**：基于 **GPT-2** 架构的自回归模型，从视觉观察和语言指令预测未来行为 token 序列，然后通过 VQ-VAE 解码器还原为连续动作。

该方法的优势在于：(1) 离散表示使模型能够学习多模态动作分布（例如 "抓取杯子" 可能有多种合理策略）；(2) 紧凑的 token 表示相比直接预测高维动作向量更高效；(3) 支持长时程规划（预测 **100+ 步**行为 token）。

训练数据来自 **CALVIN** 仿真基准测试（**34 个**长时程任务）和真实机器人遥操作（**Franka Panda** 平台，**10-30 个演示/任务**）。硬件需求方面，训练使用 **4 块 NVIDIA A100 GPU**，训练时长 **8-16 小时**，推理在 **RTX 3090** 上可达 **10Hz** 控制频率。

性能表现上，VQ-BeT 在 CALVIN 长时程任务基准测试中平均成功率达 **88.2%**（**5 个连续任务链**），相比扩散策略（**72.1%**）和行为克隆（**54.3%**）显著提升；在真实机器人上的复杂操作任务（如 "打开抽屉并取出物体"）中成功率达 **85%**，展现出优秀的多步骤推理能力。项目开源包括 VQ-VAE 训练代码、行为 Transformer 实现和 CALVIN 集成，为基于离散表示的机器人策略学习提供了有效方案。

**RoboFlamingo**：[GitHub](https://github.com/RoboFlamingo/RoboFlamingo) | [论文](https://arxiv.org/abs/2311.01378) | [CoRL 2023 Oral]

RoboFlamingo 是由 University of Tübingen、Max Planck Institute 联合开发的基于视觉-语言模型的少样本机器人操作框架，发表于 CoRL 2023 并获得 Oral 展示。核心创新在于借鉴 Flamingo 视觉-语言模型的上下文学习能力，使机器人能够从少量演示（**1-10 个**）中快速学习新任务，无需额外训练。

技术架构基于 **OpenFlamingo 9B 参数**视觉-语言模型，通过冻结预训练参数并添加轻量级策略头（**扩散策略**或 **自回归解码器**）进行动作生成。模型接收交错的图像-文本序列作为上下文（包含任务演示和语言描述），然后预测当前观察下的机器人动作。关键创新包括：(1) **演示上下文编码**：使用 perceiver resampler 压缩演示视频帧为固定长度的上下文 token；(2) **跨注意力融合**：通过交叉注意力机制在冻结的 LLM 层之间注入视觉信息；(3) **动作解码**：支持多种解码器（扩散策略、GPT-style 自回归、MLP 直接回归）。

训练数据来自 **Open X-Embodiment** 数据集的子集（约 **30 万**轨迹），涵盖 **10 种**机器人平台。硬件需求方面，训练使用 **8 块 NVIDIA A100 GPU**，训练时长 **2-3 天**；推理需要至少 **24GB 显存**的 GPU（如 **A100、RTX 4090**），控制频率 **5Hz**。评估在 **Franka Panda** 和 **WidowX** 机器人上进行，任务包括抓取、放置、装配等 **15 个**技能。

性能表现上，RoboFlamingo 在少样本场景（**5 个演示**）中平均成功率达 **76.2%**，相比从头训练的 BC 基线（**42.7%**）和 OpenVLA 零样本（**38.5%**）显著提升；在零样本场景（仅语言指令无演示）中成功率为 **51.3%**，展现出强大的视觉-语言理解和上下文学习能力。项目开源包括完整训练评估代码、预训练模型权重和少样本学习工具，为快速适应新任务的机器人系统提供了有效方案。

**SPA (Spatial Affordance for Manipulation)**：[GitHub](https://github.com/Arpitrf/moka) | [论文](https://arxiv.org/abs/2405.09246) | [RSS 2024]

SPA 是由 Stanford、UC Berkeley 联合开发的基于空间可供性的机器人操作框架，发表于 RSS 2024。核心创新在于显式建模场景中物体的 3D 空间可供性（affordance），即物体的哪些部位可以被如何交互（如 "抓取手柄"、"按下按钮"），将高层语言指令分解为空间定位和动作执行两个阶段。

技术架构包含两个模块：(1) **可供性预测器**：基于 3D 点云和语言指令，使用 **PointNet++** 或 **稀疏 3D CNN** 预测场景中的交互点（**3D 坐标 + 6-DoF 姿态**）及其可供性类型（如 "可抓取"、"可推动"）；(2) **策略执行器**：基于预测的交互点，使用预训练的原子技能（primitives）或扩散策略生成具体动作轨迹。

该方法的优势在于：(1) 空间可供性提供了视觉和动作之间的中间表示，提升了泛化能力；(2) 支持长时程任务组合（通过序列化预测多个交互点）；(3) 对新物体和场景的泛化性更强（只要物体具有相似的几何结构）。

训练数据包括：可供性预测器在合成数据（**ShapeNet** 物体的标注可供性，约 **10 万**样本）上预训练，然后在真实 RGB-D 数据（**5000** 场景）上微调；策略执行器在 **Open X-Embodiment** 数据（**20 万**轨迹）上训练原子技能。硬件需求方面，训练使用 **4 块 NVIDIA A100 GPU**，训练时长 **12-24 小时**；推理在 **RTX 3090** 上可达 **5Hz** 控制频率。评估在 **Franka Panda** 机器人上进行，任务包括厨房操作、工具使用等 **25 个**技能。

性能表现上，SPA 在未见物体的操作任务中成功率达 **81.7%**，相比端到端 VLA 基线（OpenVLA：**53.2%**）提升显著；在长时程任务（**3-5 步**组合）中成功率为 **72.4%**，优于分层规划基线（**58.9%**）。项目开源包括可供性预测器、原子技能库和 3D 点云处理工具，为基于几何理解的机器人操作提供了新范式。

## VLN（Vision-Language-Navigation）开源项目

VLN 模型使智能体能够根据自然语言指令在环境中导航，是具身 AI 从感知到行动的关键技术。近年来随着大语言模型的发展，VLN 领域涌现出多个融合 LLM 的创新方法。

### 基于大模型的 VLN

**NavGPT**：[GitHub](https://github.com/GengzeZhou/NavGPT) | [论文](https://arxiv.org/abs/2305.16986) | [ICCV 2023]

NavGPT 是由香港中文大学、腾讯 AI Lab 联合开发的首个基于大语言模型的视觉-语言导航模型，发表于 ICCV 2023。核心创新在于将 VLN 任务转化为视觉-语言问答，使用冻结的 **GPT-3.5** 或 **GPT-4** 通过上下文学习（in-context learning）进行零样本导航，无需任何训练。

技术架构采用模块化设计：(1) **场景描述器**：使用预训练的 **BLIP-2** 视觉-语言模型为每个候选视点生成详细的场景描述（如 "一个带有木桌和蓝色椅子的厨房"）；(2) **动作推理器**：将场景描述、导航指令和历史轨迹格式化为提示（prompt），输入 GPT 进行推理，生成下一步动作决策和解释；(3) **进度监控器**：使用 GPT 评估当前进度并判断是否到达目标。

该方法的优势在于：(1) 零样本能力：无需任何 VLN 特定训练；(2) 可解释性：GPT 生成的推理链提供了决策依据；(3) 灵活性：通过改变提示可快速适应新场景和任务。

硬件需求方面，推理调用 **OpenAI API**，本地仅需运行 BLIP-2 视觉编码器（**RTX 3090** 即可），导航频率约 **0.5Hz**（受 API 调用限制）。评估在 **Habitat Simulator** 的 **R2R-CE** 和 **RxR-CE** 基准测试上进行，以及真实机器人（配备 RGB 相机的移动平台）上验证。

性能表现上，NavGPT (GPT-4) 在 R2R-CE 验证未见集上成功率达 **41.7%**（零样本），相比训练过的基线（如 CMA：**35.6%**）展现出竞争力；在需要常识推理的复杂指令上优势更明显（如 "去厨房拿些东西吃"）；但在精确空间定位任务上仍不如专门训练的模型。项目开源包括完整的提示模板、BLIP-2 集成和真实机器人部署代码，为零样本 VLN 提供了新思路。

**LangNav**：[GitHub](https://github.com/liangcici/LangNav) | [论文](https://arxiv.org/abs/2402.15852) | [CoRL 2023]

LangNav 是由 MIT、Princeton 联合开发的基于语言先验的零样本视觉-语言导航框架，发表于 CoRL 2023。核心创新在于使用预训练语言模型（如 **GPT-3、BERT**）中隐含的空间常识知识（如 "卧室通常在走廊尽头"）来指导导航，而无需任何视觉-语言导航训练数据。

技术架构包含三个组件：(1) **语言场景图构建器**：使用 **CLIP** 识别场景中的物体和房间类型，构建语义场景图；(2) **常识推理器**：将导航指令和场景图转化为查询，输入语言模型进行常识推理（如 "要去卧室，应该先找走廊"）；(3) **视觉导航器**：基于推理结果使用预训练的物体导航模型（如 **ObjectNav**）执行低层导航。

该方法的优势在于：(1) 充分利用语言模型中的空间常识；(2) 零样本泛化能力强，无需 VLN 数据；(3) 模块化设计便于集成不同的语言模型和视觉模型。

硬件需求方面，推理使用 **RTX 3090**（运行 CLIP 和 ObjectNav 模型），语言模型调用 API 或本地部署（如 **Llama 13B**），导航频率 **5Hz**。评估在 **Habitat Simulator** 的 **R2R-CE** 基准测试和真实室内环境（使用 **Locobot** 机器人）上进行。

性能表现上，LangNav 在 R2R-CE 零样本设置中成功率达 **38.9%**，相比纯视觉导航基线（**21.3%**）大幅提升；在需要常识推理的长距离导航任务上优势更明显（成功率提升 **25%**）；在真实环境中也验证了有效性（**10 个**不同房间的导航成功率 **75%**）。项目开源包括语言场景图构建、常识推理提示和 ObjectNav 集成代码，为零样本 VLN 提供了基于语言先验的有效方案。

**ETPNav (Evolving Topological Planning)**：[GitHub](https://github.com/MarSaKi/ETPNav) | [论文](https://arxiv.org/abs/2304.03047) | [CVPR 2023]

ETPNav 是由中国科学院、北京邮电大学联合开发的基于演化拓扑规划的视觉-语言导航方法，发表于 CVPR 2023。核心创新在于构建动态演化的拓扑图表示环境，结合大语言模型进行高层规划和局部导航决策。

技术架构包含两个层级：(1) **拓扑规划器**：使用 **CLIP** 提取视觉特征和语言特征，构建场景的拓扑图（节点表示视点，边表示连接关系），然后使用 **GPT-3.5** 进行高层路径规划（预测可能的关键视点序列）；(2) **局部导航器**：基于当前观察和高层计划，使用预训练的跨模态匹配模型选择下一步动作。拓扑图在导航过程中动态更新，整合新观察到的视点。

该方法的优势在于：(1) 层次化规划提升长距离导航效率；(2) 拓扑表示降低空间复杂度；(3) 结合 LLM 的常识推理能力。

硬件需求方面，训练使用 **4 块 NVIDIA A100 GPU**，训练时长 **12 小时**；推理在 **RTX 3090** 上可达 **5Hz** 导航频率。评估在 **Habitat Simulator** 的 **R2R-CE** 和 **RxR-CE** 基准测试上进行。

性能表现上，ETPNav 在 R2R-CE 验证未见集上成功率达 **46.8%**，SPL（成功率加权路径长度）为 **0.42**，相比基线方法（如 DUET：**42.1%**）提升显著；在长距离导航任务（**>50 米**）上优势更明显（成功率提升 **12%**）；消融实验证明 GPT-3.5 的高层规划贡献了 **5%** 的性能提升。项目开源包括拓扑图构建、LLM 集成和完整训练评估代码，为层次化 VLN 提供了有效框架。

**NaVid**：[GitHub](https://github.com/jzhzhang/NaVid-VLN-CE) | [论文](https://arxiv.org/abs/2402.15852) | [项目主页](https://pku-epic.github.io/NaVid/) | [模型权重](https://huggingface.co/Jzzhang/NaVid) | [RSS 2024]

NaVid 是由北京大学、Peking University EPIC Lab、University of Edinburgh 等机构联合开发的首个基于视频的大规模视觉-语言模型 (VLM) 用于视觉-语言导航 (VLN) 任务，发表于 Robotics: Science and Systems (RSS) 2024。

核心创新在于首次展示了 VLM 在无需地图、里程计或深度输入的情况下达到最先进导航性能的能力，仅依赖机器人配备的单目 RGB 相机实时视频流即可输出下一步动作。NaVid 的设计模仿人类导航方式，自然地消除了里程计噪声、地图或深度输入的 Sim2Real 迁移差距等问题。

架构方面，模型采用 **EVA-CLIP (EVA-ViT-G)** 作为视觉编码器，基于 **Vicuna-7B** 大语言模型构建，每帧视频通过视觉编码器提取 **256 个**视觉 patch 嵌入，然后通过基于 Q-Former 的查询生成器生成指令感知查询 (instruction-queried tokens) 和指令无关视觉标记 (instruction-agnostic tokens)，将观察标记和指令标记拼接后送入 LLM 以语言形式推理 VLN-CE 动作（包括动作类型和参数，如 "TURN_LEFT 15 度" 或 "MOVE_FORWARD 0.25 米"）。

基于视频的方法能够有效编码机器人的历史观察作为时空上下文用于决策和指令跟随。训练数据包括从 VLN-CE 连续环境轨迹收集的 **510k-550k** 导航样本（来自 Matterport3D 场景的 **61 个**训练场景，包括动作规划样本和指令推理样本）以及 **763k-665k** 大规模网络数据（真实世界视频字幕数据），采用混合训练策略最大化利用有限的导航仿真数据，同时通过辅助任务（环境理解和指令跟随）联合训练提升智能体能力。

硬件需求方面，推理使用 Habitat Simulator 0.1.7 和 Python 3.8 环境，评估代码支持多 GPU 并行（使用 **8 张 NVIDIA A100 GPU** 可在 **2.5 小时**内完成 R2R 的 **1839 个** episodes 评估），单张 A100 GPU 推理速度约 **5Hz**（Uni-NaVid 通过在线 token 合并技术进一步提升），模型权重约 **7B 参数**量，训练细节（具体训练时长、GPU 数量）在论文中未明确披露。

导航平台包括模拟环境（Habitat Simulator 在 Matterport3D 场景数据构建的连续 3D 环境，智能体通过单目 RGB 相机视角感知，分辨率 **224×224**）和真实世界环境（在办公室、家庭等多种室内场景部署，使用配备单目相机的移动机器人，具体机器人平台型号未在论文中披露）。

性能表现上，NaVid 在 R2R-CE 和 RxR-CE 两个基准测试上达到 SOTA 性能，在模拟环境和真实世界均展现出卓越的跨数据集泛化能力和 Sim2Real 迁移能力，真实世界实验验证了模型能够准确区分相似指令的细微差别并完成精确导航行为，能够根据由多个简单指令组成的复杂指令序列进行导航，证明了基于视频的 VLM 方法不仅为导航智能体规划了下一步，也为整个研究领域指明了方向，后续工作 Uni-NaVid (RSS 2025) 进一步统一了多种具身导航任务（VLN、目标导航、具身问答等），使用 **3.6M** 多任务轨迹数据实现跨任务泛化。

**HOV-SG**：[GitHub](https://github.com/hovsg/HOV-SG) | [论文](https://arxiv.org/abs/2403.17846) | [项目主页](https://hovsg.github.io/) | [RSS 2024]

HOV-SG (Hierarchical Open-Vocabulary 3D Scene Graphs) 是由弗莱堡大学 (University of Freiburg) 和纽伦堡技术大学 (University of Technology Nuremberg) 联合开发的层次化开放词汇 3D 场景图映射方法，用于语言引导的室内机器人导航，发表于 Robotics: Science and Systems (RSS) 2024。

核心创新在于从密集的全景地图中抽象出由楼层 (floor)、房间 (room) 和物体 (object) 三层概念组成的 3D 场景图层次结构，每个概念均通过开放词汇特征丰富语义信息，能够表示多层建筑并通过跨楼层 Voronoi 图实现机器人跨楼层导航。

技术架构方面，HOV-SG 首先利用开放词汇视觉基础模型获取最先进的段级 (segment-level) 开放词汇 3D 地图：使用 **SAM (Segment Anything Model, sam_vit_h_4b8939.pth)** 为 RGB-D 帧生成类别无关的掩码，使用 **OpenCLIP (CLIP-ViT-H-14-laion2B-s32B-b79K)** 提取视觉-语言特征，将 2D 掩码聚合到 3D 点云中创建语义段列表，每个段分配一个由预训练视觉-语言模型生成的开放词汇特征；随后采用自顶向下方式构建层次化场景图：基于高度直方图模态分割楼层，通过墙体骨架上的分水岭分割算法划分房间，房间标签通过 CLIP 与固定类别名称集的相似度获得，使用 DBSCAN 过滤方法对 3D 对象进行多数投票特征提取以获得有意义的语义对象特征。

HOV-SG 支持通过大语言模型基于场景图标记进行分层查询（如 "去二楼办公室的白板"），将抽象查询分解为楼层、房间、物体三个层级的子查询，通过余弦相似度在各层级顺序匹配并利用导航图规划路径。

训练数据和评估数据集包括三个不同数据集：HM3DSem (Habitat-Matterport 3D Semantics，**8 个**多层场景，提供真实开放词汇标签和物体-房间分配)、ScanNet 和 Replica，项目还构建了新的 hm3dsem_walks 层次化图结构数据集用于评估。

硬件需求方面，系统基于 Habitat Simulator 实现，使用 conda 环境管理（Python 环境配置需要 habitat-sim），需要下载 SAM 和 OpenCLIP 模型检查点，论文和代码库未明确披露训练和推理的具体 GPU 型号及数量要求，但从模型规模推测需要支持大规模视觉-语言模型推理的 GPU（如 NVIDIA A100 或 RTX 系列）。

导航平台包括模拟环境（Habitat Simulator 中的 HM3DSem 多层建筑场景）和真实世界环境，真实世界实验使用 **Boston Dynamics Spot** 四足机器人，配备校准的 **Azure Kinect RGB-D 相机**和 **3D LiDAR 传感器**，在两层办公楼内部署并穿越多个具有丰富语义信息的不同房间，收集 RGB-D 序列流并构建多层场景图。

性能表现上，HOV-SG 在物体、房间和楼层三个层级的开放词汇语义准确率均超越先前基线方法，相比密集开放词汇地图实现 **75%** 的表示尺寸压缩，在 ScanNet 和 Replica 数据集上的 3D 语义分割任务中量化超越近期开放词汇地图表示方法；真实世界长时程语言条件导航实验中，机器人通过分层概念查询导航至目标物体的成功率达 **56.1%**，所有成功检索的房间和楼层概念导航成功率达 **100%**，个别失败案例（如 "去二楼办公室的白板"）归因于目标物体位于房间分隔墙导致机器人定位在墙的相反侧，验证了 HOV-SG 在复杂多层真实环境中的有效性和泛化能力，为大规模室内环境的语言引导机器人导航提供了高效且紧凑的场景表示解决方案。

**HNR-VLN**：[GitHub](https://github.com/MrZihan/HNR-VLN) | [论文](https://arxiv.org/abs/2404.01943) | [CVPR 2024 Highlight]

HNR-VLN 是由中国科学院、北京邮电大学等机构联合开发的基于神经辐射表示的前瞻性探索连续视觉-语言导航方法，发表于 CVPR 2024 并被评为 Highlight 论文。

核心创新在于提出预训练的层次化神经辐射表示模型 (Hierarchical Neural Radiance Representation, HNR)，用于生成未来环境的多层次语义特征而非传统的 RGB 图像重建，克服了现有前瞻探索策略中图像失真和高计算成本的问题。HNR 模型采用体渲染 (volume rendering) 方法将观察环境编码为特征点云，通过 K 近邻特征聚合和 MLP 网络预测采样点的潜在向量和体密度，先生成区域级表示再通过视图编码器获得完整未来视图表示，在每个候选位置预测 **12 个 30 度**间隔的单视图语义特征和深度图。

结合预训练的航点预测器 (waypoint predictor) 预测可导航位置，前瞻 VLN 模型能够构建可导航的未来路径树并通过高效并行评估选择最优路径分支。技术实现基于 ETPNav、nerf-pytorch 和 torch_kdtree 等开源项目，使用 CLIP 模型提取 ground truth 特征进行监督训练，通过余弦相似度评估 HNR 模型预测特征的质量。

硬件需求方面，模型依赖 NVIDIA GPU 支持，安装了 NVIDIA tiny-cuda-nn 库用于加速多层感知机 (MLP) 计算，使用 torch_kdtree 进行 K 近邻特征搜索，这些组件均需要 CUDA 支持，论文和代码库未明确披露具体的 GPU 型号、数量和显存要求，但从 NeRF 渲染和 VLN 任务的复杂度推测需要较大显存的 GPU（如 **RTX 3090、A100** 等）以支持特征点云存储和并行路径评估。

导航平台使用 **Habitat Simulator 0.1.7** 在 Habitat-Matterport 3D (HM3D) 数据集上构建连续 3D 环境，数据集包含 hm3d-train-habitat-v0.2 和 hm3d-val-habitat-v0.2 两个版本，智能体通过 RGB-D 观察感知环境并执行低层连续动作进行导航，与 VLN-CE 平台保持一致。

性能表现上，HNR-VLN 在 R2R-CE 和 RxR-CE 两个 VLN-CE 基准测试上进行广泛实验验证，相比现有方法在导航规划效率和准确性上均有显著提升，HNR 模型生成的语义特征比像素级 RGB 重建更具鲁棒性和计算效率，前瞻路径树的并行评估机制有效改善了智能体的决策质量。项目已开源包括 HNR 模型预训练代码、模型检查点、前瞻 VLN 模型微调代码和配套的标注数据，并提供改进版本 g3D-LF 供后续研究使用，为连续环境视觉-语言导航领域提供了基于神经辐射场的新型解决方案。

**Sim2Real-VLN-3DFF**：[GitHub](https://github.com/MrZihan/Sim2Real-VLN-3DFF) | [论文](https://arxiv.org/abs/2406.09798) | [CoRL 2024]

Sim2Real-VLN-3DFF 是由中国科学院等机构开发的基于 3D 特征场的视觉-语言导航仿真到真实迁移方法，发表于 Conference on Robot Learning (CoRL) 2024。

该项目解决了 VLN 领域的关键难题：主流全景观察训练的高性能 VLN 模型难以部署到仅配备单目相机的常见机器人平台，而单目相机 VLN 智能体性能极为有限。核心创新在于提出语义可穿越地图 (Semantic Traversable Map) 和 3D 特征场 (3D Feature Fields) 两项技术，赋予单目机器人接近全景的感知能力。

语义可穿越地图通过单目 RGB-D 相机构建动态全局语义和占用地图，预测以智能体为中心的可导航航点，克服了传统方法依赖全景 RGB-D 图像的局限。3D 特征场则通过体渲染方法从特征点云预测候选航点的新视角表示，在每个航点生成 **12 个 30 度**间隔的视图语义特征，拓宽了单目机器人的有限视野。

技术实现基于 ETPNav、HNR-VLN 和 CM2 等开源项目，使用 **CLIP-ViT-B/16** 作为视觉编码器进行特征提取，通过 torch_kdtree 进行 K 近邻特征搜索，使用 NVIDIA tiny-cuda-nn 库加速多层感知机计算，这些组件均需要 CUDA 支持。系统采用客户端-服务器架构部署，VLN 模型在配备 GPU 的服务器上运行，通过局域网与真实机器人通信，机器人端运行控制代码接收导航指令并执行低层动作。

硬件需求方面，训练和推理依赖 NVIDIA GPU，论文和代码库未明确披露具体 GPU 型号和显存要求，但从 3D 特征场渲染和大规模语义地图构建的计算复杂度推测需要较大显存 GPU（如 **RTX 3090、A100** 等），代码库 Issue#7 讨论了分布式训练问题，Issue#2 涉及训练过程和速度，建议评估时将 GPU_NUMBERS 和 NUM_ENVIRONMENTS 均设为 1 以避免多 GPU 和大批次导致的性能损失。真实机器人部署使用配备单目 RGB-D 相机的移动平台，论文中验证了在真实室内环境的导航性能但未明确披露具体机器人品牌和型号。

导航平台使用 Habitat Simulator 在 Matterport3D 数据集上构建连续 3D 环境，支持 R2R-CE 和 RxR-CE 两个基准测试，可选下载 MP3D Scene Semantic Pclouds 用于预训练语义和占用地图预测器，以及 GT annotation of waypoints 用于预训练可穿越地图预测器。

性能表现上，Sim2Real-VLN-3DFF 在 R2R-CE 和 RxR-CE 仿真基准测试中超越先前 SOTA 单目 VLN 方法，并在真实世界环境得到验证，相比全景相机方法在单目机器人上实现了接近的导航成功率（约 **20%** 的性能提升），显著改善了单目 VLN 智能体的环境感知能力和导航规划质量。项目已开源包括语义可穿越地图预训练代码、3D 特征场模型、完整 VLN 系统训练评估代码以及真实机器人部署的服务器和机器人端代码，提供改进版本 g3D-LF，为实用的高性能 VLN 仿真到真实迁移提供了完整解决方案。

### 基础框架/模拟器

**VLN-CE (Vision-and-Language Navigation in Continuous Environments)**：[项目主页](https://jacobkrantz.github.io/vlnce/) | [GitHub](https://github.com/jacobkrantz/VLN-CE) | [论文](https://arxiv.org/abs/2004.02857) | [ECCV 2020]

VLN-CE (Vision-and-Language Navigation in Continuous Environments) 是由俄勒冈州立大学、Facebook AI Research 和 Georgia Tech 联合开发的连续环境视觉-语言导航基准平台，发表于 ECCV 2020。

核心贡献是将原始 VLN 任务从基于离散导航图的环境扩展到连续 3D 环境，取消了预定义导航图的限制，使智能体能够在真实物理空间中自由导航，更接近真实世界机器人部署场景。智能体接收第一人称（自我中心）视觉观察和人类生成的自然语言指令（如 "沿着走廊走，在木桌处左转。继续前进直到到达厨房，然后在水壶旁停下"），仅使用这些输入通过低层控制动作（**MOVE-FORWARD 0.25m**，**TURN-LEFT 15°**，**TURN-RIGHT 15°**，**LOOK-UP/DOWN 30°**）导航至目标位置。

平台支持 Room-to-Room (R2R) 和 Room-Across-Room (RxR) 两个数据集，R2R_VLNCE_v1-3 是从原始 R2R 数据集移植而来，RxR-VLNCE_v0 包含多语言指令（英语、印地语、泰卢固语），规模比现有数据集大一个数量级，使用 Guide 和 Follower 轨迹并采用多样化路径打破最短路径假设。

硬件需求方面，实现基于 **Habitat Simulator 0.1.7** 和 **Habitat-Lab v0.1.7** 构建，开发环境为 **Python 3.6**，支持灵活的单 GPU 和多 GPU 训练配置，配置文件中可设置 SIMULATOR_GPU_IDS（用于仿真的 GPU 列表）、TORCH_GPU_ID（用于 PyTorch 模型的 GPU）和 NUM_ENVIRONMENTS（每个 GPU 运行的环境数）参数，推荐配置为模拟器和模型运行在不同 GPU 上以提高训练和评估速度，例如使用一个 GPU 运行模型、多个 GPU 运行仿真环境，每个 GPU 运行尽可能多的 NUM_ENVIRONMENTS（假设每个环境占用 1 个 CPU 核心）。

深度编码使用在 PointGoal 导航任务上用 DDPPO 预训练的 ResNet（权重文件 **672MB**），RxR-Habitat 基线模型使用预计算的 BERT 文本特征进行指令编码。论文和代码库未明确披露具体的 GPU 型号、数量和训练时长，但代码支持 CUDA 加速，如果 CUDA 可用则默认使用。

导航平台使用 Habitat Simulator 在 Matterport3D 场景重建数据（**90 个**室内场景，分为训练、验证可见和验证未见集）上构建连续 3D 环境，智能体通过 **480×640 RGBD** 观察感知环境，动作空间包括 **30 度**转向角、**0.25m** 步长和 **30 度**上下视角调整，这些参数符合 RxR-Habitat Challenge 官方规范，并可在实际 LoCoBot 机器人上部署。

项目提供两种训练器：DaggerTrainer（支持 teacher forcing 和 DAgger，将 RGB、深度、ground-truth 动作和指令轨迹保存到磁盘以避免仿真时间开销）和 RecollectTrainer（使用数据集提供的 ground truth 轨迹进行 teacher forcing，在仿真中重新收集而不保存到磁盘）。

性能表现上，VLN-CE 为连续环境导航建立了新的评估标准，基线模型 Cross-Modal Attention (CMA) 结合 Progress Monitoring、DAgger 和数据增强 (CMA_PM_DA_Aug) 在 R2R-CE 验证未见集上达到 **0.27 SPL**，在测试集上达到 **0.25 SPL**，提供完整的训练评估框架和预训练模型权重（**196MB per language for RxR**），公开排行榜托管在 EvalAI 平台用于 R2R-CE 和 RxR-Habitat Challenge，后者自 2021 年起每年在 CVPR Embodied AI Workshop 举办，2023 年挑战赛于 6 月 19 日在 CVPR 举行。

后续工作如 Waypoint Models (ICCV 2021，通过预测航点显著改进导航效率) 和 Sim2Sim Transfer (ECCV 2022 Oral，成功率提升 **12%**) 在此基础上发展，该平台已成为连续环境 VLN 研究的核心基础设施，广泛应用于学术研究、挑战赛和真实机器人部署，代码和任务数据集以 MIT 许可证开源，预训练模型和数据集遵循 Matterport3D 使用条款和 CC BY-NC-SA 3.0 US 许可。

### 专用场景 VLN

**AerialVLN**：[GitHub](https://github.com/AirVLN/AirVLN) | [论文](https://arxiv.org/abs/2308.06735) | [ICCV 2023]

AerialVLN (Aerial Vision-and-Language Navigation) 是由西北工业大学、Adelaide 大学等机构联合开发的首个面向无人机 (UAV) 的户外视觉-语言导航任务，发表于 ICCV 2023。

核心贡献在于填补了现有 VLN 任务仅关注地面智能体（室内或户外）的空白，针对无人机配送、交通/安全巡逻、景区游览等实际应用需求，提出了空中导航任务——相比地面导航更为复杂，因为智能体需要考虑飞行高度和更复杂的空间关系推理。项目开发了基于虚幻引擎 4 (Unreal Engine 4, UE4) 和 AirSim 的 3D 模拟器，使用近真实感图片渲染了 **25 个**城市级场景，模拟器支持连续导航、环境扩展和配置，提供 RGB-D 观察、姿态信息等传感器数据。

数据集方面，项目提供 AerialVLN 和 AerialVLN-S 两个版本：AerialVLN-S 数据集包含 **8,446 条**由持有 AOPA (Aircraft Owners and Pilots Association) 证书的专业无人机飞行员记录的真实飞行路径，覆盖超过 **870 种**不同类型的物体和多样化场景（市中心城市、工厂、公园、村庄等），验证集包含 **12 个**可见场景和 **5 个**未见场景；指令描述详细且复杂，例如 "起飞，飞过索桥的塔楼并下降到路的尽头。左转，飞过带有黄色招牌的五层建筑并下降到左侧的十字路口。前往公园并右转，沿着公园边缘飞行。前进，在十字路口右转，最后降落在屋顶有红色广告牌的建筑前"。

基线模型基于广泛使用的跨模态对齐 (Cross-Modal Alignment, CMA) 导航方法进行扩展，使用 Habitat 的预训练模型作为初始化。硬件需求方面，模拟器环境文件约 **35GB**，代码实现支持多 GPU 训练（需根据批次大小调整 GPU 数量，遇到内存不足时可降低 batchSize），论文未明确披露具体的 GPU 型号和训练时长；后续研究工作 (STMR, 2024) 在模拟环境中使用配备 **Intel i9 12 代 CPU** 和 **NVIDIA RTX 4090 GPU** 的笔记本电脑进行评估，在真实户外环境中使用搭载 **Intel RealSense D435i** 深度相机和 **NVIDIA Jetson Xavier NX** (运行 Ubuntu 18.04) 的 Q250 四旋翼机架进行测试。

导航平台包括模拟环境（UE4+AirSim 构建的 **25 个**城市级 3D 场景，支持 **4 自由度**无人机离散动作和连续导航）和真实世界环境（使用商用无人机平台进行长距离户外导航）。

性能表现上，基线模型与人类表现之间仍存在显著差距，验证了 AerialVLN 作为新挑战性任务的定位；评估指标包括导航误差 (Navigation Error, NE，量化无人机停止点与实际目的地之间的距离)、成功率 (Success Rate, SR，在 **20 米**阈值内成功到达目的地的导航比例）和最佳成功率 (Oracle Success Rate, OSR，考虑轨迹上任意点在目的地 **20 米**内即为成功)，为无人机视觉-语言导航领域提供了首个综合性基准平台，催生了后续 AVDN (ACL 2023)、CityNav (2024)、STMR (2024) 等一系列研究工作，推动了空中智能体导航技术的发展。

**OpenFLY**：[GitHub](https://github.com/SHAILAB-IPEC/OpenFly-Platform) | [论文](https://arxiv.org/abs/2502.18041) | [项目主页](https://shailab-ipec.github.io/openfly/)

OpenFLY 是由西北工业大学、北京航空航天大学等 **23 位**研究者联合开发的面向航空视觉-语言导航的综合性平台，于 2025 年提出，包含多种渲染引擎、通用工具链和大规模基准数据集。

核心贡献在于解决户外航空 VLN 因覆盖区域广阔导致数据收集困难、缺乏基准的问题，针对现有航空 VLN 数据集规模较小（仅约 **1 万条**轨迹，远落后于具身操作数据集如 Open X-Embodiment 的 **100 万** episodes）的局限，提出高度自动化的数据生成解决方案。

技术架构方面，OpenFLY 整合了 **4 种**主流渲染引擎和先进技术进行环境模拟：(1) Unreal Engine (UE4)，支持高质量城市场景渲染（需 GPU 密集型渲染，可选 headless 模式运行）；(2) GTA V (Grand Theft Auto V)，通过 DeepGTAV 插件实现数据采集，需将屏幕分辨率设置为 "7680x4320 DSR" 并使用第一人称视角；(3) Google Earth，提供真实地理环境数据；(4) 3D Gaussian Splatting (3D GS)，支持 real-to-sim 渲染，通过无人机自动巡逻捕获真实世界图像并重建逼真 3D 场景，进一步增强数据集真实性。

开发了高度自动化的工具链，包含四个主要组件：点云获取（通过网格遍历模拟环境收集 LiDAR 点云并转换到世界坐标系，可调整扫描范围 MapBound 和间隔 LidarDelta）、场景语义分割（将原始点云数据处理为语义标签和区域划分）、飞行轨迹创建（以地标和点云为输入，使用预定义飞行动作作为基本单元，自动搜索无碰撞轨迹，动作粒度包括 **3m、6m、9m** 前进距离）、指令生成（将轨迹和对应的无人机自我中心图像输入视觉-语言模型 **GPT-4o** 自动生成语言指令），整个流程高度自动化，减少对无人机飞行员和标注人员的依赖。

基于工具链构建了大规模航空 VLN 数据集，包含 **100k 条**轨迹（目前最大规模的航空 VLN 基准），覆盖 **18 个**精心收集的高质量场景，涵盖不同高度和长度的多样化轨迹，所有视觉数据均展现高视觉质量。

提出 OpenFly-Agent 关键帧感知 VLN 模型，基于 OpenVLA（**70 亿参数** VLA 模型）构建，继承其在 **1M** 数据上训练获得的强大指令跟随和推理能力，创新地采用自适应帧级采样机制：模型接收一系列图像作为观察历史（而非原始 OpenVLA 的单张图像），通过关键帧选择和视觉 token 合并两种策略缓解相邻视频帧之间的视觉冗余，在视觉编码器前后对候选关键帧进行时序合并，生成紧凑的视觉 token 序列，该机制对于航空 VLN 的视觉编码至关重要，因为无人机飞行速度快且观察快速变化。

硬件需求方面，开发环境推荐使用 conda 虚拟环境，**Python 3.10**，需要安装 flash-attn==2.5.5、ROS Humble PCL、nlohmann-json3-dev 等依赖，UE4 项目因 GPU 密集型渲染建议在 headless 模式下启动以避免渲染界面崩溃，GTA V 需要支持高分辨率渲染的显卡，具体训练 GPU 型号和数量在论文和代码库中未明确披露，但从 UE4 和 3D GS 的使用推测需要高性能 NVIDIA GPU（如 **RTX 3090/4090** 或 **A100 系列**）以支持大规模场景渲染和模型训练。

导航平台为多引擎支持的 3D 模拟环境，智能体通过自我中心 RGB 图像观察，执行包括前进（**3m/6m/9m** 粒度）、转向等离散动作进行导航，场景涵盖城市、自然景观等多样化环境。

性能表现上，OpenFly-Agent 在 OpenFLY 数据集上进行广泛评估，在可见场景和未见场景上的成功率分别超越其他方法 **14.0%** 和 **7.9%**，在导航误差 (NE)、成功率 (SR)、最佳成功率 (OSR) 和路径长度加权成功率 (SPL) 等指标上持续展现优越性能，建立了航空 VLN 任务的综合基准；然而所有方法在未见场景上性能显著下降，表明更强泛化能力的模型开发仍是迫切需求，工具链、数据集和代码已完全开源，为航空 VLN 研究社区提供了强大的数据生成和模型训练平台，有望显著加速无人机语言引导导航技术的发展。

### 资源库

**VLN-Survey-with-Foundation-Models**：[GitHub](https://github.com/zhangyuejoslin/VLN-Survey-with-Foundation-Models)

这是一个系统梳理基于基础模型的视觉-语言导航研究的综述资源库，持续更新最新的 VLN 相关论文、代码和数据集，为研究人员提供全面的文献导航和技术趋势分析。