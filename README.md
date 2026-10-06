# Hermes
My First Robot, Coaxial Hip-mounted Dual-actuated Parallelogram Wheel-leg Mechanism.

## 工程组成

- [wheel_leg_simulation](references/wheel_leg_simulation)：MuJoCo 建模、控制、仿真配置、测试与结果。
- [wheel_leg_hardware](references/wheel_leg_hardware)：机械/电气结构、CAD、制造文件、BOM 与装配资料。
- [参考图](documents/pic/)：轮腿机构外观与关节位置参考。

两个工程以 Git 子模块保存，各自维护独立仓库；Hermes 固定引用具体提交版本。

## 获取完整工程

先安装 Git 和 Git LFS，然后执行：

```bash
git lfs install
git clone --recurse-submodules https://github.com/Stella-CN/Hermes.git
cd Hermes
git -C references/wheel_leg_hardware lfs pull
```

已有克隆可执行 `git submodule update --init --recursive`，再执行上述 `lfs pull` 命令补齐硬件大文件。硬件子模块包含数 GiB 文件，并引用 Gitee 上的电机资料子模块，需要能访问 GitHub 和 Gitee。运行和依赖说明见各子模块 README。
