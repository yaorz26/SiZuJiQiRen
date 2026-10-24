# Git 管理约定

本仓库远端为 `git@github.com:yaorz26/SiZuJiQiRen.git`，默认分支为 `main`。
2026-10-10 整理后，Git 保存源码、启动配置、脚本、文档及少量构建依赖。
`build/`、`install/`、`log/`、录制、点云和地图等生成文件由根目录
`.gitignore` 排除；原有本地文件仍保留。

## 拉取与依赖

```bash
git clone --recurse-submodules git@github.com:yaorz26/SiZuJiQiRen.git
cd SiZuJiQiRen
```

已有克隆使用：

```bash
git pull --ff-only
git submodule update --init --recursive
```

`src/kiss-icp` 是来自 `https://github.com/PRBonn/kiss-icp.git` 的子模块。
父仓库记录精确提交，初始化时会检出该版本。更新依赖时，先审查上游
改动，再提交父仓库中的子模块指针；普通同步使用上面的命令。

## 日常提交

新功能建议使用独立分支，例如 `codex/功能名称`。提交前检查文件清单和
差异，再明确添加需要保存的源码、配置、脚本及文档：

```bash
git status --short
git diff
git add src/需要提交的包 scripts/需要提交的脚本 docs/需要提交的文档
git diff --cached --stat
git diff --cached
git commit -m "说明具体改动"
git push -u origin HEAD
```

密码、令牌、私钥和本地 `.env` 不进入仓库。生成文件无需强制添加；
重新构建产生的文件不会影响 Git 状态。

## 数据位置与历史文件

大型输出优先放在 `/media/yaorz/Elements/unitree_go2`。创建前确认
`/media/yaorz/Elements` 已挂载且目的目录可写，不自动回退到内部磁盘。
每次实验使用独立目录并保存代码版本、配置快照和输入数据索引。
工作区保存小型索引和分析说明。同一移动硬盘上的副本不算独立备份。

[历史文件索引](../../legacy_artifact_index.csv) 记录本次移出 Git 跟踪的既有
数据、源代码压缩包与旧构建记录，包括原路径、大小、SHA-256 和所在的
旧提交。这些文件仍在本地；本次整理没有将它们迁移到外盘。

此前提交里的编译文件和数据仍能从 Git 历史恢复。因此，这次整理使当前
版本只跟踪必要内容，但不会缩小已有 Git 历史或清除历史中的大文件。
如需恢复索引中的单个文件，可先确认原路径与哈希，再用
`git show 6a6d00e53e3c2dfac142d893b3117a0b20e3c737:原路径` 导出到所需
位置；大型恢复文件也应先确认外盘挂载与写入条件。
