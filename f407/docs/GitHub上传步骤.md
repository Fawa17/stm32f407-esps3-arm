# 上传到 GitHub 的步骤

本机已经装了 Git（2.54.0），两个工程目录里也都放好了 `.gitignore`。下面按"最省事"的顺序写。

---

## 0. 先决定仓库怎么分

两种都行：

- **两个仓库**（推荐，最省事）
  - `stm32-arm-f407` → `C:\STM32\smart hand`（下位机：驱动、轨迹、IAP、引导程序）
  - `stm32-arm-s3` → `C:\STM32\hand_s3`（上位机：WiFi、网页、逆运动学、OTA）
- 一个仓库两个目录：需要把两个文件夹挪到同一个父目录再 `git init`，改动多，不建议。

简历里贴链接时，两个仓库都放，或者只放 F407 那个（更能体现底层能力）。

---

## 1. 本地初始化 + 首次提交

在 PowerShell 里（一条一条执行）：

```powershell
cd "C:\STM32\smart hand"

git init
git add .
git status --short | Measure-Object -Line        # 看有多少文件，心里有数
git commit -m "STM32F407 六轴机械臂下位机：六路 PWM + 100Hz S 曲线 + 逆运动学协议 + IAP 引导程序"
git branch -M main
```

第二个工程同理：

```powershell
cd "C:\STM32\hand_s3"
git init
git add .
git commit -m "ESP32-S3 上位机：WiFi + 内置网页 + 逆运动学 + OTA 转发"
git branch -M main
```

> 提交前如果 `git status --short` 里出现 `MDK-ARM/Objects/`、`build/`、`*.axf` 之类，说明 `.gitignore` 没生效（注意 Windows 下 `git` 对大小写不敏感，子目录名要对得上）。这些目录一个就有上百 MB，千万别提交。

---

## 2. 在 GitHub 上建两个空仓库

网页操作：右上角 `+` → **New repository**

- Repository name：`stm32-arm-f407`（另一个是 `stm32-arm-s3`）
- 可见性：Public（简历要用）／Private 也行，但别人点不开
- **不要勾** "Add a README file"、".gitignore"、license —— 本地已经有了，勾了会产生冲突，第一个 push 就会报 `rejected`

建完页面会给你一个地址，形如：

```
https://github.com/<你的用户名>/stm32-arm-f407.git
```

---

## 3. 关联远程仓库并推送

```powershell
cd "C:\STM32\smart hand"
git remote add origin https://github.com/<你的用户名>/stm32-arm-f407.git
git push -u origin main
```

第一次 push 会弹窗要账号密码：

- **用户名**：你的 GitHub 用户名
- **密码**：**不是**账号密码，而是 Personal Access Token

token 生成：GitHub → 右上头像 → Settings → 最下方 Developer settings → **Personal access tokens → Tokens (classic)** → Generate new token (classic) → 勾 `repo` → 生成后**立刻复制**（只显示一次）。把它粘到密码框里。

如果嫌每次输，可以缓存：

```powershell
git config --global credential.helper manager
```

第二次工程重复第 3 步即可（`origin` 名字各自独立，不冲突）。

---

## 4. （可选）改用 SSH，之后不用再输 token

```powershell
ssh-keygen -t ed25519 -C "你的邮箱"
# 一路回车，公钥在 C:\Users\韦兴发\.ssh\id_ed25519.pub
Get-Content "$env:USERPROFILE\.ssh\id_ed25519.pub" | Set-Clipboard
```

GitHub → Settings → **SSH and GPG keys** → New SSH key → 粘贴 → 保存。然后远程地址用 SSH 形式：

```powershell
git remote set-url origin git@github.com:<你的用户名>/stm32-arm-f407.git
git push -u origin main
```

---

## 5. 推之前建议做的三件小事

1. **WiFi 密码不要进公开仓库**。`C:\STM32\hand_s3\main\app_main.c` 里现在写着 `WIFI_SSID "ESP"` / `WIFI_PASS "12345678"`。两个办法：
   - 改成从 `menuconfig`（Kconfig）读，写在 `sdkconfig.defaults` 里，仓库里放个 `sdkconfig.defaults.example`；
   - 或者直接把这两行改成 `#ifndef WIFI_SSID #define WIFI_SSID "your-ssid"` 这种占位，README 里写"改这两行再编译"。
   （热点密码本身没什么价值，但公开仓库里出现明文口令，面试官会扣分。）
2. **补一个 LICENSE**（MIT 最省事）：GitHub 仓库页 → Add file → Create new file → 文件名 `LICENSE` → 右侧 "Choose a license template" → MIT。
3. **换行符**：Windows 提交时可能刷 `LF will be replaced by CRLF` 警告，加一条全局配置就不烦了：
   ```powershell
   git config --global core.autocrlf true
   ```

---

## 6. 之后日常怎么用

```powershell
git status                 # 看改了什么
git add -A                 # 全部加入暂存区（或 git add 文件名）
git commit -m "改了什么"
git push                   # 推到 GitHub
```

改完再推一次就自动更新仓库页面，简历上的链接不用变。

---

## 7. 常见报错对照

| 报错 | 原因 | 处理 |
|---|---|---|
| `remote origin already exists` | 已经加过 origin | `git remote set-url origin <新地址>` |
| `failed to push some refs / fetch first` | 远端有 README 等初始提交 | `git pull --rebase origin main` 再 push |
| `Support for password authentication was removed` | 用了账号密码 | 改用 Personal Access Token |
| `src refspec main does not match any` | 还没 commit，或分支不叫 main | `git branch -M main` 后再 push |
| push 很慢 / 提示文件过大 | 把 `Objects/`、`build/` 加进去了 | `git rm -r --cached ` 那几项，确认 `.gitignore` 后重新提交 |

---

## 8. 想让仓库好看一点（加分项）

- README 顶部放一张实物照 + 一段接线框图（`docs\接线图.png` 现成的），把"能跑起来"直接展示；
- 把 `hand_app.bin`、`bootloader.hex`、`hand_s3.bin` 传到 **Releases**，标签写 `v1.0`，面试官不用装工具链也能刷；
- 在 README 里贴三个自检脚本的运行输出截图（`verify_s3.py`、`verify.py`、`boot_state_check.py`），比写"已测试"有说服力。