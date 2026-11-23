"""
boot_state_check.py —— 用 Python 重放引导程序的"启动计数 / 健康印章 / 回滚"状态机，
确认边界（连续几次才回滚、回滚后会不会无限回滚、盖章后计数是否真的清零）。

映射关系（与 C 代码一一对应）：
  bootloader_main.c main():  擦元数据 / 计数 / 回滚 / 记一次启动
  Backup.c Backup_Healthy(): 应用跑健康后盖章

只建模"元数据区 sector 7"的四个关键位置：
  0x00 magic   0x0C state   0x10.. slots[8]   0x30 seal
"""

import sys
ERASED = 0xFFFFFFFF
META_MAGIC = 0x54414D45
META_PENDING = 1
META_RESTORED = 2
BOOT_MARK = 0x4B4F4F42
SEAL_MAGIC = 0x4C414553
SLOTS = 8
MAX_TRIES = 3


sys.stdout.reconfigure(encoding='utf-8', errors='replace')


class Meta:
    def __init__(self):
        self.erase()

    def erase(self):
        self.magic = ERASED
        self.state = ERASED
        self.slots = [ERASED] * SLOTS
        self.seal = ERASED


    def boots(self):
        return sum(1 for s in self.slots if s == BOOT_MARK)

    def mark_boot(self):
        for i in range(SLOTS):
            if self.slots[i] != BOOT_MARK:
                self.slots[i] = BOOT_MARK
                return

    def write(self, off, val):
        """模拟"只能 1→0"的编程：目标必须还是擦除态"""
        if off == 0x00:
            assert self.magic == ERASED, "重复写 magic"
            self.magic = val
        elif off == 0x0C:
            assert self.state == ERASED, "重复写 state"
            self.state = val
        elif off == 0x30:
            assert self.seal == ERASED, "重复写 seal"
            self.seal = val
        else:
            raise AssertionError(hex(off))


RESTORES = [0]   # 模块级：erase 不该把它清掉（真实硬件里它是外部观察值）


def boot(meta, app_healthy_after, backup_ok=True, verbose=False):
    """上电一次：返回 'install' / 'restore' / 'jump'"""
    events = []

    # 1) 装新固件（这里不模拟 IAP，直接看计数逻辑）
    # 2) 上次盖章了 → 计数清零
    if meta.seal == SEAL_MAGIC:
        meta.erase()
        events.append("seal→erase")

    # 3) 数连续未盖章启动
    boots = meta.boots()

    # 4) 连续 3 次没印章 → 回滚（只回滚一次）
    if boots >= MAX_TRIES and meta.state != META_RESTORED:
        if backup_ok:
            meta.erase()
            meta.write(0x00, META_MAGIC)
            meta.write(0x0C, META_RESTORED)
            RESTORES[0] += 1
            events.append("RESTORE")

    # 5) 记一次启动
    meta.mark_boot()

    # 应用侧：跑满 3 秒就盖章
    if app_healthy_after:
        if meta.seal != SEAL_MAGIC:
            meta.write(0x30, SEAL_MAGIC)
            events.append("SEAL")

    if verbose:
        print("  boots_before=%d events=%s state=%#x slots=%d"
              % (boots, events or ["-"], meta.state & 0xFFFFFFFF, meta.boots()))
    return events


def scenario(name, boots_list, backup_ok=True):
    RESTORES[0] = 0
    m = Meta()
    print(name)
    for i, healthy in enumerate(boots_list, 1):
        ev = boot(m, healthy, backup_ok)
        print("  第 %d 次上电: healthy=%-5s -> %s" % (i, healthy, ev or ["(仅记一次启动)"]))
    print("  结果: 回滚次数=%d, state=%#x, 累计计数=%d\n"
          % (RESTORES[0], m.state & 0xFFFFFFFF, m.boots()))
    return m


# 场景 1：正常固件（每次都健康）——永远不该回滚
m = scenario("场景1：老固件一直正常", [True, True, True, True])
assert RESTORES[0] == 0

# 场景 2：新固件一上电就崩（从不盖章）——应在第 4 次上电时回滚，且只回滚一次
m = scenario("场景2：新固件一直起不来", [False] * 8)
assert RESTORES[0] == 1, RESTORES[0]

# 场景 3：回滚后恢复健康——计数清零，回到正常
m = scenario("场景3：回滚后恢复健康", [False, False, False, False, True, True])
assert RESTORES[0] == 1

# 场景 4：备份也没有（芯片没焊 / 备份无效）——不回滚，也不卡死
m = scenario("场景4：备份不可用", [False] * 6, backup_ok=False)
assert RESTORES[0] == 0

# 场景 5：偶尔启动一次健康（用户手动复位抖动）——不应误回滚
m = scenario("场景5：断续健康", [False, False, True, False, False, True, False, False])
assert RESTORES[0] == 0, m.restores

print("全部断言通过 ✅  （回滚只在'连续 3 次未盖章且未回滚过'时发生一次）")