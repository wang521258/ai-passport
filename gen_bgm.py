# -*- coding: utf-8 -*-
"""
gen_bgm.py —— 生成 main/bgm_score.h（8-bit 背景乐曲谱 + 频率表）

为什么是"曲谱"而不是"PCM"：
  16 秒 16kHz 单声道 PCM = 512KB，塞进 3.4MB 的 app 分区太亏。
  这里只存音符（128 步 × 3 声部 = 384 字节）+ 61 项频率表，
  由 bgm.c 在设备上实时合成方波/噪声 —— 数据量小到可以忽略。

曲风：Game Boy / NES 芯片音乐（欢快、C 大调、120BPM、8 小节循环）。
※ 原创旋律，只取"8bit 芯片音乐"的听感，不复制任何已有作品的主题曲。

用法：python gen_bgm.py   →  覆盖写入 main/bgm_score.h
"""
import io
import math

RATE = 16000          # 与 SND_PCM_RATE 一致
STEP = 2000           # 16 分音符 = 2000 样本 @16kHz → 120 BPM
BARS = 8
STEPS = BARS * 16     # 128

OUT = "main/bgm_score.h"

# ---------------------------------------------------------------- 和声骨架
# 8 小节：C  Am  F  G  C  Am  F  G
CHORDS = [
    # (根音 MIDI, 和弦音)
    (48, [60, 64, 67, 72]),   # C
    (45, [57, 60, 64, 69]),   # Am
    (41, [53, 57, 60, 65]),   # F
    (43, [55, 59, 62, 67]),   # G
    (48, [60, 64, 67, 72]),   # C
    (45, [57, 60, 64, 69]),   # Am
    (41, [53, 57, 60, 65]),   # F
    (43, [55, 59, 62, 67]),   # G
]

# ---------------------------------------------------------------- 主旋律
# 每小节 = [(MIDI, 占几个 16 分音符), ...]，合计必须 = 16
LEAD_BARS = [
    [(76, 2), (79, 2), (76, 2), (74, 2), (72, 4), (74, 2), (72, 2)],   # C
    [(72, 2), (76, 2), (72, 2), (71, 2), (69, 4), (71, 2), (72, 2)],   # Am
    [(69, 2), (72, 2), (69, 2), (67, 2), (65, 4), (67, 2), (69, 2)],   # F
    [(71, 2), (74, 2), (71, 2), (72, 2), (72, 8)],                     # G
    [(72, 2), (76, 2), (79, 2), (81, 2), (79, 4), (76, 2), (74, 2)],   # C
    [(76, 2), (72, 2), (69, 2), (72, 2), (74, 4), (72, 2), (69, 2)],   # Am
    [(65, 2), (67, 2), (69, 2), (72, 2), (71, 4), (69, 2), (67, 2)],   # F
    [(67, 2), (74, 2), (72, 2), (71, 2), (72, 8)],                     # G
]

# 低音：8 分音符脉冲，根音/五音交替（0=根 1=五）
BASS_PAT = [0, 0, 1, 0, 0, 1, 0, 1]

REST = 0xFF
TIE = 0xFE


def midi_to_q16(m):
    """MIDI 音高 → Q16 相位增量（每样本），满一个周期 = 65536。"""
    f = 440.0 * (2.0 ** ((m - 69) / 12.0))
    return int(round(f * 65536.0 / RATE))


def build_lead():
    out = []
    for bar in LEAD_BARS:
        assert sum(n for _, n in bar) == 16, bar
        for m, ln in bar:
            out.append(m)
            out.extend([TIE] * (ln - 1))
    assert len(out) == STEPS, len(out)
    return out


def build_bass():
    out = []
    for bar in range(BARS):
        root, _ = CHORDS[bar]
        for k in range(8):
            m = root + (7 if BASS_PAT[k] else 0)
            out.append(m)
            out.append(TIE)
    assert len(out) == STEPS
    return out


def build_arp():
    """第 3 声道：16 分音符快速琶音（12.5% 占空比的小音量闪亮音色）。"""
    out = []
    for bar in range(BARS):
        _, tones = CHORDS[bar]
        for s in range(16):
            out.append(tones[s % 4])
    assert len(out) == STEPS
    return out


def build_masks():
    """打击乐掩码，每小节 16 bit（bit i = 第 i 个 16 分音符）。"""
    snare, hat = [], []
    for _ in range(BARS):
        sn, ht = 0, 0
        for s in range(16):
            if s in (4, 12):            # 二、四拍反拍（军鼓）
                sn |= 1 << s
            elif s % 2 == 1:            # 八分音符后半（踩镲）
                ht |= 1 << s
        snare.append(sn)
        hat.append(ht)
    return snare, hat


def fmt(name, ctype, values, per_line=16, hexfmt=True):
    lines = []
    for i in range(0, len(values), per_line):
        chunk = values[i:i + per_line]
        if hexfmt:
            body = ", ".join("0x%02X" % v for v in chunk)
        else:
            body = ", ".join("%d" % v for v in chunk)
        lines.append("    " + body + ",")
    return "static const %s %s[%d] = {\n%s\n};\n" % (ctype, name, len(values), "\n".join(lines))


def main():
    lead, bass, arp = build_lead(), build_bass(), build_arp()
    snare, hat = build_masks()
    freqs = [midi_to_q16(m) for m in range(36, 97)]

    parts = []
    parts.append("// main/bgm_score.h —— 由 gen_bgm.py 生成，勿手改。")
    parts.append("//")
    parts.append("// 8-bit 背景乐曲谱（原创，仅取 Game Boy / NES 芯片音乐的听感）。")
    parts.append("// 存的是音符不是 PCM：128 步 x 3 声部 = 384 字节，设备上实时合成方波。")
    parts.append("#pragma once")
    parts.append("")
    parts.append("#include <stdint.h>")
    parts.append("")
    parts.append("#define BGM_RATE          %d   /* 合成采样率，与 SND_PCM_RATE 一致 */" % RATE)
    parts.append("#define BGM_STEP_SAMPLES  %d    /* 16 分音符帧数 = 120BPM */" % STEP)
    parts.append("#define BGM_STEPS        %d    /* %d 小节 x 16 步 = %.1f 秒一循环 */"
                 % (STEPS, BARS, STEPS * STEP / float(RATE)))
    parts.append("#define BGM_REST         0xFF /* 本步休止 */")
    parts.append("#define BGM_TIE          0xFE /* 延续上一步的音高（不重触发） */")
    parts.append("")
    parts.append("/* 声部 0：主旋律（方波 50% 占空比） */")
    parts.append(fmt("BGM_LEAD", "uint8_t", lead))
    parts.append("/* 声部 1：低音（方波 25%，根音/五音交替） */")
    parts.append(fmt("BGM_BASS", "uint8_t", bass))
    parts.append("/* 声部 2：琶音（方波 12.5%，闪亮小音量） */")
    parts.append(fmt("BGM_ARP", "uint8_t", arp))
    parts.append("/* 打击乐：军鼓（二四拍）/ 踩镲（八分反拍），每小节一个 16bit 掩码 */")
    parts.append(fmt("BGM_SNARE", "uint16_t", snare, per_line=8))
    parts.append(fmt("BGM_HAT", "uint16_t", hat, per_line=8))
    parts.append("/* MIDI 36..96 的 Q16 相位增量 @BGM_RATE（合成器直接查表，不做浮点） */")
    parts.append(fmt("BGM_FREQ_Q16", "uint16_t", freqs, per_line=8, hexfmt=False))

    with io.open(OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(parts))

    print("已生成 %s" % OUT)
    print("  循环长度 %d 步 x %d 样本 = %.1f 秒 @%dHz" % (STEPS, STEP, STEPS * STEP / float(RATE), RATE))
    print("  主旋律非休止步数 %d / %d" % (sum(1 for v in lead if v not in (REST, TIE)), STEPS))
    print("  音域 MIDI %d..%d" % (min(v for v in lead if v not in (REST, TIE)),
                                   max(v for v in lead if v not in (REST, TIE))))
    print("  频率表 %d 项，最大相位增量 %d（<65535 可容纳）" % (len(freqs), max(freqs)))


if __name__ == "__main__":
    main()
