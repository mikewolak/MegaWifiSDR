# MegaWifi SDR Receiver

A software-defined radio front end for the Sega Genesis, using the
MegaWifi ESP32-C3 cartridge to connect to remote KiwiSDR receivers
over WiFi and play live shortwave radio through the Genesis audio bus.

```
┌─────────────────────────────────────────────────────────────┐
│  ====== MEGAWIFI SDR RECEIVER ======                        │
│  Reverb: ON   [START] Settings                              │
│  sdr.hfunderground.com                S:-87                 │
│  > 14.200 MHz  USB  BW:2.8k                                │
│  [10/18] 20m Voice USB                                      │
│                                                             │
│  Vol: ████████████████░░░░ 75%                              │
│  ESP32-C3 CPU: 8%                                           │
│  [A]Scan [B]Mode [C]Band [LR]Vol                            │
│                                                             │
│  ~~~ KiwiSDR WebSocket protocol ~~~ Tune shortwave! ~~~     │
│  ══════════════════════════════════                          │
└─────────────────────────────────────────────────────────────┘
```

## Architecture

```
KiwiSDR Server (remote)
       │
  WebSocket (ws://host:8073/TIMESTAMP/SND)
       │
┌──────▼──────────────────────────────────┐
│  ESP32-C3 (MegaWifi)                    │
│                                         │
│  WebSocket Client                       │
│       │                                 │
│  SND Frame Parser (10-byte header)      │
│       │                                 │
│  Raw PCM / IMA-ADPCM Decode             │
│       │                                 │
│  Resample 12kHz → 44.1kHz              │
│       │                                 │
│  Anti-Imaging LPF (Butterworth biquad)  │
│       │                                 │
│  8-Channel Mixer ──→ Freeverb           │
│       │                  │              │
│  PWM Audio Driver (12-bit LEDC)         │
│  GPIO4(L) GPIO5(R)                      │
└──────┬──────────────────────────────────┘
       │
  RC Filter (R=2.2k, C=10nF)
       │
  Genesis Audio Bus (cart B8/B9)
```

```
┌──────────────────────┐         ┌────────────────────────────┐
│  Sega Genesis 68k    │  UART   │  ESP32-C3 (MegaWifi)       │
│                      │ 1.5Mbit │                            │
│  Tuner UI ───────────┼─────────┼→ MW_CMD_SDR_* handlers     │
│  Preset selector     │  LSD    │                            │
│  Scan engine         │  ch 0   │  KiwiSDR WebSocket Client  │
│  S-meter display     │←────────┼─ Packed status (8 bytes)   │
│  Reverb control      │         │  Freeverb send/return      │
│  Marquee scroller    │         │  CPU % profiling           │
│                      │         │                            │
│  SGDK + mw-api       │         │  ESP-IDF v5.x              │
└──────────────────────┘         └────────────────────────────┘
```

## Controls

| Button | Action |
|--------|--------|
| **A** | Toggle scan (cycles presets, pauses on signals) |
| **B** | Cycle demodulation mode (AM/USB/LSB/CW/FM) |
| **C** | Cycle band preset (18 presets) |
| **Up/Down** | Manual frequency tune (stops scan) |
| **Left/Right** | Master volume |
| **START** | Open reverb control panel |

## Scan Mode

Press **A** to start scanning. The receiver cycles through all 18 presets,
dwelling 3 seconds on each. When the S-meter detects a signal above the
squelch threshold, scanning pauses for 5 seconds. If the signal drops,
scanning resumes. Press **A** again or tune manually to stop.

## Presets (18)

| # | Frequency | Mode | Description |
|---|-----------|------|-------------|
| 1 | 5.000 MHz | AM | WWV Time Signal |
| 2 | 10.000 MHz | AM | WWV Time Signal |
| 3 | 15.000 MHz | AM | WWV Time Signal |
| 4 | 3.550 MHz | CW | 80m Morse Code |
| 5 | 3.750 MHz | LSB | 80m Voice |
| 6 | 7.030 MHz | CW | 40m Morse Code |
| 7 | 7.200 MHz | LSB | 40m Voice |
| 8 | 7.290 MHz | LSB | 40m Ragchew |
| 9 | 14.060 MHz | CW | 20m Morse Code |
| 10 | 14.200 MHz | USB | 20m Voice |
| 11 | 14.300 MHz | USB | 20m Emergency Net |
| 12 | 21.060 MHz | CW | 15m Morse Code |
| 13 | 21.200 MHz | USB | 15m Voice |
| 14 | 28.400 MHz | USB | 10m Voice |
| 15 | 9.420 MHz | AM | BBC World Service |
| 16 | 6.195 MHz | AM | Shortwave Broadcast 49m |
| 17 | 11.735 MHz | AM | Shortwave Broadcast 25m |
| 18 | 4.840 MHz | AM | WWCR Nashville |

## KiwiSDR Protocol

The ESP32-C3 connects to public KiwiSDR receivers via WebSocket:

1. TCP connect to `host:8073`
2. WebSocket upgrade: `GET /TIMESTAMP/SND HTTP/1.1`
3. `SET auth t=kiwi p=#` (public access)
4. `SET AR OK in=12000 out=12000`
5. `SET mod=am low_cut=-4900 high_cut=4900 freq=10000.000`
6. `SET compression=0` (raw PCM mode)
7. `SET dbgAudioStart=1`
8. `SET keepalive` (every 5 seconds)

Audio arrives as binary SND frames: 10-byte header + PCM samples at 12 kHz.

## DSP Chain

| Stage | Description |
|-------|-------------|
| ADPCM Decode | IMA/DVI 4:1 decompression (or raw PCM bypass) |
| Resample | 12 kHz → 44.1 kHz, fixed-point linear interpolation |
| Anti-Image LPF | 2nd-order Butterworth biquad, 5.5 kHz cutoff |
| Mixer | 8 uniform channels, per-channel vol + pan + reverb send |
| Freeverb | Jezar's algorithm, 8 comb + 4 allpass, fixed-point |
| PWM Output | 12-bit LEDC at 19.5 kHz, GPTimer ISR at 44 kHz |

## Command Protocol

| Command | ID | Direction | Description |
|---------|---:|-----------|-------------|
| `MW_CMD_SDR_CONNECT` | 72 | Gen→ESP | Connect to KiwiSDR (host + port) |
| `MW_CMD_SDR_TUNE` | 73 | Gen→ESP | Tune freq/mode/bandwidth |
| `MW_CMD_SDR_DISCONNECT` | 74 | Gen→ESP | Disconnect |
| `MW_CMD_SDR_STATUS` | 75 | Gen←ESP | Packed status (8 bytes) |

### Packed SDR Status (2 × u32, big-endian)

```
Word 0:
  [31:30] state     0=disconnected, 1=connected, 2=streaming
  [29:20] s_meter   10-bit raw signal strength
  [19:16] mode      AM/USB/LSB/CW/FM
  [15:8]  cpu_pct   ISR CPU usage 0-100%
  [7:4]   error     last error code
  [3:0]   flags     SND frame flags

Word 1:
  [31:0]  freq_hz   current frequency in Hz
```

## Required Hardware Modification

Replace C16 and C17 on the WiFi sub-board from 1 µF to **10 nF** (0805 SMD).
Stock caps create an aggressive 72 Hz low-pass — similar to a sub-woofer
crossover. Only the lowest bass frequencies pass through; all midrange,
vocals, and treble content is severely attenuated.
With 10 nF: cutoff = 7,234 Hz — passes voice and music.

## Build

### Genesis ROM

```sh
cd ~/MegaWifi/sdr_player
make
# ROM: out/sdr_player.bin
```

Requires: `m68k-elf-gcc`, SGDK at `~/sgdk`

### ESP32-C3 Firmware

```sh
cd ~/MegaWifi/mw-fw-rtos
source ~/esp/esp-idf/export.sh
# Enable audio: idf.py menuconfig → MegaWiFi options → Enable PWM audio
idf.py build
```

## License

- Genesis application: (c) 2026 Mike Wolak, all rights reserved
- Freeverb: public domain (Jezar at Dreampoint)
- micromod: BSD-3-Clause (Martin Cameron)
- Helix MP3: RPSL (RealNetworks)
- KiwiSDR protocol: public (Jezar/jks-prv)
- MegaWifi firmware: GPL-3.0 (upstream by doragasu)

---

*March 2026 — Mike Wolak <mikewolak@gmail.com>*
