# Cross-model notes: T9/DLX13 on RV1126 (vs the T8 AIVI on PX30)

Notes from rooting a retail **DEEBOT T9+ (DLX13)**, firmware `2.5.5`. Mostly
things that a T8 AIVI guide does not cover, and one item that may be worth a
caveat in the main walkthrough.

## the T9 line is two different SoCs

| firmware | SoC | arch | kernel | `product` (`/etc/fw.manifest`) |
| :--- | :--- | :--- | :--- | :--- |
| `1.x` | Rockchip PX30 | aarch64 | `4.4.185` | `zj2010_t9` |
| `2.x` | Rockchip **RV1126** | **armv7l** | `4.19.111` | `rv1126_zj2010` |

So "T9" does not imply aarch64. Any prebuilt binary you point people at needs a
32-bit ARM build for the `2.x` units. The GPT partition layout, however, is the
same as the PX30 T8 AIVI (`uboot 0x4000`, `trust 0x6000`, `mx 0x8000`,
`my 0xa000`, `sys 0xc000`, `boot1 0xd000`, `rootfs1 0x10000`, `boot2 0x33000`,
`rootfs2 0x36000`, `data 0x59000`).

## `/data/autostart` may be disabled — check `rc.conf` first

This is the one that may matter for the walkthrough, which relies on
`/data/autostart/sshd_start.sh`.

`/etc/rc.conf` ships **`!autostart.sh`**, and `/etc/rc.sysinit` skips any daemon
whose entry starts with `!`:

```sh
for daemon in ${DAEMONS}; do
  case ${daemon:0:1} in
     '!') continue;;     # Skip this daemon.
```

I dumped the SquashFS from three separate T9 images and all three disable it:

| image | SoC | `rc.conf` |
| :--- | :--- | :--- |
| `lhbd50` fw 1.4.8 | PX30 | `!autostart.sh` |
| `fqxoiu` fw 1.6.3 | PX30 | `!autostart.sh` |
| `dj8zpr` fw 2.5.5 | RV1126 | `!autostart.sh` |

On the T9 images this hook is disabled, so `sshd_start.sh` never executes unless
`rc.conf` is patched. Check on any unit:

```sh
grep -o '.\{0,1\}autostart.sh' /etc/rc.conf
```

Two techniques that work regardless, and survive a `/data` wipe or factory reset:

1. **Bind-mount over the read-only SquashFS at boot.** Mounts need no write
   access to the target filesystem, take effect immediately, and evaporate on
   reboot, so they are fully reversible:
   ```sh
   mount --bind /data/patched/autostart.sh /etc/rc.d/autostart.sh
   ```
   Same trick works to stub out the OTA scripts.
2. **Bake into both A/B `rootfs` slots.** A factory reset only clears `/data`
   (`reset_data_partition()` in `/usr/bin/factory_fun`), so an SSH daemon plus a
   start script inside SquashFS survives it. Flash the spare slot first and flip
   the `sys` flag (`boot_mode1` / `boot_mode2` at offset 0).

## `eco_passwd` product keys

Confirming `px30-sl` and `AI_px30` from the guide, plus the T9 pair —
`zj2010_t9` for PX30 and `rv1126_zj2010` for RV1126. Neither T9 key is in the
public calculator lists at time of writing. As usual the base64 result is
truncated to 8 characters before `busybox chpasswd`, and the SN is at offset
1 MiB of the `sys` partition.

## `sl` firmware section is not the MCU

The T9+ OTA bundle carries an `sl` section (35,964 bytes) in addition to `mcu`.
That is the **structured-light / TrueDetect 3D module**, flashed through
`/etc/conf/medusa/sl_ota_hook.sh` -> `mdsctl slota` on `/dev/ttyS3`. Worth
flagging so nobody assumes a section list without it means no sensor firmware.
On this SKU the module is definitely present (`/opt/sl.bin`, `/data/sl-bk/`,
`/linelaser/*` and `/setting/SlOnOff` topics), while there is **no camera**
(no `/dev/video*`, no `isp`/`mipi`/`vip` platform devices) — unlike the AIVI.

## OTA chain and why U-Boot survives

```
crond -> autoOTA.sh -> start_ota.sh -> medusa FW plugin
      -> download_fw_hook_v3.sh -> fw_upgrade.sh
```

`fw_upgrade.sh` writes only `boot1/2`, `rootfs1/2` and the `sys` boot flag — never
`uboot` or `trust`. So even after a forced OTA, the U-Boot `=>` prompt (and
`init=/bin/sh`) remains available.

## `fw_target` on 2.5.5

Still the same XOR-`0x55` obfuscated format string as the PX30 build:

```
ZWNvX2Z3X3RhcmdldCA%s1jdSAt%sbyBtYW4%dy5iaW4%s%825xx%s
  args: "ECO-PT", <platform>, <section type>, "", <section len>, "jeff-hk@126.com"
```

Open question: the `2.7.0` package for this class does not decrypt with a
`px30`-shaped key, and section 0's header byte 2 differs (`0x39` on 1.4.8 vs
`0xf1` on 2.7.0). The format string is unchanged, so the difference is in the cipher/mode or the
section-type nibble rather than the platform token.
