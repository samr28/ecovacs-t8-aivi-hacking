# Camera

## Live view on the Mac (works)

```sh
scripts/robot_camera.sh             # 640x480 window via ffplay
scripts/robot_camera.sh 1280 960    # full sensor resolution
scripts/robot_camera.sh --record out.h264
```

Pipeline (nothing installed on the robot — it's all one ssh command line):

```
/dev/video0 (ISP mainpath, NV12) ─ v4l2-ctl --stream-to=- ─▶ mpi_enc_test (Rockchip MPP hardware H.264)
   ─▶ stdout over ssh ─▶ ffplay -f h264 (low-latency flags) on the Mac
```

- ~30 fps at 640x480, ~1.2 Mbit/s (~5 KB per P-frame vs 460 KB raw NV12 — ~100× less than raw).
- `mpi_enc_test` (shipped Rockchip test tool) works with `-i /dev/stdin -o /dev/stdout -f 0` (NV12) `-t 7`
  (H.264) `-n 0` (no frame limit). It emits a single IDR then P-frames.
- **Gotcha 1 — logs on stdout:** `mpi_enc_test` prints its log lines (`mpi_enc_test: test_mpp_run encoded
  frame 60 size ...`) to **stdout**. With `-o /dev/stdout` they were spliced into the H.264 in 4 KB bursts →
  a corrupt frame every ~10 s (green smear in ffplay, red ✕ in Foxglove until the next keyframe). Fix: write
  the video to fd 3 and discard the tool's stdout: `mpi_enc_test -o /dev/fd/3 ... 3>&1 1>/dev/null 2>/dev/null | cat`
  (the `cat` makes fd 3 a plain pipe). Verified: 758 frames, 0 decode errors.
- Stream structure: one slice per frame, IDR every 60 frames (2 s), but **SPS/PPS only once at the start** —
  `robot_viz.py` caches them and prepends them to every IDR so a late-joining viewer can decode.
- Foxglove: `scripts/robot_viz.py` publishes this as `/camera` (`foxglove.CompressedVideo`, h264); view it
  in an **Image** panel.
- **Gotcha 2:** `mpi_enc_test` ignores write errors, so when the ssh client disconnects the robot-side
  pipeline keeps running and holds `/dev/video0`. The script's remote command runs a watchdog that polls
  the ssh session process and `kill 0`s the process group once it's gone (cleanup within ~1 s).
- Conflicts: `/dev/video0` is the device medusa's `camera1` (app live view) uses. Don't stream while
  the app's live video is open. Object detection (`camera0`, `/dev/video1`) is unaffected.
- Low light: auto-exposure already pins `exposure=1000` (max) in a dark room, so night images are
  dark/red-tinted. Brightening has to come from more light or post-processing.

## Hardware

- Single camera sensor behind the front window (used for AIVI object detection)
- Sensor subdev `/dev/v4l-subdev2` (`m00_b_jx_h65 2-0030`), via MIPI D-PHY into the Rockchip ISP (`rkisp1`)
- Sensor resolution: 1280x960, pixel rate 43.2 MHz
- The image includes a dark border — the edge of the camera window in the housing

## Video Devices

All from the same ISP (names from `/sys/class/video4linux/*/name`):

| Device | ISP node | Used by medusa as | Format in medusa config |
|--------|----------|-------------------|-------------------------|
| `/dev/video0` | `rkisp1_mainpath` | `camera1` — app live view | 864x480 NV12, crop top=310 left=212, MMAP |
| `/dev/video1` | `rkisp1_selfpath` | `camera0` — object detection | 1280x960 XBGR32, DMABUF |
| `/dev/video2` | `rkisp1_rawpath` | — | |
| `/dev/video3` | `rkisp1_dmapath` | — | |
| `/dev/video4` | `rkisp1-statistics` | ISP 3A loop | |
| `/dev/video5` | `rkisp1-input-params` | ISP 3A loop | |

Medusa keeps `/dev/video1`, `video4` and `video5` open even when idle (object-detection path + the
auto-exposure/white-balance loop). `/dev/video0` is free unless the app's live view is running.

The mainpath supports YUYV, 422P, NV16, NV61, YM16, NV21, NV12, NM21 and can downscale in hardware
(`--set-fmt-video=width=640,height=480,pixelformat=NV12` works).

## Capturing a Single Frame

```sh
ssh root@<robot-ip> 'v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=NV12 &&
  v4l2-ctl -d /dev/video0 --stream-mmap=4 --stream-count=5 --stream-to=-' > frames.nv12
ffmpeg -f rawvideo -pix_fmt nv12 -s 640x480 -i frames.nv12 -vf "select=eq(n\,4)" -frames:v 1 frame.jpg
```
(Skip the first few frames while auto-exposure settles.) Raw NV12 over ssh manages only ~6–9 fps at
640x480 (~2.8 MB/s) — use the H.264 path for live view.

## Medusa Camera Pipeline

Medusa (`/usr/bin/medusa`) manages all camera operations via its plugin chain. The config is at
`/etc/conf/medusa/deebot_px30_ai.conf` (copied to `/tmp/deebot_px30_ai.conf` at runtime).
Element links below are from that config's chain list.

### Object detection path
```
camera0 (/dev/video1 selfpath, 1280x960 XBGR32)
  ↔ rk_drm (RK_DRM_ELEM, GPU/RGA pre-processing)
  → objectDetector (ObjectDetect, distance table /data/distance_table)
  → rosnode (publishes /aiDetectRslt to ROS)
```

### Live video path (app remote view)
```
camera1 (/dev/video0 mainpath, 864x480 NV12, cropped)
  → rkmpp0 (Rockchip hardware H.264 encoder, 15 fps, gop 15, 512 kbps VGA / 1024 kbps 720p)
  → lvision (live video manager)
  → bumbee/linkkit (Alibaba IoT cloud, outbound TLS to port 443)
  → Ecovacs cloud relay
  → phone app
```

There is no local port serving the video stream. The robot connects outbound to Alibaba Cloud; the
app connects to the cloud, which relays between app and robot.

### Other camera-related medusa elements
- `RK_ISP` (`rk_isp1`) — image signal processor control (exposure, gain)
- `RK_DRM_ELEM` (`rk_drm`) — DRM/GPU processing, saves to `/tmp/map.out`
- `MEDIA_CONTROL` (`mediactl`) — handles picture capture/upload, calls `process_picture.sh`
- `LIVE_PWD` (`live_pwd`) — password gate for live video (`/data/config/medusa/password.conf`)
- `LVISON` (`lvision`) — live video state (`/data/config/medusa/lvision.conf`)

## Camera-Related ROS Topics

No raw image data flows through ROS. Only detection results and control signals:

| Topic | Type | Notes |
|-------|------|-------|
| `/aiDetectRslt` | `ai/AIDetResult` | Object detection results from medusa → deebot |
| `/wifimsg/wifimsgcameractrl` | `wifimsg/wifimsgcameractrl` | Camera control commands (deebot → medusa) |
| `/wifimsg/wifimsgcamerareport` | `wifimsg/wifimsgcamerareport` | Camera status reports (medusa → deebot) |
| `/task/VideoState` | `task/VideoState` | Whether live video is active |
| `/setting/AiSetting` | `setting/AiSetting` | AI feature toggle |
| `/task/AiPoints` | `task/AiPoints` | Detected object locations |

## Sensor Controls (`v4l2-ctl -d /dev/v4l-subdev2 -l`)

| Control | Range | Default | Observed (dark room, AE active) |
|---------|-------|---------|------|
| exposure | 4–1000 | 900 | 1000 |
| gain | 0–16383 | 1024 | 1024 |
| analogue_gain | 0–127 | 0 | 48 |
| vertical_blanking | 40–64575 | 40 | 40 |
| test_pattern | 0–4 | 0 | 0 |

Medusa's ISP loop drives these; manual changes will likely be overwritten.
