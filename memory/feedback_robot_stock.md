---
name: Keep robot stock
description: User wants to avoid copying files to robot - run scripts from Mac, keep robot filesystem as untouched as possible
type: feedback
---

Avoid copying scripts or files to the robot. Run everything from the Mac over SSH/network instead.

**Why:** User wants to minimize risk of bricking the robot by keeping its filesystem as stock as possible.

**How to apply:** Use SSH tunnels, run Python scripts locally against tunneled ports, only put essential persistent infrastructure (like dropbear) on the robot. When something must run on-robot, prefer one-liners over deploying scripts.
