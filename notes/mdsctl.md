# mdsctl

Command-line tool for sending JSON commands to medusa plugins via unix socket (`/tmp/mds_cmd.sock`).

## Usage

```
mdsctl <element_name> '<json>'
```

JSON always contains a `"todo"` key. Some commands also use `"cmd"` as a sub-action.

## Known commands (from scripts on the robot)

### sys0
```sh
mdsctl sys0 '{"pid_get":"get_all"}'                # returns sn, type, did, password, country, etc.
mdsctl sys0 '{"todo":"sys","cmd":"get_country"}'
mdsctl sys0 '{"get_fw_manfest":"get_all"}'
```

### audio0
```sh
mdsctl audio0 '{"todo":"audio", "cmd":"play","file":"/media/music/ZH/1","baton":1}'
mdsctl audio0 '{"todo":"audio", "cmd":"play","file_number":90}'
mdsctl audio0 '{"todo":"audio", "cmd":"PlaySound", "file_number":102}'
mdsctl audio0 '{"todo":"audio", "cmd":"get_language"}'
mdsctl audio0 '{"todo":"play_result", "result":0,"baton":1,"file":"/path"}'
mdsctl audio0 '{"todo":"mute_result", "result":0,"baton":1,"mute":"on"}'
```

### fw
```sh
mdsctl fw '{"todo":"startAutoOta"}'
mdsctl fw '{"todo":"stopAutoOta"}'
mdsctl fw '{"todo":"ota", "act":"start"}'
mdsctl fw '{"todo":"status", "status":"MDS_FW_DL_FW_FINISHED", "result":0}'
mdsctl fw '{"todo":"status", "status":"MDS_FW_UPDATE_FINISHED", "result":0}'
```

### bumbee (IoT/cloud)
```sh
mdsctl bumbee '{"todo":"QueryIotInfo"}'             # returns did, mid, resource, password
```

### rosnode
```sh
mdsctl rosnode '{"todo":"rtctl", "cmd":"getBatteryInfo"}'
mdsctl rosnode '{"todo":"bigData","item":"cloud","status":"online","bssid":"...","mac":"...","rssi":...}'
mdsctl rosnode '{"todo":"bigData","item":"network","status":"offline","reason":...}'
```

### DevStatus0
```sh
mdsctl DevStatus0 '{"todo":"dev","cmd":"wifiLedStatus","status":"on"}'      # also: off, slow_flash, fast_flash
```

### wifihandler
```sh
mdsctl wifihandler '{"todo":"OtaStatus","status":"start_download"}'
mdsctl wifihandler '{"todo":"CloudInfo","status":"online"}'
```

### linkkit (Alibaba IoT)
```sh
mdsctl linkkit '{"todo":"awss_factory_reset"}'
mdsctl linkkit '{"todo":"network_online"}'
```

### live_pwd
```sh
mdsctl live_pwd '{"todo":"livePWD", "cmd":"mulitPress"}'
```

### time0
```sh
mdsctl time0 '{"todo":"time","cmd":"pull_time"}'
mdsctl time0 '{"todo":"time","cmd":"set_time","from":"net","senconds":-1,"timezone":"+8"}'
```

## Notes

- No `mdsctl` commands for camera/video elements (`camera0`, `camera1`, `rkmpp0`, `lvision`, `mediactl`) were found in any scripts on the robot. The camera pipeline appears to be controlled entirely within medusa's internal plugin chain.
- `process_picture.sh` is called by `MEDIA_CONTROL` after a picture is already captured — it only handles uploading the JPEG to Ecovacs cloud.
