# Mecapi Firmware Holonome

Firmware du Robot Holonome pour [ESP32-S3-Tiny](https://www.waveshare.com/wiki/ESP32-S3-Tiny).
Le firmware est compilé avec [Platform.io](https://platformio.org/).


## Tasks

Tasks can be assigned priorities of 0 to (configMAX_PRIORITIES - 1).  Zero is the lowest priority.

---
### ESP32_Helper::Update  "CommandUpdate" - Default : Core 0, Priority 5
Read Serial Debug and save incomming command into queue


---
### Wifi_Helper::Update "WifiUpdate" - Core 0, Priority 10
Reconnect to wifi </br>
Reconnect to server </br>
Reconnect to Teleplot UDP </br>
Read Wifi Client and save incomming command into queue
Handle OTA

---
### Lidar::TaskLidar "TaskLidar" - Core 1, Priority 10
Read Serial Lidar and save incomming obstacle


---
### TaskMatch "TaskMatch" - Core 1, Priority 10
State machine of the Match

---
### Loop - Core 1, Priority 20 ?



### timerMotionCallback "Timer Motion" - Must not block !!! Priority MAX
Update Position
Update Trajectory
Update Motion
Update Motors
---
