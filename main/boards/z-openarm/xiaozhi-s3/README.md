# Z-OpenArm Xiaozhi ESP32-S3

This board keeps the `bread-compact-wifi` audio, OLED, buttons, Wi-Fi, and OTA
stack while adding a bounded asynchronous client for the robot-local MCP
service.

## Hardware

- ESP32-S3 N16R8
- INMP441: WS GPIO4, SCK GPIO5, DIN GPIO6
- MAX98357A: DOUT GPIO7, BCLK GPIO15, LRCK GPIO16
- SSD1306 128x32: SDA GPIO41, SCL GPIO42
- volume up/down: GPIO40/GPIO39
- BOOT/wake: GPIO0

## Robot integration

Configure these values with `menuconfig`; never commit the real bearer token:

```text
Xiaozhi Assistant
  -> Z-OpenArm Robot
     -> LAN MCP endpoint
     -> LAN MCP bearer token
```

The default endpoint is `http://192.168.31.108:8790/mcp`. The corresponding
host service is `openarm-mcp.service`, with its token stored outside Git in
`~/.config/openarm/mcp.env`.

Device MCP tools:

- `self.robot.perform`
- `self.robot.stop`
- `self.robot.rest`
- `self.robot.get_status`

Autonomous actions are restricted to the published action IDs
`idle-look-around`, `idle-neck-stretch`, and `idle-doze`. They are skipped when
the robot service is busy or unavailable and are interrupted when Xiaozhi
leaves the idle state.
