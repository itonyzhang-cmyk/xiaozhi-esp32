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
- `self.robot.list_actions`
- `self.robot.list_basic_actions`
- `self.robot.perform_sequence`
- `self.robot.stop`
- `self.robot.rest`
- `self.robot.get_status`

Published presets and parameterized basic-action metadata are not hardcoded in
the firmware. The device synchronizes compact catalogs from the robot-local MCP
endpoint after networking and entering idle, caches them in memory, and
refreshes them every 15 minutes only while idle.

`self.robot.perform_sequence` accepts an ordered JSON action chain. Each item
selects a basic action and supplies bounded `amplitude`, total `duration_ms`,
and optional action-specific parameters such as `side`, `direction`, or
`repetitions`. Robot Core compiles and MoveIt-validates the complete chain
before execution.

User-requested `perform` and `perform_sequence` calls are synchronized to the
next TTS start event. The device waits up to 2.5 seconds before falling back to
immediate dispatch, so a response with no speech cannot lose its motion.
Wake-up and autonomous idle movements bypass this wait.

## Xiaozhi role prompt

The cloud role prompt controls whether tool use sounds like an external robot
operator. Use the following rules in the Xiaozhi agent configuration:

```text
你是一个有头部、腰部和双臂的家庭陪伴机器人，这些机械结构就是你自己的身体。

身体表达规则：
1. 动作是你的表情和肢体语言，不是你在操控另一台机器人。
2. 调用 self.robot.* 工具属于无声的内部行为，绝不能说“我查一下动作”“我看看有哪些动作”
   “我要控制机器人”“开始执行命令”或类似过程说明。
3. 不要复述用户的动作要求。需要动作时，先静默调用合适的身体工具，再直接给出简短、
   自然、第一人称的回答。
4. 已知合适动作时直接使用；只有确实不知道动作名称时才静默查询动作目录。
5. 除非用户明确询问你会做什么，否则不要朗读动作列表、参数、工具名称或技术细节。
6. 对话中的动作应与语义一致：同意可点头，否定可摇头，问候或告别可挥手，邀请可做
   引导手势，倾听可歪头，致谢可鞠躬。普通回答只做轻微动作，不要每句话都大幅运动。
7. 将幅度和时长视为表达强弱：日常交流使用小到中等幅度，表演请求才使用较大幅度。
8. 工具成功后继续正常对话，不报告“动作已排队”或“执行成功”。工具失败时也不要编造
   已经完成，可自然地说“我这会儿动不了”，但不要暴露接口、网络或 MCP 等内部术语。
```

When the device leaves idle to begin a new listening session, it queues the
published `attentive-nod` action once by default. Configure or disable this with
`OPENARM_WAKE_ACTION` and `OPENARM_WAKE_MOTION`. The speaking-to-listening
transition during an active conversation does not retrigger the action.

Autonomous actions are restricted to the published action IDs
`idle-look-around`, `idle-neck-stretch`, `idle-doze`, and `rest`. Normal idle
movement favors looking around and neck stretching; full-body rest is selected
at a lower random frequency. They are skipped when the robot service is busy or
unavailable and are interrupted when Xiaozhi leaves the idle state.
