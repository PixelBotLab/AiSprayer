# -*- coding: utf-8 -*-
"""Teleoperation module for AiSprayer robot arm via handheld HID controller.

Primary input device is an 8BitDo gamepad (standard HID gamepad in X-Input /
D-Input mode), but the module is named by its responsibility (teleoperation),
not by the hardware, so other manual input devices can be added later.

This package lives in the core layer and is hardware/protocol agnostic:
it reads HID gamepad events, applies a configurable key-mapping and a safety
state machine, then emits high-level teleop commands through an injected
command sink (see docs/teleop_design.md).

The concrete wiring to ``RobotService`` is done in the app layer, keeping the
core free of business/transport concerns (driver layer <~ service layer <~ API).
"""
