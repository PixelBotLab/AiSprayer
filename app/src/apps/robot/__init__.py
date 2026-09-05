# -*- coding: utf-8 -*-
"""
Robot application package.
Provides robot REST/WebSocket APIs, request models, and robot service management.
"""
from apps.robot.services.robot_service import robot_service, RobotService

__all__ = ["robot_service", "RobotService"]
