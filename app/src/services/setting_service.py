from typing import Any, Dict
from sqlalchemy.orm import Session
import json

import sys
import os

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "app/src"))

from db.models import SysSettings
from db.database import SessionLocal

class SettingService:
    def __init__(self):
        self._db_session: Session = SessionLocal()

    def __del__(self):
        try:
            self._db_session.close()
        except:
            pass

    def get_value(self, key: str, default: Any = None) -> Any:
        setting = self._db_session.query(SysSettings).filter(SysSettings.key == key).first()
        if not setting:
            return default
        try:
            return json.loads(setting.value)
        except json.JSONDecodeError:
            return setting.value

    def set_value(self, key: str, value: Any, description: str = ""):
        setting = self._db_session.query(SysSettings).filter(SysSettings.key == key).first()
        str_val = json.dumps(value) if not isinstance(value, str) else value
        
        if setting:
            setting.value = str_val
            if description:
                setting.description = description
        else:
            new_setting = SysSettings(key=key, value=str_val, description=description)
            self._db_session.add(new_setting)
        
        self._db_session.commit()

    def get_all_settings(self) -> Dict[str, Any]:
        settings = self._db_session.query(SysSettings).all()
        result = {}
        for s in settings:
            try:
                result[s.key] = json.loads(s.value)
            except json.JSONDecodeError:
                result[s.key] = s.value
        return result
