from pydantic import BaseModel, Field
from typing import Optional, Dict, Any, List

class CalibrationModeUpdate(BaseModel):
    enabled: bool = Field(..., description="是否开启标定模式 (开启时启动角点检测并关闭深度流以节省CPU)")
    board_type: Optional[str] = Field("chessboard", description="标定板类型")
    rows: Optional[int] = Field(None, description="棋盘格行数")
    cols: Optional[int] = Field(None, description="棋盘格列数")
    square_size_mm: Optional[float] = Field(None, description="格子尺寸 (mm)")
    draw_corners: Optional[bool] = Field(True, description="是否叠加绘制角点")

class CameraCaptureReq(BaseModel):
    save_dir: Optional[str] = Field("data/calib", description="存储目录")
    prefix: Optional[str] = Field("sample", description="文件前缀")
    index: Optional[int] = Field(0, description="采样序号")
