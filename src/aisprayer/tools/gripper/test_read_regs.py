#!/usr/bin/env python3
"""
测试越疆 GetHoldRegs 读取夹爪状态寄存器
重点观察 0x07D0 状态字在各阶段的实际值
"""
import os, sys, time
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from aisprayer.core.hardware.robot.dobot_api import DobotApiDashboard

IP = "192.168.5.1"
PORT = 29999
SLAVE_ID = 9

def read_reg(dashboard, idx, addr):
    res = dashboard.GetHoldRegs(idx, addr, 1, "U16")
    if res and res.startswith("0"):
        parts = res.split("{")
        if len(parts) > 1:
            return int(parts[1].split("}")[0].strip())
    return -1

def print_status(dashboard, idx, label):
    s = read_reg(dashboard, idx, 0x07D0)
    pf = read_reg(dashboard, idx, 0x07D1)
    sf = read_reg(dashboard, idx, 0x07D2)
    pos = (pf >> 8) & 0xFF if pf >= 0 else -1
    fault = pf & 0xFF if pf >= 0 else -1
    speed_fb = (sf >> 8) & 0xFF if sf >= 0 else -1
    force_fb = sf & 0xFF if sf >= 0 else -1
    moving = "动" if s >= 0 and not (s & 0x0080) else "停"
    print(f"  [{label:12s}] [{moving}] 0x07D0={s:5d}  pos={pos:3d}  "
          f"speed_fb={speed_fb:3d}  force_fb={force_fb:3d}  fault={fault}")

def main():
    print(f"连接越疆控制器 {IP}:{PORT} ...")
    dashboard = DobotApiDashboard(IP, PORT)
    
    res = dashboard.ModbusCreate("127.0.0.1", 60000, SLAVE_ID, 1)
    print(f"  ModbusCreate: {res}")
    if not res or not res.startswith("0"):
        return
    idx = int(res.split("{")[1].split("}")[0].strip())
    print(f"  设备索引: {idx}\n")
    
    # 使能
    print("=== 使能夹爪 ===")
    dashboard.SetHoldRegs(idx, 0x03E8, 1, "{1}", "U16")
    for i in range(6):
        time.sleep(0.5)
        print_status(dashboard, idx, f"使能+{i*0.5:.1f}s")
    
    # 运动到全开 (hw position = 0)
    print("\n=== 运动到全开 (hw_pos=0) ===")
    dashboard.SetHoldRegs(idx, 0x03E9, 1, "{0}", "U16")      # pos=0<<8=0
    dashboard.SetHoldRegs(idx, 0x03EA, 1, "{25700}", "U16")   # force=100,speed=100
    dashboard.SetHoldRegs(idx, 0x03E8, 1, "{9}", "U16")       # trigger
    for i in range(8):
        time.sleep(0.3)
        print_status(dashboard, idx, f"全开+{i*0.3:.1f}s")
    
    time.sleep(1)
    
    # 运动到全闭 (hw position = 255)
    print("\n=== 运动到全闭 (hw_pos=255) ===")
    dashboard.SetHoldRegs(idx, 0x03E9, 1, "{65280}", "U16")   # pos=255<<8=65280
    dashboard.SetHoldRegs(idx, 0x03EA, 1, "{25700}", "U16")   # force=100,speed=100
    dashboard.SetHoldRegs(idx, 0x03E8, 1, "{9}", "U16")       # trigger
    for i in range(8):
        time.sleep(0.3)
        print_status(dashboard, idx, f"全闭+{i*0.3:.1f}s")
    
    time.sleep(1)
    
    # 运动到半开 (hw position = 128)
    print("\n=== 运动到半开 (hw_pos=128) ===")
    dashboard.SetHoldRegs(idx, 0x03E9, 1, "{32768}", "U16")   # pos=128<<8=32768
    dashboard.SetHoldRegs(idx, 0x03EA, 1, "{25700}", "U16")
    dashboard.SetHoldRegs(idx, 0x03E8, 1, "{9}", "U16")
    for i in range(8):
        time.sleep(0.3)
        print_status(dashboard, idx, f"半开+{i*0.3:.1f}s")
    
    print("\n=== 完成 ===")
    dashboard.ModbusClose(idx)
    dashboard.close()

if __name__ == "__main__":
    main()
