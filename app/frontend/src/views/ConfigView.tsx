import React, { useState, useEffect } from 'react';
import { Save, Server, Camera as CameraIcon, Cpu } from 'lucide-react';

const ConfigView: React.FC = () => {
  const [config, setConfig] = useState<any>({});
  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [saveSuccess, setSaveSuccess] = useState(false);

  useEffect(() => {
    fetch('http://localhost:8000/api/system/config')
      .then(res => res.json())
      .then(data => {
        setConfig(data.config || {});
        setLoading(false);
      })
      .catch(err => {
        console.error("Failed to load config:", err);
        setLoading(false);
      });
  }, []);

  const handleSave = async () => {
    setSaving(true);
    setSaveSuccess(false);
    try {
      await fetch('http://localhost:8000/api/system/config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ settings: config })
      });
      setSaveSuccess(true);
      setTimeout(() => setSaveSuccess(false), 3000);
    } catch (err) {
      console.error("Failed to save config:", err);
    } finally {
      setSaving(false);
    }
  };

  const handleChange = (key: string, value: any) => {
    setConfig({ ...config, [key]: value });
  };

  if (loading) {
    return (
      <div className="flex-1 flex items-center justify-center p-8 bg-slate-950">
        <div className="text-slate-500 animate-pulse flex flex-col items-center gap-3">
          <Server size={32} />
          <p>Loading System Configuration...</p>
        </div>
      </div>
    );
  }

  return (
    <div className="flex-1 flex flex-col bg-slate-950 p-8 overflow-y-auto custom-scrollbar">
      <div className="max-w-4xl mx-auto w-full space-y-8">
        
        {/* Header */}
        <div className="flex items-center justify-between border-b border-slate-800 pb-4">
          <div>
            <h1 className="text-2xl font-semibold text-slate-100 flex items-center gap-3">
              <Server className="text-blue-500" />
              System Configuration
            </h1>
            <p className="text-slate-400 text-sm mt-1">Manage global hardware parameters and system behaviors.</p>
          </div>
          
          <button 
            onClick={handleSave}
            disabled={saving}
            className={`px-5 py-2.5 rounded-lg text-sm font-medium shadow-md transition-all flex items-center gap-2 ${
              saveSuccess 
                ? 'bg-emerald-600 text-white shadow-emerald-900/20'
                : 'bg-blue-600 hover:bg-blue-500 text-white shadow-blue-900/20'
            } disabled:opacity-50 disabled:cursor-not-allowed`}
          >
            <Save size={16} /> 
            {saving ? 'Saving...' : saveSuccess ? 'Saved Successfully' : 'Save Changes'}
          </button>
        </div>

        <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
          
          {/* Robot Settings Card */}
          <div className="bg-slate-900/50 border border-slate-800 rounded-xl p-6 shadow-sm flex flex-col">
            <div className="flex items-center gap-2 mb-6 border-b border-slate-800/50 pb-3">
              <Cpu className="text-indigo-400" size={18} />
              <h2 className="text-lg font-medium text-slate-200">Robot Settings</h2>
            </div>
            
            <div className="space-y-5 flex-1">
              <div>
                <label className="block text-sm font-medium text-slate-400 mb-2">Robot IP Address</label>
                <input 
                  type="text" 
                  value={config.robot_ip || ''} 
                  onChange={(e) => handleChange("robot_ip", e.target.value)}
                  placeholder="e.g. 192.168.5.1"
                  className="w-full bg-slate-950 border border-slate-700 rounded-lg px-4 py-2.5 text-slate-200 focus:border-blue-500 focus:ring-1 focus:ring-blue-500 transition-all font-mono"
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-slate-400 mb-2">Robot Port</label>
                <input 
                  type="text" 
                  value={config.robot_port || ''} 
                  onChange={(e) => handleChange("robot_port", e.target.value)}
                  placeholder="e.g. 29999"
                  className="w-full bg-slate-950 border border-slate-700 rounded-lg px-4 py-2.5 text-slate-200 focus:border-blue-500 focus:ring-1 focus:ring-blue-500 transition-all font-mono"
                />
              </div>
              <p className="text-xs text-slate-500 leading-relaxed pt-2">
                Configure the network address of the Dobot robotic arm. Ensure the controller and IPC are on the same subnet.
              </p>
            </div>
          </div>

          {/* Camera Settings Card */}
          <div className="bg-slate-900/50 border border-slate-800 rounded-xl p-6 shadow-sm flex flex-col">
            <div className="flex items-center gap-2 mb-6 border-b border-slate-800/50 pb-3">
              <CameraIcon className="text-emerald-400" size={18} />
              <h2 className="text-lg font-medium text-slate-200">Calibration Board</h2>
            </div>
            
            <div className="space-y-5 flex-1">
              <div className="grid grid-cols-2 gap-4">
                <div>
                  <label className="block text-sm font-medium text-slate-400 mb-2">Columns (Width)</label>
                  <input 
                    type="number" 
                    value={config.calib_board_cols || ''} 
                    onChange={(e) => handleChange("calib_board_cols", parseInt(e.target.value))}
                    className="w-full bg-slate-950 border border-slate-700 rounded-lg px-4 py-2.5 text-slate-200 focus:border-blue-500 focus:ring-1 focus:ring-blue-500 transition-all"
                  />
                </div>
                <div>
                  <label className="block text-sm font-medium text-slate-400 mb-2">Rows (Height)</label>
                  <input 
                    type="number" 
                    value={config.calib_board_rows || ''} 
                    onChange={(e) => handleChange("calib_board_rows", parseInt(e.target.value))}
                    className="w-full bg-slate-950 border border-slate-700 rounded-lg px-4 py-2.5 text-slate-200 focus:border-blue-500 focus:ring-1 focus:ring-blue-500 transition-all"
                  />
                </div>
              </div>
              <p className="text-xs text-slate-500 leading-relaxed pt-2">
                Specify the external physical dimensions (number of squares) of the chessboard. OpenCV uses internal corners which are automatically derived (width - 1, height - 1). Standard AiSprayer board is 9x12.
              </p>
            </div>
          </div>

        </div>
      </div>
    </div>
  );
};

export default ConfigView;
