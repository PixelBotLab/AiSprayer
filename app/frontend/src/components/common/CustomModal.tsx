import React, { useState, useEffect, useRef } from 'react';
import { X, Check, AlertCircle, HelpCircle, Info, Trash2, FolderPlus } from 'lucide-react';

export type ModalType = 'input' | 'confirm' | 'alert' | 'info';

export interface ModalConfig {
  isOpen: boolean;
  type: ModalType;
  title: string;
  message?: string;
  placeholder?: string;
  defaultValue?: string;
  confirmText?: string;
  cancelText?: string;
  isDanger?: boolean;
  onConfirm?: (value?: string) => void;
  onCancel?: () => void;
}

interface CustomModalProps {
  config: ModalConfig;
  onClose: () => void;
}

export const CustomModal: React.FC<CustomModalProps> = ({ config, onClose }) => {
  const [inputValue, setInputValue] = useState('');
  const inputRef = useRef<HTMLInputElement>(null);

  useEffect(() => {
    if (config.isOpen) {
      setInputValue(config.defaultValue || '');
      if (config.type === 'input') {
        setTimeout(() => {
          inputRef.current?.focus();
          inputRef.current?.select();
        }, 50);
      }
    }
  }, [config.isOpen, config.defaultValue, config.type]);

  if (!config.isOpen) return null;

  const handleConfirm = () => {
    if (config.onConfirm) {
      config.onConfirm(config.type === 'input' ? inputValue.trim() : undefined);
    }
    onClose();
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter') {
      e.preventDefault();
      handleConfirm();
    } else if (e.key === 'Escape') {
      e.preventDefault();
      if (config.onCancel) config.onCancel();
      onClose();
    }
  };

  const getIcon = () => {
    switch (config.type) {
      case 'input':
        return <FolderPlus className="text-blue-400" size={20} />;
      case 'confirm':
        return config.isDanger ? <Trash2 className="text-red-400" size={20} /> : <HelpCircle className="text-amber-400" size={20} />;
      case 'alert':
        return <AlertCircle className="text-amber-400" size={20} />;
      case 'info':
      default:
        return <Info className="text-blue-400" size={20} />;
    }
  };

  return (
    <div className="fixed inset-0 z-[200] flex items-center justify-center p-4">
      {/* Backdrop */}
      <div 
        className="fixed inset-0 bg-black/70 backdrop-blur-sm transition-opacity animate-in fade-in duration-200"
        onClick={() => {
          if (config.onCancel) config.onCancel();
          onClose();
        }}
      />

      {/* Dialog Card */}
      <div 
        className="relative bg-slate-900 border border-slate-700/80 rounded-2xl shadow-2xl w-full max-w-md p-6 overflow-hidden flex flex-col gap-4 z-10 animate-in zoom-in-95 duration-150"
        onKeyDown={handleKeyDown}
      >
        {/* Glow accent */}
        <div className="absolute -top-12 -left-12 w-32 h-32 bg-blue-500/10 rounded-full blur-2xl pointer-events-none" />
        
        {/* Header */}
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            <div className="p-2 rounded-xl bg-slate-800 border border-slate-700/60 shadow-inner">
              {getIcon()}
            </div>
            <h3 className="text-base font-semibold text-slate-100 tracking-wide">
              {config.title}
            </h3>
          </div>
          <button
            onClick={() => {
              if (config.onCancel) config.onCancel();
              onClose();
            }}
            className="p-1 rounded-lg text-slate-400 hover:text-slate-200 hover:bg-slate-800 transition-colors"
          >
            <X size={18} />
          </button>
        </div>

        {/* Message / Description */}
        {config.message && (
          <p className="text-sm text-slate-300 leading-relaxed">
            {config.message}
          </p>
        )}

        {/* Input field if type == input */}
        {config.type === 'input' && (
          <div className="flex flex-col gap-1.5 mt-1">
            <input
              ref={inputRef}
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder={config.placeholder || "Enter value..."}
              className="w-full px-3.5 py-2.5 bg-slate-950 border border-slate-700 rounded-xl text-slate-100 placeholder-slate-500 text-sm focus:outline-none focus:border-blue-500 focus:ring-2 focus:ring-blue-500/20 transition-all font-mono"
            />
          </div>
        )}

        {/* Action Buttons */}
        <div className="flex items-center justify-end gap-2.5 mt-2 pt-2 border-t border-slate-800/80">
          {config.type !== 'alert' && config.type !== 'info' && (
            <button
              onClick={() => {
                if (config.onCancel) config.onCancel();
                onClose();
              }}
              className="px-4 py-2 rounded-xl text-xs font-medium text-slate-300 hover:text-slate-100 hover:bg-slate-800 border border-slate-700/80 transition-all"
            >
              {config.cancelText || 'Cancel'}
            </button>
          )}

          <button
            onClick={handleConfirm}
            className={`px-5 py-2 rounded-xl text-xs font-medium text-white shadow-lg transition-all active:scale-95 flex items-center gap-1.5 ${
              config.isDanger
                ? 'bg-gradient-to-r from-red-600 to-rose-600 hover:from-red-500 hover:to-rose-500 shadow-red-900/30'
                : 'bg-gradient-to-r from-blue-600 to-indigo-600 hover:from-blue-500 hover:to-indigo-500 shadow-blue-900/30'
            }`}
          >
            <Check size={14} />
            {config.confirmText || (config.type === 'alert' || config.type === 'info' ? 'Got it' : 'Confirm')}
          </button>
        </div>
      </div>
    </div>
  );
};
