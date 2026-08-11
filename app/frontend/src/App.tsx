import React, { useState } from 'react';
import CalibView from './views/CalibView';
import ConfigView from './views/ConfigView';
import Layout from './components/Layout';

function App() {
  const [activeTab, setActiveTab] = useState('calib');

  const renderContent = () => {
    switch (activeTab) {
      case 'calib':
        return <CalibView />;
      case 'config':
        return <ConfigView />;
      case 'interactive':
        return <div className="p-8 text-xl">2D Interactive Teach (Coming Soon)</div>;
      case 'auto_planner':
        return <div className="p-8 text-xl">3D Auto Planner (Coming Soon)</div>;
      case 'digital_twin':
        return <div className="p-8 text-xl">3D Digital Twin (Coming Soon)</div>;
      default:
        return null;
    }
  };

  return (
    <Layout activeTab={activeTab} setActiveTab={setActiveTab}>
      {renderContent()}
    </Layout>
  );
}

export default App;
