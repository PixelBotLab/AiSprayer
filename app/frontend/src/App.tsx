import { useState } from 'react';
import WorkspaceView from './views/WorkspaceView';
import ConfigView from './views/ConfigView';
import Layout from './components/Layout';

function App() {
  const [activeTab, setActiveTab] = useState('interactive');

  const renderContent = () => {
    switch (activeTab) {
      case 'config':
        return <ConfigView />;
      default:
        // All other tabs (calib, interactive, auto_planner, digital_twin) 
        // share the unified WorkspaceView layout
        return <WorkspaceView activeTab={activeTab} />;
    }
  };

  return (
    <Layout activeTab={activeTab} setActiveTab={setActiveTab}>
      {renderContent()}
    </Layout>
  );
}

export default App;
