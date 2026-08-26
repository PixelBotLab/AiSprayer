import { useState } from 'react';
import WorkspaceView from './views/WorkspaceView';
import ConfigView from './views/ConfigView';
import Layout from './components/Layout';

function App() {
  const [activeTab, setActiveTab] = useState('interactive');
  const [isCameraVisible, setIsCameraVisible] = useState(false);

  const renderContent = () => {
    switch (activeTab) {
      case 'config':
        return <ConfigView />;
      default:
        // All other tabs (calib, interactive, task) 
        // share the unified WorkspaceView layout
        return (
          <WorkspaceView 
            activeTab={activeTab} 
            isCameraVisible={isCameraVisible}
            setIsCameraVisible={setIsCameraVisible}
          />
        );
    }
  };

  return (
    <Layout 
      activeTab={activeTab} 
      setActiveTab={setActiveTab}
      isCameraVisible={isCameraVisible}
      setIsCameraVisible={setIsCameraVisible}
    >
      {renderContent()}
    </Layout>
  );
}

export default App;
