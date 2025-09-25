import { useState } from 'react';
import { Header } from './components/Header';
import { DroneMap } from './components/DroneMap';
import 'leaflet/dist/leaflet.css';


export default function App() {
  const [isSatelliteView, setIsSatelliteView] = useState(false);

  return (
    <div className="h-screen bg-gradient-to-br from-slate-900 via-gray-900 to-black">
      <Header
        isSatelliteView={isSatelliteView}
        onToggleSatellite={setIsSatelliteView}
      />
      <main className="h-full">
        <DroneMap isSatelliteView={isSatelliteView} />
      </main>
    </div>
  );
}