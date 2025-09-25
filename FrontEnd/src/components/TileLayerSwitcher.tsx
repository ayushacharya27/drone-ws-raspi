import { useState } from 'react';
import { TileLayer } from 'react-leaflet';
import { Satellite, Map } from 'lucide-react';
import { Switch } from './ui/switch';

export function TileLayerSwitcher() {
  const [isSatelliteView, setIsSatelliteView] = useState(false);

  const tileUrl = isSatelliteView
    ? 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'
    : 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png';

  const attribution = isSatelliteView
    ? 'Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community'
    : '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors';

  return (
    <>
      <TileLayer attribution={attribution} url={tileUrl} />
      
      {/* Satellite Toggle UI */}
      <div className="absolute top-4 right-4 z-40 flex items-center gap-3 px-4 py-2 rounded-xl bg-black/40 backdrop-blur-sm border border-white/20">
        <Map className={`w-4 h-4 ${!isSatelliteView ? 'text-cyan-400' : 'text-gray-400'}`} />
        <Switch
          checked={isSatelliteView}
          onCheckedChange={setIsSatelliteView}
          className="data-[state=checked]:bg-cyan-500"
        />
        <Satellite className={`w-4 h-4 ${isSatelliteView ? 'text-cyan-400' : 'text-gray-400'}`} />
        <span className="text-sm text-white/80">Satellite</span>
      </div>
    </>
  );
}
