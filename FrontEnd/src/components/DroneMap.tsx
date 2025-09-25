import { useState, useMemo, useEffect } from 'react';
import { MapContainer, TileLayer, CircleMarker, Polyline, useMap } from 'react-leaflet';
import { motion } from 'motion/react';
import * as L from 'leaflet';
import 'leaflet-polylinedecorator';
import { NodeInfoPanel } from './NodeInfoPanel';
import { droneNodes, droneColor } from '../data/mockData';
import { DroneNode as DroneNodeType } from '../types/drone';

interface DroneMapProps {
  isSatelliteView: boolean;
}

// Create a custom component to handle the polyline decoration
const DirectedPolyline = ({ positions, pathOptions, arrowSize }: any) => {
  const map = useMap();
  useEffect(() => {
    if (!map) return;

    // Clear any existing polylines with decorators
    map.eachLayer((layer: any) => {
      if (layer instanceof L.Polyline) {
        map.removeLayer(layer);
      }
    });

    const polyline = L.polyline(positions, pathOptions).addTo(map);

    L.polylineDecorator(polyline, {
      patterns: [
        {
          offset: '20%',
          repeat: '20%',
          symbol: L.Symbol.arrowHead({
            pixelSize: arrowSize,
            polygon: false,
            pathOptions: { stroke: true, color: 'cyan', weight: 2 },
          }),
        },
      ],
    }).addTo(map);

    return () => {
      map.removeLayer(polyline);
    };
  }, [map, positions, pathOptions, arrowSize]);

  return null;
};


export function DroneMap({ isSatelliteView }: DroneMapProps) {
  const [selectedNode, setSelectedNode] = useState<DroneNodeType | null>(null);

  const handleNodeClick = (node: DroneNodeType) => {
    setSelectedNode(node);
  };

  const handleClosePanel = () => {
    setSelectedNode(null);
  };

  // Sort nodes by priority for routing
  const sortedNodes = useMemo(() => {
    return [...droneNodes].sort((a, b) => a.priority - b.priority);
  }, []);

  // Determine the map center from the nodes' coordinates
  const mapCenter = useMemo(() => {
    if (sortedNodes.length === 0) return [34.0522, -118.2437] as [number, number];
    const latSum = sortedNodes.reduce((sum, n) => sum + n.lat, 0);
    const lonSum = sortedNodes.reduce((sum, n) => sum + n.lon, 0);
    return [latSum / sortedNodes.length, lonSum / sortedNodes.length] as [number, number];
  }, [sortedNodes]);

  const tileUrl = isSatelliteView
    ? 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'
    : 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png';

  const attribution = isSatelliteView
    ? 'Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community'
    : '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors';

  const pathCoordinates = useMemo(() => {
    return sortedNodes.map(node => [node.lat, node.lon] as [number, number]);
  }, [sortedNodes]);

  return (
    <div className="h-screen bg-gradient-to-br from-slate-900 via-gray-900 to-black overflow-hidden"
         style={{ paddingTop: '80px' }}>
      <div className="container mx-auto px-6 h-full">
        <div className="relative w-full h-full bg-slate-900/50 backdrop-blur-sm border border-white/10 rounded-2xl overflow-hidden shadow-2xl">
          <MapContainer
          center={mapCenter}
          zoom={10000}
          scrollWheelZoom={true}
          style={{ zIndex: 0 , height:"98%",width:"98%", borderRadius:"15px",margin:"10px 10px 10px 10px"}}
        >
          <TileLayer attribution={attribution} url={tileUrl} />
          
          {/* The new component for the directed polyline */}
          {pathCoordinates.length > 1 && (
            <DirectedPolyline
              positions={pathCoordinates}
              pathOptions={{ color: 'red', opacity: 1, weight: 5}}
              arrowSize={15}
            />
          )}

          {sortedNodes.map((node) => (
            <CircleMarker
              key={node.id}
              center={[node.lat, node.lon]}
              radius={18}
              pathOptions={{
                fillColor: droneColor(node),
                color: 'rgba(231, 255, 54, 1)',
                weight: 2,
                fillOpacity: 1,
                opacity: 1
              }}
              eventHandlers={{
                click: () => handleNodeClick(node),
              }}
            />
          ))}
        </MapContainer>
          
          <div className="absolute inset-0 z-10 pointer-events-none">
            {/* Radar sweep animation */}
            <motion.div
              className="absolute inset-0 opacity-20"
              style={{
                background: `conic-gradient(from 0deg, transparent 0deg, rgba(0,255,255,0.3) 30deg, transparent 60deg)`,
                transformOrigin: 'center',
              }}
              animate={{ rotate: [0, 360], }}
              transition={{ duration: 8, repeat: Infinity, ease: 'linear', }}
            />

            {/* Animated scan lines */}
            <motion.div
              className="absolute inset-0 opacity-10"
              style={{
                backgroundImage: `repeating-linear-gradient(to bottom, transparent 0, transparent 2px, rgba(0,255,255,0.1) 3px, transparent 4px)`,
              }}
              animate={{
                backgroundPositionY: ['-100%', '100%'],
              }}
              transition={{
                duration: 5,
                repeat: Infinity,
                ease: 'linear',
              }}
            />
          </div>

          <div className="absolute p-8 z-20 pointer-events-auto" style={{position:"fixed",left:"20px",bottom:"20px"}}>
            <div className="bg-black/40 backdrop-blur-xl border border-white/20 text-white p-4 rounded-xl shadow-xl">
              <div className="text-sm text-cyan-300 mb-2">System Status</div>
              <div className="grid grid-cols-3 gap-6">
                <div>
                  <p className="text-red-300">Alerts</p>
                  <p className="text-2xl font-bold">{droneNodes.filter(n => n.data.riskLevel === 'critical').length}</p>
                </div>
                <div>
                  <p className="text-cyan-300">Active Nodes</p>
                  <p className="text-2xl font-bold">{droneNodes.length}</p>
                </div>
                <div>
                  <p className="text-purple-300">Pending</p>
                  <p className="text-2xl font-bold">{droneNodes.filter(n => n.data.missionStatus === 'pending').length}</p>
                </div>
                <div>
                  <p className="text-green-300">Completed</p>
                  <p className="text-2xl font-bold">{droneNodes.filter(n => n.data.missionStatus === 'completed').length}</p>
                </div>
                <div>
                  <p className="text-yellow-300">Survivors Found</p>
                  <p className="text-2xl font-bold">{droneNodes.reduce((sum, n) => sum + n.data.survivors, 0)}</p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>

      <NodeInfoPanel node={selectedNode} onClose={handleClosePanel} />
    </div>
  );
}