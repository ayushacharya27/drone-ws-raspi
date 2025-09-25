import { motion, AnimatePresence } from 'motion/react';
import { X, MapPin, Thermometer, Droplets, Mountain, Signal, Battery, Users, Clock, AlertTriangle, CheckCircle } from 'lucide-react';
import { DroneNode } from '../types/drone';
import { droneColor } from '../data/mockData';
import { Card } from './ui/card';
import { Badge } from './ui/badge';
import { Button } from './ui/button';

interface NodeInfoPanelProps {
  node: DroneNode | null;
  onClose: () => void;
}

export function NodeInfoPanel({ node, onClose }: NodeInfoPanelProps) {
  if (!node) return null;

  const formatTimestamp = (timestamp: string) => {
    return new Date(timestamp).toLocaleString();
  };

  const getBatteryColor = (level: number) => {
    if (level > 60) return 'text-green-500';
    if (level > 30) return 'text-yellow-500';
    return 'text-red-500';
  };

  const getSignalColor = (strength: number) => {
    if (strength > 80) return 'text-green-500';
    if (strength > 50) return 'text-yellow-500';
    return 'text-red-500';
  };

  return (
    <AnimatePresence>
      <motion.div
        initial={{ x: 400, opacity: 0 }}
        animate={{ x: 0, opacity: 1 }}
        exit={{ x: 400, opacity: 0 }}
        transition={{ type: "spring", stiffness: 300, damping: 30 }}
        className="fixed right-0 top-0 h-full w-96 bg-black/40 backdrop-blur-xl border-l border-white/20 shadow-2xl z-50 overflow-y-auto"
        style={{ paddingTop: '80px' }}
      >
        <div className="h-full text-white">
          <div className="p-6">
            {/* Header */}
            <div className="flex items-center justify-between mb-6">
              <div>
                <h2 className="text-xl font-semibold text-white">{node.label}</h2>
                <div className="flex items-center gap-2 mt-2">
                </div>
              </div>
              <Button variant="ghost" size="sm" onClick={onClose} className="text-white hover:bg-white/20">
                <X className="w-4 h-4" />
              </Button>
            </div>

            {/* Mission Status */}
            <div className="mb-6 p-4 rounded-xl bg-white/10 border border-white/20">
              <div className="flex items-center justify-between mb-3">
                <h3 className="font-medium text-cyan-300">Mission Status</h3>
                <div className="flex items-center gap-2">
                  {node.data.missionStatus === 'completed' && <CheckCircle className="w-4 h-4 text-green-400" />}
                  {node.data.missionStatus === 'in-progress' && <Clock className="w-4 h-4 text-blue-400" />}
                  {node.data.missionStatus === 'pending' && <AlertTriangle className="w-4 h-4 text-orange-400" />}
                  <span className={`text-sm font-medium ${
                    node.data.missionStatus === 'completed' ? 'text-green-400' :
                    node.data.missionStatus === 'in-progress' ? 'text-blue-400' :
                    node.data.missionStatus === 'pending' ? 'text-orange-400' : 'text-red-400'
                  }`}>
                    {node.data.missionStatus.toUpperCase()}
                  </span>
                </div>
              </div>
              
              <div className="grid grid-cols-3 gap-4 text-center">
                <div>
                  <p className="text-xs text-gray-300">Priority</p>
                  <p className="text-lg font-bold" style={{ color: droneColor(node) }}>#{node.priority}</p>
                </div>
                <div>
                  <p className="text-xs text-gray-300">Risk Level</p>
                  <p className={`text-lg font-bold ${
                    node.data.riskLevel === 'critical' ? 'text-red-400' :
                    node.data.riskLevel === 'high' ? 'text-orange-400' :
                    node.data.riskLevel === 'medium' ? 'text-yellow-400' : 'text-green-400'
                  }`}>
                    {node.data.riskLevel.toUpperCase()}
                  </p>
                </div>
              </div>
            </div>

            {/* Survivors */}
            {node.data.survivors > 0 && (
              <div className="mb-6 p-4 rounded-xl bg-orange-500/20 border border-orange-400/30">
                <div className="flex items-center gap-2 mb-2">
                  <Users className="w-5 h-5 text-orange-400" />
                  <h3 className="font-medium text-orange-300">Survivors Detected</h3>
                </div>
                <p className="text-2xl font-bold text-orange-400">{node.data.survivors} individuals</p>
              </div>
            )}

            {/* GPS Coordinates */}
            <div className="mb-6">
              <div className="flex items-center gap-2 mb-3">
                <MapPin className="w-5 h-5 text-cyan-400" />
                <h3 className="font-medium text-cyan-300">GPS Coordinates</h3>
              </div>
              <div className="grid grid-cols-2 gap-4">
                <div className="bg-white/10 p-3 rounded-lg border border-white/20">
                  <p className="text-sm text-gray-300">Latitude</p>
                  <p className="font-mono text-white">{node.lat}</p>
                </div>
                <div className="bg-white/10 p-3 rounded-lg border border-white/20">
                  <p className="text-sm text-gray-300">Longitude</p>
                  <p className="font-mono text-white">{node.lon}</p>
                </div>
              </div>
            </div>


            {/* System Status */}
            <div className="mb-6">
              <h3 className="font-medium mb-3 text-blue-300">System Status</h3>
              <div className="space-y-4">
                <div className="flex items-center justify-between p-3 rounded-lg bg-white/5 border border-white/10">
                  <div className="flex items-center gap-2">
                    <Signal className={`w-4 h-4 ${getSignalColor(node.data.signalStrength)}`} />
                    <span className="text-white">Signal Strength</span>
                  </div>
                  <span className={`font-mono ${getSignalColor(node.data.signalStrength)}`}>
                    {node.data.signalStrength}%
                  </span>
                </div>
                
                <div className="flex items-center justify-between p-3 rounded-lg bg-white/5 border border-white/10">
                  <div className="flex items-center gap-2">
                    <Battery className={`w-4 h-4 ${getBatteryColor(node.data.batteryLevel)}`} />
                    <span className="text-white">Battery Level</span>
                  </div>
                  <span className={`font-mono ${getBatteryColor(node.data.batteryLevel)}`}>
                    {node.data.batteryLevel}%
                  </span>
                </div>
              </div>
            </div>

            {/* Timestamp */}
            <div className="border-t border-white/20 pt-4">
              <p className="text-sm text-gray-300">Last Updated</p>
              <p className="font-mono text-sm text-cyan-400">{formatTimestamp(node.data.timestamp)}</p>
            </div>

            {/* Actions */}
            <div className="mt-6 space-y-3">
              <Button 
                className="w-full bg-gradient-to-r from-cyan-500 to-blue-600 hover:from-cyan-600 hover:to-blue-700 text-white border-0"
              >
                Deploy Drone
              </Button>
              <div className="grid grid-cols-2 gap-2">
                <Button 
                  variant="outline" 
                  className="border-white/30 bg-white/10 text-white hover:bg-white/20"
                >
                  Export Data
                </Button>
                <Button 
                  variant="outline" 
                  className="border-white/30 bg-white/10 text-white hover:bg-white/20"
                >
                  View History
                </Button>
              </div>
            </div>
          </div>
        </div>
      </motion.div>
    </AnimatePresence>
  );
}