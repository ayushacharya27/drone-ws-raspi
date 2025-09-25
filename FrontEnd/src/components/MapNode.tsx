import { motion } from 'motion/react';
import { DroneNode } from '../types/drone';
import { droneColor } from '../data/mockData';
import { Shield, Users } from 'lucide-react';

interface MapNodeProps {
  node: DroneNode;
  isSelected: boolean;
  onClick: (node: DroneNode) => void;
}

const getRiskColor = (riskLevel: string) => {
  switch (riskLevel) {
    case 'critical': return '#ff0055';
    case 'high': return '#ff6600';
    case 'medium': return '#ffaa00';
    case 'low': return '#00ff88';
    default: return '#ffffff';
  }
};

const getStatusColor = (status: string) => {
  switch (status) {
    case 'completed': return '#00ff88';
    case 'in-progress': return '#00aaff';
    case 'pending': return '#ffaa00';
    case 'failed': return '#ff0055';
    default: return '#ffffff';
  }
};

export function MapNode({ node, isSelected, onClick }: MapNodeProps) {
  const riskColor = getRiskColor(node.data.riskLevel);
  const statusColor = getStatusColor(node.data.missionStatus);

  return (
    <motion.div
      initial={{ scale: 0, opacity: 0 }}
      animate={{ scale: 1, opacity: 1 }}
      whileHover={{ scale: 1.3 }}
      whileTap={{ scale: 0.9 }}
      className="absolute cursor-pointer rounded-full z-20"
      style={{
        width: '18px',
        height: '18px',
        background: `radial-gradient(circle, ${droneColor(node)} 50%, transparent 50%)`,
        border: `2px solid white`,
        boxShadow: `0 0 10px ${isSelected ? 'cyan' : 'transparent'}`,
        transition: 'box-shadow 0.2s ease-in-out',
        top: `calc(${node.lat} - 9px)`,
        left: `calc(${node.lon} - 9px)`,
      }}
      onClick={() => onClick(node)}
    >
      {/* Node status icon */}
      <div className="absolute -top-3 -right-3 w-6 h-6 rounded-full flex items-center justify-center">
        {node.data.missionStatus === 'in-progress' && (
          <Shield className="w-4 h-4 text-white" style={{ color: statusColor }} />
        )}
      </div>

      {/* Pulsing animation for high priority missions */}
      {(node.data.riskLevel === 'critical' || node.priority === 1) && (
        <motion.div
          className="absolute inset-0 rounded-full border-2"
          style={{ borderColor: riskColor }}
          animate={{
            scale: [1, 1.8, 1],
            opacity: [0.8, 0, 0.8],
          }}
          transition={{
            duration: 1.5,
            repeat: Infinity,
          }}
        />
      )}

      {/* Survivors indicator */}
      {node.data.survivors > 0 && (
        <div className="absolute -bottom-2 left-1/2 transform -translate-x-1/2">
          <div className="bg-orange-500 text-white px-2 py-1 rounded-full flex items-center gap-1 text-xs">
            <Users className="w-3 h-3" />
            <span>{node.data.survivors}</span>
          </div>
        </div>
      )}

      {/* Node label */}
      <div className="absolute top-14 left-1/2 transform -translate-x-1/2 whitespace-nowrap opacity-0 hover:opacity-100 transition-all duration-200 z-30">
        <div className="bg-black/90 backdrop-blur-sm text-white px-3 py-2 rounded-lg text-sm shadow-xl border border-white/20">
          <div className="font-semibold">{node.label}</div>
          <div className="text-xs opacity-80 flex items-center gap-2">
            Status: <span style={{ color: statusColor }}>{node.data.missionStatus}</span>
          </div>
        </div>
      </div>
    </motion.div>
  );
}