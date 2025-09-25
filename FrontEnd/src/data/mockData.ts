import { DroneNode } from '../types/drone';

export const droneColor = (node: DroneNode) => {
  switch (node.data.missionStatus) {
    case 'in-progress':
      return '#00aaff'; // Blue
    case 'completed':
      return '#00ff88'; // Green
    case 'pending':
      return '#ffaa00'; // Orange
    case 'failed':
      return '#ff0055'; // Red
    default:
      return '#ffffff'; // White
  }
};

export const droneNodes: DroneNode[] = [
  {
    id: 'node-1',
    label: 'Node 1',
    lat: 12.844299,
    lon:  80.154302,
    estimatedTime: 12,
    priority: 1,
    data: {
      riskLevel: 'critical',
      missionStatus: 'in-progress',
      temperature: 28.5,
      humidity: 45,
      altitude: 150,
      signalStrength: 95,
      batteryLevel: 80,
      survivors: 0,
      timestamp: '2025-09-25T10:00:00Z',
    },
  },
  {
    id: 'node-2',
    label: 'Node 2',
    lat: 12.844548,
    lon: 80.153443,
    estimatedTime: 8,
    priority: 2,
    data: {
      riskLevel: 'high',
      missionStatus: 'in-progress',
      temperature: 27.8,
      humidity: 50,
      altitude: 160,
      signalStrength: 88,
      batteryLevel: 75,
      survivors: 2,
      timestamp: '2025-09-25T10:05:00Z',
    },
  },
  {
    id: 'node-3',
    label: 'Node 3',
    lat: 12.843129,
    lon: 80.153510,
    estimatedTime: 15,
    priority: 3,
    data: {
      riskLevel: 'medium',
      missionStatus: 'pending',
      temperature: 29.1,
      humidity: 48,
      altitude: 145,
      signalStrength: 72,
      batteryLevel: 90,
      survivors: 0,
      timestamp: '2025-09-25T10:10:00Z',
    },
  },
  {
    id: 'node-4',
    label: 'Node 4',
    lat: 12.843430,
    lon: 80.152377,
    estimatedTime: 5,
    priority: 4,
    data: {
      riskLevel: 'low',
      missionStatus: 'completed',
      temperature: 26.5,
      humidity: 55,
      altitude: 130,
      signalStrength: 99,
      batteryLevel: 65,
      survivors: 5,
      timestamp: '2025-09-25T10:15:00Z',
    },
  },
];