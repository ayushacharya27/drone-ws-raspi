export interface DroneNode {
  id: string;
  label: string;
  lat: number;
  lon: number;
  estimatedTime: number;
  priority: number;
  data: {
    riskLevel: 'critical' | 'high' | 'medium' | 'low';
    missionStatus: 'in-progress' | 'completed' | 'pending' | 'failed';
    temperature: number;
    humidity: number;
    altitude: number;
    signalStrength: number;
    batteryLevel: number;
    survivors: number;
    timestamp: string;
  };
}