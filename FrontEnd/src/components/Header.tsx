import { Shield, Settings, Download, Share2, Satellite, Map, Users, AlertTriangle } from 'lucide-react';
import { Button } from './ui/button';
import { Badge } from './ui/badge';
import { Switch } from './ui/switch';

interface HeaderProps {
  isSatelliteView: boolean;
  onToggleSatellite: (enabled: boolean) => void;
}

export function Header({ isSatelliteView, onToggleSatellite }: HeaderProps) {
  return (
    <header className="fixed top-0 left-0 right-0 z-50 border-b border-white/10 bg-black/20 backdrop-blur-xl supports-[backdrop-filter]:bg-black/10">
      <div className="container mx-auto px-6 h-20 flex items-center justify-between">
        {/* Logo and Title */}
        <div className="flex items-center gap-4">
          <div className="w-12 h-12 bg-gradient-to-br from-cyan-400 to-blue-600 rounded-xl flex items-center justify-center shadow-lg shadow-cyan-500/25">
            <Shield className="w-7 h-7 text-white" />
          </div>
          <div>
            <h1 className="text-xl font-bold bg-gradient-to-r from-cyan-400 to-blue-600 bg-clip-text text-transparent">
              SENTRY RESCUE DRONE
            </h1>
            <p className="text-sm text-cyan-300/80">Autonomous Emergency Response</p>
          </div>
        </div>

        {/* Status Indicators */}
        <div className="flex items-center gap-3">
          <Badge variant="outline" className="text-white/80 border-white/20 px-3 py-1" style={{border:"solid 1px rgba(255, 159, 159, 1)",color:"rgba(230, 90, 90, 1)",backgroundColor:"rgba(90, 20, 20, 0.6)"}}>
            <AlertTriangle className="w-4 h-4 mr-2 text-red-400" />
            2 Alerts
          </Badge>
          <Badge variant="outline" className="text-white/80 border-white/20 px-3 py-1" style={{border:"solid 1px rgba(122, 188, 126, 1)",color:"rgba(122, 188, 126, 1)",backgroundColor:"rgba(3, 71, 16, 0.6)"}}>
            <Users className="w-4 h-4 mr-2" />
            6 Drones Active
          </Badge>
        </div>

        {/* Satellite Toggle */}
        <div className="flex items-center gap-3 px-4 py-2 rounded-xl bg-white/10 backdrop-blur-sm border border-white/20">
          <Map className={`w-4 h-4 ${!isSatelliteView ? 'text-cyan-400' : 'text-gray-400'}`} />
          <Switch
            checked={isSatelliteView}
            onCheckedChange={onToggleSatellite}
            className="data-[state=checked]:bg-cyan-500"
          />
          <Satellite className={`w-4 h-4 ${isSatelliteView ? 'text-cyan-400' : 'text-gray-400'}`} />
          <span className="text-sm text-white/80">Satellite</span>
        </div>

        {/* Action Buttons */}
        <div className="flex items-center gap-2">
          <Button variant="outline" size="sm" className="border-white/20 bg-white/10 backdrop-blur-sm text-white hover:bg-white/20">
            <Download className="w-4 h-4 mr-2" />
            Export
          </Button>
          <Button variant="outline" size="sm" className="border-white/20 bg-white/10 backdrop-blur-sm text-white hover:bg-white/20">
            <Share2 className="w-4 h-4 mr-2" />
            Share
          </Button>
          <Button variant="outline" size="sm" className="border-white/20 bg-white/10 backdrop-blur-sm text-white hover:bg-white/20">
            <Settings className="w-4 h-4" />
          </Button>
        </div>
      </div>
    </header>
  );
}