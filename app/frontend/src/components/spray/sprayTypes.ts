export type SprayVisualMode = 'realistic' | 'heatmap';

export interface SpraySimConfig {
  enabled: boolean;
  visualMode: SprayVisualMode;
  paintColor: string;            // Hex string e.g. '#2563eb'
  targetDistanceMm: number;      // Nominal standoff distance (mm), e.g. 150
  sprayWidthMm: number;          // Spray width at target distance (mm), e.g. 60
  flowRateMicronsPerSec: number; // Deposition rate (um/s), default 15.0
  targetMinThickness: number;    // Lower threshold for standard thickness (um), e.g. 35
  targetMaxThickness: number;    // Upper threshold for standard thickness (um), e.g. 65
}

export const DEFAULT_SPRAY_CONFIG: SpraySimConfig = {
  enabled: false,
  visualMode: 'realistic',
  paintColor: '#2563eb', // Royal Blue
  targetDistanceMm: 150,
  sprayWidthMm: 60,
  flowRateMicronsPerSec: 15.0,
  targetMinThickness: 35.0,
  targetMaxThickness: 65.0,
};

export interface SprayCoverageStats {
  totalVertices: number;
  coveredVertices: number;
  coveragePercentage: number;     // 0.0 to 100.0%
  averageThickness: number;       // In micrometers (um)
  maxThickness: number;           // In micrometers (um)
  standardComplianceRate: number; // 0.0 to 100.0% within [min, max]
}
