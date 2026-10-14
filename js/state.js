/**
 * FirmDev Studio — 3-Link Manipulator & Hydraulic Trajectory 3D
 * Module: Application State
 */

export const state = {
    // 3 Mechanical Link Lengths (meters)
    L1: 2.00, // Base Turret Height
    L2: 3.40, // Upper Arm (Shoulder-to-Elbow)
    L3: 2.60, // Forearm + End-Effector TCP

    gripperOpen: 65,
    payloadTransfer: true,
    showVelocityVector: true,
    showSampleTicks: true,
    showComparePath: true,

    helpers: {
        envelope: true,
        joints: true
    },

    // Active Folder Tab ('start' | 'end' | 'links')
    activeWaypoint: 'start',
    inputMode: 'cartesian', // 'cartesian' | 'joint'
    elbowSign: 1,           // +1 = Elbow Up, -1 = Elbow Down

    // Start Pose [A] & Target Pose [B]
    startPos: { x: 3.20, y: -2.20, z: 1.40 },
    startAngles: [0, 0, 0],
    startReachable: true,

    endPos: { x: 2.60, y: 2.50, z: 3.80 },
    endAngles: [0, 0, 0],
    endReachable: true,

    // Trajectory Interpolation Law
    interpLaw: 'quintic', // 'quintic' | 'cubic' | 'cartesian' | 'lspb'
    duration: 4.0,
    samplesCount: 100,

    // Transport Playback
    isPlaying: false,
    pingPong: true,
    direction: 1,
    speedMultiplier: 1.0,
    progress: 0.0,

    // Computed Trajectory Data
    samples: [],
    comparePositions: [],
    totalArcLength: 0,
    peakLinearSpeed: 0,

    // Live Hydraulic Telemetry (meters & percentage)
    cyl1Length: 1.24,
    cyl1Pct: 62,
    cyl2Length: 0.92,
    cyl2Pct: 48,

    // Chart Mode ('pos' | 'vel' | 'cyl')
    plotMode: 'pos'
};
