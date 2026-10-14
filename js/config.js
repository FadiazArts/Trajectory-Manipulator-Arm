/**
 * FirmDev Studio — 3-Link Manipulator & Hydraulic Trajectory 3D
 * Module: Config & Constants
 */

export const DEG2RAD = Math.PI / 180;
export const RAD2DEG = 180 / Math.PI;

export const PRESETS = {
    pickPlace: {
        startPos: { x: 3.50, y: -2.40, z: 0.70 },
        endPos: { x: 3.10, y: 2.70, z: 1.40 },
        interpLaw: 'quintic',
        duration: 4.0
    },
    highZenith: {
        // High Z elevation test to showcase non-floating hydraulic cylinder at steep upward angles
        startPos: { x: 3.40, y: -1.80, z: 1.50 },
        endPos: { x: 1.40, y: 1.20, z: 6.80 },
        interpLaw: 'quintic',
        duration: 4.5
    },
    verticalWeld: {
        startPos: { x: 3.70, y: 0.00, z: 0.85 },
        endPos: { x: 3.70, y: 0.00, z: 5.40 },
        interpLaw: 'cartesian',
        duration: 4.5
    },
    wideSweep: {
        startPos: { x: -2.40, y: -3.40, z: 2.20 },
        endPos: { x: -2.20, y: 3.50, z: 4.40 },
        interpLaw: 'lspb',
        duration: 5.0
    }
};

export const CAMERA_PRESETS = {
    iso: {
        position: [8.0, 5.8, 8.2],
        target: [0, 2.3, 0],
        activeBtnId: 'camPerspBtn',
        railBtnId: 'railBtnPersp'
    },
    top: {
        position: [0.01, 15.0, 0],
        target: [0, 0, 0],
        activeBtnId: 'camTopBtn'
    },
    side: {
        position: [0, 3.0, 11.5],
        target: [0, 2.4, 0],
        activeBtnId: 'camSideBtn'
    },
    front: {
        position: [11.5, 3.0, 0],
        target: [0, 2.4, 0],
        activeBtnId: 'camFrontBtn'
    }
};
