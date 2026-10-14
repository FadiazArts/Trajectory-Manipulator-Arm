/**
 * FirmDev Studio — 3-Link Manipulator & Hydraulic Trajectory 3D
 * Module: Forward & Inverse Kinematics (3R Anthropomorphic Arm)
 */

import { DEG2RAD, RAD2DEG } from './config.js';
import { state } from './state.js';

/**
 * Computes forward kinematics for a 3R anthropomorphic robot arm.
 * Returns joint positions (shoulder, elbow, TCP) and the 4x4 SE(3) homogeneous transform matrix.
 */
export function forwardKinematics(anglesDeg, L1 = state.L1, L2 = state.L2, L3 = state.L3) {
    const t1 = anglesDeg[0] * DEG2RAD;
    const t2 = anglesDeg[1] * DEG2RAD;
    const t3 = anglesDeg[2] * DEG2RAD;
    const t23 = t2 + t3;

    const shoulder = { x: 0, y: 0, z: L1 };

    const rElbow = L2 * Math.cos(t2);
    const zElbow = L1 + L2 * Math.sin(t2);
    const elbow = {
        x: rElbow * Math.cos(t1),
        y: rElbow * Math.sin(t1),
        z: zElbow
    };

    const rTcp = rElbow + L3 * Math.cos(t23);
    const zTcp = zElbow + L3 * Math.sin(t23);
    const tcp = {
        x: rTcp * Math.cos(t1),
        y: rTcp * Math.sin(t1),
        z: zTcp
    };

    const c1 = Math.cos(t1), s1 = Math.sin(t1);
    const c23 = Math.cos(t23), s23 = Math.sin(t23);

    const matrix = [
        [c1 * c23, -c1 * s23,  s1, tcp.x],
        [s1 * c23, -s1 * s23, -c1, tcp.y],
        [s23,       c23,       0,  tcp.z],
        [0,         0,         0,  1]
    ];

    return { shoulder, elbow, tcp, matrix };
}

/**
 * Analytical inverse kinematics solver for 3-DOF articulated arm.
 * Handles reachability boundaries with safe clamping and configurable elbow configuration (+1 or -1).
 */
export function inverseKinematics(targetX, targetY, targetZ, elbowSign = state.elbowSign, L1 = state.L1, L2 = state.L2, L3 = state.L3) {
    const theta1 = Math.atan2(targetY, targetX);
    const r = Math.hypot(targetX, targetY);
    const dz = targetZ - L1;
    let d = Math.hypot(r, dz);

    const maxReach = (L2 + L3) - 0.005;
    const minReach = Math.abs(L2 - L3) + 0.005;

    let reachable = true;
    if (d > maxReach || d < minReach) {
        reachable = false;
        d = Math.max(minReach, Math.min(maxReach, d));
    }

    let cosTheta3 = (d * d - L2 * L2 - L3 * L3) / (2 * L2 * L3);
    cosTheta3 = Math.max(-1, Math.min(1, cosTheta3));

    const sinTheta3 = -elbowSign * Math.sqrt(Math.max(0, 1 - cosTheta3 * cosTheta3));
    const theta3 = Math.atan2(sinTheta3, cosTheta3);

    const alpha = Math.atan2(dz, r);
    const beta = Math.atan2(L3 * Math.sin(theta3), L2 + L3 * Math.cos(theta3));
    const theta2 = alpha - beta;

    const anglesDeg = [
        theta1 * RAD2DEG,
        theta2 * RAD2DEG,
        theta3 * RAD2DEG
    ];

    const fk = forwardKinematics(anglesDeg, L1, L2, L3);
    return {
        angles: anglesDeg,
        reachable,
        actualPos: fk.tcp
    };
}

/**
 * Converts robot world coordinates (X forward, Y left, Z up)
 * to Three.js coordinate system (X right, Y up, Z forward).
 */
export function robotToThreeVec(pos) {
    return new THREE.Vector3(pos.x, pos.z, -pos.y);
}
