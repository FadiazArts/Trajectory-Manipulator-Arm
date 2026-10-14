/**
 * FirmDev Studio — 3-Link Manipulator & Hydraulic Trajectory 3D
 * Module: Trajectory Interpolation Engine & CSV Exporter
 */

import { state } from './state.js';
import { forwardKinematics, inverseKinematics } from './kinematics.js';

const listeners = [];

/**
 * Registers a listener callback invoked whenever trajectory samples are recomputed.
 */
export function onTrajectoryComputed(callback) {
    if (typeof callback === 'function') {
        listeners.push(callback);
    }
}

/**
 * Evaluates polynomial or LSPB time scaling law for normalized time tau in [0, 1].
 * Returns { s, ds, d2s } where s in [0, 1].
 */
export function evaluateTimeScalingLaw(tau, T, law) {
    tau = Math.max(0, Math.min(1, tau));
    if (law === 'cubic') {
        const s = 3 * Math.pow(tau, 2) - 2 * Math.pow(tau, 3);
        const ds = (6 * tau - 6 * Math.pow(tau, 2)) / T;
        const d2s = (6 - 12 * tau) / (T * T);
        return { s, ds, d2s };
    } else if (law === 'lspb') {
        const tb = 0.25;
        const vMax = 1 / (1 - tb);
        const aMax = vMax / tb;
        let s = 0, ds = 0, d2s = 0;
        if (tau <= tb) {
            s = 0.5 * aMax * tau * tau;
            ds = (aMax * tau) / T;
            d2s = aMax / (T * T);
        } else if (tau <= 1 - tb) {
            s = 0.5 * aMax * tb * tb + vMax * (tau - tb);
            ds = vMax / T;
            d2s = 0;
        } else {
            const dtRem = 1 - tau;
            s = 1 - 0.5 * aMax * dtRem * dtRem;
            ds = (aMax * dtRem) / T;
            d2s = -aMax / (T * T);
        }
        return { s, ds, d2s };
    } else {
        // Quintic (C^2 Continuous) default
        const s = 10 * Math.pow(tau, 3) - 15 * Math.pow(tau, 4) + 6 * Math.pow(tau, 5);
        const ds = (30 * Math.pow(tau, 2) - 60 * Math.pow(tau, 3) + 30 * Math.pow(tau, 4)) / T;
        const d2s = (60 * tau - 180 * Math.pow(tau, 2) + 120 * Math.pow(tau, 3)) / (T * T);
        return { s, ds, d2s };
    }
}

/**
 * Computes all trajectory sample points across the configured time duration and sample count.
 */
export function computeTrajectorySamples() {
    const N = state.samplesCount;
    const T = Math.max(0.5, state.duration);
    const samples = [];
    const comparePositions = [];

    const qA = state.startAngles;
    const qB = state.endAngles;
    const pA = forwardKinematics(qA).tcp;
    const pB = forwardKinematics(qB).tcp;

    for (let i = 0; i <= N; i++) {
        const tau = i / N;
        const t = tau * T;

        if (state.interpLaw === 'cartesian') {
            const { s } = evaluateTimeScalingLaw(tau, T, 'quintic');
            const tx = pA.x + (pB.x - pA.x) * s;
            const ty = pA.y + (pB.y - pA.y) * s;
            const tz = pA.z + (pB.z - pA.z) * s;

            const ik = inverseKinematics(tx, ty, tz, state.elbowSign);
            const fk = forwardKinematics(ik.angles);

            samples.push({
                t,
                tau,
                angles: ik.angles,
                omega: [0, 0, 0],
                alpha: [0, 0, 0],
                pos: fk.tcp,
                matrix: fk.matrix,
                speed: 0,
                velVec: { x: 0, y: 0, z: 0 }
            });

            const compQ = [
                qA[0] + (qB[0] - qA[0]) * s,
                qA[1] + (qB[1] - qA[1]) * s,
                qA[2] + (qB[2] - qA[2]) * s
            ];
            comparePositions.push(forwardKinematics(compQ).tcp);
        } else {
            const { s, ds, d2s } = evaluateTimeScalingLaw(tau, T, state.interpLaw);
            const angles = [0, 0, 0];
            const omega = [0, 0, 0];
            const alpha = [0, 0, 0];

            for (let j = 0; j < 3; j++) {
                const dq = qB[j] - qA[j];
                angles[j] = qA[j] + dq * s;
                omega[j] = dq * ds;
                alpha[j] = dq * d2s;
            }

            const fk = forwardKinematics(angles);
            samples.push({
                t,
                tau,
                angles,
                omega,
                alpha,
                pos: fk.tcp,
                matrix: fk.matrix,
                speed: 0,
                velVec: { x: 0, y: 0, z: 0 }
            });

            comparePositions.push({
                x: pA.x + (pB.x - pA.x) * s,
                y: pA.y + (pB.y - pA.y) * s,
                z: pA.z + (pB.z - pA.z) * s
            });
        }
    }

    const dt = T / N;
    let arcLen = 0;
    let peakSpeed = 0;

    for (let i = 0; i <= N; i++) {
        const prev = samples[Math.max(0, i - 1)];
        const next = samples[Math.min(N, i + 1)];
        const span = (i === 0 || i === N) ? dt : 2 * dt;

        const vx = (i === 0 || i === N) ? 0 : (next.pos.x - prev.pos.x) / span;
        const vy = (i === 0 || i === N) ? 0 : (next.pos.y - prev.pos.y) / span;
        const vz = (i === 0 || i === N) ? 0 : (next.pos.z - prev.pos.z) / span;

        samples[i].velVec = { x: vx, y: vy, z: vz };
        samples[i].speed = Math.hypot(vx, vy, vz);
        if (samples[i].speed > peakSpeed) peakSpeed = samples[i].speed;

        if (i > 0) {
            arcLen += Math.hypot(
                samples[i].pos.x - samples[i - 1].pos.x,
                samples[i].pos.y - samples[i - 1].pos.y,
                samples[i].pos.z - samples[i - 1].pos.z
            );
        }

        if (state.interpLaw === 'cartesian') {
            for (let j = 0; j < 3; j++) {
                samples[i].omega[j] = (i === 0 || i === N) ? 0 : (next.angles[j] - prev.angles[j]) / span;
            }
        }
    }

    state.samples = samples;
    state.comparePositions = comparePositions;
    state.totalArcLength = arcLen;
    state.peakLinearSpeed = peakSpeed;

    listeners.forEach(fn => fn(state));
}

/**
 * Samples instantaneous kinematic state at given progress [0..1].
 */
export function sampleTrajectoryAtProgress(prog) {
    prog = Math.max(0, Math.min(1, prog));
    const N = state.samples.length - 1;
    if (N <= 0) return null;

    const exact = prog * N;
    const i0 = Math.floor(exact);
    const i1 = Math.min(N, i0 + 1);
    const f = exact - i0;

    const s0 = state.samples[i0];
    const s1 = state.samples[i1];
    const lerp = (a, b, u) => a + (b - a) * u;

    const angles = [
        lerp(s0.angles[0], s1.angles[0], f),
        lerp(s0.angles[1], s1.angles[1], f),
        lerp(s0.angles[2], s1.angles[2], f)
    ];
    const fk = forwardKinematics(angles);

    return {
        t: lerp(s0.t, s1.t, f),
        angles,
        omega: [
            lerp(s0.omega[0], s1.omega[0], f),
            lerp(s0.omega[1], s1.omega[1], f),
            lerp(s0.omega[2], s1.omega[2], f)
        ],
        pos: fk.tcp,
        matrix: fk.matrix,
        speed: lerp(s0.speed, s1.speed, f),
        velVec: {
            x: lerp(s0.velVec.x, s1.velVec.x, f),
            y: lerp(s0.velVec.y, s1.velVec.y, f),
            z: lerp(s0.velVec.z, s1.velVec.z, f)
        }
    };
}

/**
 * Exports computed trajectory samples as a CSV spreadsheet.
 */
export function exportTrajectoryCSV() {
    if (state.samples.length === 0) return;
    const headers = [
        'sample_idx', 'time_s', 'tau',
        'theta1_deg', 'theta2_deg', 'theta3_deg',
        'omega1_deg_s', 'omega2_deg_s', 'omega3_deg_s',
        'tcp_x_m', 'tcp_y_m', 'tcp_z_m', 'linear_speed_m_s'
    ];
    const rows = [headers.join(',')];
    state.samples.forEach((s, idx) => {
        rows.push([
            idx,
            s.t.toFixed(4),
            s.tau.toFixed(4),
            s.angles[0].toFixed(3),
            s.angles[1].toFixed(3),
            s.angles[2].toFixed(3),
            s.omega[0].toFixed(3),
            s.omega[1].toFixed(3),
            s.omega[2].toFixed(3),
            s.pos.x.toFixed(4),
            s.pos.y.toFixed(4),
            s.pos.z.toFixed(4),
            s.speed.toFixed(4)
        ].join(','));
    });

    const blob = new Blob([rows.join('\n')], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = `firmdev_3dof_${state.interpLaw}.csv`;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
}
