/**
 * FirmDev Studio — 3-Link Manipulator & Hydraulic Trajectory 3D
 * Module: UI Controllers, DOM Bindings & Telemetry Updater
 */

import { DEG2RAD, PRESETS } from './config.js';
import { state } from './state.js';
import { forwardKinematics, inverseKinematics, robotToThreeVec } from './kinematics.js';
import { computeTrajectorySamples, sampleTrajectoryAtProgress } from './trajectory.js';
import {
    buildRobotAssembly,
    updateGripperOpening,
    updateHydraulicActuators,
    payloadMesh,
    velocityArrow,
    jointAxesHelpers,
    rebuildTrajectory3D,
    joint2Shoulder
} from './robotModel.js';
import {
    envelopeGroup,
    setCameraView as sceneSetCameraView,
    focusOnHydraulics as sceneFocusOnHydraulics,
    clearCameraActiveButtons
} from './scene.js';
import { renderSplineChart, setPlotMode } from './chart.js';

/**
 * Updates CSS gradient fill percentage on range inputs.
 */
export function syncRangeGradientFill(inputEl) {
    if (!inputEl) return;
    const min = parseFloat(inputEl.min) || 0;
    const max = parseFloat(inputEl.max) || 100;
    const val = parseFloat(inputEl.value) || 0;
    const pct = Math.max(0, Math.min(100, ((val - min) / (max - min)) * 100));
    inputEl.style.setProperty('--pct', `${pct.toFixed(1)}%`);
    return Math.round(pct);
}

export function switchFolderTab(tab) {
    document.getElementById('tabBtnStart').classList.toggle('is-active', tab === 'start');
    document.getElementById('tabBtnEnd').classList.toggle('is-active', tab === 'end');
    document.getElementById('tabBtnLinks').classList.toggle('is-active', tab === 'links');

    const paneWaypoints = document.getElementById('paneWaypoints');
    const paneLinks = document.getElementById('paneLinks');

    if (tab === 'links') {
        paneWaypoints.style.display = 'none';
        paneLinks.style.display = 'flex';
        syncRangeGradientFill(document.getElementById('sliderL1'));
        syncRangeGradientFill(document.getElementById('sliderL2'));
        syncRangeGradientFill(document.getElementById('sliderL3'));
        syncRangeGradientFill(document.getElementById('sliderGripper'));
    } else {
        paneWaypoints.style.display = 'flex';
        paneLinks.style.display = 'none';
        state.activeWaypoint = tab;

        const isStart = tab === 'start';
        document.getElementById('waypointCardTitle').textContent =
            isStart ? 'Koordinat Posisi Awal [A]' : 'Koordinat Posisi Akhir [B]';
        document.getElementById('waypointCardSub').textContent =
            isStart ? 'Ubah titik awal end-effector secara dinamis' : 'Ubah titik target akhir end-effector';

        const tag = document.getElementById('activePoseTag');
        tag.textContent = isStart ? 'POSE A' : 'POSE B';
        tag.className = isStart ? 'pill-tag pill-mint' : 'pill-tag pill-coral';

        document.getElementById('miniBoxA').classList.toggle('active-a', isStart);
        document.getElementById('miniBoxB').classList.toggle('active-b', !isStart);

        syncWaypointInputsUI();
    }
}

export function setInputMode(mode) {
    state.inputMode = mode;
    document.getElementById('btnModeCart').className =
        mode === 'cartesian' ? 'btn-pill is-active-mint' : 'btn-pill';
    document.getElementById('btnModeJoint').className =
        mode === 'joint' ? 'btn-pill is-active-purple' : 'btn-pill';

    document.getElementById('cartesianSlidersBox').style.display = mode === 'cartesian' ? 'flex' : 'none';
    document.getElementById('jointSlidersBox').style.display = mode === 'joint' ? 'flex' : 'none';
    syncWaypointInputsUI();
}

export function setElbowSign(sign) {
    state.elbowSign = sign;
    document.getElementById('btnElbowUp').className =
        sign === 1 ? 'btn-pill is-active-blue' : 'btn-pill';
    document.getElementById('btnElbowDown').className =
        sign === -1 ? 'btn-pill is-active-blue' : 'btn-pill';

    recomputeBothWaypointsFromCartesian();
    syncWaypointInputsUI();
    computeTrajectorySamples();
    updateLiveTelemetryUI();
}

export function recomputeBothWaypointsFromCartesian() {
    const resA = inverseKinematics(state.startPos.x, state.startPos.y, state.startPos.z, state.elbowSign);
    state.startAngles = resA.angles;
    state.startReachable = resA.reachable;

    const resB = inverseKinematics(state.endPos.x, state.endPos.y, state.endPos.z, state.elbowSign);
    state.endAngles = resB.angles;
    state.endReachable = resB.reachable;
}

export function onWaypointParamInput(axis, valStr) {
    const val = parseFloat(valStr);
    if (isNaN(val)) return;

    const targetPos = state.activeWaypoint === 'start' ? state.startPos : state.endPos;
    targetPos[axis] = val;

    const ik = inverseKinematics(targetPos.x, targetPos.y, targetPos.z, state.elbowSign);
    if (state.activeWaypoint === 'start') {
        state.startAngles = ik.angles;
        state.startReachable = ik.reachable;
    } else {
        state.endAngles = ik.angles;
        state.endReachable = ik.reachable;
    }

    syncWaypointInputsUI();
    computeTrajectorySamples();
    updateLiveTelemetryUI();
}

export function onWaypointJointInput(jointIdx, valStr) {
    const val = parseFloat(valStr);
    if (isNaN(val)) return;

    if (state.activeWaypoint === 'start') {
        state.startAngles[jointIdx] = val;
        state.startPos = { ...forwardKinematics(state.startAngles).tcp };
        state.startReachable = true;
    } else {
        state.endAngles[jointIdx] = val;
        state.endPos = { ...forwardKinematics(state.endAngles).tcp };
        state.endReachable = true;
    }

    syncWaypointInputsUI();
    computeTrajectorySamples();
    updateLiveTelemetryUI();
}

export function update5SegmentBar(containerId, valId, pct, activeClass) {
    const clamped = Math.max(0, Math.min(100, Math.round(pct)));
    const valEl = document.getElementById(valId);
    if (valEl) valEl.textContent = `${clamped}%`;
    const litCount = Math.round((clamped / 100) * 5);
    const container = document.getElementById(containerId);
    if (container) {
        const spans = container.querySelectorAll('span');
        spans.forEach((sp, idx) => {
            sp.className = idx < litCount ? activeClass : '';
        });
    }
}

export function fmtSigned(num, dec = 2) {
    return `${num >= 0 ? '+' : ''}${num.toFixed(dec)}`;
}

export function syncWaypointInputsUI() {
    const pos = state.activeWaypoint === 'start' ? state.startPos : state.endPos;
    const angles = state.activeWaypoint === 'start' ? state.startAngles : state.endAngles;

    const sX = document.getElementById('sliderPosX');
    const sY = document.getElementById('sliderPosY');
    const sZ = document.getElementById('sliderPosZ');

    sX.value = pos.x;
    document.getElementById('numPosX').value = pos.x.toFixed(2);
    document.getElementById('pctAxisX').textContent = `${syncRangeGradientFill(sX)}%`;

    sY.value = pos.y;
    document.getElementById('numPosY').value = pos.y.toFixed(2);
    document.getElementById('pctAxisY').textContent = `${syncRangeGradientFill(sY)}%`;

    sZ.value = pos.z;
    document.getElementById('numPosZ').value = pos.z.toFixed(2);
    document.getElementById('pctAxisZ').textContent = `${syncRangeGradientFill(sZ)}%`;

    const j1 = document.getElementById('sliderJoint1');
    const j2 = document.getElementById('sliderJoint2');
    const j3 = document.getElementById('sliderJoint3');

    j1.value = Math.round(angles[0]);
    document.getElementById('numJoint1').value = Math.round(angles[0]);
    document.getElementById('degJoint1').textContent = `${fmtSigned(angles[0], 0)}°`;
    syncRangeGradientFill(j1);

    j2.value = Math.round(angles[1]);
    document.getElementById('numJoint2').value = Math.round(angles[1]);
    document.getElementById('degJoint2').textContent = `${fmtSigned(angles[1], 0)}°`;
    syncRangeGradientFill(j2);

    j3.value = Math.round(angles[2]);
    document.getElementById('numJoint3').value = Math.round(angles[2]);
    document.getElementById('degJoint3').textContent = `${fmtSigned(angles[2], 0)}°`;
    syncRangeGradientFill(j3);

    // 5-segment bars
    const maxR = state.L2 + state.L3;
    const xyReachPct = (Math.hypot(pos.x, pos.y) / maxR) * 100;
    const elevPct = (pos.z / (state.L1 + maxR)) * 100;
    const shldrPct = ((angles[1] + 35) / 170) * 100;

    update5SegmentBar('segBarsReach', 'segValReach', xyReachPct, 'active-mint');
    update5SegmentBar('segBarsElev', 'segValElev', elevPct, 'active-purple');
    update5SegmentBar('segBarsShoulder', 'segValShoulder', shldrPct, 'active-blue');

    // Mini Comparison Boxes
    const fkA = forwardKinematics(state.startAngles).tcp;
    const fkB = forwardKinematics(state.endAngles).tcp;

    document.getElementById('miniXYZA').textContent =
        `${fkA.x.toFixed(2)}, ${fkA.y.toFixed(2)}, ${fkA.z.toFixed(2)}m`;
    document.getElementById('miniJointsA').textContent =
        `${Math.round(state.startAngles[0])}°,${Math.round(state.startAngles[1])}°,${Math.round(state.startAngles[2])}°`;

    document.getElementById('miniXYZB').textContent =
        `${fkB.x.toFixed(2)}, ${fkB.y.toFixed(2)}, ${fkB.z.toFixed(2)}m`;
    document.getElementById('miniJointsB').textContent =
        `${Math.round(state.endAngles[0])}°,${Math.round(state.endAngles[1])}°,${Math.round(state.endAngles[2])}°`;

    const ikBadge = document.getElementById('ikStatusBadge');
    if (state.startReachable && state.endReachable) {
        ikBadge.textContent = 'IK VALID';
        ikBadge.className = 'pill-tag pill-mint';
    } else {
        ikBadge.textContent = 'CLAMPED MAX';
        ikBadge.className = 'pill-tag pill-coral';
    }
}

export function copyPoseAtoB() {
    state.endPos = { ...state.startPos };
    state.endAngles = [...state.startAngles];
    state.endReachable = state.startReachable;
    syncWaypointInputsUI();
    computeTrajectorySamples();
    updateLiveTelemetryUI();
}

export function swapPoseAandB() {
    const tmpPos = { ...state.startPos };
    const tmpAng = [...state.startAngles];
    const tmpReach = state.startReachable;

    state.startPos = { ...state.endPos };
    state.startAngles = [...state.endAngles];
    state.startReachable = state.endReachable;

    state.endPos = tmpPos;
    state.endAngles = tmpAng;
    state.endReachable = tmpReach;

    syncWaypointInputsUI();
    computeTrajectorySamples();
    updateLiveTelemetryUI();
}

export function randomizeValidPose() {
    const rand = (min, max) => min + Math.random() * (max - min);
    const angles = [rand(-135, 135), rand(10, 105), rand(-110, -20)];
    const fk = forwardKinematics(angles);

    if (state.activeWaypoint === 'start') {
        state.startAngles = angles;
        state.startPos = { ...fk.tcp };
        state.startReachable = true;
    } else {
        state.endAngles = angles;
        state.endPos = { ...fk.tcp };
        state.endReachable = true;
    }

    syncWaypointInputsUI();
    computeTrajectorySamples();
    updateLiveTelemetryUI();
}

export function applyIndustrialPreset(presetName, slotNum) {
    [1, 2, 3, 4].forEach(n => {
        const btn = document.getElementById(`presetBtn${n}`);
        if (btn) btn.classList.toggle('is-selected', n === slotNum);
    });

    const preset = PRESETS[presetName];
    if (preset) {
        state.startPos = { ...preset.startPos };
        state.endPos = { ...preset.endPos };
        state.interpLaw = preset.interpLaw;
        state.duration = preset.duration;
    }

    document.getElementById('interpLawSelect').value = state.interpLaw;
    document.getElementById('sliderDuration').value = state.duration;
    document.getElementById('numDuration').value = state.duration.toFixed(1);
    syncRangeGradientFill(document.getElementById('sliderDuration'));

    recomputeBothWaypointsFromCartesian();
    syncWaypointInputsUI();
    onInterpLawChange(state.interpLaw);
    state.progress = 0;
    state.direction = 1;
    updateLiveTelemetryUI();
}

export function onLinkGeometryChange() {
    state.L1 = parseFloat(document.getElementById('sliderL1').value);
    state.L2 = parseFloat(document.getElementById('sliderL2').value);
    state.L3 = parseFloat(document.getElementById('sliderL3').value);

    document.getElementById('numL1').value = state.L1.toFixed(2);
    document.getElementById('numL2').value = state.L2.toFixed(2);
    document.getElementById('numL3').value = state.L3.toFixed(2);

    document.getElementById('lblValL1').textContent = `${state.L1.toFixed(2)} m`;
    document.getElementById('lblValL2').textContent = `${state.L2.toFixed(2)} m`;
    document.getElementById('lblValL3').textContent = `${state.L3.toFixed(2)} m`;
    document.getElementById('maxReachTag').textContent = `R_MAX: ${(state.L2 + state.L3).toFixed(1)}m`;

    syncRangeGradientFill(document.getElementById('sliderL1'));
    syncRangeGradientFill(document.getElementById('sliderL2'));
    syncRangeGradientFill(document.getElementById('sliderL3'));

    buildRobotAssembly();
    recomputeBothWaypointsFromCartesian();
    syncWaypointInputsUI();
    computeTrajectorySamples();
    updateLiveTelemetryUI();
}

export function onLinkGeometryNumChange(linkKey, valStr) {
    const val = parseFloat(valStr);
    if (isNaN(val)) return;
    document.getElementById('slider' + linkKey).value = val;
    onLinkGeometryChange();
}

export function onGripperInput(valStr) {
    const val = Math.max(0, Math.min(100, parseFloat(valStr) || 0));
    state.gripperOpen = val;
    const sGrip = document.getElementById('sliderGripper');
    sGrip.value = val;
    document.getElementById('numGripper').value = Math.round(val);
    document.getElementById('lblValGrip').textContent = `${Math.round(val)}%`;
    syncRangeGradientFill(sGrip);
    updateGripperOpening(val);
}

export function onInterpLawChange(val) {
    state.interpLaw = val;
    const shortNames = {
        quintic: 'QUINTIC C²',
        cubic: 'CUBIC C¹',
        cartesian: 'LINEAR IK',
        lspb: 'LSPB CRUISE'
    };
    document.getElementById('kpiLawShort').textContent = shortNames[val];
    document.getElementById('lawSpaceBadge').textContent = val === 'cartesian' ? 'CARTESIAN' : 'JOINT';
    computeTrajectorySamples();
    updateLiveTelemetryUI();
}

export function onTrajectoryTimingChange() {
    const sDur = document.getElementById('sliderDuration');
    const sSamp = document.getElementById('sliderSamples');
    state.duration = parseFloat(sDur.value);
    state.samplesCount = parseInt(sSamp.value, 10);

    document.getElementById('numDuration').value = state.duration.toFixed(1);
    document.getElementById('lblDuration').textContent = `${state.duration.toFixed(1)} s`;
    document.getElementById('numSamples').value = state.samplesCount;
    document.getElementById('lblSamples').textContent = `${state.samplesCount} pt`;

    syncRangeGradientFill(sDur);
    syncRangeGradientFill(sSamp);

    computeTrajectorySamples();
    updateLiveTelemetryUI();
}

export function onDurationNumInput(valStr) {
    const v = Math.max(1.0, Math.min(10.0, parseFloat(valStr) || 4.0));
    document.getElementById('sliderDuration').value = v;
    onTrajectoryTimingChange();
}

export function onSamplesNumInput(valStr) {
    const v = Math.max(20, Math.min(200, parseInt(valStr, 10) || 100));
    document.getElementById('sliderSamples').value = v;
    onTrajectoryTimingChange();
}

export function setCameraView(preset) {
    sceneSetCameraView(preset);
}

export function focusOnHydraulics() {
    const shldrPos = new THREE.Vector3();
    joint2Shoulder.getWorldPosition(shldrPos);
    sceneFocusOnHydraulics(shldrPos);
}

export function toggleSceneHelper(key) {
    state.helpers[key] = !state.helpers[key];
    if (key === 'envelope') {
        if (envelopeGroup) envelopeGroup.visible = state.helpers.envelope;
        document.getElementById('railBtnEnvelope').classList.toggle('is-active', state.helpers.envelope);
    } else if (key === 'joints') {
        jointAxesHelpers.forEach(h => { h.visible = state.helpers.joints; });
        document.getElementById('railBtnJoints').classList.toggle('is-active', state.helpers.joints);
    }
}

export function togglePayloadTransfer() {
    state.payloadTransfer = !state.payloadTransfer;
    payloadMesh.visible = state.payloadTransfer;
    document.getElementById('railBtnPayload').classList.toggle('is-active', state.payloadTransfer);
    updateLiveTelemetryUI();
}

export function toggleComparePath() {
    state.showComparePath = !state.showComparePath;
    document.getElementById('toggleCompareBtn').classList.toggle('is-active', state.showComparePath);
    rebuildTrajectory3D();
}

export function toggleWaypointTicks() {
    state.showSampleTicks = !state.showSampleTicks;
    document.getElementById('toggleTicksBtn').classList.toggle('is-active', state.showSampleTicks);
    rebuildTrajectory3D();
}

export function toggleVelocityVector() {
    state.showVelocityVector = !state.showVelocityVector;
    document.getElementById('toggleVelVecBtn').classList.toggle('is-active', state.showVelocityVector);
    velocityArrow.visible = state.showVelocityVector;
    updateLiveTelemetryUI();
}

export function updatePlayPauseButton() {
    const btn = document.getElementById('playPauseBtn');
    if (!btn) return;
    if (state.isPlaying) {
        btn.className = 'btn-pill is-active-mint';
        btn.innerHTML = '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 320 512" width="11" height="11" fill="currentColor" aria-hidden="true" style="display: inline-block; vertical-align: -1px; margin-right: 6px;"><path d="M48 64C21.5 64 0 85.5 0 112L0 400c0 26.5 21.5 48 48 48l32 0c26.5 0 48-21.5 48-48l0-288c0-26.5-21.5-48-48-48L48 64zm192 0c-26.5 0-48 21.5-48 48l0 288c0 26.5 21.5 48 48 48l32 0c26.5 0 48-21.5 48-48l0-288c0-26.5-21.5-48-48-48l-32 0z"/></svg>PAUSE';
        btn.title = 'Jeda Simulasi [Spasi]';
    } else {
        btn.className = 'btn-pill is-active-red';
        btn.innerHTML = '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 384 512" width="11" height="11" fill="currentColor" aria-hidden="true" style="display: inline-block; vertical-align: -1px; margin-right: 6px;"><path d="M73 39c-14.8-9.1-33.4-9.4-48.5-.9S0 62.6 0 80L0 432c0 17.4 9.4 33.4 24.5 41.9s33.7 8.1 48.5-.9L361 297c14.3-8.7 23-24.2 23-41s-8.7-32.2-23-41L73 39z"/></svg>PLAY';
        btn.title = 'Jalankan Simulasi [Spasi]';
    }
}

export function togglePlayback() {
    state.isPlaying = !state.isPlaying;
    updatePlayPauseButton();
    if (state.isPlaying && state.progress >= 1.0 && !state.pingPong) {
        state.progress = 0;
    }
}

export function resetTimeline() {
    state.progress = 0;
    state.direction = 1;
    updateLiveTelemetryUI();
}

export function jumpToProgress(targetProg) {
    if (state.isPlaying) togglePlayback();
    state.progress = targetProg;
    updateLiveTelemetryUI();
}

export function togglePingPongMode() {
    state.pingPong = !state.pingPong;
    const btn = document.getElementById('pingPongBtn');
    btn.className = state.pingPong ? 'btn-pill is-active-purple' : 'btn-pill';
    if (!state.pingPong) state.direction = 1;
}

export function setPlaybackSpeed(mult) {
    state.speedMultiplier = mult;
    document.getElementById('spd05Btn').className = mult === 0.5 ? 'btn-pill is-active-blue' : 'btn-pill';
    document.getElementById('spd10Btn').className = mult === 1.0 ? 'btn-pill is-active-blue' : 'btn-pill';
    document.getElementById('spd20Btn').className = mult === 2.0 ? 'btn-pill is-active-blue' : 'btn-pill';
}

export function onScrubberSlide(val) {
    if (state.isPlaying) togglePlayback();
    state.progress = parseFloat(val) / 1000;
    updateLiveTelemetryUI();
}

export function initKeyboardShortcuts() {
    window.addEventListener('keydown', e => {
        if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
        if (e.code === 'Space') {
            e.preventDefault();
            togglePlayback();
        }
    });
}

/**
 * Updates full live telemetry UI across HUD, KPI cards, rings, pillars, and chart.
 */
export function updateLiveTelemetryUI() {
    const cur = sampleTrajectoryAtProgress(state.progress);
    if (!cur) return;

    // 1. Update 3D Robot & Non-Floating Telescopic Hydraulic Cylinders
    updateHydraulicActuators(cur.angles);

    // 2. Payload Cube & Velocity Vector
    if (state.payloadTransfer) {
        payloadMesh.position.copy(robotToThreeVec(cur.pos));
        payloadMesh.rotation.y = cur.angles[0] * DEG2RAD;
        const txt = document.getElementById('payloadStatusText');
        if (txt) {
            txt.textContent = state.progress < 0.98 ? 'Benda Kerja: Sedang Diangkut' : 'Benda Kerja: Tiba di Target [B]';
        }
    } else {
        const txt = document.getElementById('payloadStatusText');
        if (txt) txt.textContent = 'Benda Kerja: Nonaktif';
    }

    if (state.showVelocityVector && cur.speed > 0.02) {
        velocityArrow.visible = true;
        velocityArrow.position.copy(robotToThreeVec(cur.pos));
        const vDir = new THREE.Vector3(cur.velVec.x, cur.velVec.z, -cur.velVec.y).normalize();
        velocityArrow.setDirection(vDir);
        velocityArrow.setLength(Math.min(2.2, 0.35 + cur.speed * 0.45), 0.20, 0.10);
    } else {
        velocityArrow.visible = false;
    }

    // 3. Top KPI Header Cards
    if (state.samples && state.samples.length > 0) {
        const endTarget = state.samples[state.samples.length - 1].pos;
        const distRem = Math.hypot(
            endTarget.x - cur.pos.x,
            endTarget.y - cur.pos.y,
            endTarget.z - cur.pos.z
        );

        const elSpeed = document.getElementById('kpiLinearSpeed');
        if (elSpeed) elSpeed.textContent = cur.speed.toFixed(2);
        const elPeak = document.getElementById('kpiPeakSpeed');
        if (elPeak) elPeak.textContent = `+${state.peakLinearSpeed.toFixed(2)} max`;
        const elArc = document.getElementById('kpiArcLength');
        if (elArc) elArc.textContent = state.totalArcLength.toFixed(2);
        const elRem = document.getElementById('kpiDistRem');
        if (elRem) elRem.textContent = distRem.toFixed(2);
        const elProg = document.getElementById('kpiProgressPct');
        if (elProg) elProg.textContent = `${Math.round(state.progress * 100)}%`;
    }

    // Hydraulic Cylinder Stroke in mm + Tick Strip
    const cylMm = Math.round(state.cyl1Length * 1000);
    const elCylMm = document.getElementById('kpiCylStrokeMm');
    if (elCylMm) elCylMm.textContent = cylMm.toLocaleString('en-US');
    const strip = document.getElementById('headerTickStrip');
    if (strip) {
        const tickSpans = strip.querySelectorAll('span');
        const litTicks = Math.round((state.cyl1Pct / 100) * tickSpans.length);
        tickSpans.forEach((sp, i) => {
            sp.className = i < litTicks ? 'lit-mint' : '';
        });
    }

    // 4. Transport Scrubber & Stepper Nodes
    const scrubber = document.getElementById('timelineScrubber');
    if (scrubber) {
        scrubber.value = Math.round(state.progress * 1000);
        syncRangeGradientFill(scrubber);
    }
    const readout = document.getElementById('timeReadout');
    if (readout) {
        readout.textContent = `t = ${cur.t.toFixed(2)} / ${state.duration.toFixed(1)}s`;
    }

    const milestones = [
        { id: 'stepNode0', p: 0.0 },
        { id: 'stepNode25', p: 0.25 },
        { id: 'stepNode50', p: 0.50 },
        { id: 'stepNode75', p: 0.75 },
        { id: 'stepNode100', p: 1.0 }
    ];
    milestones.forEach(m => {
        const el = document.getElementById(m.id);
        if (el) {
            el.classList.toggle('passed', state.progress >= m.p - 0.02);
            el.classList.toggle('current', Math.abs(state.progress - m.p) < 0.08);
        }
    });

    // 5. Featured Indigo Card (TCP XYZ)
    const rHoriz = Math.hypot(cur.pos.x, cur.pos.y);
    const elRBadge = document.getElementById('tcpRadiusBadge');
    if (elRBadge) elRBadge.textContent = `R = ${rHoriz.toFixed(2)}m`;
    const elTeleX = document.getElementById('teleX');
    if (elTeleX) elTeleX.textContent = `${fmtSigned(cur.pos.x, 2)}m`;
    const elTeleY = document.getElementById('teleY');
    if (elTeleY) elTeleY.textContent = `${fmtSigned(cur.pos.y, 2)}m`;
    const elTeleZ = document.getElementById('teleZ');
    if (elTeleZ) elTeleZ.textContent = `${fmtSigned(cur.pos.z, 2)}m`;

    // 6. Concentric Multi-Ring Gauge & Hydraulic Pillars
    const maxReach = state.L2 + state.L3;
    const reach3D = Math.hypot(cur.pos.x, cur.pos.y, cur.pos.z - state.L1);
    const reachPct = Math.min(100, Math.round((reach3D / maxReach) * 100));

    const elDonut = document.getElementById('donutCenterVal');
    if (elDonut) elDonut.textContent = `${reachPct}%`;
    const elRingReach = document.getElementById('ringReach');
    if (elRingReach) elRingReach.style.strokeDashoffset = (314.16 * (1 - reachPct / 100)).toFixed(1);
    const elRingCyl1 = document.getElementById('ringCyl1');
    if (elRingCyl1) elRingCyl1.style.strokeDashoffset = (245.04 * (1 - state.cyl1Pct / 100)).toFixed(1);
    const elRingCyl2 = document.getElementById('ringCyl2');
    if (elRingCyl2) elRingCyl2.style.strokeDashoffset = (175.93 * (1 - state.cyl2Pct / 100)).toFixed(1);

    const pCyl1 = document.getElementById('pillarCyl1');
    if (pCyl1) pCyl1.style.height = `${state.cyl1Pct}%`;
    const pVal1 = document.getElementById('pillarVal1');
    if (pVal1) pVal1.textContent = `${state.cyl1Pct}%`;

    const pCyl2 = document.getElementById('pillarCyl2');
    if (pCyl2) pCyl2.style.height = `${state.cyl2Pct}%`;
    const pVal2 = document.getElementById('pillarVal2');
    if (pVal2) pVal2.textContent = `${state.cyl2Pct}%`;

    const bTxt1 = document.getElementById('barTxtCyl1');
    if (bTxt1) bTxt1.textContent = `${state.cyl1Length.toFixed(2)} m (${state.cyl1Pct}%)`;
    const bFill1 = document.getElementById('barFillCyl1');
    if (bFill1) bFill1.style.width = `${state.cyl1Pct}%`;

    const bTxt2 = document.getElementById('barTxtCyl2');
    if (bTxt2) bTxt2.textContent = `${state.cyl2Length.toFixed(2)} m (${state.cyl2Pct}%)`;
    const bFill2 = document.getElementById('barFillCyl2');
    if (bFill2) bFill2.style.width = `${state.cyl2Pct}%`;

    // 7. Joint State Readouts & Equalizer Bars
    const j1El = document.getElementById('j1Readout');
    if (j1El) j1El.textContent = `${fmtSigned(cur.angles[0], 1)}° | ${Math.round(cur.omega[0])}°/s`;
    const j2El = document.getElementById('j2Readout');
    if (j2El) j2El.textContent = `${fmtSigned(cur.angles[1], 1)}° | ${Math.round(cur.omega[1])}°/s`;
    const j3El = document.getElementById('j3Readout');
    if (j3El) j3El.textContent = `${fmtSigned(cur.angles[2], 1)}° | ${Math.round(cur.omega[2])}°/s`;

    // Dynamic Stacked Bar Proportions
    const a1 = Math.abs(cur.angles[0]) + 15;
    const a2 = Math.abs(cur.angles[1]) + 15;
    const a3 = Math.abs(cur.angles[2]) + 15;
    const sumA = a1 + a2 + a3;
    const seg1 = document.getElementById('stackSeg1');
    if (seg1) seg1.style.width = `${((a1 / sumA) * 100).toFixed(1)}%`;
    const seg2 = document.getElementById('stackSeg2');
    if (seg2) seg2.style.width = `${((a2 / sumA) * 100).toFixed(1)}%`;
    const seg3 = document.getElementById('stackSeg3');
    if (seg3) seg3.style.width = `${((a3 / sumA) * 100).toFixed(1)}%`;

    // 8-Bar Capsule Equalizer Animation
    const eqBars = document.getElementById('equalizerBars');
    if (eqBars) {
        const eqFills = eqBars.querySelectorAll('.cap-bar-fill');
        const eqHeights = [
            Math.min(95, 25 + Math.abs(cur.angles[0]) * 0.45),
            Math.min(95, 20 + Math.abs(cur.omega[0]) * 1.1),
            Math.min(95, 25 + Math.abs(cur.angles[1]) * 0.55),
            Math.min(95, 20 + Math.abs(cur.omega[1]) * 1.1),
            state.cyl1Pct,
            Math.min(95, 25 + Math.abs(cur.angles[2]) * 0.50),
            Math.min(95, 20 + Math.abs(cur.omega[2]) * 1.1),
            state.cyl2Pct
        ];
        eqFills.forEach((el, idx) => {
            el.style.height = `${Math.max(14, Math.round(eqHeights[idx]))}%`;
        });
    }

    // 8. 4x4 Homogeneous Matrix SE(3)
    for (let r = 0; r < 4; r++) {
        for (let c = 0; c < 4; c++) {
            const cell = document.getElementById(`m${r}${c}`);
            if (cell) {
                const val = cur.matrix[r][c];
                cell.textContent = r < 3 ? fmtSigned(val, 2) : val.toFixed(2);
            }
        }
    }

    renderSplineChart();
}
