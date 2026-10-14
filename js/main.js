/**
 * FirmDev Studio — 3-Link Manipulator & Hydraulic Trajectory 3D
 * Module: Main Application Bootstrap & Animation Loop
 */

import { state } from './state.js';
import { initScene, scene, camera, renderer, controls } from './scene.js';
import { buildRobotAssembly, rebuildTrajectory3D } from './robotModel.js';
import { computeTrajectorySamples, exportTrajectoryCSV, onTrajectoryComputed } from './trajectory.js';
import { initChart, renderSplineChart, setPlotMode } from './chart.js';
import {
    syncRangeGradientFill,
    switchFolderTab,
    setInputMode,
    setElbowSign,
    recomputeBothWaypointsFromCartesian,
    onWaypointParamInput,
    onWaypointJointInput,
    syncWaypointInputsUI,
    copyPoseAtoB,
    swapPoseAandB,
    randomizeValidPose,
    applyIndustrialPreset,
    onLinkGeometryChange,
    onLinkGeometryNumChange,
    onGripperInput,
    onInterpLawChange,
    onTrajectoryTimingChange,
    onDurationNumInput,
    onSamplesNumInput,
    setCameraView,
    focusOnHydraulics,
    toggleSceneHelper,
    togglePayloadTransfer,
    toggleComparePath,
    toggleWaypointTicks,
    toggleVelocityVector,
    updatePlayPauseButton,
    togglePlayback,
    resetTimeline,
    jumpToProgress,
    togglePingPongMode,
    setPlaybackSpeed,
    onScrubberSlide,
    updateLiveTelemetryUI,
    initKeyboardShortcuts
} from './ui.js';

// Hook trajectory updates to 3D trajectory rebuild and chart redraw
onTrajectoryComputed(() => {
    rebuildTrajectory3D();
    renderSplineChart();
});

// Expose interaction handlers to window scope for seamless HTML inline event bindings
Object.assign(window, {
    switchFolderTab,
    setInputMode,
    setElbowSign,
    recomputeBothWaypointsFromCartesian,
    onWaypointParamInput,
    onWaypointJointInput,
    syncWaypointInputsUI,
    copyPoseAtoB,
    swapPoseAandB,
    randomizeValidPose,
    applyIndustrialPreset,
    onLinkGeometryChange,
    onLinkGeometryNumChange,
    onGripperInput,
    onInterpLawChange,
    onTrajectoryTimingChange,
    onDurationNumInput,
    onSamplesNumInput,
    setCameraView,
    focusOnHydraulics,
    toggleSceneHelper,
    togglePayloadTransfer,
    toggleComparePath,
    toggleWaypointTicks,
    toggleVelocityVector,
    updatePlayPauseButton,
    togglePlayback,
    resetTimeline,
    jumpToProgress,
    togglePingPongMode,
    setPlaybackSpeed,
    onScrubberSlide,
    setPlotMode,
    exportTrajectoryCSV
});

// Expose app state for debugging & telemetry inspection in browser console
window.__FIRMDEV_STATE__ = state;

let lastTick = performance.now();
let containerEl = null;

export function handleViewportResize() {
    if (!containerEl) {
        containerEl = document.getElementById('canvasContainer');
    }
    if (containerEl && camera && renderer) {
        const rect = containerEl.getBoundingClientRect();
        if (rect.width > 0 && rect.height > 0) {
            camera.aspect = rect.width / rect.height;
            camera.updateProjectionMatrix();
            renderer.setSize(rect.width, rect.height);
        }
    }
    renderSplineChart();
}

export function animateLoop(now) {
    requestAnimationFrame(animateLoop);

    const dt = Math.min(0.1, (now - lastTick) / 1000);
    lastTick = now;

    if (state.isPlaying) {
        const deltaProg = (dt * state.speedMultiplier * state.direction) / state.duration;
        state.progress += deltaProg;

        if (state.progress >= 1.0) {
            if (state.pingPong) {
                state.progress = 1.0;
                state.direction = -1;
            } else {
                state.progress = 0.0;
            }
        } else if (state.progress <= 0.0) {
            state.progress = 0.0;
            state.direction = 1;
        }

        updateLiveTelemetryUI();
    }

    if (controls) controls.update();
    if (renderer && scene && camera) renderer.render(scene, camera);
}

export function initApp() {
    // Explicitly guarantee simulation starts in paused state at t = 0
    state.isPlaying = false;
    state.progress = 0.0;
    state.direction = 1;

    containerEl = document.getElementById('canvasContainer');
    initScene(containerEl);
    initChart(document.getElementById('kinematicChartCanvas'));

    buildRobotAssembly(scene);
    recomputeBothWaypointsFromCartesian();
    syncWaypointInputsUI();

    syncRangeGradientFill(document.getElementById('sliderDuration'));
    syncRangeGradientFill(document.getElementById('sliderSamples'));

    computeTrajectorySamples();
    handleViewportResize();
    updateLiveTelemetryUI();
    updatePlayPauseButton();
    initKeyboardShortcuts();

    window.addEventListener('resize', handleViewportResize);

    lastTick = performance.now();
    requestAnimationFrame(animateLoop);
}

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initApp);
} else {
    initApp();
}
