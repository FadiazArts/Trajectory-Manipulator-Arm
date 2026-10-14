/**
 * FirmDev Studio — 3-Link Manipulator & Hydraulic Trajectory 3D
 * Module: Kinematic Spline Chart & Floating Callout Badge
 */

import { state } from './state.js';

let chartCanvas = null;

export function initChart(canvasEl) {
    chartCanvas = canvasEl || document.getElementById('kinematicChartCanvas');
}

export function setPlotMode(mode) {
    state.plotMode = mode;
    const tabPos = document.getElementById('plotTabPos');
    const tabVel = document.getElementById('plotTabVel');
    const tabCyl = document.getElementById('plotTabCyl');
    if (tabPos) tabPos.className = mode === 'pos' ? 'btn-pill is-active-mint' : 'btn-pill';
    if (tabVel) tabVel.className = mode === 'vel' ? 'btn-pill is-active-mint' : 'btn-pill';
    if (tabCyl) tabCyl.className = mode === 'cyl' ? 'btn-pill is-active-mint' : 'btn-pill';

    const sub = document.getElementById('chartSubLabel');
    if (sub) {
        if (mode === 'pos') sub.textContent = 'Sudut Sendi θ₁, θ₂, θ₃ terhadap waktu (derajat)';
        else if (mode === 'vel') sub.textContent = 'Kecepatan Sudut ω₁, ω₂, ω₃ terhadap waktu (°/s)';
        else sub.textContent = 'Koordinat Kartesian X, Y, Z End-Effector (meter)';
    }

    renderSplineChart();
}

export function renderSplineChart() {
    if (!chartCanvas) {
        chartCanvas = document.getElementById('kinematicChartCanvas');
    }
    if (!chartCanvas || !state.samples || state.samples.length === 0) return;

    const rect = chartCanvas.getBoundingClientRect();
    if (rect.width === 0 || rect.height === 0) return;

    const dpr = window.devicePixelRatio || 1;
    const targetW = Math.floor(rect.width * dpr);
    const targetH = Math.floor(rect.height * dpr);
    if (chartCanvas.width !== targetW || chartCanvas.height !== targetH) {
        chartCanvas.width = targetW;
        chartCanvas.height = targetH;
    }

    const ctx = chartCanvas.getContext('2d');
    ctx.save();
    ctx.scale(dpr, dpr);

    const W = rect.width;
    const H = rect.height;
    const padL = 36, padR = 14, padT = 24, padB = 18;
    const plotW = W - padL - padR;
    const plotH = H - padT - padB;

    ctx.clearRect(0, 0, W, H);

    let s1 = [], s2 = [], s3 = [];
    if (state.plotMode === 'pos') {
        s1 = state.samples.map(s => s.angles[0]);
        s2 = state.samples.map(s => s.angles[1]);
        s3 = state.samples.map(s => s.angles[2]);
    } else if (state.plotMode === 'vel') {
        s1 = state.samples.map(s => s.omega[0]);
        s2 = state.samples.map(s => s.omega[1]);
        s3 = state.samples.map(s => s.omega[2]);
    } else {
        s1 = state.samples.map(s => s.pos.x);
        s2 = state.samples.map(s => s.pos.y);
        s3 = state.samples.map(s => s.pos.z);
    }

    let minVal = Infinity, maxVal = -Infinity;
    [s1, s2, s3].forEach(arr => {
        arr.forEach(v => {
            if (v < minVal) minVal = v;
            if (v > maxVal) maxVal = v;
        });
    });

    if (Math.abs(maxVal - minVal) < 1e-3) {
        minVal -= 1;
        maxVal += 1;
    } else {
        const pad = (maxVal - minVal) * 0.15;
        minVal -= pad;
        maxVal += pad;
    }

    const valToY = v => padT + plotH - ((v - minVal) / (maxVal - minVal)) * plotH;

    // Subtle Horizontal Grid Lines
    ctx.strokeStyle = 'rgba(160, 169, 192, 0.08)';
    ctx.lineWidth = 1;
    ctx.fillStyle = '#727C94';
    ctx.font = '500 10px "Poppins", sans-serif';
    ctx.textAlign = 'right';

    for (let r = 0; r <= 3; r++) {
        const v = minVal + (r / 3) * (maxVal - minVal);
        const y = Math.round(valToY(v)) + 0.5;
        ctx.beginPath();
        ctx.moveTo(padL, y);
        ctx.lineTo(W - padR, y);
        ctx.stroke();

        const lbl = state.plotMode === 'cyl' ? v.toFixed(1) : Math.round(v).toString();
        ctx.fillText(lbl, padL - 6, y + 3);
    }

    // Draw Smooth Curves + Area Gradient under Primary Curve
    const traces = [
        { data: s3, color: '#FF862E', width: 1.6 },
        { data: s2, color: '#9646EF', width: 1.8 },
        { data: s1, color: '#43D1A7', width: 2.2, fillArea: true }
    ];

    const N = s1.length - 1;

    traces.forEach(tr => {
        if (tr.fillArea) {
            const grad = ctx.createLinearGradient(0, padT, 0, padT + plotH);
            grad.addColorStop(0, 'rgba(67, 209, 167, 0.22)');
            grad.addColorStop(1, 'rgba(67, 209, 167, 0.0)');
            ctx.fillStyle = grad;
            ctx.beginPath();
            tr.data.forEach((v, idx) => {
                const x = padL + (idx / N) * plotW;
                const y = valToY(v);
                if (idx === 0) ctx.moveTo(x, y);
                else ctx.lineTo(x, y);
            });
            ctx.lineTo(padL + plotW, padT + plotH);
            ctx.lineTo(padL, padT + plotH);
            ctx.closePath();
            ctx.fill();
        }

        ctx.strokeStyle = tr.color;
        ctx.lineWidth = tr.width;
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';
        ctx.beginPath();
        tr.data.forEach((v, idx) => {
            const x = padL + (idx / N) * plotW;
            const y = valToY(v);
            if (idx === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        });
        ctx.stroke();
    });

    // Playhead Vertical Line & Glowing Callout Dot
    const cursorX = padL + state.progress * plotW;
    const exactIdx = Math.min(N, Math.max(0, Math.round(state.progress * N)));
    const activeVal = s2[exactIdx];
    const cursorY = valToY(activeVal);

    ctx.strokeStyle = 'rgba(255, 255, 255, 0.22)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cursorX, padT);
    ctx.lineTo(cursorX, padT + plotH);
    ctx.stroke();

    // Outer Glow Halo on Curve Point
    ctx.fillStyle = 'rgba(67, 209, 167, 0.28)';
    ctx.beginPath();
    ctx.arc(cursorX, cursorY, 8, 0, Math.PI * 2);
    ctx.fill();

    // Crisp White Dot
    ctx.fillStyle = '#FFFFFF';
    ctx.beginPath();
    ctx.arc(cursorX, cursorY, 4, 0, Math.PI * 2);
    ctx.fill();

    // Floating Teal Callout Badge ('1.802' style)
    const badgeText = state.plotMode === 'cyl'
        ? `${activeVal.toFixed(2)}m`
        : `${activeVal.toFixed(1)}°`;
    const badgeW = 46;
    const badgeH = 17;
    const bx = Math.max(padL + 2, Math.min(W - padR - badgeW - 2, cursorX - badgeW / 2));
    const by = Math.max(3, cursorY - 25);

    ctx.fillStyle = '#00E5B8';
    ctx.beginPath();
    ctx.roundRect(bx, by, badgeW, badgeH, 4);
    ctx.fill();

    ctx.fillStyle = '#0D1916';
    ctx.font = '700 10px "Poppins", sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(badgeText, bx + badgeW / 2, by + 12);

    ctx.restore();
}
