/**
 * FirmDev Studio — 3-Link Manipulator & Hydraulic Trajectory 3D
 * Module: Three.js 3D Scene, Camera, Lighting & Viewport Environment
 */

import { state } from './state.js';

export let scene;
export let camera;
export let renderer;
export let controls;
export let envelopeGroup;
export let worldAxes;

/**
 * Initializes the Three.js 3D scene, cameras, renderers, lights, and floor environment.
 */
export function initScene(container) {
    if (!container) {
        container = document.getElementById('canvasContainer');
    }

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x131722);
    scene.fog = new THREE.FogExp2(0x131722, 0.016);

    camera = new THREE.PerspectiveCamera(42, 1, 0.1, 120);
    camera.position.set(8.0, 5.8, 8.2);

    const isMobile = typeof window !== 'undefined' && window.innerWidth <= 768;

    renderer = new THREE.WebGLRenderer({
        antialias: !isMobile,
        powerPreference: isMobile ? 'default' : 'high-performance'
    });
    renderer.setPixelRatio(isMobile ? 1.0 : Math.min(window.devicePixelRatio || 1, 2));

    if (isMobile) {
        renderer.shadowMap.enabled = false;
    } else {
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    }
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.15;
    container.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.07;
    controls.target.set(0, 2.3, 0);
    controls.maxPolarAngle = Math.PI / 2 - 0.01;
    controls.minDistance = 2.0;
    controls.maxDistance = 26;

    // Ambient & Studio Lighting
    scene.add(new THREE.AmbientLight(0xdbeafe, isMobile ? 0.85 : 0.68));

    const hemiLight = new THREE.HemisphereLight(0xe2e8f0, 0x1e293b, 0.65);
    hemiLight.position.set(0, 18, 0);
    scene.add(hemiLight);

    const keyLight = new THREE.DirectionalLight(0xffffff, isMobile ? 1.25 : 1.38);
    keyLight.position.set(9, 16, 9);
    if (!isMobile) {
        keyLight.castShadow = true;
        keyLight.shadow.mapSize.width = 2048;
        keyLight.shadow.mapSize.height = 2048;
        keyLight.shadow.camera.near = 1;
        keyLight.shadow.camera.far = 38;
        const sb = 9.5;
        keyLight.shadow.camera.left = -sb;
        keyLight.shadow.camera.right = sb;
        keyLight.shadow.camera.top = sb;
        keyLight.shadow.camera.bottom = -sb;
        keyLight.shadow.bias = -0.0005;
    } else {
        keyLight.castShadow = false;
    }
    scene.add(keyLight);

    const rimLight = new THREE.DirectionalLight(0x9646ef, 0.48);
    rimLight.position.set(-9, 8, -8);
    scene.add(rimLight);

    // Studio Floor & Grid
    const floorMesh = new THREE.Mesh(
        new THREE.PlaneGeometry(46, 46),
        new THREE.MeshStandardMaterial({ color: 0x10131c, roughness: 0.88, metalness: 0.14 })
    );
    floorMesh.rotation.x = -Math.PI / 2;
    floorMesh.receiveShadow = true;
    scene.add(floorMesh);

    const floorGrid = new THREE.GridHelper(20, 20, 0x2f374c, 0x1c2130);
    floorGrid.position.y = 0.005;
    scene.add(floorGrid);

    // World Reference Triad (RGB axes)
    worldAxes = new THREE.Group();
    worldAxes.position.set(0, 0.02, 0);
    worldAxes.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 0, 0), 1.6, 0xf73463, 0.18, 0.09));
    worldAxes.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(0, 0, 0), 1.6, 0x43d1a7, 0.18, 0.09));
    worldAxes.add(new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), 1.6, 0x4da6ff, 0.18, 0.09));
    scene.add(worldAxes);

    // Workspace Envelope Ring & Arc Group
    envelopeGroup = new THREE.Group();
    scene.add(envelopeGroup);
    rebuildEnvelopeVisual();

    return { scene, camera, renderer, controls, envelopeGroup };
}

/**
 * Reconstructs the reachable workspace boundary ring & boundary arc.
 */
export function rebuildEnvelopeVisual() {
    if (!envelopeGroup) return;
    while (envelopeGroup.children.length > 0) envelopeGroup.remove(envelopeGroup.children[0]);
    const isMobile = typeof window !== 'undefined' && window.innerWidth <= 768;
    const maxR = state.L2 + state.L3;
    const ringMesh = new THREE.Mesh(
        new THREE.RingGeometry(maxR - 0.04, maxR, isMobile ? 48 : 96),
        new THREE.MeshBasicMaterial({ color: 0x43d1a7, side: THREE.DoubleSide, transparent: true, opacity: 0.42 })
    );
    ringMesh.rotation.x = -Math.PI / 2;
    ringMesh.position.y = 0.01;
    envelopeGroup.add(ringMesh);

    const arcSteps = isMobile ? 32 : 64;
    const arcPts = [];
    for (let i = 0; i <= arcSteps; i++) {
        const ang = (i / arcSteps) * Math.PI;
        arcPts.push(new THREE.Vector3(Math.cos(ang) * maxR, state.L1 + Math.sin(ang) * maxR, 0));
    }
    const arcLine = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(arcPts),
        new THREE.LineDashedMaterial({ color: 0x455ddc, dashSize: 0.2, gapSize: 0.14, transparent: true, opacity: 0.48 })
    );
    arcLine.computeLineDistances();
    envelopeGroup.add(arcLine);
}

/**
 * Clears active button styling across camera buttons.
 */
export function clearCameraActiveButtons() {
    ['camPerspBtn', 'camHydraulicBtn', 'camTopBtn', 'camSideBtn', 'camFrontBtn'].forEach(id => {
        const el = document.getElementById(id);
        if (el) el.classList.remove('is-active');
    });
    const railPersp = document.getElementById('railBtnPersp');
    if (railPersp) railPersp.classList.remove('is-active');
    const railHyd = document.getElementById('railBtnHydraulics');
    if (railHyd) railHyd.classList.remove('is-active');
}

/**
 * Sets camera position and orientation according to view preset.
 */
export function setCameraView(preset) {
    if (!camera || !controls) return;
    clearCameraActiveButtons();
    if (preset === 'iso') {
        camera.position.set(8.0, 5.8, 8.2);
        controls.target.set(0, 2.3, 0);
        const camBtn = document.getElementById('camPerspBtn');
        if (camBtn) camBtn.classList.add('is-active');
        const railBtn = document.getElementById('railBtnPersp');
        if (railBtn) railBtn.classList.add('is-active');
    } else if (preset === 'top') {
        camera.position.set(0.01, 15.0, 0);
        controls.target.set(0, 0, 0);
        const camBtn = document.getElementById('camTopBtn');
        if (camBtn) camBtn.classList.add('is-active');
    } else if (preset === 'side') {
        camera.position.set(0, 3.0, 11.5);
        controls.target.set(0, 2.4, 0);
        const camBtn = document.getElementById('camSideBtn');
        if (camBtn) camBtn.classList.add('is-active');
    } else if (preset === 'front') {
        camera.position.set(11.5, 3.0, 0);
        controls.target.set(0, 2.4, 0);
        const camBtn = document.getElementById('camFrontBtn');
        if (camBtn) camBtn.classList.add('is-active');
    }
    controls.update();
}

/**
 * Zooms camera directly into shoulder trunnion and hydraulic clevis bracket.
 */
export function focusOnHydraulics(shoulderPos) {
    if (!camera || !controls) return;
    clearCameraActiveButtons();
    const hydBtn = document.getElementById('camHydraulicBtn');
    if (hydBtn) hydBtn.classList.add('is-active');
    const railHyd = document.getElementById('railBtnHydraulics');
    if (railHyd) railHyd.classList.add('is-active');

    camera.position.set(shoulderPos.x + 2.6, shoulderPos.y + 1.1, shoulderPos.z + 2.8);
    controls.target.set(shoulderPos.x + 0.5, shoulderPos.y - 0.1, shoulderPos.z);
    controls.update();
}
