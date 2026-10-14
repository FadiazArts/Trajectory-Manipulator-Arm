/**
 * FirmDev Studio — 3-Link Manipulator & Hydraulic Trajectory 3D
 * Module: 3D Robot Assembly, Telescopic Actuators, Gripper & 3D Markers
 */

import { DEG2RAD } from './config.js';
import { state } from './state.js';
import { robotToThreeVec } from './kinematics.js';
import { rebuildEnvelopeVisual } from './scene.js';

// Industrial PBR Materials
export const robotMats = {
    primaryCoat: new THREE.MeshStandardMaterial({ color: 0xff862e, metalness: 0.40, roughness: 0.28 }),
    castIron: new THREE.MeshStandardMaterial({ color: 0x222631, metalness: 0.75, roughness: 0.36 }),
    structuralSteel: new THREE.MeshStandardMaterial({ color: 0x2f374c, metalness: 0.72, roughness: 0.34 }),
    anodizedDark: new THREE.MeshStandardMaterial({ color: 0x141720, metalness: 0.85, roughness: 0.28 }),
    chromeSteel: new THREE.MeshStandardMaterial({ color: 0xe2e8f0, metalness: 0.94, roughness: 0.12 }),
    cylinderBody: new THREE.MeshStandardMaterial({ color: 0x1e2433, metalness: 0.80, roughness: 0.26 }),
    wiperSeal: new THREE.MeshStandardMaterial({ color: 0x43d1a7, metalness: 0.50, roughness: 0.30 }),
    tcpBeacon: new THREE.MeshBasicMaterial({ color: 0x00e5b8 }),
    payloadMat: new THREE.MeshStandardMaterial({ color: 0x455ddc, metalness: 0.38, roughness: 0.32 })
};

// Hierarchical Scene Graph
export const robotRoot = new THREE.Group();
export const joint1Turret = new THREE.Group();
export const joint2Shoulder = new THREE.Group();
export const joint3Elbow = new THREE.Group();
export const endEffectorGroup = new THREE.Group();

// Dynamic Hydraulic Cylinders & Gripper Finger Handles
export let hydraulicPairs = [];
export let leftFingerGroup = null;
export let rightFingerGroup = null;
export let jointAxesHelpers = [];

// Payload Cube & Velocity Arrow Helper
export const payloadMesh = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.26, 0.26), robotMats.payloadMat);
payloadMesh.castShadow = true;

export const velocityArrow = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 0, 0),
    0.001,
    0x00e5b8,
    0.22,
    0.12
);

// Pose Markers [A] & [B]
export const markersGroup = new THREE.Group();
export let poseAMarker = null;
export let poseBMarker = null;

// Trajectory 3D Path Lines & Sample Ticks
export const trajectoryGroup = new THREE.Group();
export let activeTrajectoryLine = null;
export let compareTrajectoryLine = null;
export const sampleTicksGroup = new THREE.Group();
trajectoryGroup.add(sampleTicksGroup);

export function createPoseMarker3D(colorHex) {
    const g = new THREE.Group();
    const core = new THREE.Mesh(
        new THREE.OctahedronGeometry(0.15, 0),
        new THREE.MeshStandardMaterial({ color: colorHex, roughness: 0.22, metalness: 0.45 })
    );
    g.add(core);

    const ring = new THREE.Mesh(
        new THREE.RingGeometry(0.22, 0.27, 32),
        new THREE.MeshBasicMaterial({ color: colorHex, side: THREE.DoubleSide })
    );
    ring.rotation.x = -Math.PI / 2;
    g.add(ring);

    const stem = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, -1, 0)]),
        new THREE.LineBasicMaterial({ color: colorHex, transparent: true, opacity: 0.42 })
    );
    stem.name = 'verticalDrop';
    g.add(stem);
    return g;
}

export function updatePoseMarkerPosition(marker, pos) {
    if (!marker || !pos) return;
    const v = robotToThreeVec(pos);
    marker.position.copy(v);
    const stem = marker.getObjectByName('verticalDrop');
    if (stem) stem.scale.y = v.y;
}

export function createJointAxisTriad(size = 0.65) {
    const g = new THREE.Group();
    g.add(new THREE.AxesHelper(size));
    g.visible = state.helpers.joints;
    jointAxesHelpers.push(g);
    return g;
}

export function createServoDrum(radius, length) {
    const g = new THREE.Group();
    const drum = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, length, 32), robotMats.castIron);
    drum.rotation.x = Math.PI / 2;
    g.add(drum);

    const ringGeo = new THREE.CylinderGeometry(radius * 1.03, radius * 1.03, length * 0.12, 32);
    const ringL = new THREE.Mesh(ringGeo, robotMats.primaryCoat);
    ringL.rotation.x = Math.PI / 2;
    ringL.position.z = length * 0.38;
    g.add(ringL);

    const ringR = ringL.clone();
    ringR.position.z = -length * 0.38;
    g.add(ringR);

    const cap = new THREE.Mesh(new THREE.CylinderGeometry(radius * 0.74, radius * 0.80, length + 0.06, 24), robotMats.structuralSteel);
    cap.rotation.x = Math.PI / 2;
    g.add(cap);

    return g;
}

/**
 * Creates a Telescopic Hydraulic Cylinder pair (Base Barrel + Wiper Gland + Chrome Piston Rod)
 * that scales and aims dynamically every frame so it NEVER floats or separates.
 */
export function createTelescopicCylinder(parentBase, basePos, parentRod, rodPos, barrelRadius = 0.085, rodRadius = 0.052) {
    // 1. Base Pivot Group (Attached to Clevis Pin on Link A)
    const basePivot = new THREE.Group();
    basePivot.position.copy(basePos);
    parentBase.add(basePivot);

    // Rear Clevis Eye Ring around Pin
    const rearEye = new THREE.Mesh(
        new THREE.CylinderGeometry(barrelRadius * 1.15, barrelRadius * 1.15, 0.09, 20),
        robotMats.castIron
    );
    rearEye.rotation.x = Math.PI / 2;
    basePivot.add(rearEye);

    // Rear End Cap (Fixed length at z = 0..0.08)
    const rearCap = new THREE.Mesh(
        new THREE.CylinderGeometry(barrelRadius * 1.08, barrelRadius * 1.08, 0.08, 20),
        robotMats.anodizedDark
    );
    rearCap.geometry.rotateX(Math.PI / 2);
    rearCap.position.z = 0.04;
    basePivot.add(rearCap);

    // Scalable Cylinder Barrel (Unit length 1.0 along +Z axis)
    const barrelGeo = new THREE.CylinderGeometry(barrelRadius, barrelRadius, 1.0, 24);
    barrelGeo.translate(0, 0.5, 0);
    barrelGeo.rotateX(Math.PI / 2);
    const barrelMesh = new THREE.Mesh(barrelGeo, robotMats.cylinderBody);
    basePivot.add(barrelMesh);

    // Wiper Seal Gland Collar at Tip of Barrel
    const glandCollar = new THREE.Mesh(
        new THREE.CylinderGeometry(barrelRadius * 1.12, barrelRadius * 1.08, 0.09, 24),
        robotMats.wiperSeal
    );
    glandCollar.geometry.rotateX(Math.PI / 2);
    basePivot.add(glandCollar);

    // 2. Rod Pivot Group (Attached to Clevis Pin on Link B)
    const rodPivot = new THREE.Group();
    rodPivot.position.copy(rodPos);
    parentRod.add(rodPivot);

    // Rod End Clevis Eye Ring around Pin
    const rodEye = new THREE.Mesh(
        new THREE.CylinderGeometry(rodRadius * 1.55, rodRadius * 1.55, 0.085, 20),
        robotMats.chromeSteel
    );
    rodEye.rotation.x = Math.PI / 2;
    rodPivot.add(rodEye);

    // Scalable Chrome Piston Rod (Unit length 1.0 along +Z axis)
    const rodGeo = new THREE.CylinderGeometry(rodRadius, rodRadius, 1.0, 20);
    rodGeo.translate(0, 0.5, 0);
    rodGeo.rotateX(Math.PI / 2);
    const rodMesh = new THREE.Mesh(rodGeo, robotMats.chromeSteel);
    rodPivot.add(rodMesh);

    return {
        basePivot,
        rodPivot,
        barrelMesh,
        glandCollar,
        rodMesh
    };
}

function buildGripperJaw() {
    const jaw = new THREE.Group();
    const base = new THREE.Mesh(new THREE.BoxGeometry(0.17, 0.09, 0.055), robotMats.chromeSteel);
    base.position.set(0.085, 0, 0);
    jaw.add(base);
    const pad = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.085, 0.04), robotMats.primaryCoat);
    pad.position.set(0.20, 0, 0);
    jaw.add(pad);
    return jaw;
}

/**
 * Builds the complete 3-DOF robot manipulator assembly into the Three.js scene.
 */
export function buildRobotAssembly(sceneRef) {
    if (sceneRef) {
        if (!robotRoot.parent) sceneRef.add(robotRoot);
        if (!payloadMesh.parent) sceneRef.add(payloadMesh);
        if (!velocityArrow.parent) sceneRef.add(velocityArrow);
        if (!markersGroup.parent) {
            sceneRef.add(markersGroup);
            poseAMarker = createPoseMarker3D(0x43d1a7);
            poseBMarker = createPoseMarker3D(0xf73463);
            markersGroup.add(poseAMarker);
            markersGroup.add(poseBMarker);
        }
        if (!trajectoryGroup.parent) sceneRef.add(trajectoryGroup);
    }

    hydraulicPairs = [];
    jointAxesHelpers = [];

    while (robotRoot.children.length > 0) robotRoot.remove(robotRoot.children[0]);
    while (joint1Turret.children.length > 0) joint1Turret.remove(joint1Turret.children[0]);
    while (joint2Shoulder.children.length > 0) joint2Shoulder.remove(joint2Shoulder.children[0]);
    while (joint3Elbow.children.length > 0) joint3Elbow.remove(joint3Elbow.children[0]);
    while (endEffectorGroup.children.length > 0) endEffectorGroup.remove(endEffectorGroup.children[0]);

    robotRoot.add(joint1Turret);
    joint1Turret.add(joint2Shoulder);
    joint2Shoulder.add(joint3Elbow);
    joint3Elbow.add(endEffectorGroup);

    const L1 = state.L1;
    const L2 = state.L2;
    const L3 = state.L3;

    // 1. HEAVY BOLTED PEDESTAL BASE
    const baseGroup = new THREE.Group();
    const flangeBottom = new THREE.Mesh(new THREE.CylinderGeometry(1.12, 1.22, 0.16, 36), robotMats.castIron);
    flangeBottom.position.y = 0.08;
    baseGroup.add(flangeBottom);

    const boltGeo = new THREE.CylinderGeometry(0.042, 0.042, 0.22, 10);
    for (let i = 0; i < 12; i++) {
        const a = (i / 12) * Math.PI * 2;
        const b = new THREE.Mesh(boltGeo, robotMats.chromeSteel);
        b.position.set(Math.cos(a) * 1.04, 0.11, Math.sin(a) * 1.04);
        baseGroup.add(b);
    }

    const slewRing = new THREE.Mesh(new THREE.CylinderGeometry(0.86, 0.96, 0.24, 36), robotMats.anodizedDark);
    slewRing.position.y = 0.28;
    baseGroup.add(slewRing);
    robotRoot.add(baseGroup);

    // 2. LINK 1: ROTATING TURRET COLUMN + SOLID WELDED TRUNNION CLEVIS BRACKET
    const turretBaseH = 0.40;
    const turretDisk = new THREE.Mesh(new THREE.CylinderGeometry(0.84, 0.88, 0.20, 32), robotMats.primaryCoat);
    turretDisk.position.y = turretBaseH + 0.10;
    joint1Turret.add(turretDisk);
    joint1Turret.add(createJointAxisTriad(0.75));

    const colH = Math.max(0.55, L1 - (turretBaseH + 0.20));
    const colCenterY = turretBaseH + 0.20 + colH / 2;

    const sidePlateGeo = new THREE.BoxGeometry(0.58, colH, 0.15);
    const leftPlate = new THREE.Mesh(sidePlateGeo, robotMats.primaryCoat);
    leftPlate.position.set(0, colCenterY, 0.33);
    joint1Turret.add(leftPlate);

    const rightPlate = new THREE.Mesh(sidePlateGeo, robotMats.primaryCoat);
    rightPlate.position.set(0, colCenterY, -0.33);
    joint1Turret.add(rightPlate);

    const spineBox = new THREE.Mesh(new THREE.BoxGeometry(0.46, colH * 0.92, 0.52), robotMats.castIron);
    spineBox.position.set(-0.04, colCenterY, 0);
    joint1Turret.add(spineBox);

    // Forward Trunnion & Clevis Nose Bracket
    const clevisY = Math.max(0.68, L1 - 0.54);
    const clevisX = 0.42;

    const turretClevisBracket = new THREE.Group();
    turretClevisBracket.position.set(0, clevisY, 0);
    joint1Turret.add(turretClevisBracket);

    const noseBridge = new THREE.Mesh(new THREE.BoxGeometry(0.52, 0.26, 0.62), robotMats.structuralSteel);
    noseBridge.position.set(0.20, 0, 0);
    turretClevisBracket.add(noseBridge);

    const gussetGeo = new THREE.BoxGeometry(0.44, 0.32, 0.09);
    [-0.21, 0.21].forEach(zOff => {
        const gusset = new THREE.Mesh(gussetGeo, robotMats.primaryCoat);
        gusset.position.set(0.22, -0.10, zOff);
        gusset.rotation.z = 0.35;
        turretClevisBracket.add(gusset);

        const ear = new THREE.Mesh(
            new THREE.CylinderGeometry(0.13, 0.13, 0.11, 24),
            robotMats.structuralSteel
        );
        ear.rotation.x = Math.PI / 2;
        ear.position.set(clevisX, 0, zOff);
        turretClevisBracket.add(ear);
    });

    const basePinShaft = new THREE.Mesh(
        new THREE.CylinderGeometry(0.048, 0.048, 0.64, 20),
        robotMats.chromeSteel
    );
    basePinShaft.rotation.x = Math.PI / 2;
    basePinShaft.position.set(clevisX, 0, 0);
    turretClevisBracket.add(basePinShaft);

    // 3. LINK 2: UPPER ARM (L2) + LOWER BELLY CLEVIS LUG
    joint2Shoulder.position.set(0, L1, 0);
    joint2Shoulder.add(createServoDrum(0.40, 0.90));
    joint2Shoulder.add(createJointAxisTriad(0.70));

    const beamGeo = new THREE.BoxGeometry(L2, 0.32, 0.13);
    const beamLeft = new THREE.Mesh(beamGeo, robotMats.primaryCoat);
    beamLeft.position.set(L2 / 2, 0, 0.25);
    joint2Shoulder.add(beamLeft);

    const beamRight = new THREE.Mesh(beamGeo, robotMats.primaryCoat);
    beamRight.position.set(L2 / 2, 0, -0.25);
    joint2Shoulder.add(beamRight);

    const webCore = new THREE.Mesh(new THREE.BoxGeometry(L2 * 0.86, 0.22, 0.38), robotMats.castIron);
    webCore.position.set(L2 / 2, 0, 0);
    joint2Shoulder.add(webCore);

    const armPinX = Math.min(0.86, L2 * 0.36);
    const armPinY = -0.22;

    const bellyBracket = new THREE.Group();
    bellyBracket.position.set(armPinX, 0, 0);
    joint2Shoulder.add(bellyBracket);

    const bellyWeb = new THREE.Mesh(new THREE.BoxGeometry(0.46, 0.30, 0.56), robotMats.structuralSteel);
    bellyWeb.position.set(0, -0.12, 0);
    bellyBracket.add(bellyWeb);

    [-0.21, 0.21].forEach(zOff => {
        const lugEar = new THREE.Mesh(
            new THREE.CylinderGeometry(0.12, 0.12, 0.10, 24),
            robotMats.primaryCoat
        );
        lugEar.rotation.x = Math.PI / 2;
        lugEar.position.set(0, armPinY, zOff);
        bellyBracket.add(lugEar);
    });

    const armPinShaft = new THREE.Mesh(
        new THREE.CylinderGeometry(0.046, 0.046, 0.62, 20),
        robotMats.chromeSteel
    );
    armPinShaft.rotation.x = Math.PI / 2;
    armPinShaft.position.set(0, armPinY, 0);
    bellyBracket.add(armPinShaft);

    // Twin Synchronized Shoulder Hydraulic Cylinders
    [-0.21, 0.21].forEach(zOff => {
        const cyl = createTelescopicCylinder(
            joint1Turret,
            new THREE.Vector3(clevisX, clevisY, zOff),
            joint2Shoulder,
            new THREE.Vector3(armPinX, armPinY, zOff),
            0.078,
            0.048
        );
        cyl.role = 'shoulder';
        hydraulicPairs.push(cyl);
    });

    // Upper Clevis Bracket on Link 2 for Elbow Actuator
    const elbowCylBaseX = Math.max(0.65, L2 - 0.78);
    const elbowCylBaseY = 0.24;

    const upperClevisGroup = new THREE.Group();
    upperClevisGroup.position.set(elbowCylBaseX, 0, 0);
    joint2Shoulder.add(upperClevisGroup);

    const upperSaddle = new THREE.Mesh(new THREE.BoxGeometry(0.38, 0.26, 0.42), robotMats.structuralSteel);
    upperSaddle.position.set(0, 0.12, 0);
    upperClevisGroup.add(upperSaddle);

    const upperEar = new THREE.Mesh(new THREE.CylinderGeometry(0.10, 0.10, 0.14, 20), robotMats.primaryCoat);
    upperEar.rotation.x = Math.PI / 2;
    upperEar.position.set(0, elbowCylBaseY, 0);
    upperClevisGroup.add(upperEar);

    const upperPin = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 0.42, 16), robotMats.chromeSteel);
    upperPin.rotation.x = Math.PI / 2;
    upperPin.position.set(0, elbowCylBaseY, 0);
    upperClevisGroup.add(upperPin);

    // 4. LINK 3: FOREARM (L3) + WELDED HORN CLEVIS
    joint3Elbow.position.set(L2, 0, 0);
    joint3Elbow.add(createServoDrum(0.31, 0.72));
    joint3Elbow.add(createJointAxisTriad(0.65));

    const wristLen = 0.42;
    const forearmBodyLen = Math.max(0.55, L3 - wristLen);

    const counterMass = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.26, 0.44, 24), robotMats.castIron);
    counterMass.rotation.z = Math.PI / 2;
    counterMass.position.set(-0.20, 0, 0);
    joint3Elbow.add(counterMass);

    const forearmTubeGeo = new THREE.CylinderGeometry(0.15, 0.24, forearmBodyLen, 24);
    forearmTubeGeo.rotateZ(-Math.PI / 2);
    const forearmTube = new THREE.Mesh(forearmTubeGeo, robotMats.primaryCoat);
    forearmTube.position.set(forearmBodyLen / 2, 0, 0);
    joint3Elbow.add(forearmTube);

    const forearmRib = new THREE.Mesh(new THREE.BoxGeometry(forearmBodyLen * 0.76, 0.12, 0.42), robotMats.castIron);
    forearmRib.position.set(forearmBodyLen * 0.44, 0, 0);
    joint3Elbow.add(forearmRib);

    const forearmPinX = Math.min(0.52, forearmBodyLen * 0.36);
    const forearmPinY = 0.22;

    const forearmHorn = new THREE.Group();
    forearmHorn.position.set(forearmPinX, 0, 0);
    joint3Elbow.add(forearmHorn);

    const hornWeb = new THREE.Mesh(new THREE.BoxGeometry(0.30, 0.24, 0.26), robotMats.structuralSteel);
    hornWeb.position.set(0, 0.12, 0);
    forearmHorn.add(hornWeb);

    const hornEar = new THREE.Mesh(new THREE.CylinderGeometry(0.095, 0.095, 0.12, 20), robotMats.primaryCoat);
    hornEar.rotation.x = Math.PI / 2;
    hornEar.position.set(0, forearmPinY, 0);
    forearmHorn.add(hornEar);

    const hornPin = new THREE.Mesh(new THREE.CylinderGeometry(0.038, 0.038, 0.30, 16), robotMats.chromeSteel);
    hornPin.rotation.x = Math.PI / 2;
    hornPin.position.set(0, forearmPinY, 0);
    forearmHorn.add(hornPin);

    // Elbow Telescopic Hydraulic Cylinder
    const elbowCyl = createTelescopicCylinder(
        joint2Shoulder,
        new THREE.Vector3(elbowCylBaseX, elbowCylBaseY, 0),
        joint3Elbow,
        new THREE.Vector3(forearmPinX, forearmPinY, 0),
        0.068,
        0.042
    );
    elbowCyl.role = 'elbow';
    hydraulicPairs.push(elbowCyl);

    // 5. END-EFFECTOR PARALLEL 2-JAW GRIPPER
    endEffectorGroup.position.set(forearmBodyLen, 0, 0);

    const wristCollar = new THREE.Mesh(new THREE.CylinderGeometry(0.16, 0.16, 0.12, 24), robotMats.chromeSteel);
    wristCollar.rotation.z = Math.PI / 2;
    wristCollar.position.set(0.06, 0, 0);
    endEffectorGroup.add(wristCollar);

    const gripperRail = new THREE.Mesh(new THREE.BoxGeometry(0.15, 0.22, 0.50), robotMats.anodizedDark);
    gripperRail.position.set(0.17, 0, 0);
    endEffectorGroup.add(gripperRail);

    leftFingerGroup = new THREE.Group();
    rightFingerGroup = new THREE.Group();

    leftFingerGroup.add(buildGripperJaw());
    rightFingerGroup.add(buildGripperJaw());
    endEffectorGroup.add(leftFingerGroup);
    endEffectorGroup.add(rightFingerGroup);

    const tcpMarker = new THREE.Mesh(new THREE.SphereGeometry(0.048, 16, 16), robotMats.tcpBeacon);
    tcpMarker.position.set(wristLen, 0, 0);
    endEffectorGroup.add(tcpMarker);

    updateGripperOpening(state.gripperOpen);

    robotRoot.traverse(obj => {
        if (obj.isMesh) {
            obj.castShadow = true;
            obj.receiveShadow = true;
        }
    });

    rebuildEnvelopeVisual();
}

export function updateGripperOpening(percent) {
    if (!leftFingerGroup || !rightFingerGroup) return;
    const zOffset = 0.04 + (percent / 100) * 0.16;
    leftFingerGroup.position.set(0.24, 0, zOffset);
    rightFingerGroup.position.set(0.24, 0, -zOffset);
}

/**
 * Updates all telescopic hydraulic cylinders dynamically:
 * Base barrel looks at rod clevis; piston rod looks at base clevis;
 * scales both components with exact overlap so they never float or detach.
 */
export function updateHydraulicActuators(anglesDeg) {
    joint1Turret.rotation.y = anglesDeg[0] * DEG2RAD;
    joint2Shoulder.rotation.z = anglesDeg[1] * DEG2RAD;
    joint3Elbow.rotation.z = anglesDeg[2] * DEG2RAD;

    robotRoot.updateMatrixWorld(true);

    const pBase = new THREE.Vector3();
    const pRod = new THREE.Vector3();

    hydraulicPairs.forEach(cyl => {
        cyl.basePivot.getWorldPosition(pBase);
        cyl.rodPivot.getWorldPosition(pRod);

        const dist = Math.max(0.25, pBase.distanceTo(pRod));

        cyl.basePivot.lookAt(pRod);
        cyl.rodPivot.lookAt(pBase);

        const barrelLen = dist * 0.56;
        const rodLen = dist * 0.62;

        cyl.barrelMesh.scale.set(1, 1, barrelLen);
        cyl.glandCollar.position.set(0, 0, barrelLen);
        cyl.rodMesh.scale.set(1, 1, rodLen);

        if (cyl.role === 'shoulder') {
            state.cyl1Length = dist;
            state.cyl1Pct = Math.round(Math.max(8, Math.min(98, ((dist - 0.52) / 0.92) * 100)));
        } else if (cyl.role === 'elbow') {
            state.cyl2Length = dist;
            state.cyl2Pct = Math.round(Math.max(8, Math.min(98, ((dist - 0.45) / 0.85) * 100)));
        }
    });
}

/**
 * Rebuilds the 3D trajectory line, comparative path, and milestone tick spheres.
 */
export function rebuildTrajectory3D() {
    if (activeTrajectoryLine) {
        trajectoryGroup.remove(activeTrajectoryLine);
        activeTrajectoryLine.geometry.dispose();
        activeTrajectoryLine = null;
    }
    if (compareTrajectoryLine) {
        trajectoryGroup.remove(compareTrajectoryLine);
        compareTrajectoryLine.geometry.dispose();
        compareTrajectoryLine = null;
    }
    while (sampleTicksGroup.children.length > 0) {
        sampleTicksGroup.remove(sampleTicksGroup.children[0]);
    }

    if (state.samples.length === 0) return;

    const mainPts = state.samples.map(s => robotToThreeVec(s.pos));
    activeTrajectoryLine = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(mainPts),
        new THREE.LineBasicMaterial({ color: 0x00e5b8, linewidth: 2 })
    );
    trajectoryGroup.add(activeTrajectoryLine);

    if (state.showComparePath && state.comparePositions.length > 1) {
        const compPts = state.comparePositions.map(p => robotToThreeVec(p));
        compareTrajectoryLine = new THREE.Line(
            new THREE.BufferGeometry().setFromPoints(compPts),
            new THREE.LineDashedMaterial({ color: 0x9646ef, dashSize: 0.16, gapSize: 0.12, transparent: true, opacity: 0.72 })
        );
        compareTrajectoryLine.computeLineDistances();
        trajectoryGroup.add(compareTrajectoryLine);
    }

    if (state.showSampleTicks) {
        const tickGeo = new THREE.SphereGeometry(0.038, 10, 10);
        const tickMat = new THREE.MeshBasicMaterial({ color: 0x4da6ff });
        const step = Math.max(2, Math.floor(state.samples.length / 20));
        for (let i = step; i < state.samples.length - 1; i += step) {
            const tick = new THREE.Mesh(tickGeo, tickMat);
            tick.position.copy(robotToThreeVec(state.samples[i].pos));
            sampleTicksGroup.add(tick);
        }
    }

    if (poseAMarker && poseBMarker) {
        updatePoseMarkerPosition(poseAMarker, state.samples[0].pos);
        updatePoseMarkerPosition(poseBMarker, state.samples[state.samples.length - 1].pos);
    }
}
