import GUI from 'lil-gui';
import { type Vec3, vec3 } from 'mathcat';
import {
    addOffMeshConnection,
    findPath,
    getNodeRefType,
    NodeType,
    OffMeshConnectionDirection,
    type QueryFilter,
} from 'navcat';
import { generateSoloNavMesh, type SoloNavMeshInput, type SoloNavMeshOptions } from 'navcat/blocks';
import {
    createNavMeshHelper,
    createNavMeshOffMeshConnectionsHelper,
    createNavMeshPolyHelper,
    type DebugObject,
    getPositionsAndIndices,
} from 'navcat/three';
import * as THREE from 'three';
import { LineGeometry, OrbitControls } from 'three/examples/jsm/Addons.js';
import { Line2 } from 'three/examples/jsm/lines/webgpu/Line2.js';
import { Line2NodeMaterial } from 'three/webgpu';
import { createExample } from './common/example-base';
import { createFlag } from './common/flag';
import { type JumpLink, type JumpLinkBuilderConfig, type JumpLinkType, buildJumpLinks, extractBorderEdges } from './common/jump-link-builder';
import { loadGLTF } from './common/load-gltf';

/* setup example scene */
const container = document.getElementById('root')!;
const { scene, camera, renderer } = await createExample(container);

camera.position.set(-2, 10, 10);

const orbitControls = new OrbitControls(camera, renderer.domElement);
orbitControls.enableDamping = true;

const navTestModel = await loadGLTF('./models/nav-test.glb');
scene.add(navTestModel.scene);

const walkableMeshes: THREE.Mesh[] = [];
scene.traverse((object) => {
    if (object instanceof THREE.Mesh) {
        walkableMeshes.push(object);
    }
});

const [positions, indices] = getPositionsAndIndices(walkableMeshes);

/* navmesh + jump-link config */
const navConfig = {
    cellSize: 0.15,
    cellHeight: 0.15,
    walkableRadiusWorld: 0.1,
    walkableClimbWorld: 0.5,
    walkableHeightWorld: 0.25,
    walkableSlopeAngleDegrees: 45,
    minRegionArea: 8,
    mergeRegionArea: 20,
    maxSimplificationError: 1.3,
    maxEdgeLength: 12,
    maxVerticesPerPoly: 5,
    detailSampleDistanceVoxels: 6,
    detailSampleMaxErrorVoxels: 1,
};

const jumpConfig: JumpLinkBuilderConfig = {
    cellSize: navConfig.cellSize,
    cellHeight: navConfig.cellHeight,
    agentRadius: navConfig.walkableRadiusWorld,
    agentHeight: navConfig.walkableHeightWorld,
    agentClimb: navConfig.walkableClimbWorld,
    groundTolerance: 0.05,
    startDistance: 0.3,
    endDistance: 2.0,
    minHeight: -0.5,
    maxHeight: 2.0,
    jumpHeight: 1.5,
};

const visualConfig = {
    jumpType: 'jump' as JumpLinkType | 'both',
    showBorderEdges: true,
    showJumpSpines: true,
    showOffMeshConnections: true,
    showNavMesh: true,
};

/* visuals */
type Visual = { object: THREE.Object3D; dispose: () => void };

const transientVisuals: Visual[] = [];
const pathVisuals: Visual[] = [];

const addTransient = (visual: Visual) => {
    transientVisuals.push(visual);
    scene.add(visual.object);
};
const clearTransient = () => {
    while (transientVisuals.length > 0) {
        const v = transientVisuals.pop()!;
        scene.remove(v.object);
        v.dispose();
    }
};
const addPathVisual = (visual: Visual) => {
    pathVisuals.push(visual);
    scene.add(visual.object);
};
const clearPathVisuals = () => {
    while (pathVisuals.length > 0) {
        const v = pathVisuals.pop()!;
        scene.remove(v.object);
        v.dispose();
    }
};

const wrapDebug = (helper: DebugObject): Visual => ({ object: helper.object, dispose: helper.dispose });

const lineFromPoints = (points: THREE.Vector3[], color: THREE.ColorRepresentation, linewidth: number): Visual => {
    const geometry = new LineGeometry();
    geometry.setFromPoints(points);
    const material = new Line2NodeMaterial({ color, linewidth, worldUnits: true });
    const line = new Line2(geometry, material);
    return {
        object: line,
        dispose: () => {
            geometry.dispose();
            material.dispose();
        },
    };
};

/* generate navmesh */
const navMeshInput: SoloNavMeshInput = { positions, indices };

let navMesh: ReturnType<typeof generateSoloNavMesh>['navMesh'];
let intermediates: ReturnType<typeof generateSoloNavMesh>['intermediates'];

const buildNavMesh = () => {
    const walkableRadiusVoxels = Math.ceil(navConfig.walkableRadiusWorld / navConfig.cellSize);
    const walkableClimbVoxels = Math.ceil(navConfig.walkableClimbWorld / navConfig.cellHeight);
    const walkableHeightVoxels = Math.ceil(navConfig.walkableHeightWorld / navConfig.cellHeight);
    const detailSampleDistance = navConfig.detailSampleDistanceVoxels < 0.9 ? 0 : navConfig.cellSize * navConfig.detailSampleDistanceVoxels;
    const detailSampleMaxError = navConfig.cellHeight * navConfig.detailSampleMaxErrorVoxels;

    const options: SoloNavMeshOptions = {
        cellSize: navConfig.cellSize,
        cellHeight: navConfig.cellHeight,
        walkableRadiusWorld: navConfig.walkableRadiusWorld,
        walkableRadiusVoxels,
        walkableClimbWorld: navConfig.walkableClimbWorld,
        walkableClimbVoxels,
        walkableHeightWorld: navConfig.walkableHeightWorld,
        walkableHeightVoxels,
        walkableSlopeAngleDegrees: navConfig.walkableSlopeAngleDegrees,
        borderSize: 0,
        minRegionArea: navConfig.minRegionArea,
        mergeRegionArea: navConfig.mergeRegionArea,
        maxSimplificationError: navConfig.maxSimplificationError,
        maxEdgeLength: navConfig.maxEdgeLength,
        maxVerticesPerPoly: navConfig.maxVerticesPerPoly,
        detailSampleDistance,
        detailSampleMaxError,
    };

    const result = generateSoloNavMesh(navMeshInput, options);
    navMesh = result.navMesh;
    intermediates = result.intermediates;
};

/* run jump-link builder + render */
let jumpLinks: JumpLink[] = [];
let connectionIds: number[] = [];

const regenerate = () => {
    clearPathVisuals();
    clearTransient();
    connectionIds = [];

    buildNavMesh();

    /* run jump-link builder */
    console.time('buildJumpLinks');
    jumpLinks = [];
    if (visualConfig.jumpType === 'jump' || visualConfig.jumpType === 'both') {
        jumpLinks.push(...buildJumpLinks(navMesh, intermediates.heightfield, 'jump', jumpConfig));
    }
    if (visualConfig.jumpType === 'climb-down' || visualConfig.jumpType === 'both') {
        jumpLinks.push(...buildJumpLinks(navMesh, intermediates.heightfield, 'climb-down', jumpConfig));
    }
    console.timeEnd('buildJumpLinks');
    console.log(`generated ${jumpLinks.length} jump links`);

    /* register each link as an off-mesh connection (midpoint of the jump's lateral spread) */
    const startMid = vec3.create();
    const endMid = vec3.create();
    for (const link of jumpLinks) {
        const a = link.startSamples[0].p;
        const b = link.startSamples[link.startSamples.length - 1].p;
        const c = link.endSamples[0].p;
        const d = link.endSamples[link.endSamples.length - 1].p;
        startMid[0] = (a[0] + b[0]) * 0.5;
        startMid[1] = (a[1] + b[1]) * 0.5;
        startMid[2] = (a[2] + b[2]) * 0.5;
        endMid[0] = (c[0] + d[0]) * 0.5;
        endMid[1] = (c[1] + d[1]) * 0.5;
        endMid[2] = (c[2] + d[2]) * 0.5;
        const id = addOffMeshConnection(navMesh, {
            start: [startMid[0], startMid[1], startMid[2]],
            end: [endMid[0], endMid[1], endMid[2]],
            radius: jumpConfig.agentRadius,
            direction: OffMeshConnectionDirection.START_TO_END,
            area: 0,
            flags: 0xffffff,
        });
        connectionIds.push(id);
    }

    /* visuals */
    if (visualConfig.showNavMesh) {
        const navMeshHelper = createNavMeshHelper(navMesh);
        navMeshHelper.object.position.y += 0.1;
        addTransient(wrapDebug(navMeshHelper));
    }

    if (visualConfig.showBorderEdges) {
        const edges = extractBorderEdges(navMesh);
        for (const e of edges) {
            addTransient(
                lineFromPoints(
                    [new THREE.Vector3(e.sp[0], e.sp[1] + 0.05, e.sp[2]), new THREE.Vector3(e.sq[0], e.sq[1] + 0.05, e.sq[2])],
                    0xff3344,
                    0.05,
                ),
            );
        }
    }

    if (visualConfig.showJumpSpines) {
        for (const link of jumpLinks) {
            const color = link.type === 'jump' ? 0xffd60a : 0x4cc9f0;
            const left: THREE.Vector3[] = [];
            const right: THREE.Vector3[] = [];
            for (let i = 0; i < link.nspine; i++) {
                left.push(new THREE.Vector3(link.spine0[i * 3], link.spine0[i * 3 + 1], link.spine0[i * 3 + 2]));
                right.push(new THREE.Vector3(link.spine1[i * 3], link.spine1[i * 3 + 1], link.spine1[i * 3 + 2]));
            }
            addTransient(lineFromPoints(left, color, 0.05));
            addTransient(lineFromPoints(right, color, 0.05));
        }
    }

    if (visualConfig.showOffMeshConnections) {
        addTransient(wrapDebug(createNavMeshOffMeshConnectionsHelper(navMesh)));
    }

    updateStats();
    updatePath();
};

/* pathfinding interaction */
const queryFilter: QueryFilter = {
    passFilter: () => true,
    getCost: (pa, pb) => vec3.distance(pa, pb),
};

let start: Vec3 = [-2.2, 0.26, 4.71];
let end: Vec3 = [3.4, 2.8, 3.6];
const halfExtents: Vec3 = [1, 1, 1];

const updatePath = () => {
    clearPathVisuals();

    const startFlag = createFlag(0x2196f3);
    startFlag.object.position.set(...start);
    addPathVisual(startFlag);
    const endFlag = createFlag(0x00ff00);
    endFlag.object.position.set(...end);
    addPathVisual(endFlag);

    const pathResult = findPath(navMesh, start, end, halfExtents, queryFilter);
    if (!pathResult.success) return;

    const { path, nodePath } = pathResult;
    if (nodePath) {
        for (const node of nodePath.path) {
            if (getNodeRefType(node) === NodeType.POLY) {
                const polyHelper = createNavMeshPolyHelper(navMesh, node);
                polyHelper.object.position.y += 0.15;
                addPathVisual({
                    object: polyHelper.object,
                    dispose: () => polyHelper.dispose(),
                });
            }
        }
    }

    if (path) {
        for (let i = 0; i < path.length; i++) {
            const point = path[i];
            const dotMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
            const dotGeometry = new THREE.SphereGeometry(0.15);
            const dot = new THREE.Mesh(dotGeometry, dotMaterial);
            dot.position.set(...point.position);
            addPathVisual({
                object: dot,
                dispose: () => {
                    dotGeometry.dispose();
                    dotMaterial.dispose();
                },
            });
            if (i > 0) {
                const prev = path[i - 1].position;
                addPathVisual(
                    lineFromPoints(
                        [new THREE.Vector3(...prev), new THREE.Vector3(...point.position)],
                        'yellow',
                        0.1,
                    ),
                );
            }
        }
    }
};

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
const getPointOnNavMesh = (event: PointerEvent): Vec3 | null => {
    const rect = renderer.domElement.getBoundingClientRect();
    pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);
    const hits = raycaster.intersectObjects(walkableMeshes, true);
    if (hits.length > 0) {
        const p = hits[0].point;
        return [p.x, p.y, p.z];
    }
    return null;
};

let moving: 'start' | 'end' | null = null;
renderer.domElement.addEventListener('pointerdown', (event) => {
    event.preventDefault();
    const point = getPointOnNavMesh(event);
    if (!point) return;
    if (event.button === 0) {
        moving = moving === 'start' ? null : 'start';
        renderer.domElement.style.cursor = moving ? 'crosshair' : '';
        start = point;
    } else if (event.button === 2) {
        moving = moving === 'end' ? null : 'end';
        renderer.domElement.style.cursor = moving ? 'crosshair' : '';
        end = point;
    }
    updatePath();
});
renderer.domElement.addEventListener('pointermove', (event) => {
    if (!moving) return;
    const point = getPointOnNavMesh(event);
    if (!point) return;
    if (moving === 'start') start = point;
    else end = point;
    updatePath();
});
renderer.domElement.addEventListener('contextmenu', (e) => e.preventDefault());

/* stats */
const statsDiv = document.createElement('div');
statsDiv.style.position = 'absolute';
statsDiv.style.top = '10px';
statsDiv.style.left = '10px';
statsDiv.style.color = 'white';
statsDiv.style.fontFamily = 'monospace';
statsDiv.style.fontSize = '12px';
statsDiv.style.background = 'rgba(0,0,0,0.7)';
statsDiv.style.padding = '10px';
statsDiv.style.borderRadius = '4px';
container.appendChild(statsDiv);

const updateStats = () => {
    const jumpCount = jumpLinks.filter((l) => l.type === 'jump').length;
    const climbCount = jumpLinks.filter((l) => l.type === 'climb-down').length;
    statsDiv.innerHTML = `<div style="font-weight:bold;color:#00aaff;margin-bottom:6px;">Jump Link Stats</div>` +
        `<div>edges scanned: ${extractBorderEdges(navMesh).length}</div>` +
        `<div>jumps: <span style="color:#ffd60a;">${jumpCount}</span></div>` +
        `<div>climb-downs: <span style="color:#4cc9f0;">${climbCount}</span></div>` +
        `<div>total: ${jumpLinks.length}</div>`;
};

/* gui */
const gui = new GUI();
gui.title('Auto Off-Mesh Connections');

const typeFolder = gui.addFolder('Builder');
typeFolder.add(visualConfig, 'jumpType', ['jump', 'climb-down', 'both']).onChange(regenerate);
typeFolder.add(jumpConfig, 'startDistance', 0, 2, 0.05).onChange(regenerate);
typeFolder.add(jumpConfig, 'endDistance', 0.5, 5, 0.1).onChange(regenerate);
typeFolder.add(jumpConfig, 'jumpHeight', 0.1, 5, 0.1).onChange(regenerate);
typeFolder.add(jumpConfig, 'minHeight', -3, 0, 0.1).onChange(regenerate);
typeFolder.add(jumpConfig, 'maxHeight', 0, 5, 0.1).onChange(regenerate);
typeFolder.add(jumpConfig, 'agentClimb', 0.1, 2, 0.05).onChange(regenerate);
typeFolder.add(jumpConfig, 'agentRadius', 0.05, 1, 0.01).onChange(regenerate);
typeFolder.add(jumpConfig, 'groundTolerance', 0, 0.5, 0.01).onChange(regenerate);

const debugFolder = gui.addFolder('Debug');
debugFolder.add(visualConfig, 'showNavMesh').onChange(regenerate);
debugFolder.add(visualConfig, 'showBorderEdges').onChange(regenerate);
debugFolder.add(visualConfig, 'showJumpSpines').onChange(regenerate);
debugFolder.add(visualConfig, 'showOffMeshConnections').onChange(regenerate);

gui.add({ regenerate }, 'regenerate').name('Rebuild');

regenerate();

/* loop */
const update = () => {
    requestAnimationFrame(update);
    orbitControls.update();
    renderer.render(scene, camera);
};
update();
