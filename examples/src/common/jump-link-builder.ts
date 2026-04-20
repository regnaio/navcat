import { type Vec3, vec3 } from 'mathcat';
import {
    ANY_QUERY_FILTER,
    createGetPolyHeightResult,
    getNodeByRef,
    getPolyHeight,
    type Heightfield,
    type NavMesh,
    type NodeRef,
    queryPolygons,
} from 'navcat';

export type JumpLinkType = 'jump' | 'climb-down';

export type JumpLinkBuilderConfig = {
    /**
     * Sample density along edges and trajectories — match the navmesh.
     *
     * xz-plane voxel size of the navmesh build, in world units.
     * Controls how densely we sample along each edge and along each trajectory.
     * Should match the `cellSize` used to generate the navmesh; if it's smaller
     * we waste work, if larger we may miss thin gaps.
     */
    cellSize: number;

    /**
     * Sample density along trajectories vertically — match the navmesh.
     *
     * y-axis voxel size of the navmesh build, in world units.
     * Used to convert solid-heightfield span indices back to world y coordinates,
     * and as a step size (along with cellSize) when walking a trajectory for
     * collision checks. Should match the `cellHeight` used to generate the navmesh.
     */
    cellHeight: number;

    /**
     * Landing must be at least 2·r wide; fan starts 2·r from edge.
     *
     * Agent radius in world units. Drives two things:
     *   1. The landing fan starts at 2·agentRadius out from the edge — i.e. the
     *      nearest sampled landing position leaves enough lateral room for the
     *      agent's body to clear the wall it just jumped off of.
     *   2. A discovered jump segment is rejected if its lateral width is less
     *      than 2·agentRadius, since the agent wouldn't physically fit on it.
     */
    agentRadius: number;

    /**
     * Trajectory body-cylinder for ceiling / wall checks.
     *
     * Agent height in world units. Used during trajectory collision checks: at
     * every step along a candidate arc we sweep a vertical box from
     * `p.y + groundTolerance` to `p.y + agentHeight` and reject the trajectory
     * if any solid-heightfield span overlaps it. So this is "is there a ceiling
     * or overhang in the way?".
     */
    agentHeight: number;

    /**
     * How far below the edge the takeoff probes; max Δy between two landing
     * samples joined into the same link.
     *
     * Maximum step-up the agent can climb without jumping, in world units. Used
     * for two things:
     *   1. The takeoff sample is placed `agentClimb` BELOW the edge's y, so we
     *      probe ground that's still on-foot reachable from the edge polygon.
     *   2. Flood-fill grouping of valid landing samples joins two neighbours
     *      only if |Δy| < agentClimb — this keeps each emitted jump link
     *      pointing at one continuous ledge rather than spanning a step.
     */
    agentClimb: number;

    /**
     * y-bias so the trajectory's own ground doesn't self-collide.
     *
     * Vertical clearance above the trajectory, in world units. The trajectory's
     * own ground would otherwise count as a collision (since the path runs along
     * it); this lifts the test box up by groundTolerance so we only catch real
     * obstacles (overhangs, walls). Typical values are a few cm.
     */
    groundTolerance: number;

    /**
     * How far back from the edge the takeoff sits.
     *
     * Perpendicular offset of the takeoff sample from the edge, in world units.
     * The "start" segment is `edge + az·startDistance + ay·(-agentClimb)` — i.e.
     * we sample ground `startDistance` units perpendicular to the edge (into the
     * source polygon) and slightly below it. Small values (~0.3) keep the sample
     * adjacent to the edge.
     */
    startDistance: number;

    /**
     * Furthest landing distance — "max jump length".
     *
     * Distance from the edge to the outermost landing sample, in world units.
     * The landing fan is sampled along `az` from `2·agentRadius` out to
     * `endDistance`. Larger values let the algorithm discover longer jumps, at
     * the cost of more heightfield work per edge.
     */
    endDistance: number;

    /**
     * Lower bound (relative to edge.y) of the landing-search y-window — negative
     * to look below the edge.
     *
     * Lower bound (relative to the edge's y) of the y-window in which we look
     * for landing surfaces, in world units. Typically negative — e.g. -0.5
     * means "consider landing surfaces up to 0.5 units below the edge". Together
     * with maxHeight this defines the vertical search range of the ground probe
     * at each landing-fan sample.
     */
    minHeight: number;

    /**
     * Upper bound (relative to edge.y) of the landing-search y-window —
     * positive to also discover jump-UP links.
     *
     * Upper bound (relative to the edge's y) of the y-window in which we look
     * for landing surfaces, in world units. Typically positive — e.g. 2.0 means
     * "also consider landing surfaces up to 2 units above the edge", which lets
     * the algorithm discover jump-UP links as well as jump-down/across.
     */
    maxHeight: number;

    /**
     * Parabolic arc peak — "how high can the agent jump".
     *
     * Peak height of the parabolic jump arc above the higher of the takeoff and
     * landing points, in world units. Larger values let trajectories clear
     * taller obstacles between takeoff and landing, at the cost of accepting
     * jumps that aren't physically realistic for the agent. Unused for
     * `'climb-down'` trajectories.
     */
    jumpHeight: number;
};

export type Edge = { sp: Vec3; sq: Vec3 };

export type GroundSample = {
    p: Vec3;
    validHeight: boolean;
    validTrajectory: boolean;
};

export type GroundSegment = {
    p: Vec3;
    q: Vec3;
    height: number;
    gsamples: GroundSample[];
};

export type Trajectory = (out: Vec3, start: Vec3, end: Vec3, u: number) => Vec3;

export type JumpLink = {
    type: JumpLinkType;
    start: GroundSegment;
    end: GroundSegment;
    startSamples: GroundSample[];
    endSamples: GroundSample[];
    trajectory: Trajectory;
    nspine: number;
    spine0: Float32Array;
    spine1: Float32Array;
};

const SPINE_POINTS = 8;

/* ---------- trajectories ---------- */

const lerp = (a: number, b: number, u: number) => (1 - u) * a + u * b;

export const jumpTrajectory = (jumpHeight: number): Trajectory => {
    return (out, start, end, u) => {
        const ys = start[1];
        const ye = end[1];
        let y: number;
        if (u <= 0) {
            y = ys;
        } else if (u >= 1) {
            y = ye;
        } else {
            // two-parabola arc whose peak heights are jumpHeight above the higher of (start, end)
            let h1: number;
            let h2: number;
            if (ys >= ye) {
                h1 = jumpHeight;
                h2 = jumpHeight + ys - ye;
            } else {
                h1 = jumpHeight + ys - ye;
                h2 = jumpHeight;
            }
            const t = Math.sqrt(h1) / (Math.sqrt(h1) + Math.sqrt(h2));
            if (u <= t) {
                const v = 1 - u / t;
                y = ys + h1 - h1 * v * v;
            } else {
                const v = (u - t) / (1 - t);
                y = ys + h1 - h2 * v * v;
            }
        }
        out[0] = lerp(start[0], end[0], u);
        out[1] = y;
        out[2] = lerp(start[2], end[2], u);
        return out;
    };
};

export const climbDownTrajectory: Trajectory = (out, start, end, u) => {
    // walk horizontally for the first half, then drop vertically for the second half
    const horiz = Math.min(2 * u, 1);
    const vert = Math.max(0, 2 * u - 1);
    out[0] = lerp(start[0], end[0], horiz);
    out[1] = lerp(start[1], end[1], vert);
    out[2] = lerp(start[2], end[2], horiz);
    return out;
};

/* ---------- edge extraction ---------- */

export const extractBorderEdges = (navMesh: NavMesh): Edge[] => {
    const edges: Edge[] = [];
    for (const tileId of Object.keys(navMesh.tiles)) {
        const tile = navMesh.tiles[tileId];
        const verts = tile.vertices;
        for (const poly of tile.polys) {
            const nv = poly.vertices.length;
            for (let j = 0; j < nv; j++) {
                // border edges have neis[j] === 0 after finalizePolyNeighbours.
                // non-zero with POLY_NEIS_FLAG_EXT_LINK = portal (tile boundary, not a cliff).
                if (poly.neis[j] !== 0) continue;

                const nj = (j + 1) % nv;
                const va = poly.vertices[j] * 3;
                const vb = poly.vertices[nj] * 3;
                // recast4j reverses sp/sq so the perpendicular basis points outward
                edges.push({
                    sp: [verts[vb], verts[vb + 1], verts[vb + 2]],
                    sq: [verts[va], verts[va + 1], verts[va + 2]],
                });
            }
        }
    }
    return edges;
};

/* ---------- sampler construction ---------- */

type EdgeSampler = {
    edge: Edge;
    ax: Vec3;
    ay: Vec3;
    az: Vec3;
    start: GroundSegment;
    end: GroundSegment[];
    trajectory: Trajectory;
    type: JumpLinkType;
};

const trans2d = (out: Vec3, ax: Vec3, ay: Vec3, x: number, y: number): Vec3 => {
    out[0] = ax[0] * x + ay[0] * y;
    out[1] = ax[1] * x + ay[1] * y;
    out[2] = ax[2] * x + ay[2] * y;
    return out;
};

const addOffset = (out: Vec3, base: Vec3, offset: Vec3): Vec3 => {
    out[0] = base[0] + offset[0];
    out[1] = base[1] + offset[1];
    out[2] = base[2] + offset[2];
    return out;
};

const createGroundSegment = (): GroundSegment => ({
    p: vec3.create(),
    q: vec3.create(),
    height: 0,
    gsamples: [],
});

const buildJumpSampler = (cfg: JumpLinkBuilderConfig, edge: Edge): EdgeSampler => {
    const ax = vec3.sub(vec3.create(), edge.sq, edge.sp);
    vec3.normalize(ax, ax);
    const az: Vec3 = [ax[2], 0, -ax[0]];
    vec3.normalize(az, az);
    const ay: Vec3 = [0, 1, 0];

    const offset = vec3.create();
    const start = createGroundSegment();
    start.height = cfg.agentClimb * 2;
    trans2d(offset, az, ay, cfg.startDistance, -cfg.agentClimb);
    addOffset(start.p, edge.sp, offset);
    addOffset(start.q, edge.sq, offset);

    const dx = cfg.endDistance - 2 * cfg.agentRadius;
    const nFanSamples = Math.max(2, Math.ceil(dx / cfg.cellSize));
    const heightRange = cfg.maxHeight - cfg.minHeight;

    const end: GroundSegment[] = [];
    for (let j = 0; j < nFanSamples; j++) {
        const v = j / (nFanSamples - 1);
        const ox = 2 * cfg.agentRadius + dx * v;
        trans2d(offset, az, ay, ox, cfg.minHeight);
        const seg = createGroundSegment();
        seg.height = heightRange;
        addOffset(seg.p, edge.sp, offset);
        addOffset(seg.q, edge.sq, offset);
        end.push(seg);
    }

    return {
        edge,
        ax,
        ay,
        az,
        start,
        end,
        trajectory: jumpTrajectory(cfg.jumpHeight),
        type: 'jump',
    };
};

const buildClimbDownSampler = (cfg: JumpLinkBuilderConfig, edge: Edge): EdgeSampler => {
    const ax = vec3.sub(vec3.create(), edge.sq, edge.sp);
    vec3.normalize(ax, ax);
    const az: Vec3 = [ax[2], 0, -ax[0]];
    vec3.normalize(az, az);
    const ay: Vec3 = [0, 1, 0];

    const offset = vec3.create();
    const start = createGroundSegment();
    start.height = cfg.agentClimb * 2;
    trans2d(offset, az, ay, cfg.startDistance, -cfg.agentClimb);
    addOffset(start.p, edge.sp, offset);
    addOffset(start.q, edge.sq, offset);

    trans2d(offset, az, ay, cfg.endDistance, cfg.minHeight);
    const end = createGroundSegment();
    end.height = cfg.maxHeight - cfg.minHeight;
    addOffset(end.p, edge.sp, offset);
    addOffset(end.q, edge.sq, offset);

    return {
        edge,
        ax,
        ay,
        az,
        start,
        end: [end],
        trajectory: climbDownTrajectory,
        type: 'climb-down',
    };
};

/* ---------- ground sampling ---------- */

type HeightProbe = { found: boolean; height: number };

const probeNavMeshHeight = (
    out: HeightProbe,
    navMesh: NavMesh,
    point: Vec3,
    cellSize: number,
    heightRange: number,
): HeightProbe => {
    const halfX = cellSize;
    const halfZ = cellSize;
    const halfY = heightRange;
    const bounds: [number, number, number, number, number, number] = [
        point[0] - halfX,
        point[1] - halfY,
        point[2] - halfZ,
        point[0] + halfX,
        point[1] + halfY,
        point[2] + halfZ,
    ];

    const polys: NodeRef[] = queryPolygons(navMesh, bounds, ANY_QUERY_FILTER);

    out.found = false;
    out.height = point[1];

    const maxHeight = point[1] + heightRange;
    let best = point[1];

    const result = _probeNavMeshHeight_polyHeightResult;
    for (const ref of polys) {
        const node = getNodeByRef(navMesh, ref);
        const tile = navMesh.tiles[node.tileId];
        if (!tile) continue;
        const poly = tile.polys[node.polyIndex];
        if (!poly) continue;
        const r = getPolyHeight(result, tile, poly, node.polyIndex, point);
        if (!r.success) continue;
        const y = r.height;
        if (y > best && y < maxHeight) {
            best = y;
            out.found = true;
            out.height = y;
        }
    }

    return out;
};
const _probeNavMeshHeight_polyHeightResult = createGetPolyHeightResult();

const sampleGroundSegment = (navMesh: NavMesh, seg: GroundSegment, nSamples: number, cellSize: number): void => {
    seg.gsamples = new Array(nSamples);
    const probe: HeightProbe = { found: false, height: 0 };
    for (let i = 0; i < nSamples; i++) {
        const u = i / (nSamples - 1);
        const px = lerp(seg.p[0], seg.q[0], u);
        const py = lerp(seg.p[1], seg.q[1], u);
        const pz = lerp(seg.p[2], seg.q[2], u);
        const point: Vec3 = [px, py, pz];
        probeNavMeshHeight(probe, navMesh, point, cellSize, seg.height);
        const sample: GroundSample = {
            p: [px, probe.found ? probe.height : py, pz],
            validHeight: probe.found,
            validTrajectory: false,
        };
        seg.gsamples[i] = sample;
    }
};

const sampleGround = (navMesh: NavMesh, es: EdgeSampler, cellSize: number): void => {
    const dx = es.start.p[0] - es.start.q[0];
    const dz = es.start.p[2] - es.start.q[2];
    const dist = Math.sqrt(dx * dx + dz * dz);
    const nSamples = Math.max(2, Math.ceil(dist / cellSize));
    sampleGroundSegment(navMesh, es.start, nSamples, cellSize);
    for (const end of es.end) {
        sampleGroundSegment(navMesh, end, nSamples, cellSize);
    }
};

/* ---------- trajectory collision check ---------- */

const heightfieldColumnBlocks = (
    solid: Heightfield,
    x: number,
    yMin: number,
    yMax: number,
    z: number,
): boolean => {
    const ix = Math.floor((x - solid.bounds[0]) / solid.cellSize);
    const iz = Math.floor((z - solid.bounds[2]) / solid.cellSize);
    if (ix < 0 || iz < 0 || ix >= solid.width || iz >= solid.height) {
        return false;
    }
    let span = solid.spans[ix + iz * solid.width];
    while (span !== null) {
        const symin = solid.bounds[1] + span.min * solid.cellHeight;
        const symax = solid.bounds[1] + span.max * solid.cellHeight;
        if (!(yMin > symax || yMax < symin)) {
            return true;
        }
        span = span.next;
    }
    return false;
};

const trajectoryClear = (
    cfg: JumpLinkBuilderConfig,
    solid: Heightfield,
    pa: Vec3,
    pb: Vec3,
    trajectory: Trajectory,
): boolean => {
    const stepSize = Math.min(cfg.cellSize, cfg.cellHeight);
    const dx = pa[0] - pb[0];
    const dy = Math.abs(pa[1] - pb[1]);
    const dz = pa[2] - pb[2];
    const totalDist = Math.sqrt(dx * dx + dz * dz) + dy;
    const nSteps = Math.max(2, Math.ceil(totalDist / stepSize));
    const p = vec3.create();
    for (let i = 0; i < nSteps; i++) {
        const u = i / (nSteps - 1);
        trajectory(p, pa, pb, u);
        if (heightfieldColumnBlocks(solid, p[0], p[1] + cfg.groundTolerance, p[1] + cfg.agentHeight, p[2])) {
            return false;
        }
    }
    return true;
};

const sampleTrajectories = (cfg: JumpLinkBuilderConfig, solid: Heightfield, es: EdgeSampler): void => {
    const nSamples = es.start.gsamples.length;
    for (let i = 0; i < nSamples; i++) {
        const startSample = es.start.gsamples[i];
        if (!startSample.validHeight) continue;
        for (const end of es.end) {
            const endSample = end.gsamples[i];
            if (!endSample.validHeight) continue;
            if (!trajectoryClear(cfg, solid, startSample.p, endSample.p, es.trajectory)) {
                continue;
            }
            startSample.validTrajectory = true;
            endSample.validTrajectory = true;
        }
    }
};

/* ---------- jump segment grouping (flood fill) ---------- */

type JumpSegment = { groundSegment: number; startSample: number; samples: number };

const buildJumpSegments = (cfg: JumpLinkBuilderConfig, es: EdgeSampler): JumpSegment[] => {
    const cols = es.end.length;
    const rows = es.start.gsamples.length; // sample index along the edge
    // grid[i][j] : -1 unvisited, -2 invalid, >=0 region id
    const grid: number[][] = new Array(rows);
    for (let i = 0; i < rows; i++) {
        grid[i] = new Array(cols).fill(-1);
    }

    let region = 0;
    const queue: Array<[number, number]> = [];
    const tryEnqueue = (i: number, j: number, h: number) => {
        if (i < 0 || i >= rows || j < 0 || j >= cols) return;
        const sample = es.end[j].gsamples[i];
        if (!sample.validTrajectory) return;
        if (Math.abs(sample.p[1] - h) >= cfg.agentClimb) return;
        if (grid[i][j] !== -1) return;
        queue.push([i, j]);
    };

    for (let j = 0; j < cols; j++) {
        for (let i = 0; i < rows; i++) {
            if (grid[i][j] !== -1) continue;
            const sample = es.end[j].gsamples[i];
            if (!sample.validTrajectory) {
                grid[i][j] = -2;
                continue;
            }
            queue.length = 0;
            queue.push([i, j]);
            while (queue.length > 0) {
                const [ci, cj] = queue.shift()!;
                if (grid[ci][cj] !== -1) continue;
                grid[ci][cj] = region;
                const h = es.end[cj].gsamples[ci].p[1];
                tryEnqueue(ci + 1, cj, h);
                tryEnqueue(ci - 1, cj, h);
                tryEnqueue(ci, cj + 1, h);
                tryEnqueue(ci, cj - 1, h);
            }
            region++;
        }
    }

    const segments: JumpSegment[] = [];
    for (let r = 0; r < region; r++) {
        segments.push({ groundSegment: 0, startSample: 0, samples: 0 });
    }

    // For each region, pick the (groundSegment column) with the longest contiguous run.
    for (let j = 0; j < cols; j++) {
        let runLen = 0;
        let runRegion = -2;
        for (let i = 0; i <= rows; i++) {
            const cell = i < rows ? grid[i][j] : -2;
            if (cell !== runRegion || i === rows) {
                if (runRegion >= 0 && segments[runRegion].samples < runLen) {
                    segments[runRegion].samples = runLen;
                    segments[runRegion].startSample = i - runLen;
                    segments[runRegion].groundSegment = j;
                }
                runRegion = cell;
                runLen = 1;
            } else {
                runLen++;
            }
        }
    }

    return segments;
};

/* ---------- top-level builders ---------- */

const dist2DSqr = (a: Vec3, b: Vec3) => {
    const dx = a[0] - b[0];
    const dz = a[2] - b[2];
    return dx * dx + dz * dz;
};

const finalizeJumpLinks = (cfg: JumpLinkBuilderConfig, es: EdgeSampler, segments: JumpSegment[]): JumpLink[] => {
    const links: JumpLink[] = [];
    const minWidthSqr = 4 * cfg.agentRadius * cfg.agentRadius;
    for (const seg of segments) {
        if (seg.samples < 2) continue;
        const sIdxA = seg.startSample;
        const sIdxB = seg.startSample + seg.samples - 1;
        const sp = es.start.gsamples[sIdxA].p;
        const sq = es.start.gsamples[sIdxB].p;
        const end = es.end[seg.groundSegment];
        const ep = end.gsamples[sIdxA].p;
        const eq = end.gsamples[sIdxB].p;
        const widthSqr = Math.min(dist2DSqr(sp, sq), dist2DSqr(ep, eq));
        if (widthSqr < minWidthSqr) continue;

        const spine0 = new Float32Array(SPINE_POINTS * 3);
        const spine1 = new Float32Array(SPINE_POINTS * 3);
        const point = vec3.create();
        for (let k = 0; k < SPINE_POINTS; k++) {
            const u = k / (SPINE_POINTS - 1);
            es.trajectory(point, sp, ep, u);
            spine0[k * 3] = point[0];
            spine0[k * 3 + 1] = point[1];
            spine0[k * 3 + 2] = point[2];
            es.trajectory(point, sq, eq, u);
            spine1[k * 3] = point[0];
            spine1[k * 3 + 1] = point[1];
            spine1[k * 3 + 2] = point[2];
        }

        links.push({
            type: es.type,
            start: es.start,
            end,
            startSamples: es.start.gsamples.slice(sIdxA, sIdxA + seg.samples),
            endSamples: end.gsamples.slice(sIdxA, sIdxA + seg.samples),
            trajectory: es.trajectory,
            nspine: SPINE_POINTS,
            spine0,
            spine1,
        });
    }
    return links;
};

export const buildJumpLinks = (
    navMesh: NavMesh,
    solidHeightfield: Heightfield,
    type: JumpLinkType,
    cfg: JumpLinkBuilderConfig,
): JumpLink[] => {
    const edges = extractBorderEdges(navMesh);
    const out: JumpLink[] = [];
    for (const edge of edges) {
        const es = type === 'jump' ? buildJumpSampler(cfg, edge) : buildClimbDownSampler(cfg, edge);
        sampleGround(navMesh, es, cfg.cellSize);
        sampleTrajectories(cfg, solidHeightfield, es);
        const segments = buildJumpSegments(cfg, es);
        for (const link of finalizeJumpLinks(cfg, es, segments)) {
            out.push(link);
        }
    }
    return out;
};
