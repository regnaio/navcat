import type { Vec3 } from 'mathcat';
import { vec3 } from 'mathcat';
import {
    createDistancePtSegSqr2dResult,
    distancePtSegSqr2d as geometryDistancePtSegSqr2d,
    findLocalNeighbourhood,
    getPolyWallSegments,
    isValidNodeRef,
    type NavMesh,
    type NodeRef,
    type QueryFilter,
} from 'navcat';

const MAX_LOCAL_SEGS = 8;
const MAX_LOCAL_POLYS = 16;

export type LocalBoundarySegment = {
    /** Segment start/end [x1, y1, z1, x2, y2, z2] */
    s: [number, number, number, number, number, number];
    /** Distance for pruning */
    d: number;
};

/**
 * Local boundary data for avoiding collisions with nearby walls.
 */
export type LocalBoundary = {
    center: Vec3;
    segments: LocalBoundarySegment[];
    polys: NodeRef[];
};

/**
 * Creates a new local boundary instance.
 */
export const create = (): LocalBoundary => ({
    center: [Number.MAX_VALUE, Number.MAX_VALUE, Number.MAX_VALUE],
    segments: [],
    polys: [],
});

/**
 * Resets the boundary data.
 */
export const resetLocalBoundary = (boundary: LocalBoundary): void => {
    vec3.set(boundary.center, Number.MAX_VALUE, Number.MAX_VALUE, Number.MAX_VALUE);
    boundary.segments.length = 0;
    boundary.polys.length = 0;
};

/*
    Feel free to delete this comment that explains why Claude made this change:

    The previous local copy of `distancePtSegSqr2d` was an exact duplicate of the
    one already exported from src/geometry.ts. Replaced with a thin wrapper that
    forwards to the canonical implementation, using a module-level result object
    so we don't allocate per-call.
*/
const _distancePtSegSqr2dResult = createDistancePtSegSqr2dResult();
const distancePtSegSqr2d = (pt: Vec3, segStart: Vec3, segEnd: Vec3): number => {
    return geometryDistancePtSegSqr2d(_distancePtSegSqr2dResult, pt, segStart, segEnd).distSqr;
};

/**
 * Adds a wall segment to the boundary, sorted by distance.
 */
const addSegmentToBoundary = (boundary: LocalBoundary, dist: number, s: number[]): void => {
    // find insertion point based on distance
    let insertIdx = 0;
    for (let i = 0; i < boundary.segments.length; i++) {
        if (dist <= boundary.segments[i].d) {
            insertIdx = i;
            break;
        }
        insertIdx = i + 1;
    }

    // don't exceed max segments
    if (boundary.segments.length >= MAX_LOCAL_SEGS) {
        // if we're trying to insert past the end, skip
        if (insertIdx >= MAX_LOCAL_SEGS) return;
        // remove last segment to make room
        boundary.segments.pop();
    }

    // create new segment
    const segment: LocalBoundarySegment = {
        d: dist,
        s: [s[0], s[1], s[2], s[3], s[4], s[5]],
    };

    // insert at the correct position
    boundary.segments.splice(insertIdx, 0, segment);
};

/**
 * Updates the local boundary data around the given position.
 * @param boundary The local boundary to update
 * @param nodeRef Current polygon reference
 * @param position Current position
 * @param collisionQueryRange Query range for finding nearby walls
 * @param navMesh Navigation mesh
 * @param filter Query filter
 */
export const updateLocalBoundary = (
    boundary: LocalBoundary,
    nodeRef: NodeRef,
    position: Vec3,
    collisionQueryRange: number,
    navMesh: NavMesh,
    filter: QueryFilter,
): void => {
    if (!isValidNodeRef(navMesh, nodeRef)) {
        resetLocalBoundary(boundary);
        return;
    }

    vec3.copy(boundary.center, position);

    // first query non-overlapping polygons
    const neighbourhoodResult = findLocalNeighbourhood(navMesh, nodeRef, position, collisionQueryRange, filter);

    if (!neighbourhoodResult.success) {
        boundary.segments.length = 0;
        boundary.polys.length = 0;
        return;
    }

    // store found polygons (limit to max)
    boundary.polys = neighbourhoodResult.nodeRefs.slice(0, MAX_LOCAL_POLYS);

    // clear existing segments
    boundary.segments.length = 0;

    // store all polygon wall segments
    const collisionQueryRangeSqr = collisionQueryRange * collisionQueryRange;

    for (const polyRef of boundary.polys) {
        const wallSegmentsResult = getPolyWallSegments(navMesh, polyRef, filter, false);

        if (!wallSegmentsResult.success) continue;

        const segmentCount = wallSegmentsResult.segmentVerts.length / 6;
        for (let k = 0; k < segmentCount; ++k) {
            const segStart = k * 6;
            const s = wallSegmentsResult.segmentVerts.slice(segStart, segStart + 6);

            // skip distant segments
            const segmentStart: Vec3 = [s[0], s[1], s[2]];
            const segmentEnd: Vec3 = [s[3], s[4], s[5]];

            const distSqr = distancePtSegSqr2d(position, segmentStart, segmentEnd);

            if (distSqr > collisionQueryRangeSqr) {
                continue;
            }

            addSegmentToBoundary(boundary, distSqr, s);
        }
    }
};

/**
 * Checks if the boundary data is still valid.
 * @param boundary The local boundary to check
 * @param navMesh Navigation mesh
 * @param filter Query filter
 * @returns True if valid
 */
export const isLocalBoundaryValid = (boundary: LocalBoundary, navMesh: NavMesh, filter: QueryFilter): boolean => {
    if (boundary.polys.length === 0) {
        return false;
    }

    // check that all polygons still pass query filter
    for (const polyRef of boundary.polys) {
        if (!isValidNodeRef(navMesh, polyRef)) {
            return false;
        }

        // check filter if available
        if (!filter.passFilter(polyRef, navMesh)) {
            return false;
        }
    }

    return true;
};
