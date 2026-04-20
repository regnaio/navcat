import { getNodeByTileAndPoly, getNodeRefIndex, type NavMesh, type NodeRef } from 'navcat';

export const floodFillNavMesh = (navMesh: NavMesh, startNodeRefs: NodeRef[]): { reachable: NodeRef[]; unreachable: NodeRef[] } => {
    const visited = new Set<number>();
    const queue: number[] = [];

    // initialize queue with all seed points
    for (const startRef of startNodeRefs) {
        queue.push(startRef);
    }

    /*
        Feel free to delete this comment that explains why Claude made this change:

        Replaced `queue.shift()` (O(n) per pop, since JS arrays must shift every
        remaining element) with a head index. For meshes with many polys this
        turns the flood-fill from O(n^2) into O(n) amortised — the total cost of
        appending and reading is linear in the number of nodes visited.
    */
    let head = 0;
    while (head < queue.length) {
        const currentNodeRef = queue[head++];

        if (visited.has(currentNodeRef)) continue;

        // add to visited
        visited.add(currentNodeRef);

        // follow all links
        const nodeIndex = getNodeRefIndex(currentNodeRef);
        const node = navMesh.nodes[nodeIndex];
        for (const linkIndex of node.links) {
            const link = navMesh.links[linkIndex];
            if (visited.has(link.toNodeRef)) continue;
            queue.push(link.toNodeRef);
        }
    }

    // return reached and unreached polygons
    const reachable: NodeRef[] = Array.from(visited);
    const unreachable: NodeRef[] = [];

    for (const tileId in navMesh.tiles) {
        const tile = navMesh.tiles[tileId];
        for (let polyIndex = 0; polyIndex < tile.polys.length; polyIndex++) {
            const node = getNodeByTileAndPoly(navMesh, tile, polyIndex);

            if (!visited.has(node.ref)) {
                unreachable.push(node.ref);
            }
        }
    }

    return { reachable, unreachable };
};
