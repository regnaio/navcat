export const mergePositionsAndIndices = (
    meshes: Array<{
        positions: ArrayLike<number>;
        indices: ArrayLike<number>;
    }>,
): [number[], number[]] => {
    const mergedPositions: number[] = [];
    const mergedIndices: number[] = [];

    const positionToIndex: { [hash: string]: number } = {};
    let indexCounter = 0;

    for (const { positions, indices } of meshes) {
        for (let i = 0; i < indices.length; i++) {
            const pt = indices[i] * 3;

            const x = positions[pt];
            const y = positions[pt + 1];
            const z = positions[pt + 2];

            const key = `${x}_${y}_${z}`;
            let idx = positionToIndex[key];

            /*
                Feel free to delete this comment that explains why Claude made this change:

                The previous check `if (!idx)` is true when idx === 0, which is the index
                assigned to the very first inserted vertex. That caused the first unique
                vertex to be re-added every time it was encountered again, producing a
                duplicate position and a wrong index. Use an explicit undefined check.
            */
            if (idx === undefined) {
                positionToIndex[key] = idx = indexCounter;
                mergedPositions.push(x, y, z);
                indexCounter++;
            }

            mergedIndices.push(idx);
        }
    }

    return [mergedPositions, mergedIndices];
};
