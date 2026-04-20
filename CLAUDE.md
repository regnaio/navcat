# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

This is a pnpm workspace. Use `pnpm` (not npm/yarn). Workspace packages: [examples/](examples/), [docs/snippets/](docs/snippets/), [website/](website/).

- **Build the library**: `pnpm build` — wipes `dist/` and runs Rollup, producing three ES bundles (`dist/index.js`, `dist/blocks.js`, `dist/three.js`) plus `.d.ts` files.
- **Test**: `pnpm test` (vitest). Run a single test file with `pnpm test tst/heightfield.test.ts`; a single test with `pnpm test -t "test name"`.
- **Lint / format**: `pnpm lint` and `pnpm format` (biome — 4-space indent, single quotes, line width 130, configured in [biome.json](biome.json)). Both use `--write` so they auto-fix.
- **Rebuild README.md**: `pnpm docs` — runs [docs/build.js](docs/build.js) which expands custom tags (`<TOC />`, `<Snippet />`, `<RenderType />`, `<Example />`, etc.) in [docs/README.template.md](docs/README.template.md) into [README.md](README.md). **Never edit `README.md` directly** — edit the template and rebuild.
- **API docs**: `pnpm typedoc` produces TypeDoc HTML in `dist-typedoc/`.
- **Examples / website dev**: `cd examples && pnpm dev` or `cd website && pnpm dev` (Vite). They consume the built library via `workspace:*`, so run `pnpm build` at the root first if you change library source.

## Architecture

navcat is a JS navigation-mesh construction and querying library. Three module entry points, each a separate Rollup bundle:

| Entry point | Source | Purpose |
| --- | --- | --- |
| `navcat` | [src/](src/) | Low-level building blocks: generation pipeline + querying APIs. Only `mathcat` as a runtime dep. |
| `navcat/blocks` | [blocks/](blocks/) | Higher-level abstractions and presets built on top of `navcat`. Depends on `navcat`. |
| `navcat/three` | [three/](three/) | Optional Three.js debug helpers and geometry adapters. Three is an optional peer dep. |

The split is enforced as a contribution rule (see [CONTRIBUTING.md](CONTRIBUTING.md)): core building blocks go in `src/`; presets and assembled workflows go in `blocks/`.

### Core library (`src/`)

- [src/generate/](src/generate/) — the navmesh build pipeline, executed in this order: input triangle mesh → heightfield (rasterize) → compact heightfield → compact heightfield regions → contour set → poly mesh + poly mesh detail → nav mesh tile. Each stage is a separate file; `nav-mesh-tile.ts` produces the runtime tile format. [src/generate/build-context.ts](src/generate/build-context.ts) collects timings/logs across stages.
- [src/query/](src/query/) — runtime navmesh APIs. [nav-mesh.ts](src/query/nav-mesh.ts) defines the `NavMesh` data structure; [nav-mesh-api.ts](src/query/nav-mesh-api.ts) is the public mutation/inspection surface (add/remove tiles, off-mesh connections, node lookup). [find-path.ts](src/query/find-path.ts), [find-straight-path.ts](src/query/find-straight-path.ts), [find-smooth-path.ts](src/query/find-smooth-path.ts), [nav-mesh-search.ts](src/query/nav-mesh-search.ts) implement A*/string-pulling/funnel/raycast queries. [bv-tree.ts](src/query/bv-tree.ts) is the per-tile BVH. Node references (`NodeRef`) are packed integers — see [node.ts](src/query/node.ts).
- [src/geometry.ts](src/geometry.ts), [src/index-pool.ts](src/index-pool.ts), [src/debug.ts](src/debug.ts) — shared utilities. `geometry` is re-exported as a namespace from `src/index.ts`.

### Blocks (`blocks/`)

- [blocks/generators/](blocks/generators/) — `generateSoloNavMesh` and `generateTiledNavMesh` wire the full `src/generate/` pipeline behind a single options object.
- [blocks/agents/](blocks/agents/) — crowd simulation, local boundary, obstacle avoidance, path corridor. Each is exported as a namespace (`crowd`, `localBoundary`, `obstacleAvoidance`, `pathCorridor`).
- [blocks/search/](blocks/search/) — flood-fill helpers (e.g. pruning unreachable polys).
- [blocks/geometry/](blocks/geometry/) — chunky tri mesh (spatial index for input triangles, used by tiled generation) and `mergePositionsAndIndices`.

### Conventions

- **Coordinate system**: right-handed, indices in counter-clockwise winding order (OpenGL convention). Externally-authored navmeshes must share vertices between adjacent polygons.
- **Math types** come from `mathcat`: `Vec3` is `[x, y, z]`; `Box3` is the **flat** form `[minX, minY, minZ, maxX, maxY, maxZ]` (changed in 0.2.0 — see [CHANGELOG.md](CHANGELOG.md)).
- **Data structures are JSON-serializable** — no class instances, no methods on data. APIs are free functions taking the data as the first argument.
- **Node references must not be cached as objects** — nodes are pooled and reused when tiles are removed/re-added. Always look up via `getNodeByRef(navMesh, nodeRef)`.
- TypeScript is strict with `noUnusedLocals` / `noUnusedParameters` / `noImplicitReturns`; `noEmit` is true (Rollup emits).
- Biome rules disable `noNonNullAssertion`, `noParameterAssign`, `noExplicitAny`, `noUselessContinue` — these are intentionally allowed.

### Tests ([tst/](tst/))

Vitest, no separate config — tests sit at the repo root in `tst/` and import from `src/` directly. New algorithms in `src/` should get a matching `*.test.ts`.
