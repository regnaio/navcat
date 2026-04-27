# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

`navcat` is a pure-JavaScript navigation mesh construction and querying library for 3D floor-based navigation, inspired by (but not a port of) Recast/Detour. It targets games, simulations, and creative web apps and is engine-agnostic — Three.js is supported via an optional adapter, but core APIs have no rendering dependencies.

## Repository layout

This is a pnpm workspace. The root package is the `navcat` library; sub-packages under `examples/`, `docs/snippets/`, and `website/` consume it via `workspace:*`.

- `src/` — the `navcat` core entrypoint
  - `src/generate/` — navmesh generation pipeline (heightfield → compact heightfield → regions → contours → poly mesh → detail mesh → tile)
  - `src/query/` — pathfinding, raycast, A*, string pulling, nav-mesh API (`nav-mesh-api.ts` is the central runtime mutator/accessor)
- `blocks/` — the `navcat/blocks` entrypoint: higher-level presets and abstractions built on top of `navcat`
  - `generators/` — `generateSoloNavMesh`, `generateTiledNavMesh`
  - `agents/` — crowd simulation, path corridor, local boundary, obstacle avoidance
  - `geometry/` — `chunkyTriMesh` spatial partitioning, mesh merging
  - `search/` — flood fill
- `three/` — the `navcat/three` entrypoint: Three.js debug rendering and geometry extraction
- `tst/` — vitest tests (note: `tst`, not `test`)
- `docs/` — `README.template.md` (source of truth for README) and `build.js` that expands snippets into `README.md`
- `examples/` — Vite-based example apps; one HTML entry per `example-*.ts` (used by the website)
- `website/` — Vite app for navcat.dev; `website/build.sh` orchestrates typedoc + examples + website builds for GitHub Pages

The three entry points are intentionally separate: when adding functionality, decide whether it belongs in `navcat` (low-level building blocks) or `navcat/blocks` (presets/abstractions built on core). See `CONTRIBUTING.md`.

## Commands

Run from the repo root unless noted:

- `pnpm run build` — build all three bundles (`dist/index.js`, `dist/blocks.js`, `dist/three.js`) via rollup, plus type declarations
- `pnpm test` — run vitest (use `pnpm test <pattern>` for a single test file, e.g. `pnpm test heightfield`)
- `pnpm run lint` — biome lint with `--write` (autofixes)
- `pnpm run format` — biome format with `--write`
- `pnpm run typedoc` — generate API docs into `dist-typedoc/`
- `pnpm run docs` — regenerate `README.md` from `docs/README.template.md`. **Always run this after editing the template** — never edit `README.md` directly (see `CONTRIBUTING.md`).
- `pnpm --filter navcat-examples dev` — start the examples Vite dev server
- `pnpm --filter website dev` — start the website Vite dev server

CI (`.github/workflows/main.yml`) runs `pnpm run build`, builds the website, then runs `pnpm test`.

## Conventions and gotchas

- **Tooling:** Biome (not ESLint/Prettier) for both lint and format — single quotes, 4-space indent, 130-column lines. `noNonNullAssertion`, `noExplicitAny`, and `noParameterAssign` are intentionally disabled.
- **Coordinate system:** right-handed, OpenGL conventions, counter-clockwise triangle winding. Externally-imported navmeshes must share vertices between adjacent polygons (indexed geometry).
- **Math types come from `mathcat`:** `Vec3` is `[x, y, z]`. `Box3` is a **flat 6-element array** `[minX, minY, minZ, maxX, maxY, maxZ]` (changed in v0.2.0 from `[Vec3, Vec3]`).
- **Invalid node refs:** use the exported `INVALID_NODE_REF` constant (value `-1`), not `0`. Compare with strict equality / explicit null checks, not truthy/falsy checks — `0` is a valid ref. Several past bugs (see `CHANGELOG.md` 0.1.3, 0.1.6) came from falsy checks.
- **No runtime `three` dependency in core:** `three` is an optional peer dep used only by `navcat/three`. The rollup config marks `three`, `mathcat`, `navcat`, and `navcat/blocks` as external for the relevant bundles — preserve those externals when modifying `rollup.config.mjs`.
- **TypeScript:** `tsconfig.json` covers `./src`, `./blocks`, `./three` together with `strict`, `noUnusedLocals`, `noUnusedParameters`, `noImplicitReturns`. The `noEmit: true` / `emitDeclarationOnly` split means rollup handles JS, tsc handles `.d.ts`.
- **Hot paths avoid allocations.** Pathfinding, crowd, and raycast code reuses scratch objects and pools (`IndexPool`, e.g. `nodeIndexPool`, `linkIndexPool` on `NavMesh`). Match that style when editing — don't introduce per-frame allocations in query/agent code.
- **API naming:** standardized on `position` for 3D points and `nodeRef` (or `startNodeRef` / `endNodeRef`) for polygon refs. Don't reintroduce `point` or bare `ref`.
