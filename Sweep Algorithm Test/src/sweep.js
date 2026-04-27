import * as THREE from 'three'
import { Brush, Evaluator, ADDITION } from 'three-bvh-csg'
import { MeshBVH, NOT_INTERSECTED, INTERSECTED } from 'three-mesh-bvh'
import polygonClipping from 'polygon-clipping'
import earcut from 'earcut'

function multiPolyToShapes(multiPoly) {
  const shapes = []
  for (const polygon of multiPoly) {
    const outer = polygon[0]
    if (!outer || outer.length < 3) continue
    const shape = new THREE.Shape()
    shape.moveTo(outer[0][0], outer[0][1])
    for (let i = 1; i < outer.length - 1; i++) shape.lineTo(outer[i][0], outer[i][1])
    shape.closePath()
    for (let h = 1; h < polygon.length; h++) {
      const hole = polygon[h]
      if (hole.length < 3) continue
      const path = new THREE.Path()
      path.moveTo(hole[0][0], hole[0][1])
      for (let i = 1; i < hole.length - 1; i++) path.lineTo(hole[i][0], hole[i][1])
      path.closePath()
      shape.holes.push(path)
    }
    shapes.push(shape)
  }
  return shapes
}

// ── Plane-mesh intersection ───────────────────────────────────────────────────
function planeMeshIntersect(geometry, planeNormal, planeD) {
  const pos = geometry.attributes.position
  const triCount = Math.floor(pos.count / 3)
  const segments = []

  for (let t = 0; t < triCount; t++) {
    const b = t * 3
    const v = [0, 1, 2].map(i => new THREE.Vector3(
      pos.getX(b + i), pos.getY(b + i), pos.getZ(b + i)
    ))
    const d = v.map(p => p.dot(planeNormal) - planeD)

    const ints = []
    for (let e = 0; e < 3; e++) {
      const j = (e + 1) % 3
      if ((d[e] > 0) !== (d[j] > 0)) {
        const frac = d[e] / (d[e] - d[j])
        ints.push(v[e].clone().lerp(v[j], frac))
      }
    }
    if (ints.length === 2) segments.push(ints)
  }
  return segments
}

// ── Stitch unordered segments into closed loops ───────────────────────────────
function stitchSegments(segments, eps) {
  if (segments.length === 0) return []
  const eps2 = eps * eps
  const inv = 1.0 / eps
  const key = v =>
    `${Math.round(v.x * inv)},${Math.round(v.y * inv)},${Math.round(v.z * inv)}`

  // Build adjacency: each endpoint key → list of {segIdx, endIdx}
  const adj = new Map()
  for (let i = 0; i < segments.length; i++) {
    for (let e = 0; e < 2; e++) {
      const k = key(segments[i][e])
      if (!adj.has(k)) adj.set(k, [])
      adj.get(k).push({ i, e })
    }
  }

  const used = new Uint8Array(segments.length)
  const loops = []

  for (let start = 0; start < segments.length; start++) {
    if (used[start]) continue
    used[start] = 1

    // Start the loop with the first endpoint; then greedily follow chain
    const loop = [segments[start][0].clone()]
    let cur = segments[start][1].clone()
    const startPt = loop[0]

    for (;;) {
      // Check if we've closed back to the start
      if (cur.distanceToSquared(startPt) <= eps2 * 4) break

      // Look for an unused segment whose endpoint matches cur
      const cands = adj.get(key(cur)) ?? []
      let advanced = false
      for (const { i, e } of cands) {
        if (used[i]) continue
        used[i] = 1
        const next = segments[i][e === 0 ? 1 : 0]
        loop.push(cur)
        cur = next.clone()
        advanced = true
        break
      }
      if (!advanced) break
    }

    if (loop.length >= 3) loops.push(loop)
  }

  console.log(`[contour] stitched ${segments.length} segs -> ${loops.length} loop(s)`)
  return loops
}

// ── BVH-accelerated plane-mesh intersection ───────────────────────────────────
// Requires a MeshBVH built on the geometry. O(log T + K) per slice instead of O(T).
function planeMeshIntersectBVH(bvh, planeNormal, planeD) {
  const nx = planeNormal.x, ny = planeNormal.y, nz = planeNormal.z
  const segments = []

  bvh.shapecast({
    intersectsBounds: (box) => {
      // AABB support: min/max projection onto plane normal
      const projMin = (nx >= 0 ? nx * box.min.x : nx * box.max.x)
                    + (ny >= 0 ? ny * box.min.y : ny * box.max.y)
                    + (nz >= 0 ? nz * box.min.z : nz * box.max.z)
      const projMax = (nx >= 0 ? nx * box.max.x : nx * box.min.x)
                    + (ny >= 0 ? ny * box.max.y : ny * box.min.y)
                    + (nz >= 0 ? nz * box.max.z : nz * box.min.z)
      return (planeD >= projMin && planeD <= projMax) ? INTERSECTED : NOT_INTERSECTED
    },
    intersectsTriangle: (tri) => {
      const va = tri.a, vb = tri.b, vc = tri.c
      const da = va.dot(planeNormal) - planeD
      const db = vb.dot(planeNormal) - planeD
      const dc = vc.dot(planeNormal) - planeD
      const v = [va, vb, vc]
      const d = [da, db, dc]
      const ints = []
      for (let e = 0; e < 3; e++) {
        const j = (e + 1) % 3
        if ((d[e] > 0) !== (d[j] > 0)) {
          const frac = d[e] / (d[e] - d[j])
          ints.push(v[e].clone().lerp(v[j], frac))
        }
      }
      if (ints.length === 2) segments.push(ints)
      return false  // continue traversal
    }
  })

  return segments
}

// ── Orthonormal basis inside the cutting plane ────────────────────────────────
function makeBasis(planeNormal) {
  const n   = planeNormal.clone().normalize()
  const arb = Math.abs(n.x) < 0.9
    ? new THREE.Vector3(1, 0, 0)
    : new THREE.Vector3(0, 1, 0)
  const lx = new THREE.Vector3().crossVectors(n, arb).normalize()
  const ly = new THREE.Vector3().crossVectors(n, lx).normalize()
  return { lx, ly }
}

// ── Project 3D loops onto the plane's local 2D frame ─────────────────────────
function projectTo2D(loops3D, lx, ly) {
  return loops3D.map(loop =>
    loop.map(p => new THREE.Vector2(p.dot(lx), p.dot(ly)))
  )
}

function buildPolyArgs(loops2D) {
  const rings = loops2D
    .filter(l => l.length >= 3)
    .map(l => l.map(p => [p.x, p.y]))
  if (!rings.length) return []

  function area2(r) {
    let a = 0
    for (let i = 0, n = r.length; i < n; i++) {
      const j = (i + 1) % n
      a += r[i][0] * r[j][1] - r[j][0] * r[i][1]
    }
    return a
  }

  // Winding-independent point-in-ring (ray cast)
  function pip(px, py, r) {
    let inside = false
    for (let i = 0, j = r.length - 1; i < r.length; j = i++) {
      const [xi, yi] = r[i], [xj, yj] = r[j]
      if ((yi > py) !== (yj > py) && px < (xj - xi) * (py - yi) / (yj - yi) + xi)
        inside = !inside
    }
    return inside
  }

  // Count how many other rings contain ring[i]'s first point.
  // Depth 0,2,4,... = outer boundary; depth 1,3,5,... = hole.
  // This is winding-independent — works regardless of stitching order.
  const n = rings.length
  const depths = new Int32Array(n)
  for (let i = 0; i < n; i++) {
    const [px, py] = rings[i][0]
    for (let j = 0; j < n; j++) {
      if (i !== j && pip(px, py, rings[j])) depths[i]++
    }
  }

  // Ensure each ring has correct winding convention:
  //   even depth (outer) → CCW  (area2 > 0)
  //   odd  depth (hole)  → CW   (area2 < 0)
  const oriented = rings.map((r, i) => {
    const a = area2(r)
    const shouldBeCCW = depths[i] % 2 === 0
    if ((shouldBeCCW && a < 0) || (!shouldBeCCW && a > 0)) return [...r].reverse()
    return r
  })

  // Assign holes to their direct parent outer ring (smallest area that contains them)
  const polys = []
  const outerIndices = []
  for (let i = 0; i < n; i++) {
    if (depths[i] % 2 === 0) { polys.push([oriented[i]]); outerIndices.push(i) }
  }

  for (let i = 0; i < n; i++) {
    if (depths[i] % 2 !== 1) continue  // not a direct hole
    // Find the smallest containing outer ring
    const [px, py] = rings[i][0]
    let bestPoly = null
    let bestArea = Infinity
    for (let k = 0; k < outerIndices.length; k++) {
      const oi = outerIndices[k]
      const a = Math.abs(area2(oriented[oi]))
      if (a < bestArea && pip(px, py, oriented[oi])) { bestArea = a; bestPoly = polys[k] }
    }
    if (bestPoly) bestPoly.push(oriented[i])
  }

  // Deeper nesting (depth 2 = island inside a hole) → recurse as separate outer poly
  for (let i = 0; i < n; i++) {
    if (depths[i] % 2 === 0 && depths[i] >= 2) {
      // Already added in the outer loop above, no extra work needed
    }
  }

  return polys.length > 0 ? polys.map(p => [p]) : []
}

function canonicalizeRing(ring) {
  const points = ring.map(([x, y]) => [Math.round(x * 1e4), Math.round(y * 1e4)])
  if (points.length > 1) {
    const first = points[0]
    const last = points[points.length - 1]
    if (first[0] === last[0] && first[1] === last[1]) points.pop()
  }
  if (points.length === 0) return ''

  const serialize = pts => pts.map(([x, y]) => `${x}:${y}`).join(';')
  const rotateMin = pts => {
    let minIdx = 0
    for (let i = 1; i < pts.length; i++) {
      const [ax, ay] = pts[i]
      const [bx, by] = pts[minIdx]
      if (ax < bx || (ax === bx && ay < by)) minIdx = i
    }
    return pts.slice(minIdx).concat(pts.slice(0, minIdx))
  }

  const forward = rotateMin(points)
  const backward = rotateMin([...points].reverse())
  const forwardKey = serialize(forward)
  const backwardKey = serialize(backward)
  return forwardKey < backwardKey ? forwardKey : backwardKey
}

function sliceSignature(multiPoly) {
  return multiPoly
    .map(polygon => polygon.map(canonicalizeRing).sort().join('|'))
    .sort()
    .join('||')
}

async function unionBrushesBalanced(brushes, evaluator, onProgress) {
  if (brushes.length === 0) throw new Error('No brushes to union.')
  if (brushes.length === 1) {
    onProgress?.(1)
    return brushes[0]
  }

  let completed = 0
  const totalOps = brushes.length - 1
  let current = brushes

  while (current.length > 1) {
    const next = []
    for (let i = 0; i < current.length; i += 2) {
      if (i + 1 >= current.length) {
        next.push(current[i])
        continue
      }
      next.push(evaluator.evaluate(current[i], current[i + 1], ADDITION))
      completed++
      onProgress?.(completed / totalOps)
      await new Promise(r => setTimeout(r, 0))
    }
    current = next
  }

  return current[0]
}

// ── Debug helper ──────────────────────────────────────────────────────────────
function debugGeometry(label, geometry) {
  if (!geometry) { console.warn(`[sweep] ${label}: null`); return }
  const { start, count } = geometry.drawRange ?? {}
  console.group(`[sweep] ${label}`)
  console.log('drawRange:', { start, count })
  for (const [name, attr] of Object.entries(geometry.attributes ?? {})) {
    if (!attr) { console.warn(`  ${name}: UNDEFINED`); continue }
    const t = attr.array?.constructor?.name ?? (attr.isInterleavedBufferAttribute ? '[interleaved]' : 'NO ARRAY')
    console.log(`  ${name}: itemSize=${attr.itemSize} count=${attr.count} type=${t}`)
  }
  console.groupEnd()
}

// ── Extract the used portion of a CSG result geometry ────────────────────────
function extractCSGResult(geometry) {
  debugGeometry('CSG result (raw)', geometry)
  let geom = geometry
  const hasInterleaved = Object.values(geom.attributes ?? {}).some(a => a?.isInterleavedBufferAttribute)
  if (hasInterleaved) {
    geom = geom.toNonIndexed()
  }

  const { start, count } = geom.drawRange
  const posAttr = geom.attributes.position
  if (!posAttr) throw new Error('[sweep] CSG result has no position attribute')

  const totalVerts = posAttr.count
  const vertCount  = count === Infinity ? totalVerts : Math.min(count, totalVerts)
  console.log(`[sweep] extracting ${vertCount}/${totalVerts} verts`)

  const result = new THREE.BufferGeometry()
  for (const [name, attr] of Object.entries(geom.attributes)) {
    if (!attr?.array) continue
    const { itemSize, normalized, array } = attr
    const from = start * itemSize
    const to   = Math.min((start + vertCount) * itemSize, array.length)
    result.setAttribute(name, new THREE.BufferAttribute(array.slice(from, to), itemSize, normalized))
  }
  if (!result.attributes.position)
    throw new Error('[sweep] Extracted geometry has no position attribute')
  if (geom.index) {
    const src   = geom.index.array
    const slice = src.slice(start, start + vertCount)
    if (slice.length > 0) {
      const minIdx = slice.reduce((a, b) => Math.min(a, b), Infinity)
      result.setIndex(Array.from(slice).map(i => i - minIdx))
    }
  }
  result.computeVertexNormals()
  return result
}

// =============================================================================
// PUBLIC API
// =============================================================================

export function extractContour(geometry, direction, { layerHeight = 0.2 } = {}) {
  const dir = new THREE.Vector3(direction.x, direction.y, direction.z)
  if (dir.lengthSq() < 1e-10) throw new Error('Sweep direction is zero vector')
  dir.normalize()

  let geom = geometry.clone()
  if (geom.index) geom = geom.toNonIndexed()

  geom.computeBoundingBox()
  const bbSize = new THREE.Vector3()
  geom.boundingBox.getSize(bbSize)
  const eps = Math.max(bbSize.x, bbSize.y, bbSize.z) * 1e-5

  const pos = geom.attributes.position
  let eMin = Infinity, eMax = -Infinity
  for (let i = 0; i < pos.count; i++) {
    const d = pos.getX(i) * dir.x + pos.getY(i) * dir.y + pos.getZ(i) * dir.z
    if (d < eMin) eMin = d
    if (d > eMax) eMax = d
  }

  const { lx, ly } = makeBasis(dir)

  // Build BVH once — O(T log T) up front, then O(log T + K) per slice instead of O(T)
  const bvh = new MeshBVH(geom)

  const allLoops3D = []
  const slices = []
  let totalSlices = 0
  let slicesWithContours = 0

  for (let planeD = eMin + layerHeight * 0.5; planeD < eMax; planeD += layerHeight) {
    totalSlices++
    const segments = planeMeshIntersectBVH(bvh, dir, planeD)
    if (segments.length === 0) continue
    const loops3D = stitchSegments(segments, eps)
    if (loops3D.length === 0) continue

    allLoops3D.push(...loops3D)
    slicesWithContours++

    const loops2D = projectTo2D(loops3D, lx, ly)
    const polyArgs = buildPolyArgs(loops2D)
    if (polyArgs.length === 0) continue

    let unionResult
    try {
      // buildPolyArgs returns MultiPolygon[] — each element is already a valid MultiPolygon.
      // Single-polygon: use directly. Multi: union all into one MultiPolygon.
      unionResult = polyArgs.length === 1
        ? polyArgs[0]
        : polygonClipping.union(...polyArgs)
    } catch (e) {
      console.warn(`[contour] slice union at ${planeD.toFixed(3)} failed:`, e.message)
      unionResult = polyArgs[0]
    }

    const shapes = multiPolyToShapes(unionResult)
    if (shapes?.length > 0) {
      slices.push({
        planeD,
        poly: unionResult,   // raw MultiPolygon — kept for accumulateContours
        shapes,
        signature: sliceSignature(unionResult),
      })
    }
  }

  console.log(`[contour] ${slicesWithContours}/${totalSlices} slices, ${allLoops3D.length} loops, ${slices.length} profiles`)

  if (slices.length === 0)
    throw new Error('No cross-section found. Try a different sweep direction.')

  const mergedSlices = []
  for (const slice of slices) {
    const prev = mergedSlices[mergedSlices.length - 1]
    const expectedGap = prev ? Math.abs(slice.planeD - prev.planeDEnd - layerHeight) : Infinity
    if (prev && prev.signature === slice.signature && expectedGap < layerHeight * 0.25) {
      prev.planeDEnd = slice.planeD
      continue
    }

    mergedSlices.push({
      planeDStart: slice.planeD,
      planeDEnd: slice.planeD,
      poly:      slice.poly,
      shapes:    slice.shapes,
      signature: slice.signature,
    })
  }

  const stats = {
    inputVertices:      pos.count,
    layerHeight,
    totalSlices,
    slicesWithContours,
    loops:              allLoops3D.length,
    totalContourPoints: allLoops3D.reduce((s, l) => s + l.length, 0),
    profileShapes:      slices.length,
    mergedExtrusions:   mergedSlices.length,
    meshExtentMin:      eMin,
    meshExtentMax:      eMax,
  }
  console.log('[contour] stats:', stats)

  return { loops3D: allLoops3D, slices, mergedSlices, lx, ly, meshExtentMin: eMin, meshExtentMax: eMax, stats }
}

/**
 * STEP 1b — Accumulate contour profiles forward.
 *
 * Replaces each slice's 2D profile with the union of itself and every
 * preceding slice's profile (a running forward union).
 *
 * Effect: the profile can only grow or stay the same as you move along
 * the sweep axis.  Consecutive identical profiles get run-merged into
 * one long prism, dramatically reducing the number of CSG operations.
 *
 * NOTE: this changes the swept solid's cross-section to always be a
 * superset of the start-end geometry — use when you want the
 * "maximum material" envelope rather than the exact Minkowski sum.
 */
export function accumulateContours(contour) {
  const { slices, lx, ly, meshExtentMin, meshExtentMax, stats } = contour

  let accPoly = null
  const accSlices = []

  for (const slice of slices) {
    try {
      accPoly = accPoly === null
        ? slice.poly
        : polygonClipping.union(accPoly, slice.poly)
    } catch (e) {
      console.warn(`[accumulate] union failed at ${slice.planeD.toFixed(3)}:`, e.message)
      // keep accPoly as-is (no regression)
    }

    const shapes = multiPolyToShapes(accPoly)
    if (shapes?.length > 0) {
      accSlices.push({
        planeD:    slice.planeD,
        poly:      accPoly,
        shapes,
        signature: sliceSignature(accPoly),
      })
    }
  }

  if (accSlices.length === 0) throw new Error('Accumulation produced no valid slices.')

  // Reconstruct loops3D from the accumulated 2D polygons so the viewer shows
  // the grown profiles.  Un-project: p3D = planeD·dir + px·lx + py·ly
  const dir3 = new THREE.Vector3().crossVectors(lx, ly).normalize()
  const accLoops3D = []
  for (const slice of accSlices) {
    const origin = dir3.clone().multiplyScalar(slice.planeD)
    for (const polygon of slice.poly) {
      for (const ring of polygon) {
        if (ring.length < 3) continue
        const loop = ring
          .slice(0, ring.length)  // may be closed (first==last) — keep all for display
          .map(([px, py]) =>
            origin.clone()
              .addScaledVector(lx, px)
              .addScaledVector(ly, py)
          )
        accLoops3D.push(loop)
      }
    }
  }

  // Re-run the merge step on the now-monotone profiles
  const layerHeight = stats.layerHeight
  const mergedSlices = []
  for (const slice of accSlices) {
    const prev = mergedSlices[mergedSlices.length - 1]
    const gap  = prev ? Math.abs(slice.planeD - prev.planeDEnd - layerHeight) : Infinity
    if (prev && prev.signature === slice.signature && gap < layerHeight * 0.25) {
      prev.planeDEnd = slice.planeD
      continue
    }
    mergedSlices.push({
      planeDStart: slice.planeD,
      planeDEnd:   slice.planeD,
      poly:        slice.poly,
      shapes:      slice.shapes,
      signature:   slice.signature,
    })
  }

  console.log(`[accumulate] ${accSlices.length} slices -> ${mergedSlices.length} merged extrusions`)

  const newStats = {
    ...stats,
    profileShapes:    accSlices.length,
    mergedExtrusions: mergedSlices.length,
    accumulated:      true,
  }

  return { ...contour, loops3D: accLoops3D, slices: accSlices, mergedSlices, stats: newStats }
}

/**
 * STEP 1c — Staircase loft: build a watertight BufferGeometry directly from
 * the accumulated (monotone-nested) contour stack.
 *
 * Algorithm:
 *   – Bottom cap  : triangulate first slab's polygon facing -dir
 *   – Per slab    : vertical wall quads around every ring (outer + holes)
 *   – Per shelf   : polygon-difference of adjacent slabs, triangulated facing +dir
 *   – Top cap     : triangulate last slab's polygon facing +dir
 *
 * Cost: O(N · V log V)  — no CSG, no voxelization, no three-bvh-csg.
 * Requires accumulateContours() to have been called (mergedSlices must carry poly).
 */
export function loftContoursToMesh(contour) {
  const { mergedSlices, lx, ly, stats } = contour
  if (!mergedSlices?.length) throw new Error('No merged slices. Run extractContour then accumulateContours first.')
  if (!mergedSlices[0].poly) throw new Error('mergedSlices missing poly field — please re-run Extract Contour.')

  const dir3 = new THREE.Vector3().crossVectors(lx, ly).normalize()
  const layerHeight = stats.layerHeight

  // Cache basis components for the hot-path
  const lxx = lx.x, lxy = lx.y, lxz = lx.z
  const lyx = ly.x, lyy = ly.y, lyz = ly.z
  const d3x = dir3.x, d3y = dir3.y, d3z = dir3.z

  // Use plain JS arrays for push(), convert to typed arrays at the end
  const positions = []
  const normals   = []

  // Emit one vertex given 2D coords (px,py) in the lx/ly plane at sweep-axis height h
  function pushV(px, py, h) {
    positions.push(
      h*d3x + px*lxx + py*lyx,
      h*d3y + px*lxy + py*lyy,
      h*d3z + px*lxz + py*lyz
    )
  }

  // ── Cap triangulation ─────────────────────────────────────────────────────
  // normalSign: +1 → faces +dir3 (top / upward shelf), -1 → faces -dir3 (bottom)
  function triangulateCap(multiPoly, h, normalSign) {
    const nx = normalSign * d3x
    const ny = normalSign * d3y
    const nz = normalSign * d3z
    for (const polygon of multiPoly) {
      if (!polygon?.length) continue
      const outer = polygon[0]
      if (!outer || outer.length < 3) continue

      // Build flat earcut input. polygon-clipping rings are closed (first == last) — drop last.
      const verts = []
      const holeIdxs = []

      const olen = outer.length
      const outerClosed = olen > 1 && outer[0][0] === outer[olen-1][0] && outer[0][1] === outer[olen-1][1]
      for (let i = 0; i < (outerClosed ? olen - 1 : olen); i++) verts.push(outer[i][0], outer[i][1])

      for (let hi = 1; hi < polygon.length; hi++) {
        const hole = polygon[hi]
        if (!hole || hole.length < 3) continue
        holeIdxs.push(verts.length >> 1)  // vertex index of hole start
        const hlen = hole.length
        const holeClosed = hlen > 1 && hole[0][0] === hole[hlen-1][0] && hole[0][1] === hole[hlen-1][1]
        for (let i = 0; i < (holeClosed ? hlen - 1 : hlen); i++) verts.push(hole[i][0], hole[i][1])
      }

      if (verts.length < 6) continue  // fewer than 3 vertices

      const indices = earcut(verts, holeIdxs.length > 0 ? holeIdxs : null)
      for (let t = 0; t < indices.length; t += 3) {
        // Flip winding for downward-facing cap so normal points -dir3
        const ia = indices[t]
        const ib = normalSign > 0 ? indices[t+1] : indices[t+2]
        const ic = normalSign > 0 ? indices[t+2] : indices[t+1]
        pushV(verts[ia*2], verts[ia*2+1], h)
        pushV(verts[ib*2], verts[ib*2+1], h)
        pushV(verts[ic*2], verts[ic*2+1], h)
        normals.push(nx,ny,nz, nx,ny,nz, nx,ny,nz)
      }
    }
  }

  // ── Vertical wall quads ───────────────────────────────────────────────────
  // For an outer ring (CCW): outward normal = (dy*lx − dx*ly).normalize()
  // For a hole ring  (CW) : outward normal = −(dy*lx − dx*ly).normalize()
  // Wall winding: CCW when viewed from the outward normal.
  // Outer CCW ring: a→b is CCW, outward = right of travel.
  //   Tri1: BL,BR,TR  Tri2: BL,TR,TL  (both CCW from outside) ✔
  // Hole CW ring: a→b is CW, outward = left of travel = -(dy*lx−dx*ly).
  //   Same vertex layout but we must flip winding to keep CCW from outward side:
  //   Tri1: BL,TR,BR  Tri2: BL,TL,TR
  function generateWalls(ring, hLow, hHigh) {
    const rlen = ring.length
    if (rlen < 3) return
    const closed = ring[0][0] === ring[rlen-1][0] && ring[0][1] === ring[rlen-1][1]
    const n = closed ? rlen - 1 : rlen

    // Detect ring winding via signed area (positive = CCW in standard math coords)
    let area2 = 0
    for (let i = 0; i < n; i++) {
      const j = (i + 1) % n
      area2 += ring[i][0] * ring[j][1] - ring[j][0] * ring[i][1]
    }
    const isCCW = area2 > 0  // outer ring = CCW, hole ring = CW (area2 < 0)

    for (let i = 0; i < n; i++) {
      const [ax, ay] = ring[i]
      const [bx, by] = ring[(i + 1) % n]
      const dx = bx - ax, dy = by - ay
      // Outward normal: (dy*lx − dx*ly) for CCW, negated for CW
      const sign = isCCW ? 1 : -1
      const wx = sign*(dy*lxx - dx*lyx)
      const wy = sign*(dy*lxy - dx*lyy)
      const wz = sign*(dy*lxz - dx*lyz)
      const wl = Math.sqrt(wx*wx + wy*wy + wz*wz)
      if (wl < 1e-10) continue
      const invWl = 1 / wl
      const wnx = wx*invWl, wny = wy*invWl, wnz = wz*invWl

      if (isCCW) {
        // Outer ring: winding CCW from outside: BL,BR,TR and BL,TR,TL
        pushV(ax,ay,hLow);  pushV(bx,by,hLow);  pushV(bx,by,hHigh)
        normals.push(wnx,wny,wnz, wnx,wny,wnz, wnx,wny,wnz)
        pushV(ax,ay,hLow);  pushV(bx,by,hHigh); pushV(ax,ay,hHigh)
        normals.push(wnx,wny,wnz, wnx,wny,wnz, wnx,wny,wnz)
      } else {
        // Hole ring: flip winding to keep CCW from outward (inward) side: BL,TR,BR and BL,TL,TR
        pushV(ax,ay,hLow);  pushV(bx,by,hHigh); pushV(bx,by,hLow)
        normals.push(wnx,wny,wnz, wnx,wny,wnz, wnx,wny,wnz)
        pushV(ax,ay,hLow);  pushV(ax,ay,hHigh); pushV(bx,by,hHigh)
        normals.push(wnx,wny,wnz, wnx,wny,wnz, wnx,wny,wnz)
      }
    }
  }

  // ── Main staircase loop ───────────────────────────────────────────────────
  const hBottom = mergedSlices[0].planeDStart - layerHeight * 0.5

  // Bottom cap (faces -dir3)
  triangulateCap(mergedSlices[0].poly, hBottom, -1)

  for (let i = 0; i < mergedSlices.length; i++) {
    const slab = mergedSlices[i]
    const hLow  = slab.planeDStart - layerHeight * 0.5
    const hHigh = slab.planeDEnd   + layerHeight * 0.5

    // Vertical walls for every ring (outer boundary + holes)
    for (const polygon of slab.poly) {
      if (!polygon?.length) continue
      for (const ring of polygon) {
        if (ring.length >= 3) generateWalls(ring, hLow, hHigh)
      }
    }

    // Shelf at the top interface with the next slab
    if (i + 1 < mergedSlices.length) {
      const nextPoly = mergedSlices[i + 1].poly

      // Upward shelf: area of nextPoly not in thisPoly (solid grew outward)
      let upShelf = []
      try { upShelf = polygonClipping.difference(nextPoly, slab.poly) } catch (_) {}
      if (upShelf.length > 0) triangulateCap(upShelf, hHigh, +1)

      // Downward shelf: area of thisPoly not in nextPoly (rare for accumulated)
      let downShelf = []
      try { downShelf = polygonClipping.difference(slab.poly, nextPoly) } catch (_) {}
      if (downShelf.length > 0) triangulateCap(downShelf, hHigh, -1)
    }
  }

  // Top cap (faces +dir3)
  const last = mergedSlices[mergedSlices.length - 1]
  const hTop = last.planeDEnd + layerHeight * 0.5
  triangulateCap(last.poly, hTop, +1)

  const posArr = new Float32Array(positions)
  const norArr = new Float32Array(normals)

  const geom = new THREE.BufferGeometry()
  geom.setAttribute('position', new THREE.BufferAttribute(posArr, 3))
  geom.setAttribute('normal',   new THREE.BufferAttribute(norArr, 3))

  console.log(`[loft] ${posArr.length / 9} triangles, ${posArr.length / 3} vertices`)
  return geom
}

export async function computeSweptVolume(geometry, direction, distance, numSamples, onProgress, contour = null) {
  const dir = new THREE.Vector3(direction.x, direction.y, direction.z)
  if (dir.lengthSq() < 1e-10) throw new Error('Sweep direction is a zero vector')
  dir.normalize()

  // ── Fast path: merged per-slice Minkowski sum ─────────────────────────────
  // Exact optimization:
  // 1. Consecutive identical slice profiles are merged into one longer prism.
  // 2. Remaining prisms are unioned in a balanced tree, not a growing chain.
  if (contour?.mergedSlices?.length > 0) {
    const { mergedSlices, lx, ly } = contour
    console.log(`[sweep] Fast path: ${mergedSlices.length} merged extrusions from ${contour.slices.length} slices, distance=${distance.toFixed(3)}`)

    const evaluator = new Evaluator()
    evaluator.useGroups = false
    evaluator.attributes = ['position', 'normal']
    const mat = new THREE.MeshStandardMaterial()
    const brushes = []

    for (let i = 0; i < mergedSlices.length; i++) {
      const { planeDStart, planeDEnd, shapes } = mergedSlices[i]
      const origin3D = dir.clone().multiplyScalar(planeDStart)
      const sliceTransform = new THREE.Matrix4().set(
        lx.x, ly.x, dir.x, origin3D.x,
        lx.y, ly.y, dir.y, origin3D.y,
        lx.z, ly.z, dir.z, origin3D.z,
        0,    0,    0,     1
      )

      let sliceGeom = new THREE.ExtrudeGeometry(shapes, {
        depth: distance + (planeDEnd - planeDStart),
        steps: 1,
        bevelEnabled: false,
      })
      sliceGeom.applyMatrix4(sliceTransform)
      sliceGeom.deleteAttribute('uv')
      if (sliceGeom.index) sliceGeom = sliceGeom.toNonIndexed()
      sliceGeom.computeVertexNormals()

      const brush = new Brush(sliceGeom, mat)
      brush.updateMatrixWorld()
      brushes.push(brush)
    }

    const result = await unionBrushesBalanced(brushes, evaluator, onProgress)
    return extractCSGResult(result.geometry)
  }

  // ── Slow path: CSG union ──────────────────────────────────────────────────
  console.log('[sweep] Slow path: CSG union')
  let baseGeom = geometry.clone()
  if (baseGeom.index) baseGeom = baseGeom.toNonIndexed()
  baseGeom.computeVertexNormals()

  const evaluator = new Evaluator()
  evaluator.useGroups = false
  evaluator.attributes = Object.keys(baseGeom.attributes)

  const mat = new THREE.MeshStandardMaterial()
  let result = new Brush(baseGeom.clone(), mat)
  result.updateMatrixWorld()

  for (let i = 1; i <= numSamples; i++) {
    const t      = i / numSamples
    const offset = dir.clone().multiplyScalar(t * distance)
    const brush  = new Brush(baseGeom.clone(), mat)
    brush.position.copy(offset)
    brush.updateMatrixWorld()
    try {
      result = evaluator.evaluate(result, brush, ADDITION)
    } catch (e) {
      throw new Error(`CSG union failed at step ${i}/${numSamples}: ${e.message}`)
    }
    onProgress?.(t)
    await new Promise(resolve => setTimeout(resolve, 0))
  }

  return extractCSGResult(result.geometry)
}

