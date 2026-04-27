import { useState, useCallback } from 'react'
import { Canvas } from '@react-three/fiber'
import * as THREE from 'three'
import { STLLoader } from 'three/addons/loaders/STLLoader.js'
import { STLExporter } from 'three/addons/exporters/STLExporter.js'
import { extractContour, accumulateContours, loftContoursToMesh } from './sweep.js'
import { Scene } from './Scene.jsx'

const PRESETS = [
  { label: '+X', dir: { x:  1, y: 0, z:  0 } },
  { label: '-X', dir: { x: -1, y: 0, z:  0 } },
  { label: '+Y', dir: { x:  0, y: 1, z:  0 } },
  { label: '-Y', dir: { x:  0, y:-1, z:  0 } },
  { label: '+Z', dir: { x:  0, y: 0, z:  1 } },
  { label: '-Z', dir: { x:  0, y: 0, z: -1 } },
]

export default function App() {
  const [originalGeometry, setOriginalGeometry] = useState(null)
  const [sweptGeometry,    setSweptGeometry]    = useState(null)
  const [fileName,         setFileName]         = useState('')
  const [showOriginal,     setShowOriginal]     = useState(true)
  const [error,            setError]            = useState('')

  const [contour,      setContour]      = useState(null)
  const [accumulated,  setAccumulated]  = useState(false)  // true after Step 1b
  const [loftStats,    setLoftStats]    = useState(null)   // set after Step 1c
  const [layerHeight,  setLayerHeight]  = useState(0.2)

  const [direction,  setDirection]  = useState({ x: 0, y: 0, z: 1 })
  const [useCustom,  setUseCustom]  = useState(false)
  const [custom,     setCustom]     = useState({ x: '0', y: '0', z: '1' })

  /* ── STL loading ─────────────────────────────────────────────── */
  const parseSTL = useCallback((arrayBuffer, name) => {
    try {
      const loader   = new STLLoader()
      const geometry = loader.parse(arrayBuffer)
      geometry.computeVertexNormals()
      geometry.center()
      geometry.computeBoundingBox()

      const size = new THREE.Vector3()
      geometry.boundingBox.getSize(size)
      const maxDim = Math.max(size.x, size.y, size.z)

      setOriginalGeometry(geometry)
      setSweptGeometry(null)
      setContour(null)
      setFileName(name)
      setShowOriginal(true)
      setError('')
    } catch (e) {
      setError('Failed to load STL: ' + e.message)
    }
  }, [])

  const handleFile = useCallback(e => {
    const file = e.target.files?.[0]
    if (!file) return
    const reader = new FileReader()
    reader.onload = ev => parseSTL(ev.target.result, file.name)
    reader.readAsArrayBuffer(file)
  }, [parseSTL])

  const handleDrop = useCallback(e => {
    e.preventDefault()
    const file = e.dataTransfer.files?.[0]
    if (!file) return
    const reader = new FileReader()
    reader.onload = ev => parseSTL(ev.target.result, file.name)
    reader.readAsArrayBuffer(file)
  }, [parseSTL])

  /* ── Step 1: Extract contour ────────────────────────────────── */
  const handleExtractContour = useCallback(() => {
    if (!originalGeometry) return
    setError('')
    const dir = useCustom
      ? { x: parseFloat(custom.x) || 0, y: parseFloat(custom.y) || 0, z: parseFloat(custom.z) || 0 }
      : direction
    try {
      const result = extractContour(originalGeometry, dir, { layerHeight })
      setContour(result)
      setAccumulated(false)
    } catch (e) {
      console.error('[contour] extraction failed:', e)
      setError('Contour extraction failed: ' + e.message)
    }
  }, [originalGeometry, direction, custom, useCustom, layerHeight])

  /* ── Step 1b: Accumulate contours ───────────────────────────── */
  const handleAccumulate = useCallback(() => {
    if (!contour) return
    setError('')
    try {
      const result = accumulateContours(contour)
      setContour(result)
      setAccumulated(true)
      setLoftStats(null)
    } catch (e) {
      console.error('[accumulate] failed:', e)
      setError('Accumulation failed: ' + e.message)
    }
  }, [contour])

  /* ── Step 1c: Build lofted mesh ────────────────────────────── */
  const handleLoftMesh = useCallback(() => {
    if (!contour || !accumulated) return
    setError('')
    try {
      const t0 = performance.now()
      const geom = loftContoursToMesh(contour)
      const ms = (performance.now() - t0).toFixed(1)
      const triCount = geom.attributes.position.count / 3
      setSweptGeometry(geom)
      setLoftStats({ ms, triCount })
      setShowOriginal(false)
    } catch (e) {
      console.error('[loft] failed:', e)
      setError('Loft mesh failed: ' + e.message)
    }
  }, [contour, accumulated])

  /* ── Export ──────────────────────────────────────────────────── */
  const handleExport = useCallback(() => {
    if (!sweptGeometry) return
    const mesh     = new THREE.Mesh(sweptGeometry, new THREE.MeshStandardMaterial())
    const exporter = new STLExporter()
    const data     = exporter.parse(mesh, { binary: true })
    const blob     = new Blob([data], { type: 'application/octet-stream' })
    const url      = URL.createObjectURL(blob)
    const a        = document.createElement('a')
    a.href         = url
    a.download     = fileName.replace(/\.stl$/i, '') + '_swept.stl'
    a.click()
    URL.revokeObjectURL(url)
  }, [sweptGeometry, fileName])

  /* ── Derived ─────────────────────────────────────────────────── */
  const triCount = originalGeometry
    ? Math.round((originalGeometry.index
        ? originalGeometry.index.count
        : originalGeometry.attributes.position.count) / 3)
    : 0

  const sweepDir = useCustom
    ? { x: parseFloat(custom.x) || 0, y: parseFloat(custom.y) || 0, z: parseFloat(custom.z) || 0 }
    : direction

  const isDirActive = d =>
    !useCustom && d.x === direction.x && d.y === direction.y && d.z === direction.z

  /* ── Render ──────────────────────────────────────────────────── */
  return (
    <div className="app" onDrop={handleDrop} onDragOver={e => e.preventDefault()}>

      {/* ─── Control panel ───────────────────────────────────────── */}
      <aside className="panel">
        <div className="panel-header">
          <h1>Swept Volume</h1>
          <p className="subtitle">MRSweptVolume · Algorithm 1</p>
        </div>

        {/* Model */}
        <section>
          <span className="section-label">MODEL</span>
          <label className="btn btn-secondary file-btn">
            Load STL
            <input type="file" accept=".stl" onChange={handleFile} hidden />
          </label>
          {fileName
            ? <div className="file-info">
                <span className="file-name">{fileName}</span>
                <span className="badge">{triCount.toLocaleString()} tris</span>
              </div>
            : <p className="hint">or drag &amp; drop an STL here</p>
          }
        </section>

        {/* Direction */}
        <section>
          <span className="section-label">DIRECTION</span>
          <div className="dir-grid">
            {PRESETS.map(({ label, dir }) => (
              <button
                key={label}
                className={'btn dir-btn' + (isDirActive(dir) ? ' active' : '')}
                onClick={() => { setDirection(dir); setUseCustom(false); setContour(null) }}
              >
                {label}
              </button>
            ))}
          </div>
          <label className="toggle-label">
            <input type="checkbox" checked={useCustom}
              onChange={e => { setUseCustom(e.target.checked); setContour(null) }} />
            Custom direction
          </label>
          {useCustom && (
            <div className="custom-inputs">
              {['x', 'y', 'z'].map(axis => (
                <label key={axis} className="axis-input">
                  <span>{axis}</span>
                  <input
                    type="number" step="0.1"
                    value={custom[axis]}
                    onChange={e => setCustom(prev => ({ ...prev, [axis]: e.target.value }))}
                  />
                </label>
              ))}
            </div>
          )}
        </section>

        {/* ── Step 1: Extract Contour ───────────────────────────────── */}
        {originalGeometry && (
          <section className="step-section">
            <span className="section-label">STEP 1 · EXTRACT CONTOUR</span>
            <p className="hint">Slices every {layerHeight}mm. Each slice's exact 2D cross-section is extruded by the full sweep distance — the correct linear Minkowski sum. Produces 3D-shaped ends matching the mesh geometry (no flat caps).</p>

            <span className="section-label">
              LAYER HEIGHT&nbsp;<span className="value-badge">{layerHeight.toFixed(2)} mm</span>
            </span>
            <input
              type="range" min="0.1" max="0.4" step="0.05"
              value={layerHeight}
              onChange={e => { setLayerHeight(parseFloat(e.target.value)); setContour(null) }}
            />

            <button
              className="btn btn-secondary"
              onClick={handleExtractContour}
              disabled={!originalGeometry}
            >
              {contour ? 'Re-extract Contour' : 'Extract Contour'}
            </button>

            {contour && (
              <div className="contour-stats">
                <div className="stat-row">
                  <span className="stat-label">Layer ht</span>
                  <span className="stat-value">{contour.stats.layerHeight.toFixed(2)} mm</span>
                </div>
                <div className="stat-row">
                  <span className="stat-label">Slices</span>
                  <span className="stat-value">{contour.stats.slicesWithContours} / {contour.stats.totalSlices}</span>
                </div>
                <div className="stat-row">
                  <span className="stat-label">Loops</span>
                  <span className="stat-value success">{contour.stats.loops}</span>
                </div>
                <div className="stat-row">
                  <span className="stat-label">Profile shapes</span>
                  <span className="stat-value">{contour.stats.profileShapes}</span>
                </div>
              </div>
            )}

            {!contour && <p className="hint warn-hint">⚠ Extract the contour first for a fast, accurate sweep using ExtrudeGeometry.</p>}
          </section>
        )}

        {/* ── Step 1b: Accumulate Contours ─────────────────────────────────── */}
        {contour && (
          <section className="step-section">
            <span className="section-label">STEP 1b · ACCUMULATE CONTOURS</span>
            <p className="hint">
              Each slice’s 2D profile is replaced by the union of itself and every
              preceding slice. Profiles grow monotonically, so consecutive identical
              ones get merged into one long prism — fewer CSG operations, faster sweep.
            </p>

            <button
              className={'btn btn-secondary' + (accumulated ? ' active' : '')}
              onClick={handleAccumulate}
              disabled={!contour}
            >
              {accumulated ? 'Re-accumulate' : 'Accumulate Contours'}
            </button>

            {accumulated && (
              <div className="contour-stats">
                <div className="stat-row">
                  <span className="stat-label">Merged prisms</span>
                  <span className="stat-value success">{contour.stats.mergedExtrusions}</span>
                </div>
                <div className="stat-row">
                  <span className="stat-label">Reduction</span>
                  <span className="stat-value">
                    {contour.stats.profileShapes} → {contour.stats.mergedExtrusions}
                    &nbsp;({Math.round((1 - contour.stats.mergedExtrusions / contour.stats.profileShapes) * 100)}% fewer)
                  </span>
                </div>
              </div>
            )}
          </section>
        )}

        {/* ── Step 1c: Loft Mesh ─────────────────────────────────────────── */}
        {accumulated && (
          <section className="step-section">
            <span className="section-label">STEP 1c · BUILD LOFT MESH</span>
            <p className="hint">
              Directly triangulates the accumulated contour stack into a watertight mesh.
              No CSG — uses BVH-accelerated slicing, earcut triangulation, and typed arrays.
              Vertical walls + horizontal shelves + top/bottom caps.
            </p>

            <button
              className={'btn btn-primary' + (loftStats ? ' active' : '')}
              onClick={handleLoftMesh}
            >
              {loftStats ? 'Re-build Loft Mesh' : 'Build Loft Mesh (fast)'}
            </button>

            {loftStats && (
              <div className="contour-stats">
                <div className="stat-row">
                  <span className="stat-label">Triangles</span>
                  <span className="stat-value success">{loftStats.triCount.toLocaleString()}</span>
                </div>
                <div className="stat-row">
                  <span className="stat-label">Build time</span>
                  <span className="stat-value">{loftStats.ms} ms</span>
                </div>
              </div>
            )}
          </section>
        )}

        {/* Visibility */}
        {originalGeometry && (
          <section>
            <span className="section-label">VISIBILITY</span>
            <label className="toggle-label">
              <input type="checkbox" checked={showOriginal}
                onChange={e => setShowOriginal(e.target.checked)} />
              Show original model
            </label>
          </section>
        )}

        {/* Export */}
        {sweptGeometry && (
          <section>
            <button className="btn btn-success" onClick={handleExport}>
              Export STL
            </button>
          </section>
        )}

        {error && <div className="error-box">{error}</div>}
      </aside>

      {/* ─── 3D Viewport ─────────────────────────────────────────── */}
      <main className="viewport">
        {!originalGeometry && (
          <div className="empty-state">
            <p>Load an STL file to get started</p>
          </div>
        )}
        <Canvas
          camera={{ position: [80, 60, 120], fov: 45, near: 0.1, far: 5000 }}
          shadows
          gl={{ antialias: true }}
        >
          <Scene
            originalGeometry={originalGeometry}
            sweptGeometry={sweptGeometry}
            contourLoops={contour?.loops3D ?? null}
            showOriginal={showOriginal}
            sweepDirection={sweepDir}
          />
        </Canvas>
      </main>
    </div>
  )
}
