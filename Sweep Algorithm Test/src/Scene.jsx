import { useEffect, useMemo } from 'react'
import { useThree } from '@react-three/fiber'
import { OrbitControls, GizmoHelper, GizmoViewport, Grid } from '@react-three/drei'
import * as THREE from 'three'

/**
 * Fits the camera to the given geometry's bounding sphere.
 * Must live inside <Canvas> so it can access useThree().
 */
function AutoFitCamera({ geometry }) {
  const { camera, controls } = useThree()

  useEffect(() => {
    if (!geometry) return
    geometry.computeBoundingSphere()
    const sphere = geometry.boundingSphere
    if (!sphere) return

    const dist = sphere.radius * 3.5
    camera.position.set(
      sphere.center.x + dist * 0.6,
      sphere.center.y + dist * 0.5,
      sphere.center.z + dist
    )
    camera.lookAt(sphere.center)
    camera.updateProjectionMatrix()

    if (controls) {
      controls.target.copy(sphere.center)
      controls.update()
    }
  }, [geometry, camera, controls])

  return null
}

export function Scene({ originalGeometry, sweptGeometry, contourLoops, showOriginal, sweepDirection }) {
  const dirVec = useMemo(() => {
    const v = new THREE.Vector3(sweepDirection.x, sweepDirection.y, sweepDirection.z)
    return v.lengthSq() > 0 ? v.normalize() : new THREE.Vector3(0, 0, 1)
  }, [sweepDirection])

  const arrowOrigin = useMemo(() => {
    if (!originalGeometry) return new THREE.Vector3()
    originalGeometry.computeBoundingBox()
    const c = new THREE.Vector3()
    originalGeometry.boundingBox.getCenter(c)
    return c
  }, [originalGeometry])

  const arrowLength = useMemo(() => {
    if (!originalGeometry) return 10
    originalGeometry.computeBoundingBox()
    const s = new THREE.Vector3()
    originalGeometry.boundingBox.getSize(s)
    return Math.max(s.x, s.y, s.z)
  }, [originalGeometry])

  // Build LineLoop geometries from the raw Vector3 loop arrays (memoised)
  const lineGeometries = useMemo(() => {
    if (!contourLoops) return []
    return contourLoops.map(loop => {
      const pts = new Float32Array(loop.length * 3)
      loop.forEach((v, j) => {
        pts[j * 3]     = v.x
        pts[j * 3 + 1] = v.y
        pts[j * 3 + 2] = v.z
      })
      const g = new THREE.BufferGeometry()
      g.setAttribute('position', new THREE.BufferAttribute(pts, 3))
      return g
    })
  }, [contourLoops])

  // Fit camera whenever the displayed geometry changes
  const fitTarget = sweptGeometry ?? originalGeometry

  return (
    <>
      <color attach="background" args={['#0d0f16']} />

      <ambientLight intensity={0.5} />
      <directionalLight position={[15, 20, 10]} intensity={1.2} castShadow />
      <directionalLight position={[-10, -5, -10]} intensity={0.3} />

      {fitTarget && <AutoFitCamera geometry={fitTarget} />}

      <Grid
        args={[1000, 1000]}
        cellSize={5}
        cellThickness={0.4}
        cellColor="#1e2236"
        sectionSize={25}
        sectionThickness={0.8}
        sectionColor="#2a3050"
        fadeDistance={400}
        position={[0, -50, 0]}
      />

      {/* Original mesh */}
      {originalGeometry && showOriginal && (
        <mesh geometry={originalGeometry}>
          <meshStandardMaterial
            color="#4477ff"
            transparent
            opacity={(sweptGeometry || contourLoops) ? 0.15 : 1.0}
            wireframe={!!(sweptGeometry || contourLoops)}
            depthWrite={!(sweptGeometry || contourLoops)}
          />
        </mesh>
      )}

      {/* Contour cross-section — actual plane-intersection loops as 3D lines */}
      {!sweptGeometry && lineGeometries.map((geom, i) => (
        <lineLoop key={i} geometry={geom}>
          <lineBasicMaterial color="#00e5cc" />
        </lineLoop>
      ))}

      {/* Swept volume mesh */}
      {sweptGeometry && (
        <mesh geometry={sweptGeometry}>
          <meshStandardMaterial color="#ff7722" roughness={0.45} metalness={0.1} side={2} />
        </mesh>
      )}

      {/* Sweep direction arrow — only shown before step 1 */}
      {originalGeometry && !sweptGeometry && !contourLoops && (
        <arrowHelper
          args={[
            dirVec,
            arrowOrigin,
            arrowLength * 0.6,
            0x00ff99,
            arrowLength * 0.1,
            arrowLength * 0.06,
          ]}
        />
      )}

      <OrbitControls makeDefault enableDamping dampingFactor={0.08} />

      <GizmoHelper alignment="bottom-right" margin={[80, 80]}>
        <GizmoViewport />
      </GizmoHelper>
    </>
  )
}
