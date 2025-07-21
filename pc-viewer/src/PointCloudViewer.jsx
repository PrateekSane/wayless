import { OrbitControls } from "@react-three/drei";
import { Canvas, useLoader } from "@react-three/fiber";
import React, { Suspense, useMemo } from "react";
import * as THREE from "three";
import { PCDLoader } from "three/examples/jsm/loaders/PCDLoader";

function ColoredPCDScene() {
  // load the raw PCD points object
  const points = useLoader(PCDLoader, "/models/mycloud.pcd");

  // once loaded, create a new geometry with per‑vertex colors
  const coloredGeom = useMemo(() => {
    const geom = points.geometry.clone();
    const pos = geom.attributes.position.array;
    const N = pos.length / 3;

    // compute max distance (for normalization)
    geom.computeBoundingSphere();
    const maxDist = geom.boundingSphere.radius || 1;

    // build a color array
    const colors = new Float32Array(N * 3);
    for (let i = 0; i < N; i++) {
      const x = pos[i * 3 + 0];
      const y = pos[i * 3 + 1];
      const z = pos[i * 3 + 2];
      const d = Math.sqrt(x * x + y * y + z * z);
      const t = THREE.MathUtils.clamp(d / maxDist, 0, 1);
      // hue: 0.7→0.0 as distance goes 0→max (blue→red)
      const c = new THREE.Color().setHSL((1 - t) * 0.7, 1.0, 0.5);
      colors.set(c.toArray(), i * 3);
    }
    geom.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    return geom;
  }, [points]);

  return (
    <points geometry={coloredGeom}>
      <pointsMaterial vertexColors size={0.05} sizeAttenuation />
    </points>
  );
}

export default function PointCloudViewer() {
  return (
    // make the canvas fill the viewport
    <div
      style={{ width: "100vw", height: "100vh", margin: 0, overflow: "hidden" }}
    >
      <Canvas
        style={{ width: "100%", height: "100%" }}
        camera={{ position: [0, 0, 10], fov: 75 }}
      >
        <ambientLight intensity={0.5} />
        <Suspense fallback={null}>
          <ColoredPCDScene />
        </Suspense>
        <OrbitControls enablePan enableZoom enableRotate />
      </Canvas>
    </div>
  );
}
