import React, { useEffect, useRef, useState } from "react";
// switch to the maintained Foxglove fork if you can:
import Worldview, { Axes, Cubes, Points } from "@foxglove/regl-worldview";
// (or) import from "@foxglove/regl-worldview" if you’ve installed that
import { loadPointClouds } from "./bagLoader";

export default function PointCloudViewer() {
  const [scans, setScans] = useState([]);
  const [frame, setFrame] = useState(0);
  const playRef = useRef(null);

  // Controlled camera state
  const [cameraState, setCameraState] = useState({
    // 15 m back on Y, 5 m up on Z
    position: { x: 0, y: -15, z: 5 },
    lookAt: { x: 0, y: 0, z: 0 },
  });

  // load bag → scans
  useEffect(() => {
    loadPointClouds("/velo_21.bag")
      .then((loaded) => {
        console.log(`✅ Loaded ${loaded.length} scans`);
        if (loaded[0]?.length) {
          console.log("   first point:", loaded);
        }
        setScans(loaded);
      })
      .catch(console.error);
  }, []);

  // spacebar play/pause
  useEffect(() => {
    function onKey(e) {
      if (e.code === "Space") {
        if (playRef.current) {
          clearInterval(playRef.current);
          playRef.current = null;
        } else {
          playRef.current = setInterval(
            () => setFrame((f) => (f + 1) % scans.length),
            200
          );
        }
      }
    }
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [scans]);

  // Log camera state so you can tweak it
  useEffect(() => {
    console.log("CameraState:", cameraState);
  }, [cameraState]);

  return (
    <Worldview
      style={{ width: "100vw", height: "100vh" }}
      backgroundColor={{ r: 0, g: 0, b: 0, a: 1 }}
      // switch to a controlled camera
      cameraState={cameraState}
      // allow the user to orbit by right‑drag, WASD, etc.
      onCameraStateChange={setCameraState}
    >
      {/* 1. Axes triad (X=red, Y=green, Z=blue) */}
      <Axes length={5} />

      {/* 2. Static test point at origin (red, size 5) */}
      <Points
        pointSize={5}
        points={[
          {
            position: { x: 0, y: 0, z: 0 },
            color: { r: 1, g: 0, b: 0, a: 1 },
          },
        ]}
      />

      {/* 3. Your dynamic LiDAR scan (green points) */}
      {scans[frame] && (
        <Points
          pointSize={0.1}
          points={scans[frame].map((p) => ({
            position: p,
            color: { r: 0, g: 1, b: 0, a: 1 },
          }))}
        />
      )}

      {/* 4. Ego‑car cube at origin (semi‑transparent blue) */}
      <Cubes>
        {[
          {
            pose: {
              position: { x: 0, y: 0, z: 0 },
              orientation: { x: 0, y: 0, z: 0, w: 1 },
            },
            scale: { x: 1.5, y: 1.5, z: 0.5 },
            color: { r: 0, g: 0, b: 1, a: 0.2 },
          },
        ]}
      </Cubes>
      <Points
        pointSize={5}
        points={[
          {
            position: { x: 0, y: 0, z: 1 }, // lifted above cube
            color: { r: 1, g: 0, b: 0, a: 1 },
          },
        ]}
      />
    </Worldview>
  );
}
