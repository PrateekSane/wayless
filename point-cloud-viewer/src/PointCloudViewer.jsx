import Worldview, { Axes, Cubes, Points } from "@foxglove/regl-worldview";
import React, { useEffect, useRef, useState } from "react";
import { loadPointClouds } from "./bagLoader";

export default function PointCloudViewer() {
  const pointSize = 50;

  const [firstFrame, setFirstFrame] = useState([]);

  // load bag → scans
  useEffect(() => {
    loadPointClouds("/velo_21.bag")
      .then((loaded) => {
        if (loaded.length > 0) {
          setFirstFrame([loaded[0][0]]);
          console.log("APPLES BANANAS ", [loaded[0][0]]);
        }
      })
      .catch(console.error);
  }, []);

  const [cameraState, setCameraState] = useState({
    // 15 m back on Y, 5 m up on Z
    position: { x: 0, y: -15, z: 5 },
    lookAt: { x: 0, y: 0, z: 0 },
  });

  const marker = {
    points: firstFrame,
    scale: { x: pointSize, y: pointSize, z: pointSize },
    color: { r: 0, g: 1, b: 0, a: 1 }, // make them green
    pose: {
      position: { x: 0, y: 0, z: 0 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    },
  };

  return (
    <Worldview
      style={{ width: "100vw", height: "100vh" }}
      // switch to a controlled camera
      cameraState={cameraState}
      // allow the user to orbit by right‑drag, WASD, etc.
      onCameraStateChange={setCameraState}
    >
      <Axes length={10} />

      {/*firstFrame.length > 0 && (
        <Points points={[marker]} />
      )*/}

      {firstFrame.length > 0 && <Points>{[marker]}</Points>}

      <Cubes>
        {[
          {
            pose: {
              position: { x: 0, y: 0, z: 0 },
              orientation: { x: 0, y: 0, z: 0, w: 1 },
            },
            scale: { x: 1, y: 1, z: 1 },
            color: { r: 0, g: 0, b: 1, a: 0.2 },
          },
        ]}
      </Cubes>
    </Worldview>
  );
}

/*
      {scans[frame] && (
        <Points
          pointSize={0.1}
          points={scans[frame].map((p) => ({
            position: p,
            color: { r: 0, g: 1, b: 0, a: 1 },
          }))}
        />
      )}
        */

/*
  const [scans, setScans] = useState([]);
  const [frame, setFrame] = useState(0);
  const playRef = useRef(null);

  // Controlled camera state



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
*/
