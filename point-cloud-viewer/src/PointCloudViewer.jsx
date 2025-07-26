import Worldview, { Axes, Cubes, Points } from "@foxglove/regl-worldview";
import React, { KeyboardEvent, useEffect, useRef, useState } from "react";
import { loadPointClouds } from "./bagLoader";

export default function PointCloudViewer() {
  // camera methods
  const [cameraState, setCameraState] = useState({
    // 15 m back on Y, 5 m up on Z
    position: { x: 0, y: -15, z: 5 },
    lookAt: { x: 0, y: 0, z: 0 },
  });

  const [allSweeps, setAllSweeps] = useState([]);
  const [sweepIndex, setSweepIndex] = useState(0);
  const playRef = useRef(null);

  useEffect(() => {
    async function loadAndChunk() {
      const loaded = await loadPointClouds("/velo_21.bag");
      setAllSweeps(loaded);
    }
    loadAndChunk();
  }, []);

  // Frame handling
  const [frame, setFrame] = useState([]);
  useEffect(() => {
    async function combineFrames() {
      // num frames to combine
      const chunkSize = 4;
      try {
        // take the first `chunkSize` frames and flatten into one big array
        const curFrame = allSweeps
          .slice(sweepIndex * chunkSize, (sweepIndex + 1) * chunkSize)
          .flat();

        setFrame(curFrame);
      } catch (err) {
        console.error("Failed to load point clouds:", err);
      }
    }

    combineFrames();
  }, [allSweeps, sweepIndex]);

  useEffect(() => {
    function onKeyDown(e) {
      if (e.code !== "Space" || allSweeps.length === 0) {
        return;
      }
      if (playRef.current) {
        // already playing → stop
        clearInterval(playRef.current);
        playRef.current = null;
      } else {
        // not playing → start auto‑advance
        playRef.current = window.setInterval(() => {
          setSweepIndex((i) => (i + 1) % allSweeps.length);
        }, 200); // 5 Hz
      }
    }

    window.addEventListener("keydown", onKeyDown);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      if (playRef.current) clearInterval(playRef.current);
    };
  }, [allSweeps.length]);

  // Points handling
  const pointSize = 5;
  const marker = {
    points: frame,
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

      {frame.length > 0 && <Points>{[marker]}</Points>}

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
