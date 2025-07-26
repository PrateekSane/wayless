import Worldview, { Cubes, Points } from "@foxglove/regl-worldview";
import React, { useEffect, useRef, useState } from "react";
import { loadPointClouds } from "./bagLoader";
import {
  createDistanceColorMarkers,
  groupPointsByDistance,
} from "./distanceColorUtils";

export default function PointCloudViewer() {
  // camera methods
  const [cameraState, setCameraState] = useState({
    // 15 m back on Y, 5 m up on Z
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
    const interval = 100;
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
        }, interval);
      }
    }

    window.addEventListener("keydown", onKeyDown);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      if (playRef.current) clearInterval(playRef.current);
    };
  }, [allSweeps.length]);

  // Points handling with discrete distance-based coloring
  const pointSize = 2;

  const markers = React.useMemo(() => {
    if (frame.length === 0) return [];

    const pointGroups = groupPointsByDistance(frame);
    return createDistanceColorMarkers(pointGroups, pointSize);
  }, [frame]);

  return (
    <Worldview
      style={{ width: "100vw", height: "100vh" }}
      // switch to a controlled camera
      cameraState={cameraState}
      // allow the user to orbit by right‑drag, WASD, etc.
      onCameraStateChange={setCameraState}
    >
      {markers.length > 0 && <Points>{markers}</Points>}

      <Cubes>
        {[
          {
            pose: {
              position: { x: 0, y: 0, z: 0 },
              orientation: { x: 0, y: 0, z: 0, w: 1 },
            },
            scale: { x: 3, y: 2, z: 2 },
            color: { r: 0, g: 0, b: 1, a: 1 },
          },
        ]}
      </Cubes>
    </Worldview>
  );
}
