import { loadPointClouds } from "@/lib/bagLoader";
import Worldview, { Cubes, Points } from "@foxglove/regl-worldview";
import React, { useEffect, useRef, useState } from "react";

const bagFiles = import.meta.glob("../assets/*.bag", {
  query: "?url",
  import: "default",
  eager: true,
});

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
    let cancelled = false;
    async function loadSequentialBags() {
      const urls = Object.values(bagFiles).sort();
      console.log("Bags to load:", urls);

      for (const url of urls) {
        if (cancelled) break;
        console.log("⏳ loading", url);
        try {
          // load the entire bag at once
          const scans = await loadPointClouds(url);
          console.log(`✅ loaded ${scans.length} scans from`, url);
          // append all scans from this bag
          setAllSweeps((prev) => [...prev, ...scans]);
        } catch (err) {
          console.error("❌ failed to load", url, err);
        }
      }
    }

    loadSequentialBags();
    return () => {
      cancelled = true;
    };
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

  // Points handling with distance-based coloring
  const pointSize = 2;

  // Simplified approach - use fewer bins and stable objects
  const markers = React.useMemo(() => {
    if (frame.length === 0) return [];

    // Use fewer bins to reduce complexity
    const numBins = 5;

    // Calculate distances
    const distances = frame.map((point) =>
      Math.sqrt(point.x * point.x + point.y * point.y + point.z * point.z)
    );

    const minDistance = Math.min(...distances);
    const maxDistance = Math.max(...distances);
    const distanceRange = maxDistance - minDistance;

    if (distanceRange === 0) {
      return [
        {
          points: frame,
          scale: { x: pointSize, y: pointSize, z: pointSize },
          color: { r: 0, g: 1, b: 0, a: 1 },
          pose: {
            position: { x: 0, y: 0, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
          },
        },
      ];
    }

    // Group points into bins
    const bins = Array.from({ length: numBins }, () => []);

    frame.forEach((point, index) => {
      const distance = distances[index];
      let normalizedDistance = (distance - minDistance) / distanceRange;
      // Start the green->red transition earlier by compressing the range
      normalizedDistance = Math.min(normalizedDistance * 2, 1.0);
      const binIndex = Math.min(
        Math.floor(normalizedDistance * numBins),
        numBins - 1
      );
      bins[binIndex].push(point);
    });

    // Create stable marker objects
    const result = [];
    bins.forEach((points, binIndex) => {
      if (points.length > 0) {
        const normalizedDistance = binIndex / (numBins - 1);
        result.push({
          points: points,
          scale: { x: pointSize, y: pointSize, z: pointSize },
          color: {
            r: normalizedDistance,
            g: 1 - normalizedDistance,
            b: 0,
            a: 1,
          },
          pose: {
            position: { x: 0, y: 0, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
          },
        });
      }
    });

    return result;
  }, [frame]);

  return (
    <Worldview
      style={{ width: "80vw", height: "80vh" }}
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
