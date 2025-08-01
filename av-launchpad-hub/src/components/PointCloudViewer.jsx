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
  const [isPlaying, setIsPlaying] = useState(false);
  const [isLoading, setIsLoading] = useState(true);
  const [loadProgress, setLoadProgress] = useState(0);
  const playRef = useRef(null);

  useEffect(() => {
    let cancelled = false;
    async function loadSequentialBags() {
      const urls = Object.values(bagFiles).sort();
      console.log("Bags to load:", urls);
      const totalFiles = urls.length;

      for (let i = 0; i < urls.length; i++) {
        if (cancelled) break;
        const url = urls[i];
        console.log("⏳ loading", url);
        try {
          // load the entire bag at once
          const scans = await loadPointClouds(url);
          console.log(`✅ loaded ${scans.length} scans from`, url);
          // append all scans from this bag
          setAllSweeps((prev) => [...prev, ...scans]);
          // update progress
          setLoadProgress(((i + 1) / totalFiles) * 100);
        } catch (err) {
          console.error("❌ failed to load", url, err);
        }
      }
      
      // loading complete
      setIsLoading(false);
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

  const togglePlayPause = () => {
    if (allSweeps.length === 0) return;
    
    if (playRef.current) {
      // already playing → stop
      clearInterval(playRef.current);
      playRef.current = null;
      setIsPlaying(false);
    } else {
      // not playing → start auto‑advance
      const interval = 100;
      playRef.current = window.setInterval(() => {
        setSweepIndex((i) => {
          const nextIndex = i + 1;
          const maxFrames = Math.floor(allSweeps.length / 4);
          if (nextIndex >= maxFrames) {
            clearInterval(playRef.current);
            playRef.current = null;
            setIsPlaying(false);
            return 0; // Reset to first frame
          }
          return nextIndex;
        });
      }, interval);
      setIsPlaying(true);
    }
  };

  useEffect(() => {
    function onKeyDown(e) {
      if (e.code !== "Space") {
        return;
      }
      e.preventDefault();
      togglePlayPause();
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

    const thresholds = [4, 8, 12, 16, 20, Infinity];

    if (distances.length === 0) {
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
    const bins = Array.from({ length: thresholds.length }, () => []);

    frame.forEach((point, index) => {
      const distance = distances[index];
      let binIndex = 0;
      for (let i = 0; i < thresholds.length; i++) {
        if (distance <= thresholds[i]) {
          binIndex = i;
          break;
        }
      }
      bins[binIndex].push(point);
    });

    // Create stable marker objects
    const result = [];
    bins.forEach((points, binIndex) => {
      if (points.length > 0) {
        const normalizedDistance = binIndex / (thresholds.length - 1);
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
    <div className="relative w-full h-full flex flex-col">
      <div className="flex-1 relative">
        <Worldview
          style={{ width: "80vw", height: "80vh" }}
          // switch to a controlled camera
          cameraState={cameraState}
          // allow the user to orbit by right‑drag, WASD, etc.
          onCameraStateChange={setCameraState}
        >
          {!isLoading && markers.length > 0 && <Points>{markers}</Points>}

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
      
        {/* Loading overlay */}
        {isLoading && (
          <div className="absolute inset-0 bg-black bg-opacity-50 flex items-center justify-center">
            <div className="bg-white p-6 rounded-lg shadow-lg max-w-md w-full mx-4">
              <h3 className="text-lg font-semibold text-gray-800 mb-4">Loading Point Cloud Data</h3>
              <div className="w-full bg-gray-200 rounded-full h-4 mb-2">
                <div 
                  className="bg-blue-600 h-4 rounded-full transition-all duration-300 ease-out"
                  style={{ width: `${loadProgress}%` }}
                ></div>
              </div>
              <div className="text-sm text-gray-600 text-center">
                {Math.round(loadProgress)}% complete
              </div>
            </div>
          </div>
        )}
        
        {/* Status box */}
        <div className="absolute top-4 left-4 bg-black bg-opacity-75 text-white p-3 rounded-lg">
          <div className="text-sm space-y-1">
            <div>Points: {frame.length.toLocaleString()}</div>
            {allSweeps.length > 0 && (
              <div>{isPlaying ? "▶️ Playing" : "⏸️ Paused"}</div>
            )}
            <div className="text-xs text-gray-300 mt-2 space-y-1">
              <div>Press SPACE to play/pause</div>
              <div>Click and drag to move around</div>
              <div>Double-click to change angle</div>
            </div>
          </div>
        </div>
      </div>

      {/* Video-style Controls */}
      {!isLoading && allSweeps.length > 0 && (
        <div className="bg-gray-900 text-white p-4 flex flex-col space-y-3" style={{ width: "80vw" }}>
          {/* Progress Bar */}
          <div className="w-full bg-gray-600 rounded-full h-2 cursor-pointer">
            <div 
              className="bg-red-500 h-2 rounded-full transition-all duration-100"
              style={{ 
                width: `${((sweepIndex + 1) / Math.floor(allSweeps.length / 4)) * 100}%` 
              }}
            ></div>
          </div>
          
          {/* Controls Row */}
          <div className="flex items-center justify-between">
            <div className="flex items-center space-x-4">
              <button
                onClick={togglePlayPause}
                className="bg-transparent hover:bg-gray-700 text-white p-2 rounded-full transition-colors"
                disabled={allSweeps.length === 0}
              >
                {isPlaying ? "⏸️" : "▶️"}
              </button>
              <span className="text-sm">
                {sweepIndex + 1} / {Math.floor(allSweeps.length / 4)}
              </span>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
