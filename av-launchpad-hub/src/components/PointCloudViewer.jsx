import { useBagLoader } from "@/hooks/useBagLoader";
import { useFrameProcessor } from "@/hooks/useFrameProcessor";
import { usePlaybackControls } from "@/hooks/usePlaybackControls";
import {
  createPointCloudMarkers,
  createVehicleCube,
} from "@/lib/pointCloudUtils";
import Worldview, { Cubes, Points } from "@foxglove/regl-worldview";
import React, { useEffect, useRef, useState } from "react";
import { LoadingOverlay } from "./LoadingOverlay";
import { StatusBox } from "./StatusBox";
import { VideoControls } from "./VideoControls";

export default function PointCloudViewer() {
  const [cameraState, setCameraState] = useState({
    position: { x: 0, y: -15, z: 5 },
    lookAt: { x: 0, y: 0, z: 0 },
  });
  const [wasLoading, setWasLoading] = useState(false);
  const containerRef = useRef(null);

  const { allSweeps, isLoading, loadProgress } = useBagLoader();
  const { sweepIndex, isPlaying, togglePlayPause } =
    usePlaybackControls(allSweeps);
  const frame = useFrameProcessor(allSweeps, sweepIndex);

  const markers = React.useMemo(() => createPointCloudMarkers(frame), [frame]);
  const vehicleCube = React.useMemo(() => createVehicleCube(), []);

  // Track loading state without auto-scrolling
  useEffect(() => {
    setWasLoading(isLoading);
  }, [isLoading, wasLoading]);

  return (
    <div ref={containerRef} className="relative w-full h-full flex flex-col">
      <div className="flex-1 relative">
        {isLoading ? (
          <LoadingOverlay loadProgress={loadProgress} />
        ) : (
          <>
            <div className="text-center py-4">
              <h1 className="text-2xl font-bold text-gray-800">
                Explore a Point Cloud
              </h1>
              <h2 className="text-xl text-gray-800">
                Lombard Street, San Francisco
              </h2>
            </div>
            <div className="relative flex justify-center">
              <div
                className="relative"
                style={{ width: "80vw", maxWidth: "100%", height: "80vh" }}
              >
                <Worldview
                  style={{ width: "100%", height: "100%" }}
                  cameraState={cameraState}
                  onCameraStateChange={setCameraState}
                >
                  {markers.length > 0 && <Points>{markers}</Points>}
                  <Cubes>{vehicleCube}</Cubes>
                </Worldview>
                <StatusBox
                  allSweeps={allSweeps}
                  isPlaying={isPlaying}
                  frame={frame}
                />
              </div>
            </div>
          </>
        )}
      </div>

      {!isLoading && allSweeps.length > 0 && (
        <VideoControls
          allSweeps={allSweeps}
          sweepIndex={sweepIndex}
          isPlaying={isPlaying}
          togglePlayPause={togglePlayPause}
        />
      )}
    </div>
  );
}
