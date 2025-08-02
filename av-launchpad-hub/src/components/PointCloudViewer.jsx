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

  // Handle smooth scroll when loading completes
  useEffect(() => {
    if (wasLoading && !isLoading) {
      // Wait 1 second for content to render before scrolling
      setTimeout(() => {
        window.scrollTo({
          top: document.body.scrollHeight,
          behavior: "smooth",
        });
      }, 300);
    }
    setWasLoading(isLoading);
  }, [isLoading, wasLoading]);

  return (
    <div ref={containerRef} className="relative w-full h-full flex flex-col">
      <div className="flex-1 relative">
        {isLoading ? (
          <LoadingOverlay loadProgress={loadProgress} />
        ) : (
          <div>
            <Worldview
              style={{ width: "80vw", height: "80vh" }}
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
