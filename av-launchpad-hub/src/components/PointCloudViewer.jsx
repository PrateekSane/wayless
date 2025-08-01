import Worldview, { Cubes, Points } from "@foxglove/regl-worldview";
import React, { useState } from "react";
import { useBagLoader } from "@/hooks/useBagLoader";
import { useFrameProcessor } from "@/hooks/useFrameProcessor";
import { usePlaybackControls } from "@/hooks/usePlaybackControls";
import { createPointCloudMarkers, createVehicleCube } from "@/lib/pointCloudUtils";
import { LoadingOverlay } from "./LoadingOverlay";
import { StatusBox } from "./StatusBox";
import { VideoControls } from "./VideoControls";

export default function PointCloudViewer() {
  const [cameraState, setCameraState] = useState({
    position: { x: 0, y: -15, z: 5 },
    lookAt: { x: 0, y: 0, z: 0 },
  });

  const { allSweeps, isLoading, loadProgress } = useBagLoader();
  const { sweepIndex, isPlaying, togglePlayPause } = usePlaybackControls(allSweeps);
  const frame = useFrameProcessor(allSweeps, sweepIndex);

  const markers = React.useMemo(() => createPointCloudMarkers(frame), [frame]);
  const vehicleCube = React.useMemo(() => createVehicleCube(), []);

  return (
    <div className="relative w-full h-full flex flex-col">
      <div className="flex-1 relative">
        <Worldview
          style={{ width: "80vw", height: "80vh" }}
          cameraState={cameraState}
          onCameraStateChange={setCameraState}
        >
          {!isLoading && markers.length > 0 && <Points>{markers}</Points>}
          <Cubes>{vehicleCube}</Cubes>
        </Worldview>
      
        {isLoading && <LoadingOverlay loadProgress={loadProgress} />}
        <StatusBox allSweeps={allSweeps} isPlaying={isPlaying} frame={frame} />
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
