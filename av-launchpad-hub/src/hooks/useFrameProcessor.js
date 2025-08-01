import { useEffect, useState } from "react";

export function useFrameProcessor(allSweeps, sweepIndex) {
  const [frame, setFrame] = useState([]);

  useEffect(() => {
    async function combineFrames() {
      const chunkSize = 4;
      try {
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

  return frame;
}