import { useEffect, useRef, useState } from "react";

export function usePlaybackControls(allSweeps) {
  const [sweepIndex, setSweepIndex] = useState(0);
  const [isPlaying, setIsPlaying] = useState(false);
  const playRef = useRef(null);

  const togglePlayPause = () => {
    if (allSweeps.length === 0) return;

    if (playRef.current) {
      clearInterval(playRef.current);
      playRef.current = null;
      setIsPlaying(false);
    } else {
      const interval = 100;
      playRef.current = window.setInterval(() => {
        setSweepIndex((i) => {
          const nextIndex = i + 1;
          const maxFrames = Math.floor(allSweeps.length / 4);
          if (nextIndex >= maxFrames) {
            clearInterval(playRef.current);
            playRef.current = null;
            setIsPlaying(false);
            return 0;
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

  return {
    sweepIndex,
    setSweepIndex,
    isPlaying,
    togglePlayPause,
  };
}
