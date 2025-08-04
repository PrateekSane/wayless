export function StatusBox({ allSweeps, isPlaying, frame }) {
  return (
    <div className="absolute top-4 left-4 bg-black bg-opacity-75 text-white p-3 rounded-lg">
      <div className="text-sm space-y-1">
        {allSweeps.length > 0 && (
          <div>{isPlaying ? "▶️ Playing" : "⏸️ Paused"}</div>
        )}
        <div>Points: {frame.length.toLocaleString()}</div>
        <div className="text-xs text-gray-300 mt-2 space-y-1">
          <div>Press SPACE to play/pause</div>
          <div>Click and drag to move around</div>
          <div>Use two fingers/double-click to change angle</div>
        </div>
      </div>
    </div>
  );
}
