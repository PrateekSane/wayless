export function VideoControls({ allSweeps, sweepIndex, isPlaying, togglePlayPause }) {
  const maxFrames = Math.floor(allSweeps.length / 4);
  const progressPercentage = ((sweepIndex + 1) / maxFrames) * 100;

  return (
    <div className="bg-gray-900 text-white p-4 flex flex-col space-y-3" style={{ width: "80vw" }}>
      <div className="w-full bg-gray-600 rounded-full h-2 cursor-pointer">
        <div 
          className="bg-red-500 h-2 rounded-full transition-all duration-100"
          style={{ width: `${progressPercentage}%` }}
        ></div>
      </div>
      
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
            {sweepIndex + 1} / {maxFrames}
          </span>
        </div>
      </div>
    </div>
  );
}