import ProgressBar from "./ProgressBar";

export function LoadingOverlay({ loadProgress }) {
  return (
    <div className="absolute inset-0 bg-black bg-opacity-50 flex items-center justify-center">
      <div className="bg-white p-6 rounded-lg shadow-lg max-w-md w-full mx-4">
        <ProgressBar
          progress={loadProgress}
          title="Loading Point Cloud Data"
          description="Please wait while we process the data..."
          showPercentage={true}
          className="w-full"
        />
      </div>
    </div>
  );
}
