export function LoadingOverlay({ loadProgress }) {
  return (
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
  );
}