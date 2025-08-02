import React from "react";

export function ProgressBar({ 
  progress, 
  title = "Loading...", 
  description,
  showPercentage = true,
  className = "",
  barClassName = ""
}) {
  return (
    <div className={`w-full space-y-2 ${className}`}>
      {title && (
        <div className="text-center space-y-1">
          <h3 className="text-lg font-medium text-foreground">{title}</h3>
          {description && (
            <p className="text-sm text-muted-foreground">{description}</p>
          )}
        </div>
      )}
      
      <div className={`w-full bg-secondary rounded-full h-3 ${barClassName}`}>
        <div
          className="bg-primary h-3 rounded-full transition-all duration-300 ease-out"
          style={{ width: `${Math.max(0, Math.min(100, progress))}%` }}
        />
      </div>
      
      {showPercentage && (
        <div className="text-center text-sm text-muted-foreground">
          {Math.round(progress)}% complete
        </div>
      )}
    </div>
  );
}

export function LoadingScreen({ 
  progress, 
  title = "Loading...", 
  description,
  className = ""
}) {
  return (
    <div className={`flex flex-col items-center justify-center min-h-[80vh] space-y-6 ${className}`}>
      <ProgressBar 
        progress={progress}
        title={title}
        description={description}
        className="w-96"
      />
    </div>
  );
}