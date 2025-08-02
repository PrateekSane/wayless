import React from "react";

export default function ProgressBar({
  progress,
  title = "Loading...",
  description,
  showPercentage = true,
  className = "",
  barClassName = "",
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
