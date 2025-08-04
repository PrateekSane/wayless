import React from "react";

interface ContextSectionProps {
  youtubeUrl?: string;
  title?: string;
  description?: string;
}

const ContextSection: React.FC<ContextSectionProps> = ({
  youtubeUrl = "https://www.youtube.com/embed/dQw4w9WgXcQ",
  title = "Building the Future of AV Data",
  description = "Our platform revolutionizes how autonomous vehicle teams collect, process, and analyze sensor data. With advanced point cloud visualization and real-time processing capabilities, we're enabling the next generation of self-driving technology through comprehensive data infrastructure solutions."
}) => {
  return (
    <section className="relative py-20 bg-card/20 backdrop-blur-sm">
      <div className="max-w-7xl mx-auto px-6">
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-12 items-center">
          {/* Left side - YouTube video */}
          <div className="relative">
            <div className="aspect-video rounded-lg overflow-hidden shadow-2xl border border-border/20">
              <iframe
                src={youtubeUrl}
                title="AV Platform Overview"
                className="w-full h-full"
                allowFullScreen
                allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
              />
            </div>
          </div>

          {/* Vertical divider */}
          <div className="hidden lg:block absolute left-1/2 top-20 bottom-20 w-px bg-gradient-to-b from-transparent via-border to-transparent transform -translate-x-1/2" />

          {/* Right side - Text content */}
          <div className="space-y-6">
            <h2 className="text-4xl font-bold bg-gradient-primary bg-clip-text text-transparent">
              {title}
            </h2>
            <p className="text-lg text-muted-foreground leading-relaxed">
              {description}
            </p>
            <div className="pt-4">
              <div className="inline-flex items-center gap-2 bg-primary/10 border border-primary/20 rounded-full px-4 py-2">
                <div className="w-2 h-2 bg-primary rounded-full animate-pulse" />
                <span className="text-sm text-primary font-medium">
                  Live Data Processing
                </span>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
};

export default ContextSection;