import React from "react";

interface ContextSectionProps {
  youtubeUrl?: string;
  title?: string;
  description?: string;
}

const ContextSection: React.FC<ContextSectionProps> = ({
  youtubeUrl = "https://www.youtube.com/embed/H7yCXRUB-rc",
  title = "Building the Future of AV Data",
  description = "Creating a sensor mount, called a Rig, and deploying them on rideshare vehicles across the country to collect long tail AV data. It's of utmost importance to make sure AVs are safe. Achieving this requires having the data to understand the behavior of AVs in various situations.",
}) => {
  return (
    <section
      data-section="context"
      className="relative py-20 bg-card/20 backdrop-blur-sm"
    >
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
          </div>
        </div>
      </div>
    </section>
  );
};

export default ContextSection;
