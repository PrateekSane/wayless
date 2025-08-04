import { Button } from "@/components/ui/button";
import { ArrowRight, Zap } from "lucide-react";

const HeroSection = () => {
  const scrollToContext = () => {
    const contextSection = document.querySelector('[data-section="context"]');
    if (contextSection) {
      contextSection.scrollIntoView({ behavior: "smooth", block: "start" });
    }
  };

  const scrollToBottom = () => {
    const scrollHeight = document.body.scrollHeight;
    const windowHeight = window.innerHeight;
    const targetPosition = scrollHeight - windowHeight - 100; // Stop 100px above the actual bottom
    window.scrollTo({ top: targetPosition, behavior: "smooth" });
  };

  return (
    <section className="min-h-screen flex items-center justify-center relative overflow-hidden">
      {/* Background effects */}
      <div className="absolute inset-0 bg-gradient-glow opacity-20" />
      <div className="absolute inset-0">
        <div className="absolute top-1/4 left-1/4 w-96 h-96 bg-primary/10 rounded-full blur-3xl animate-pulse" />
        <div className="absolute bottom-1/4 right-1/4 w-80 h-80 bg-accent/10 rounded-full blur-3xl animate-pulse delay-1000" />
      </div>

      {/* Grid pattern overlay */}
      <div
        className="absolute inset-0 opacity-[0.03]"
        style={{
          backgroundImage: `
            linear-gradient(hsl(200 100% 50% / 0.1) 1px, transparent 1px),
            linear-gradient(90deg, hsl(200 100% 50% / 0.1) 1px, transparent 1px)
          `,
          backgroundSize: "50px 50px",
        }}
      />

      <div className="relative z-10 text-center max-w-4xl mx-auto px-6">
        <div className="inline-flex items-center gap-2 bg-secondary/50 backdrop-blur-sm border border-border/50 rounded-full px-4 py-2 mb-8">
          <Zap size={16} className="text-primary" />
          <span className="text-sm text-muted-foreground">
            Next-generation AV data platform
          </span>
        </div>

        <h1 className="text-6xl md:text-8xl font-bold mb-6 leading-tight">
          <span className="bg-gradient-primary bg-clip-text text-transparent">
            The future of
          </span>
          <br />
          <span className="text-foreground">AV data is here</span>
        </h1>

        <p className="text-xl text-muted-foreground mb-12 max-w-2xl mx-auto leading-relaxed">
          Revolutionary data infrastructure for long-tail autonomous vehicle
          development.
        </p>

        <div className="flex flex-col sm:flex-row gap-4 justify-center">
          <Button
            size="lg"
            className="bg-gradient-primary hover:shadow-glow transition-all duration-300 group"
            onClick={scrollToContext}
          >
            See a Rig
            <ArrowRight
              size={16}
              className="ml-2 group-hover:translate-x-1 transition-transform"
            />
          </Button>
          <Button
            size="lg"
            variant="outline"
            className="border-border/50 bg-card/20 backdrop-blur-sm hover:bg-card/40 hover:border-primary/50 transition-all duration-300"
            onClick={scrollToBottom}
          >
            Explore a Point Cloud
          </Button>
        </div>
      </div>
    </section>
  );
};

export default HeroSection;
