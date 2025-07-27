import { Button } from "@/components/ui/button";
import { Database } from "lucide-react";

const Navigation = () => {
  return (
    <nav className="fixed top-0 left-0 right-0 z-50 p-6">
      <div className="flex items-center justify-between max-w-7xl mx-auto">
        <div className="text-2xl font-bold bg-gradient-primary bg-clip-text text-transparent">
          Rig
        </div>
        
        <Button 
          variant="outline" 
          className="gap-2 border-border/50 bg-card/50 backdrop-blur-sm hover:bg-card/80 hover:shadow-glow transition-all duration-300"
        >
          <Database size={16} />
          Data Schema & Collection Setup
        </Button>
      </div>
    </nav>
  );
};

export default Navigation;