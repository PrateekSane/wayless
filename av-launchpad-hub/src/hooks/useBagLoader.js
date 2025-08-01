import { loadPointClouds } from "@/lib/bagLoader";
import { useEffect, useState } from "react";

const bagFiles = import.meta.glob("../assets/*.bag", {
  query: "?url",
  import: "default",
  eager: true,
});

export function useBagLoader() {
  const [allSweeps, setAllSweeps] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  const [loadProgress, setLoadProgress] = useState(0);

  useEffect(() => {
    let cancelled = false;
    
    async function loadSequentialBags() {
      const urls = Object.values(bagFiles).sort();
      console.log("Bags to load:", urls);
      const totalFiles = urls.length;

      for (let i = 0; i < urls.length; i++) {
        if (cancelled) break;
        const url = urls[i];
        console.log("⏳ loading", url);
        try {
          const scans = await loadPointClouds(url);
          console.log(`✅ loaded ${scans.length} scans from`, url);
          setAllSweeps((prev) => [...prev, ...scans]);
          setLoadProgress(((i + 1) / totalFiles) * 100);
        } catch (err) {
          console.error("❌ failed to load", url, err);
        }
      }
      
      setIsLoading(false);
    }

    loadSequentialBags();
    return () => {
      cancelled = true;
    };
  }, []);

  return { allSweeps, isLoading, loadProgress };
}