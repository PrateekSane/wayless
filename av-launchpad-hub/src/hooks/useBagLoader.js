import { useEffect, useState } from "react";
import { open } from "rosbag";
import bagsManifest from "../bags.json";

export function useBagLoader() {
  const [allSweeps, setAllSweeps] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  const [loadProgress, setLoadProgress] = useState(0);

  useEffect(() => {
    let cancelled = false;

    // normalize base URL (no trailing slash)
    const rawBase = import.meta.env.VITE_BLOB_BASE_URL;
    const base = rawBase.replace(/\/+$/, "");

    // pick URLs based on mode
    let urls;
    if (import.meta.env.MODE === "development") {
      console.log("dev mode");
      urls = Object.values(
        import.meta.glob("../assets/*.bag", {
          query: "?url",
          import: "default",
          eager: true,
        })
      );
    } else if (import.meta.env.MODE === "production") {
      console.log("prod mode");
      urls = bagsManifest.map((name) => `${base}/${name}`);
    } else {
      throw new Error(`Unsupported MODE: ${import.meta.env.MODE}`);
    }

    urls.sort();

    async function loadParallelBags() {
      const total = urls.length;
      let completed = 0;

      const results = await Promise.all(
        urls.map(async (url, index) => {
          if (cancelled) return { index, scans: [] };
          console.log("⏳ loading", url);
          try {
            const scans = await loadPointClouds(url);
            console.log(`✅ loaded ${scans.length} scans from`, url);
            return { index, scans };
          } catch (err) {
            console.error("❌ failed to load", url, err);
            return { index, scans: [] };
          } finally {
            completed += 1;
            setLoadProgress((completed / total) * 100);
          }
        })
      );

      if (!cancelled) {
        // Flatten all scans in the correct order
        const orderedScans = results.flatMap((result) => result.scans);
        setAllSweeps(orderedScans);
        setIsLoading(false);
      }
    }

    loadParallelBags();

    return () => {
      cancelled = true;
    };
  }, []);

  return { allSweeps, isLoading, loadProgress };
}

async function loadPointClouds(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`HTTP ${res.status} fetching ${url}`);
  const blob = await res.blob();
  console.log(res);
  console.log("[loadPointClouds]", url, blob.type, blob.size);
  const bag = await open(blob);

  const scans = [];
  await bag.readMessages({ topics: ["/velodyne_points"] }, ({ message }) => {
    const { data, point_step, is_bigendian } = message;
    const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
    const le = !is_bigendian;
    const pts = [];
    for (let i = 0; i < data.byteLength / point_step; i++) {
      const b = i * point_step;
      pts.push({
        x: view.getFloat32(b + 0, le),
        y: view.getFloat32(b + 4, le),
        z: view.getFloat32(b + 8, le),
      });
    }
    scans.push(pts);
  });

  return scans;
}
