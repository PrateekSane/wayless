import { useEffect, useState } from "react";
import { open } from "rosbag";
import bagsManifest from "../bags.json";

const base = import.meta.env.VITE_BLOB_BASE_URL;
const manifestPath = import.meta.env.VITE_BAG_MANIFEST_PATH;

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
    let urls;

    const mode = import.meta.env.MODE;
    if (mode === "development") {
      // local files in dev
      urls = Object.values(
        import.meta.glob("../assets/*.bag", {
          query: "?url",
          import: "default",
          eager: true,
        })
      );
    } else if (mode === "production") {
      // blobs in prod
      const base = import.meta.env.VITE_BLOB_BASE_URL;
      urls = bagsManifest.map((name) => `${base}/${name}`);
    } else {
      throw new Error(`Unsupported MODE: ${mode}`);
    }

    urls.sort();
    console.log("Bags to load:", urls);

    async function loadSequentialBags() {
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

async function loadPointClouds(url) {
  const res = await fetch(url);
  const blob = await res.blob();
  const bag = await open(blob);

  const scans = [];
  await bag.readMessages({ topics: ["/velodyne_points"] }, ({ message }) => {
    const { data, point_step, is_bigendian } = message;
    // view only the payload bytes
    const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
    const littleEndian = !is_bigendian;
    const count = Math.floor(data.byteLength / point_step);
    const pts = [];
    for (let i = 0; i < count; i++) {
      const base = i * point_step;
      const x = view.getFloat32(base + 0, littleEndian);
      const y = view.getFloat32(base + 4, littleEndian);
      const z = view.getFloat32(base + 8, littleEndian);
      pts.push({ x, y, z });
    }
    scans.push(pts);
  });
  return scans;
}
