import { open } from "rosbag";

export async function loadPointClouds(url) {
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
