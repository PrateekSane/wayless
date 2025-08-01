export function createPointCloudMarkers(frame, pointSize = 2) {
  if (frame.length === 0) return [];

  const distances = frame.map((point) =>
    Math.sqrt(point.x * point.x + point.y * point.y + point.z * point.z)
  );

  const thresholds = [4, 8, 12, 16, 20, Infinity];

  if (distances.length === 0) {
    return [
      {
        points: frame,
        scale: { x: pointSize, y: pointSize, z: pointSize },
        color: { r: 0, g: 1, b: 0, a: 1 },
        pose: {
          position: { x: 0, y: 0, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      },
    ];
  }

  const bins = Array.from({ length: thresholds.length }, () => []);

  frame.forEach((point, index) => {
    const distance = distances[index];
    let binIndex = 0;
    for (let i = 0; i < thresholds.length; i++) {
      if (distance <= thresholds[i]) {
        binIndex = i;
        break;
      }
    }
    bins[binIndex].push(point);
  });

  const result = [];
  bins.forEach((points, binIndex) => {
    if (points.length > 0) {
      const normalizedDistance = binIndex / (thresholds.length - 1);
      result.push({
        points: points,
        scale: { x: pointSize, y: pointSize, z: pointSize },
        color: {
          r: normalizedDistance,
          g: 1 - normalizedDistance,
          b: 0,
          a: 1,
        },
        pose: {
          position: { x: 0, y: 0, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      });
    }
  });

  return result;
}

export function createVehicleCube() {
  return [
    {
      pose: {
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      },
      scale: { x: 3, y: 2, z: 2 },
      color: { r: 0, g: 0, b: 1, a: 1 },
    },
  ];
}