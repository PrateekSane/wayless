// distanceColorUtils.js

// Define discrete color buckets
const COLOR_BUCKETS = [
  { r: 0, g: 1, b: 0, a: 1 }, // Green - closest
  { r: 0.5, g: 1, b: 0, a: 1 }, // Yellow-green
  { r: 1, g: 1, b: 0, a: 1 }, // Yellow
  { r: 1, g: 0.5, b: 0, a: 1 }, // Orange
  { r: 1, g: 0, b: 0, a: 1 }, // Red - farthest
];

/**
 * Calculate squared distance from origin (avoids expensive sqrt)
 */
export function calculateSquaredDistance(point) {
  return point.x * point.x + point.y * point.y + point.z * point.z;
}

/**
 * Get discrete color bucket index based on normalized squared distance
 */
export function getColorBucketIndex(normalizedSquaredDistance) {
  // Start the transition earlier by compressing the range
  const adjustedDistance = Math.min(normalizedSquaredDistance * 2.5, 1.0);
  return Math.min(
    Math.floor(adjustedDistance * COLOR_BUCKETS.length),
    COLOR_BUCKETS.length - 1
  );
}

/**
 * Group points into discrete distance/color buckets using squared distances
 */
export function groupPointsByDistance(points) {
  if (points.length === 0) return [];

  // Calculate squared distances (no sqrt needed!)
  const squaredDistances = points.map(calculateSquaredDistance);
  const minSquaredDistance = Math.min(...squaredDistances);
  const maxSquaredDistance = Math.max(...squaredDistances);
  const squaredDistanceRange = maxSquaredDistance - minSquaredDistance;

  // Handle case where all points are same distance
  if (squaredDistanceRange === 0) {
    return [
      {
        points: points,
        color: COLOR_BUCKETS[0], // Use closest color (green)
      },
    ];
  }

  // Group points into buckets
  const buckets = Array.from({ length: COLOR_BUCKETS.length }, () => []);

  points.forEach((point, index) => {
    const squaredDistance = squaredDistances[index];
    const normalizedSquaredDistance =
      (squaredDistance - minSquaredDistance) / squaredDistanceRange;
    const bucketIndex = getColorBucketIndex(normalizedSquaredDistance);
    buckets[bucketIndex].push(point);
  });

  // Return only non-empty buckets with their colors
  return buckets
    .map((points, bucketIndex) => {
      if (points.length === 0) return null;
      return {
        points: points,
        color: COLOR_BUCKETS[bucketIndex],
      };
    })
    .filter(Boolean);
}

/**
 * Create markers for worldview rendering
 */
export function createDistanceColorMarkers(pointGroups, pointSize = 2) {
  return pointGroups.map((group) => ({
    points: group.points,
    scale: { x: pointSize, y: pointSize, z: pointSize },
    color: group.color,
    pose: {
      position: { x: 0, y: 0, z: 0 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    },
  }));
}
