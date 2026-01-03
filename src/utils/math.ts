import { Quaternion, Vector3 } from 'three';

export function vec3FromArray(values: number[], target = new Vector3()): Vector3 {
  return target.set(values[0] ?? 0, values[1] ?? 0, values[2] ?? 0);
}

export function axisAngleQuaternion(
  axis: number[],
  angle: number,
  target = new Quaternion()
): Quaternion {
  const axisVec = vec3FromArray(axis);
  if (axisVec.lengthSq() === 0) {
    return target.identity();
  }
  axisVec.normalize();
  return target.setFromAxisAngle(axisVec, angle);
}

export function clamp(value: number, min?: number, max?: number): number {
  if (min !== undefined && value < min) return min;
  if (max !== undefined && value > max) return max;
  return value;
}

export function toArray(vec: Vector3): [number, number, number] {
  return [vec.x, vec.y, vec.z];
}
