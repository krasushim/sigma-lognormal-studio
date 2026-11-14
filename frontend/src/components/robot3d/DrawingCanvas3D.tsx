import { useMemo } from "react";
import { Quaternion, Vector3 } from "three";
import type { CanvasMappingParams } from "../../math/canvasMapping";


interface Props {
  mapping: CanvasMappingParams;
}

/**
 * 描画用の仮想キャンバス (平板) を配置する。
 * Three.js の PlaneGeometry は初期状態で法線 +Z を向くので、指定法線に合わせて回転する。
 */
export function DrawingCanvas3D({ mapping }: Props) {
  const quat = useMemo(() => {
    const defaultNormal = new Vector3(0, 0, 1);
    const target = new Vector3(
      mapping.canvasNormal[0], mapping.canvasNormal[1], mapping.canvasNormal[2],
    ).normalize();
    return new Quaternion().setFromUnitVectors(defaultNormal, target);
  }, [mapping.canvasNormal]);

  return (
    <group
      position={mapping.canvasCenter}
      quaternion={[quat.x, quat.y, quat.z, quat.w]}
    >
      {/* Front canvas surface */}
      <mesh receiveShadow>
        <planeGeometry args={[mapping.canvasWidth, mapping.canvasHeight]} />
        <meshStandardMaterial
          color="#f8fafc"
          roughness={0.85}
          metalness={0}
          side={2}
        />
      </mesh>
      {/* Canvas frame (thin border) */}
      <mesh position={[0, 0, -0.005]}>
        <planeGeometry args={[mapping.canvasWidth + 0.02, mapping.canvasHeight + 0.02]} />
        <meshStandardMaterial color="#1e293b" roughness={0.6} />
      </mesh>
    </group>
  );
}
