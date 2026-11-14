export function SceneLights() {
  return (
    <>
      <ambientLight intensity={0.5} />
      <directionalLight
        position={[2, 3, 2]}
        intensity={0.8}
        castShadow
        shadow-mapSize-width={1024}
        shadow-mapSize-height={1024}
      />
      <directionalLight position={[-2, 2, -1]} intensity={0.3} />
    </>
  );
}
