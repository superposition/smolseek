/**
 * MapViewer — Babylon.js 3D point cloud renderer in React.
 *
 * Ported from smolROS web/viewer.js. Renders live point cloud with
 * height-based coloring, trajectory, robot marker, and cache markers.
 */

import { useEffect, useRef, useCallback } from "react";
import {
  Engine,
  Scene,
  ArcRotateCamera,
  HemisphericLight,
  Vector3,
  Color3,
  Color4,
  Mesh,
  MeshBuilder,
  StandardMaterial,
  VertexData,
} from "@babylonjs/core";
import type { PointCloudData } from "../hooks/useGameSocket";
import type { Cache } from "../types";

interface MapViewerProps {
  onPointCloud: (cb: (data: PointCloudData) => void) => void;
  onTrajectory: (cb: (data: Float32Array) => void) => void;
  caches: Cache[];
  selectedCacheId: string | null;
  onCacheSelect: (cacheId: string) => void;
  robotTarget: { x: number; y: number } | null;
}

const REBUILD_THROTTLE_MS = 1000;

export default function MapViewer({
  onPointCloud,
  onTrajectory,
  caches,
  selectedCacheId,
  onCacheSelect,
  robotTarget,
}: MapViewerProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const sceneRef = useRef<Scene | null>(null);
  const engineRef = useRef<Engine | null>(null);

  // Mutable refs for point cloud state
  const pointsRef = useRef<{ x: number; y: number; z: number }[]>([]);
  const pointsMeshRef = useRef<Mesh | null>(null);
  const trajectoryLineRef = useRef<Mesh | null>(null);
  const robotMarkerRef = useRef<Mesh | null>(null);
  const cacheMeshesRef = useRef<Map<string, Mesh>>(new Map());
  const lastRebuildRef = useRef(0);
  const rebuildPendingRef = useRef(false);

  // Height → color mapping (blue → cyan → green → yellow → red)
  const heightToColor = useCallback((h: number): [number, number, number] => {
    const t = Math.max(0, Math.min(1, (h - -1.0) / (3.0 - -1.0)));
    if (t < 0.25) { const s = t / 0.25; return [0, s, 1]; }
    if (t < 0.5) { const s = (t - 0.25) / 0.25; return [0, 1, 1 - s]; }
    if (t < 0.75) { const s = (t - 0.5) / 0.25; return [s, 1, 0]; }
    const s = (t - 0.75) / 0.25; return [1, 1 - s, 0];
  }, []);

  // Rebuild point cloud mesh
  const rebuildPointCloud = useCallback(() => {
    const scene = sceneRef.current;
    const pts = pointsRef.current;
    if (!scene || pts.length === 0) return;

    if (pointsMeshRef.current) {
      pointsMeshRef.current.dispose();
      pointsMeshRef.current = null;
    }

    const positions = new Float32Array(pts.length * 3);
    const colors = new Float32Array(pts.length * 4);

    for (let i = 0; i < pts.length; i++) {
      const p = pts[i];
      // ROS → Babylon: X=X, Y=Z(up), Z=Y
      positions[i * 3] = p.x;
      positions[i * 3 + 1] = p.z;
      positions[i * 3 + 2] = p.y;

      const [r, g, b] = heightToColor(p.z);
      colors[i * 4] = r;
      colors[i * 4 + 1] = g;
      colors[i * 4 + 2] = b;
      colors[i * 4 + 3] = 1.0;
    }

    const mesh = new Mesh("pointCloud", scene);
    const vertexData = new VertexData();
    vertexData.positions = positions;
    vertexData.colors = colors;
    const indices = new Uint32Array(pts.length);
    for (let i = 0; i < pts.length; i++) indices[i] = i;
    vertexData.indices = indices;
    vertexData.applyToMesh(mesh);

    const mat = new StandardMaterial("pcMat", scene);
    mat.emissiveColor = new Color3(1, 1, 1);
    mat.disableLighting = true;
    mat.pointsCloud = true;
    mat.pointSize = 3;
    // @ts-expect-error — useVertexColors exists on StandardMaterial
    mat.useVertexColors = true;
    mesh.material = mat;

    pointsMeshRef.current = mesh;
    lastRebuildRef.current = Date.now();
    rebuildPendingRef.current = false;
  }, [heightToColor]);

  const scheduleRebuild = useCallback(() => {
    if (rebuildPendingRef.current) return;
    const elapsed = Date.now() - lastRebuildRef.current;
    if (elapsed >= REBUILD_THROTTLE_MS) {
      rebuildPointCloud();
    } else {
      rebuildPendingRef.current = true;
      setTimeout(() => rebuildPointCloud(), REBUILD_THROTTLE_MS - elapsed);
    }
  }, [rebuildPointCloud]);

  // Initialize Babylon.js
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const engine = new Engine(canvas, true, { preserveDrawingBuffer: true });
    const scene = new Scene(engine);
    scene.clearColor = new Color4(0.08, 0.08, 0.14, 1.0);
    engineRef.current = engine;
    sceneRef.current = scene;

    // Camera
    const camera = new ArcRotateCamera(
      "camera", -Math.PI / 4, Math.PI / 3, 10,
      Vector3.Zero(), scene
    );
    camera.attachControl(canvas, true);
    camera.lowerRadiusLimit = 1;
    camera.upperRadiusLimit = 50;
    camera.wheelPrecision = 20;

    // Light
    const light = new HemisphericLight("light", new Vector3(0, 1, 0), scene);
    light.intensity = 0.8;

    // Ground grid
    const gridSize = 20;
    const gridLines: Vector3[][] = [];
    for (let i = -gridSize / 2; i <= gridSize / 2; i++) {
      gridLines.push([new Vector3(i, 0, -gridSize / 2), new Vector3(i, 0, gridSize / 2)]);
      gridLines.push([new Vector3(-gridSize / 2, 0, i), new Vector3(gridSize / 2, 0, i)]);
    }
    const grid = MeshBuilder.CreateLineSystem("grid", { lines: gridLines }, scene);
    grid.color = new Color3(0.2, 0.2, 0.3);
    grid.alpha = 0.3;

    // Robot marker (green sphere)
    const robot = MeshBuilder.CreateSphere("robot", { diameter: 0.12 }, scene);
    const robotMat = new StandardMaterial("robotMat", scene);
    robotMat.diffuseColor = new Color3(0, 1, 0.5);
    robotMat.emissiveColor = new Color3(0, 0.5, 0.25);
    robot.material = robotMat;
    robot.isVisible = false;
    robotMarkerRef.current = robot;

    engine.runRenderLoop(() => scene.render());

    const handleResize = () => engine.resize();
    window.addEventListener("resize", handleResize);

    return () => {
      window.removeEventListener("resize", handleResize);
      engine.dispose();
    };
  }, []);

  // Register point cloud callback
  useEffect(() => {
    onPointCloud((data: PointCloudData) => {
      const pts: { x: number; y: number; z: number }[] = [];
      for (let i = 0; i < data.points.length; i += 3) {
        pts.push({
          x: data.points[i],
          y: data.points[i + 1],
          z: data.points[i + 2],
        });
      }
      if (data.isDelta) {
        pointsRef.current = pointsRef.current.concat(pts);
      } else {
        pointsRef.current = pts;
      }
      scheduleRebuild();
    });
  }, [onPointCloud, scheduleRebuild]);

  // Register trajectory callback
  useEffect(() => {
    onTrajectory((data: Float32Array) => {
      const scene = sceneRef.current;
      if (!scene) return;

      if (trajectoryLineRef.current) {
        trajectoryLineRef.current.dispose();
      }

      const path: Vector3[] = [];
      for (let i = 0; i < data.length; i += 3) {
        path.push(new Vector3(data[i], data[i + 2], data[i + 1]));
      }
      if (path.length < 2) return;

      const line = MeshBuilder.CreateLines("trajectory", { points: path }, scene);
      line.color = new Color3(0, 1, 0.5);
      trajectoryLineRef.current = line;

      // Update robot position to last trajectory point
      const last = path[path.length - 1];
      if (robotMarkerRef.current) {
        robotMarkerRef.current.position = last;
        robotMarkerRef.current.isVisible = true;
      }
    });
  }, [onTrajectory]);

  // Update cache markers (D6)
  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) return;

    const existing = cacheMeshesRef.current;
    const seen = new Set<string>();

    for (const cache of caches) {
      seen.add(cache.id);
      let mesh = existing.get(cache.id);

      if (!mesh) {
        mesh = MeshBuilder.CreateSphere(
          `cache_${cache.id}`, { diameter: 0.15 }, scene
        );
        mesh.position = new Vector3(cache.x, cache.z || 0.1, cache.y);

        // Make clickable
        mesh.isPickable = true;
        mesh.metadata = { cacheId: cache.id };

        existing.set(cache.id, mesh);
      }

      // Material based on state
      const mat = new StandardMaterial(`cacheMat_${cache.id}`, scene);
      if (cache.collected) {
        mat.diffuseColor = new Color3(0.4, 0.4, 0.4);
        mat.emissiveColor = new Color3(0.1, 0.1, 0.1);
        mat.alpha = 0.3;
      } else if (cache.id === selectedCacheId) {
        mat.diffuseColor = new Color3(1, 1, 1);
        mat.emissiveColor = new Color3(0.8, 0.8, 0.3);
      } else {
        mat.diffuseColor = new Color3(1, 0.84, 0);
        mat.emissiveColor = new Color3(0.5, 0.42, 0);
      }
      mesh.material = mat;

      // Pulsing animation for uncollected
      if (!cache.collected) {
        const t = Date.now() / 1000;
        const s = 1.0 + 0.1 * Math.sin(t * 3 + cache.x * 10);
        mesh.scaling = new Vector3(s, s, s);
      } else {
        mesh.scaling = new Vector3(1, 1, 1);
      }
    }

    // Remove stale meshes
    for (const [id, mesh] of existing) {
      if (!seen.has(id)) {
        mesh.dispose();
        existing.delete(id);
      }
    }
  }, [caches, selectedCacheId]);

  // Robot target marker
  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene || !robotTarget) return;
    if (robotMarkerRef.current) {
      robotMarkerRef.current.position = new Vector3(
        robotTarget.x, 0.05, robotTarget.y
      );
      robotMarkerRef.current.isVisible = true;
    }
  }, [robotTarget]);

  // Click handler for cache selection
  useEffect(() => {
    const scene = sceneRef.current;
    if (!scene) return;

    const handler = scene.onPointerObservable.add((pointerInfo) => {
      if (pointerInfo.type === 1) { // POINTERPICK
        const mesh = pointerInfo.pickInfo?.pickedMesh;
        const cacheId = mesh?.metadata?.cacheId;
        if (cacheId) {
          onCacheSelect(cacheId);
        }
      }
    });

    return () => {
      scene.onPointerObservable.remove(handler);
    };
  }, [onCacheSelect]);

  return (
    <canvas
      ref={canvasRef}
      style={{ width: "100%", height: "100%", display: "block" }}
    />
  );
}
