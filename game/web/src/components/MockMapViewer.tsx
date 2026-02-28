/**
 * MockMapViewer — Babylon.js 3D scene with simulated point cloud growth,
 * trajectory, robot marker, and cache markers. Used in Storybook to show
 * the bot scanning and mapping the environment in real-time.
 */

import { useEffect, useRef } from "react";
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
import type { Cache } from "../types";

interface MockMapViewerProps {
  caches: Cache[];
  selectedCacheId: string | null;
  onCacheSelect?: (id: string) => void;
  /** Scan speed: points added per frame batch */
  scanRate?: number;
}

// Height → color (blue → cyan → green → yellow → red)
function heightToColor(h: number): [number, number, number] {
  const t = Math.max(0, Math.min(1, (h + 0.5) / 2.5));
  if (t < 0.25) { const s = t / 0.25; return [0, s, 1]; }
  if (t < 0.5) { const s = (t - 0.25) / 0.25; return [0, 1, 1 - s]; }
  if (t < 0.75) { const s = (t - 0.5) / 0.25; return [s, 1, 0]; }
  const s = (t - 0.75) / 0.25; return [1, 1 - s, 0];
}

// Generate a ring of noisy points simulating a LiDAR sweep
function generateSweep(
  cx: number, cy: number, angle: number, radius: number, count: number
): { x: number; y: number; z: number }[] {
  const pts: { x: number; y: number; z: number }[] = [];
  const spread = Math.PI * 0.6;
  for (let i = 0; i < count; i++) {
    const a = angle - spread / 2 + Math.random() * spread;
    const r = radius * (0.3 + Math.random() * 0.7);
    const x = cx + Math.cos(a) * r;
    const y = cy + Math.sin(a) * r;
    // Height: ground plane with some noise + "walls" at edges
    const distFromCenter = Math.sqrt(x * x + y * y);
    const wallHeight = distFromCenter > 4 ? (distFromCenter - 4) * 0.8 : 0;
    const z = wallHeight + (Math.random() - 0.5) * 0.15;
    pts.push({ x, y, z });
  }
  return pts;
}

// Patrol path the robot follows
function getPatrolPoint(t: number): { x: number; y: number; angle: number } {
  const loops = t * 0.15;
  // Figure-8 patrol
  const x = 3.0 * Math.sin(loops);
  const y = 2.0 * Math.sin(loops * 2);
  const dx = 3.0 * Math.cos(loops) * 0.15;
  const dy = 2.0 * Math.cos(loops * 2) * 0.3;
  const angle = Math.atan2(dy, dx);
  return { x, y, angle };
}

export default function MockMapViewer({
  caches,
  selectedCacheId,
  onCacheSelect,
  scanRate = 12,
}: MockMapViewerProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const engine = new Engine(canvas, true, { preserveDrawingBuffer: true });
    const scene = new Scene(engine);
    scene.clearColor = new Color4(0.02, 0.02, 0.04, 1.0);

    // Camera — tilted top-down
    const camera = new ArcRotateCamera(
      "cam", -Math.PI / 4, Math.PI / 3.5, 12,
      Vector3.Zero(), scene
    );
    camera.attachControl(canvas, true);
    camera.lowerRadiusLimit = 3;
    camera.upperRadiusLimit = 30;
    camera.wheelPrecision = 15;

    // Light
    const light = new HemisphericLight("light", new Vector3(0, 1, 0), scene);
    light.intensity = 0.6;

    // Grid
    const gridSize = 20;
    const gridLines: Vector3[][] = [];
    for (let i = -gridSize / 2; i <= gridSize / 2; i++) {
      gridLines.push([new Vector3(i, 0, -gridSize / 2), new Vector3(i, 0, gridSize / 2)]);
      gridLines.push([new Vector3(-gridSize / 2, 0, i), new Vector3(gridSize / 2, 0, i)]);
    }
    const grid = MeshBuilder.CreateLineSystem("grid", { lines: gridLines }, scene);
    grid.color = new Color3(0.15, 0.15, 0.25);
    grid.alpha = 0.25;

    // Robot marker
    const robot = MeshBuilder.CreateSphere("robot", { diameter: 0.18 }, scene);
    const robotMat = new StandardMaterial("robotMat", scene);
    robotMat.diffuseColor = new Color3(0, 1, 0.5);
    robotMat.emissiveColor = new Color3(0, 0.6, 0.3);
    robot.material = robotMat;

    // Scan cone visualization — tip at robot, wide end outward
    const cone = MeshBuilder.CreateCylinder("cone", {
      diameterTop: 3,
      diameterBottom: 0,
      height: 3.5,
      tessellation: 16,
    }, scene);
    const coneMat = new StandardMaterial("coneMat", scene);
    coneMat.diffuseColor = new Color3(0, 1, 0.5);
    coneMat.emissiveColor = new Color3(0, 0.3, 0.15);
    coneMat.alpha = 0.06;
    cone.material = coneMat;

    // Cache markers
    const cacheMeshes = new Map<string, Mesh>();
    for (const cache of caches) {
      const m = MeshBuilder.CreateSphere(`cache_${cache.id}`, { diameter: 0.2 }, scene);
      m.position = new Vector3(cache.x, (cache.z || 0.1) + 0.1, cache.y);
      m.isPickable = true;
      m.metadata = { cacheId: cache.id };

      const mat = new StandardMaterial(`cacheMat_${cache.id}`, scene);
      if (cache.collected) {
        mat.diffuseColor = new Color3(0.3, 0.3, 0.3);
        mat.emissiveColor = new Color3(0.1, 0.1, 0.1);
        mat.alpha = 0.3;
      } else if (cache.id === selectedCacheId) {
        mat.diffuseColor = new Color3(1, 1, 1);
        mat.emissiveColor = new Color3(0.9, 0.9, 0.4);
      } else {
        mat.diffuseColor = new Color3(1, 0.84, 0);
        mat.emissiveColor = new Color3(0.5, 0.42, 0);
      }
      m.material = mat;
      cacheMeshes.set(cache.id, m);
    }

    // Click handler
    if (onCacheSelect) {
      scene.onPointerObservable.add((info) => {
        if (info.type === 1) {
          const cid = info.pickInfo?.pickedMesh?.metadata?.cacheId;
          if (cid) onCacheSelect(cid);
        }
      });
    }

    // Point cloud state
    let allPoints: { x: number; y: number; z: number }[] = [];
    let pcMesh: Mesh | null = null;
    let trajLine: Mesh | null = null;
    const trajectory: Vector3[] = [];
    let tick = 0;

    // Shared material — reuse across rebuilds to avoid flicker
    const pcMat = new StandardMaterial("pcMat", scene);
    pcMat.emissiveColor = new Color3(1, 1, 1);
    pcMat.disableLighting = true;
    pcMat.pointsCloud = true;
    pcMat.pointSize = 2.5;
    // @ts-expect-error — useVertexColors
    pcMat.useVertexColors = true;

    // Point count overlay
    const counterDiv = document.createElement("div");
    counterDiv.style.cssText =
      "position:absolute;bottom:12px;left:12px;font-family:'Geist Mono',monospace;" +
      "font-size:11px;color:rgba(228,228,231,0.4);pointer-events:none;z-index:1;";
    canvas.parentElement?.appendChild(counterDiv);

    function rebuildCloud() {
      if (allPoints.length === 0) return;

      const positions = new Float32Array(allPoints.length * 3);
      const colors = new Float32Array(allPoints.length * 4);

      for (let i = 0; i < allPoints.length; i++) {
        const p = allPoints[i];
        positions[i * 3] = p.x;
        positions[i * 3 + 1] = p.z;
        positions[i * 3 + 2] = p.y;

        const [r, g, b] = heightToColor(p.z);
        colors[i * 4] = r;
        colors[i * 4 + 1] = g;
        colors[i * 4 + 2] = b;
        colors[i * 4 + 3] = 0.85;
      }

      // Build new mesh first, then swap
      const newMesh = new Mesh("pc", scene);
      const vd = new VertexData();
      vd.positions = positions;
      vd.colors = colors;
      const indices = new Uint32Array(allPoints.length);
      for (let i = 0; i < allPoints.length; i++) indices[i] = i;
      vd.indices = indices;
      vd.applyToMesh(newMesh);
      newMesh.material = pcMat;

      // Dispose old mesh after new one is ready
      if (pcMesh) pcMesh.dispose();
      pcMesh = newMesh;
    }

    function updateTrajectory() {
      if (trajectory.length < 2) return;

      const newLine = MeshBuilder.CreateLines("traj_new", {
        points: trajectory.slice(-500),
      }, scene);
      newLine.color = new Color3(0, 0.8, 0.4);
      newLine.alpha = 0.5;

      if (trajLine) trajLine.dispose();
      trajLine = newLine;
    }

    // Animation loop — simulate robot scanning
    let frameCount = 0;
    const beforeRender = () => {
      tick += 0.016; // ~60fps
      frameCount++;

      const patrol = getPatrolPoint(tick);

      // Smooth robot movement via lerp
      const targetPos = new Vector3(patrol.x, 0.09, patrol.y);
      robot.position = Vector3.Lerp(robot.position, targetPos, 0.3);

      // Scan FOV cone — narrow tip at robot, wide end fans outward
      const coneLen = 1.75; // half cone height, so tip sits at robot
      cone.position = new Vector3(
        patrol.x + Math.cos(patrol.angle) * coneLen,
        0.05,
        patrol.y + Math.sin(patrol.angle) * coneLen,
      );
      cone.rotation.x = Math.PI / 2;
      cone.rotation.y = patrol.angle - Math.PI / 2;

      // Add scan points every other frame to reduce churn
      if (frameCount % 2 === 0) {
        const newPts = generateSweep(patrol.x, patrol.y, patrol.angle, 4.0, scanRate);
        allPoints = allPoints.concat(newPts);

        // Cap at 60k points
        if (allPoints.length > 60000) {
          allPoints = allPoints.slice(allPoints.length - 60000);
        }
      }

      // Record trajectory (every 4 frames to keep line smooth, not noisy)
      if (frameCount % 4 === 0) {
        trajectory.push(new Vector3(patrol.x, 0.05, patrol.y));
        if (trajectory.length > 800) trajectory.splice(0, trajectory.length - 800);
      }

      // Rebuild every 45 frames (~0.75s) — less frequent = less flicker
      if (frameCount % 45 === 0) {
        rebuildCloud();
        updateTrajectory();
        counterDiv.textContent = `${allPoints.length.toLocaleString()} points | scanning`;
      }

      // Pulse cache markers
      for (const [id, mesh] of cacheMeshes) {
        const cache = caches.find((c) => c.id === id);
        if (cache && !cache.collected) {
          const s = 1.0 + 0.12 * Math.sin(tick * 3 + cache.x * 10);
          mesh.scaling = new Vector3(s, s, s);
        }
      }
    };

    scene.registerBeforeRender(beforeRender);
    engine.runRenderLoop(() => scene.render());

    const handleResize = () => engine.resize();
    window.addEventListener("resize", handleResize);

    return () => {
      scene.unregisterBeforeRender(beforeRender);
      window.removeEventListener("resize", handleResize);
      counterDiv.remove();
      engine.dispose();
    };
  }, [caches, selectedCacheId, onCacheSelect, scanRate]);

  return (
    <canvas
      ref={canvasRef}
      style={{ width: "100%", height: "100%", display: "block" }}
    />
  );
}
