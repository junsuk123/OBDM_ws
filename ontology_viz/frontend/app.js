import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const state = {
  graph: null,
  selected: null,
  eventSource: null,
  liveWanted: false,
  topologySignature: "",
  nodeObjects: new Map(),
  glowObjects: new Map(),
  edgeObjects: new Map(),
  nodeLabels: new Map(),
  edgeLabels: new Map(),
  labels: [],
  clock: new THREE.Clock(),
  raycaster: new THREE.Raycaster(),
  pointer: new THREE.Vector2(),
};

const el = {
  canvasHost: document.querySelector("#graphCanvas"),
  title: document.querySelector("#graphTitle"),
  meta: document.querySelector("#graphMeta"),
  status: document.querySelector("#statusLine"),
  source: document.querySelector("#sourceInput"),
  live: document.querySelector("#liveButton"),
  load: document.querySelector("#loadButton"),
  reset: document.querySelector("#resetButton"),
  selectedTitle: document.querySelector("#selectedTitle"),
  metadata: document.querySelector("#metadataPanel"),
  connections: document.querySelector("#connectionPanel"),
  trace: document.querySelector("#tracePanel"),
};

const palette = {
  evidence: 0x24a3a6,
  semantic: 0x3d7bd9,
  inference: 0xc15ba4,
  decision: 0xf0b33f,
  prediction: 0x43a66d,
  trajectory: 0xe1683a,
  terminal: 0xde3e5f,
  Entity: 0x8e9aa3,
};

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x071014);
scene.fog = new THREE.Fog(0x071014, 70, 190);

const camera = new THREE.PerspectiveCamera(58, 1, 0.1, 500);
camera.position.set(0, 24, 72);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
el.canvasHost.append(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 24;
controls.maxDistance = 160;

const graphGroup = new THREE.Group();
scene.add(graphGroup);

scene.add(new THREE.AmbientLight(0x9fb8c6, 0.7));
const keyLight = new THREE.DirectionalLight(0xffffff, 1.5);
keyLight.position.set(18, 28, 20);
scene.add(keyLight);
const rimLight = new THREE.PointLight(0x39d5ff, 2.4, 120);
rimLight.position.set(-30, 18, 28);
scene.add(rimLight);

const grid = new THREE.GridHelper(140, 28, 0x24424a, 0x16292f);
grid.position.y = -24;
scene.add(grid);

function validateGraph(graph) {
  if (!graph || typeof graph !== "object") throw new Error("Graph must be an object.");
  if (!Array.isArray(graph.nodes)) throw new Error("Graph nodes must be a list.");
  if (!Array.isArray(graph.edges)) throw new Error("Graph edges must be a list.");
  const ids = new Set(graph.nodes.map((node) => node.id));
  graph.edges.forEach((edge) => {
    if (!ids.has(edge.source) || !ids.has(edge.target)) {
      throw new Error(`Broken edge: ${edge.source} -> ${edge.target}`);
    }
  });
  return graph;
}

async function loadGraph(source) {
  setStatus(`loading ${source}`);
  const response = await fetch(source, { cache: "no-store" });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  setGraph(await response.json(), `loaded ${source}`, { forceLayout: true });
}

function setGraph(graph, message, options = {}) {
  const nextGraph = validateGraph(graph);
  const signature = topologySignature(nextGraph);
  const sameTopology = !options.forceLayout && signature === state.topologySignature && state.nodeObjects.size > 0;
  state.graph = prepareLayout(nextGraph, { sameTopology });
  if (!selectionExists(state.selected)) {
    state.selected = state.graph.selected?.node_id || state.graph.nodes[0]?.id || null;
  }
  state.topologySignature = signature;
  renderHeader();
  if (sameTopology) {
    updateGraphValues();
  } else {
    renderGraph();
  }
  renderInspector();
  setStatus(message || "graph updated");
}

function renderHeader() {
  const graph = state.graph;
  el.title.textContent = graph.graph_id || "Ontology 3D Monitor";
  el.meta.textContent = `${graph.nodes.length} nodes, ${graph.edges.length} edges, stamp ${graph.stamp ?? "n/a"}`;
}

function topologySignature(graph) {
  const nodes = graph.nodes.map((node) => node.id).sort().join("|");
  const edges = graph.edges.map(edgeKey).sort().join("|");
  return `${nodes}::${edges}`;
}

function prepareLayout(graph, options = {}) {
  const oldPositions = new Map();
  state.nodeObjects.forEach((object, id) => oldPositions.set(id, object.position.clone()));

  const count = Math.max(1, graph.nodes.length);
  graph.nodes.forEach((node, index) => {
    const old = oldPositions.get(node.id);
    if (old) {
      node.position = old;
      return;
    }
    const ring = 18 + (index % 3) * 12;
    const angle = (Math.PI * 2 * index) / count;
    node.position = new THREE.Vector3(
      Math.cos(angle) * ring,
      ((index % 5) - 2) * 6,
      Math.sin(angle) * ring,
    );
  });

  if (options.sameTopology) return graph;

  const byId = new Map(graph.nodes.map((node) => [node.id, node]));
  for (let tick = 0; tick < 220; tick += 1) {
    for (let i = 0; i < graph.nodes.length; i += 1) {
      for (let j = i + 1; j < graph.nodes.length; j += 1) {
        const a = graph.nodes[i];
        const b = graph.nodes[j];
        const delta = a.position.clone().sub(b.position);
        const dist = Math.max(0.1, delta.length());
        const force = Math.min(1.1, 95 / (dist * dist));
        delta.normalize().multiplyScalar(force);
        a.position.add(delta);
        b.position.sub(delta);
      }
    }
    graph.edges.forEach((edge) => {
      const source = byId.get(edge.source);
      const target = byId.get(edge.target);
      const delta = target.position.clone().sub(source.position);
      const dist = Math.max(0.1, delta.length());
      const desired = 18 + (1 - edgeStrength(edge)) * 13;
      const force = (dist - desired) * 0.012;
      delta.normalize().multiplyScalar(force);
      source.position.add(delta);
      target.position.sub(delta);
    });
    graph.nodes.forEach((node) => node.position.multiplyScalar(0.997));
  }
  return graph;
}

function renderGraph() {
  graphGroup.clear();
  state.nodeObjects.clear();
  state.glowObjects.clear();
  state.edgeObjects.clear();
  state.nodeLabels.clear();
  state.edgeLabels.clear();
  state.labels.forEach((label) => label.remove());
  state.labels = [];

  const byId = new Map(state.graph.nodes.map((node) => [node.id, node]));
  state.graph.edges.forEach((edge) => {
    const source = byId.get(edge.source);
    const target = byId.get(edge.target);
    const active = isActiveEdge(edge);
    const material = new THREE.LineBasicMaterial({
      color: active ? 0x80f8ff : 0x66818d,
      transparent: true,
      opacity: active ? 0.95 : 0.42,
    });
    const geometry = new THREE.BufferGeometry().setFromPoints([source.position, target.position]);
    const line = new THREE.Line(geometry, material);
    line.userData = { kind: "edge", id: edgeKey(edge), edge };
    graphGroup.add(line);
    state.edgeObjects.set(edgeKey(edge), line);

    const label = makeLabel(edge.label || edge.type || "related_to", "edge-label");
    label.dataset.edgeId = edgeKey(edge);
    state.edgeLabels.set(edgeKey(edge), label);
    state.labels.push(label);
    el.canvasHost.append(label);
  });

  state.graph.nodes.forEach((node) => {
    const active = isActiveNode(node);
    const selected = state.selected === node.id;
    const radius = nodeRadius(node);
    const color = palette[node.type] || palette.Entity;
    const material = new THREE.MeshStandardMaterial({
      color,
      emissive: new THREE.Color(active || selected ? color : 0x000000),
      emissiveIntensity: active || selected ? 0.85 : 0.12,
      roughness: 0.38,
      metalness: 0.08,
    });
    const mesh = new THREE.Mesh(new THREE.SphereGeometry(1, 32, 20), material);
    mesh.position.copy(node.position);
    mesh.scale.setScalar(radius);
    mesh.userData = {
      kind: "node",
      id: node.id,
      node,
      baseRadius: radius,
      targetRadius: radius,
      displayRadius: radius,
      targetActivation: activation(node),
      displayActivation: activation(node),
    };
    graphGroup.add(mesh);
    state.nodeObjects.set(node.id, mesh);

    const glow = new THREE.Mesh(
      new THREE.SphereGeometry(1, 32, 16),
      new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity: active || selected ? (selected ? 0.24 : 0.14) : 0,
        blending: THREE.AdditiveBlending,
        depthWrite: false,
      }),
    );
    glow.position.copy(node.position);
    glow.scale.setScalar(radius * 1.55);
    glow.userData = { glowFor: node.id };
    graphGroup.add(glow);
    state.glowObjects.set(node.id, glow);

    const label = makeLabel(`${node.label || node.id}\n${activation(node).toFixed(2)}`, "node-label");
    label.dataset.nodeId = node.id;
    state.nodeLabels.set(node.id, label);
    state.labels.push(label);
    el.canvasHost.append(label);
  });

  renderTrace();
}

function updateGraphValues() {
  const byId = new Map(state.graph.nodes.map((node) => [node.id, node]));
  state.graph.nodes.forEach((node) => {
    const mesh = state.nodeObjects.get(node.id);
    if (!mesh) return;
    const active = isActiveNode(node);
    const selected = state.selected === node.id;
    const color = palette[node.type] || palette.Entity;
    const radius = nodeRadius(node);
    mesh.userData.node = node;
    mesh.userData.targetRadius = radius;
    mesh.userData.targetActivation = activation(node);
    mesh.material.color.setHex(color);
    mesh.material.emissive.setHex(active || selected ? color : 0x000000);
    mesh.material.emissiveIntensity = active || selected ? 0.85 : 0.12;

    const glow = state.glowObjects.get(node.id);
    if (glow) {
      glow.material.color.setHex(color);
      glow.material.opacity = active || selected ? (selected ? 0.24 : 0.14) : 0;
    }

    const label = state.nodeLabels.get(node.id);
    if (label) {
      const shown = mesh.userData.displayActivation ?? activation(node);
      label.textContent = `${node.label || node.id}\n${shown.toFixed(2)}`;
    }
  });

  state.graph.edges.forEach((edge) => {
    const line = state.edgeObjects.get(edgeKey(edge));
    if (!line) return;
    const source = byId.get(edge.source);
    const target = byId.get(edge.target);
    if (source && target) {
      const position = line.geometry.attributes.position;
      position.setXYZ(0, source.position.x, source.position.y, source.position.z);
      position.setXYZ(1, target.position.x, target.position.y, target.position.z);
      position.needsUpdate = true;
      line.geometry.computeBoundingSphere();
    }
    const active = isActiveEdge(edge);
    line.userData.edge = edge;
    line.material.color.setHex(active ? 0x80f8ff : 0x66818d);
    line.material.opacity = active ? 0.95 : 0.42;
    const label = state.edgeLabels.get(edgeKey(edge));
    if (label) label.textContent = edge.label || edge.type || "related_to";
  });

  renderTrace();
}

function makeLabel(text, className) {
  const label = document.createElement("div");
  label.className = className;
  label.textContent = text;
  return label;
}

function animate() {
  requestAnimationFrame(animate);
  const t = state.clock.getElapsedTime();
  state.nodeObjects.forEach((mesh) => {
    const node = mesh.userData.node;
    mesh.userData.displayActivation = lerpNumber(
      mesh.userData.displayActivation ?? activation(node),
      mesh.userData.targetActivation ?? activation(node),
      0.34,
    );
    mesh.userData.displayRadius = lerpNumber(
      mesh.userData.displayRadius ?? mesh.userData.baseRadius ?? 1,
      mesh.userData.targetRadius ?? mesh.userData.baseRadius ?? 1,
      0.34,
    );
    mesh.userData.baseRadius = mesh.userData.displayRadius;
    const pulse = isActiveNode(node) || state.selected === node.id ? 1 + Math.sin(t * 4.5) * 0.07 : 1;
    mesh.scale.setScalar((mesh.userData.baseRadius || 1) * pulse);
    mesh.material.emissiveIntensity = isActiveNode(node) || state.selected === node.id
      ? 0.72 + Math.sin(t * 5.5) * 0.22
      : 0.1;
    const label = state.nodeLabels.get(node.id);
    if (label) label.textContent = `${node.label || node.id}\n${(mesh.userData.displayActivation ?? activation(node)).toFixed(2)}`;
  });
  state.glowObjects.forEach((glow, nodeId) => {
    const mesh = state.nodeObjects.get(nodeId);
    if (!mesh) return;
    const node = mesh.userData.node;
    const visible = isActiveNode(node) || state.selected === nodeId;
    const pulse = visible ? 1.55 + Math.sin(t * 4.2) * 0.08 : 1.55;
    glow.scale.setScalar((mesh.userData.baseRadius || 1) * pulse);
    glow.material.opacity = visible ? (state.selected === nodeId ? 0.24 : 0.14) : 0;
  });
  controls.update();
  renderer.render(scene, camera);
  positionLabels();
}

function lerpNumber(current, target, amount) {
  if (!Number.isFinite(current)) return target;
  if (!Number.isFinite(target)) return current;
  return current + (target - current) * amount;
}

function positionLabels() {
  const rect = renderer.domElement.getBoundingClientRect();
  state.labels.forEach((label) => {
    let position = null;
    if (label.dataset.nodeId) {
      position = state.nodeObjects.get(label.dataset.nodeId)?.position;
    } else if (label.dataset.edgeId) {
      const line = state.edgeObjects.get(label.dataset.edgeId);
      if (line) {
        const arr = line.geometry.attributes.position.array;
        position = new THREE.Vector3(
          (arr[0] + arr[3]) / 2,
          (arr[1] + arr[4]) / 2,
          (arr[2] + arr[5]) / 2,
        );
      }
    }
    if (!position) return;
    const projected = position.clone().project(camera);
    const visible = projected.z < 1;
    label.style.display = visible ? "block" : "none";
    label.style.left = `${(projected.x * 0.5 + 0.5) * rect.width}px`;
    label.style.top = `${(-projected.y * 0.5 + 0.5) * rect.height}px`;
  });
}

function renderInspector() {
  if (!state.graph) return;
  const selected = getSelectedObject();
  el.selectedTitle.textContent = selected?.label || selected?.id || edgeKey(selected) || "None";
  el.metadata.textContent = JSON.stringify(selected || state.graph, null, 2);
  renderConnections(selected);
}

function renderConnections(selected) {
  el.connections.innerHTML = "";
  if (!selected || !selected.id || !state.graph.nodes.some((node) => node.id === selected.id)) {
    el.connections.textContent = "Select a node to inspect live relations.";
    return;
  }
  const related = state.graph.edges.filter((edge) => edge.source === selected.id || edge.target === selected.id);
  if (!related.length) {
    el.connections.textContent = "No direct connections.";
    return;
  }
  related.forEach((edge) => {
    const row = document.createElement("button");
    row.type = "button";
    row.className = "connection-row";
    row.textContent = `${edge.source} ${edge.label || edge.type || "related_to"} ${edge.target} (${edgeStrength(edge).toFixed(2)})`;
    row.addEventListener("click", () => {
      state.selected = { edge: edgeKey(edge) };
      renderInspector();
    });
    el.connections.append(row);
  });
}

function renderTrace() {
  el.trace.innerHTML = "";
  const trace = state.graph?.reasoning_trace || [];
  if (!trace.length) {
    el.trace.textContent = "No reasoning trace in this snapshot.";
    return;
  }
  trace.forEach((step) => {
    const item = document.createElement("div");
    item.className = "trace-step";
    item.textContent = `${step.step || ""} ${step.description || "Inference step"} (${Number(step.confidence ?? 0).toFixed(2)})`;
    el.trace.append(item);
  });
}

function getSelectedObject() {
  if (!state.graph || !state.selected) return null;
  if (typeof state.selected === "object" && state.selected.edge) {
    return state.graph.edges.find((edge) => edgeKey(edge) === state.selected.edge) || null;
  }
  return state.graph.nodes.find((node) => node.id === state.selected) || null;
}

function selectionExists(selection) {
  if (!selection || !state.graph) return false;
  if (typeof selection === "object" && selection.edge) {
    return state.graph.edges.some((edge) => edgeKey(edge) === selection.edge);
  }
  return state.graph.nodes.some((node) => node.id === selection);
}

function edgeKey(edge) {
  return edge?.id || `${edge?.source}->${edge?.target}:${edge?.label || edge?.type || "related_to"}`;
}

function activation(node) {
  return clamp01(Number(node.attributes?.activation ?? node.score ?? node.confidence ?? 0.5));
}

function importance(node) {
  return clamp01(Number(node.importance ?? node.attributes?.importance ?? activation(node)));
}

function nodeRadius(node) {
  return 1.35 + importance(node) * 2.4 + degree(node.id) * 0.09;
}

function degree(nodeId) {
  return (state.graph?.edges || []).filter((edge) => edge.source === nodeId || edge.target === nodeId).length;
}

function edgeStrength(edge) {
  return clamp01(Number(edge.confidence ?? edge.weight ?? edge.attributes?.weight ?? 0.5));
}

function isActiveNode(node) {
  return Boolean(node.active) || activation(node) >= 0.72;
}

function isActiveEdge(edge) {
  return Boolean(edge.active) || edgeStrength(edge) >= 0.62;
}

function clamp01(value) {
  return Math.max(0, Math.min(1, Number.isFinite(value) ? value : 0));
}

function resize() {
  const rect = el.canvasHost.getBoundingClientRect();
  renderer.setSize(rect.width, rect.height, false);
  camera.aspect = rect.width / Math.max(1, rect.height);
  camera.updateProjectionMatrix();
}

function resetCamera() {
  camera.position.set(0, 24, 72);
  controls.target.set(0, 0, 0);
  controls.update();
}

function setStatus(message, isError = false) {
  el.status.textContent = message;
  el.status.classList.toggle("error", isError);
}

function liveSource() {
  const source = el.source.value.trim();
  if (source.includes("/events")) return source;
  if (source.includes("/snapshot")) return source.replace("/snapshot", "/events");
  return source || "http://127.0.0.1:8765/events";
}

function startLive() {
  stopLive();
  state.liveWanted = true;
  const source = liveSource();
  state.eventSource = new EventSource(source);
  el.live.classList.add("active");
  setStatus(`waiting for ${source}`);
  state.eventSource.onopen = () => setStatus(`live connected ${source}`);
  state.eventSource.addEventListener("graph", (event) => {
    setGraph(JSON.parse(event.data), `live update ${new Date().toLocaleTimeString()}`);
  });
  state.eventSource.onerror = () => {
    if (state.liveWanted) setStatus(`waiting for ${source}`);
  };
}

function stopLive() {
  state.liveWanted = false;
  if (state.eventSource) {
    state.eventSource.close();
    state.eventSource = null;
  }
  el.live.classList.remove("active");
}

renderer.domElement.addEventListener("pointerdown", (event) => {
  const rect = renderer.domElement.getBoundingClientRect();
  state.pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  state.pointer.y = -(((event.clientY - rect.top) / rect.height) * 2 - 1);
  state.raycaster.setFromCamera(state.pointer, camera);
  const hits = state.raycaster.intersectObjects([...state.nodeObjects.values(), ...state.edgeObjects.values()]);
  if (!hits.length) return;
  const hit = hits[0].object.userData;
  state.selected = hit.kind === "edge" ? { edge: hit.id } : hit.id;
  renderGraph();
  renderInspector();
});

el.live.addEventListener("click", () => {
  if (state.eventSource) stopLive();
  else startLive();
});

el.load.addEventListener("click", () => {
  stopLive();
  const source = el.source.value.trim();
  const snapshot = source.includes("/events") ? source.replace("/events", "/snapshot") : source;
  loadGraph(snapshot || "/viewer/data/demo_graph.json").catch((error) => setStatus(error.message, true));
});

el.reset.addEventListener("click", resetCamera);
window.addEventListener("resize", resize);
window.addEventListener("beforeunload", stopLive);

resize();
animate();
loadGraph("/viewer/data/demo_graph.json")
  .catch((error) => setStatus(error.message, true))
  .finally(startLive);
