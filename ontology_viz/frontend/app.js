const state = {
  graph: null,
  selected: null,
  activeTrace: null,
  hiddenNodeTypes: new Set(),
  hiddenEdgeTypes: new Set(),
  showInferred: true,
  showTrace: true,
  search: "",
  transform: { x: 0, y: 0, scale: 1 },
  dragging: null,
  panning: null,
  eventSource: null,
  liveWanted: false,
  pollTimer: null,
};

const palette = [
  "#0d7c86",
  "#d4612f",
  "#6c7a12",
  "#7a5cc8",
  "#b33f62",
  "#2472a4",
  "#8a6b21",
  "#23845a",
];

const el = {
  svg: document.querySelector("#graphSvg"),
  viewport: document.querySelector("#viewport"),
  edgeLayer: document.querySelector("#edgeLayer"),
  nodeLayer: document.querySelector("#nodeLayer"),
  title: document.querySelector("#graphTitle"),
  meta: document.querySelector("#graphMeta"),
  status: document.querySelector("#statusLine"),
  metadata: document.querySelector("#metadataPanel"),
  trace: document.querySelector("#tracePanel"),
  source: document.querySelector("#sourceInput"),
  load: document.querySelector("#loadButton"),
  live: document.querySelector("#liveButton"),
  poll: document.querySelector("#pollButton"),
  reset: document.querySelector("#resetButton"),
  search: document.querySelector("#searchInput"),
  inferred: document.querySelector("#inferredToggle"),
  traceToggle: document.querySelector("#traceToggle"),
  nodeFilters: document.querySelector("#nodeTypeFilters"),
  edgeFilters: document.querySelector("#edgeTypeFilters"),
};

function edgeKey(edge) {
  return edge.id || `${edge.source}->${edge.target}:${edge.label || edge.type || "related_to"}`;
}

function typeColor(type) {
  const types = [...new Set((state.graph?.nodes || []).map((node) => node.type || "Entity"))].sort();
  const index = Math.max(0, types.indexOf(type || "Entity"));
  return palette[index % palette.length];
}

function validateGraph(graph) {
  if (!graph || typeof graph !== "object") throw new Error("Graph must be an object.");
  if (!Array.isArray(graph.nodes)) throw new Error("Graph nodes must be a list.");
  if (!Array.isArray(graph.edges)) throw new Error("Graph edges must be a list.");
  const nodeIds = new Set();
  graph.nodes.forEach((node, index) => {
    if (!node.id) throw new Error(`nodes[${index}].id is required.`);
    if (nodeIds.has(node.id)) throw new Error(`Duplicate node id: ${node.id}`);
    nodeIds.add(node.id);
  });
  graph.edges.forEach((edge, index) => {
    if (!nodeIds.has(edge.source)) throw new Error(`edges[${index}].source is missing: ${edge.source}`);
    if (!nodeIds.has(edge.target)) throw new Error(`edges[${index}].target is missing: ${edge.target}`);
  });
  return graph;
}

async function loadGraph(source) {
  setStatus(`loading ${source}`);
  const response = await fetch(source, { cache: "no-store" });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  setGraph(await response.json(), `loaded ${source}`);
}

function setGraph(graph, message, options = {}) {
  const nextGraph = validateGraph(graph);
  const previousSelection = state.selected;
  const selectedNode = nextGraph.selected?.node_id || nextGraph.nodes[0]?.id || null;
  state.graph = nextGraph;
  state.selected = selectionStillExists(previousSelection) ? previousSelection : selectedNode;
  if (!options.preserveTrace) state.activeTrace = null;
  if (!options.preserveFilters) {
    state.hiddenNodeTypes.clear();
    state.hiddenEdgeTypes.clear();
  }
  layoutGraph();
  if (!options.preserveView) resetView();
  render();
  setStatus(message || "graph loaded");
}

function liveSource() {
  const configured = el.source.value.trim();
  if (!configured || configured === "data/demo_graph.json") return "http://127.0.0.1:8765/events";
  if (configured.includes("/events")) return configured;
  if (configured.includes("/snapshot")) return configured.replace("/snapshot", "/events");
  return "http://127.0.0.1:8765/events";
}

function selectionStillExists(selection) {
  if (!selection || !state.graph) return false;
  if (typeof selection === "object" && selection.edge) {
    return state.graph.edges.some((edge) => edgeKey(edge) === selection.edge);
  }
  return state.graph.nodes.some((node) => node.id === selection);
}

function layoutGraph() {
  const graph = state.graph;
  const viewportWidth = Math.max(760, el.svg.clientWidth || 900);
  const viewportHeight = Math.max(520, el.svg.clientHeight || 650);
  const nodeCount = Math.max(1, graph.nodes.length);
  const columns = Math.max(3, Math.ceil(Math.sqrt(nodeCount * 1.35)));
  const rows = Math.max(2, Math.ceil(nodeCount / columns));
  const width = Math.max(viewportWidth, columns * 360 + 360, 1280);
  const height = Math.max(viewportHeight, rows * 300 + 320, 860);
  const centerX = width / 2;
  const centerY = height / 2;
  const radius = Math.min(width, height) * 0.43;
  const nodes = graph.nodes;

  nodes.forEach((node, index) => {
    const angle = (Math.PI * 2 * index) / Math.max(1, nodes.length) - Math.PI / 2;
    const ring = index % 2 === 0 ? radius : radius * 0.72;
    node.x = centerX + Math.cos(angle) * ring;
    node.y = centerY + Math.sin(angle) * ring;
  });

  const byId = new Map(nodes.map((node) => [node.id, node]));
  for (let tick = 0; tick < 180; tick += 1) {
    for (let i = 0; i < nodes.length; i += 1) {
      for (let j = i + 1; j < nodes.length; j += 1) {
        const a = nodes[i];
        const b = nodes[j];
        const dx = a.x - b.x || 0.01;
        const dy = a.y - b.y || 0.01;
        const distance = Math.max(1, Math.hypot(dx, dy));
        const minDistance = nodeClearance(a) + nodeClearance(b);
        const collision = Math.max(0, minDistance - distance) * 0.11;
        const force = 16500 / (distance * distance) + collision;
        a.x += (dx / distance) * force;
        a.y += (dy / distance) * force;
        b.x -= (dx / distance) * force;
        b.y -= (dy / distance) * force;
      }
    }

    graph.edges.forEach((edge) => {
      const source = byId.get(edge.source);
      const target = byId.get(edge.target);
      if (!source || !target) return;
      const dx = target.x - source.x;
      const dy = target.y - source.y;
      const distance = Math.max(1, Math.hypot(dx, dy));
      const desired = edgeDistance(edge, source, target);
      const force = (distance - desired) * 0.008;
      source.x += (dx / distance) * force;
      source.y += (dy / distance) * force;
      target.x -= (dx / distance) * force;
      target.y -= (dy / distance) * force;
    });

    nodes.forEach((node) => {
      node.x += (centerX - node.x) * 0.004;
      node.y += (centerY - node.y) * 0.004;
      node.x = Math.max(150, Math.min(width - 150, node.x));
      node.y = Math.max(130, Math.min(height - 130, node.y));
    });
  }
}

function nodeClearance(node) {
  const label = node.label || node.id || "";
  const type = node.type || "Entity";
  return 88 + Math.min(90, Math.max(label.length, type.length) * 3.4);
}

function edgeDistance(edge, source, target) {
  const labelLength = String(edge.label || edge.type || "related_to").length;
  return Math.max(280, nodeClearance(source) + nodeClearance(target) + labelLength * 3.5);
}

function render() {
  if (!state.graph) return;
  ensureDefs();
  renderHeader();
  renderFilters();
  renderEdges();
  renderNodes();
  renderInspector();
  renderTrace();
  applyTransform();
}

function renderHeader() {
  const graph = state.graph;
  el.title.textContent = graph.graph_id || "Ontology Graph";
  el.meta.textContent = `${graph.nodes.length} nodes, ${graph.edges.length} edges, stamp ${graph.stamp ?? "n/a"}`;
}

function ensureDefs() {
  if (el.svg.querySelector("defs")) return;
  const defs = document.createElementNS("http://www.w3.org/2000/svg", "defs");
  defs.innerHTML = `
    <marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
      <path d="M 0 0 L 10 5 L 0 10 z" fill="#80909a"></path>
    </marker>
  `;
  el.svg.prepend(defs);
}

function renderFilters() {
  const nodeCounts = countBy(state.graph.nodes, (node) => node.type || "Entity");
  const edgeCounts = countBy(state.graph.edges, (edge) => edge.type || edge.label || "related_to");
  renderFilterList(el.nodeFilters, nodeCounts, state.hiddenNodeTypes, typeColor);
  renderFilterList(el.edgeFilters, edgeCounts, state.hiddenEdgeTypes, () => "#80909a");
}

function renderFilterList(container, counts, hiddenSet, colorFn) {
  container.innerHTML = "";
  [...counts.keys()].sort().forEach((type) => {
    const label = document.createElement("label");
    label.className = "filter-item";
    label.innerHTML = `
      <span class="swatch" style="background:${colorFn(type)}"></span>
      <span>${escapeHtml(type)}</span>
      <span class="count">${counts.get(type)}</span>
    `;
    const input = document.createElement("input");
    input.type = "checkbox";
    input.checked = !hiddenSet.has(type);
    input.addEventListener("change", () => {
      if (input.checked) hiddenSet.delete(type);
      else hiddenSet.add(type);
      render();
    });
    label.prepend(input);
    container.append(label);
  });
}

function renderEdges() {
  const byId = new Map(state.graph.nodes.map((node) => [node.id, node]));
  el.edgeLayer.innerHTML = "";
  state.graph.edges.forEach((edge, index) => {
    const source = byId.get(edge.source);
    const target = byId.get(edge.target);
    if (!source || !target) return;
    const id = edgeKey(edge);
    const group = document.createElementNS("http://www.w3.org/2000/svg", "g");
    const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
    const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
    const classes = ["edge"];
    if (edge.inferred) classes.push("inferred");
    if (isReasoningEdge(id)) classes.push("reasoning");
    if (isEdgeFiltered(edge)) classes.push("filtered");
    line.setAttribute("class", classes.join(" "));
    line.setAttribute("x1", source.x);
    line.setAttribute("y1", source.y);
    line.setAttribute("x2", target.x);
    line.setAttribute("y2", target.y);
    line.addEventListener("click", (event) => {
      event.stopPropagation();
      state.selected = { edge: id };
      renderInspector();
    });
    const dx = target.x - source.x;
    const dy = target.y - source.y;
    const distance = Math.max(1, Math.hypot(dx, dy));
    const offset = 18 + (index % 3) * 8;
    const direction = index % 2 === 0 ? 1 : -1;
    label.setAttribute("class", `edge-label${isEdgeFiltered(edge) ? " filtered" : ""}`);
    label.setAttribute("x", (source.x + target.x) / 2 + (-dy / distance) * offset * direction);
    label.setAttribute("y", (source.y + target.y) / 2 + (dx / distance) * offset * direction);
    label.textContent = edge.label || edge.type || "related_to";
    group.append(line, label);
    el.edgeLayer.append(group);
  });
}

function renderNodes() {
  el.nodeLayer.innerHTML = "";
  state.graph.nodes.forEach((node) => {
    const group = document.createElementNS("http://www.w3.org/2000/svg", "g");
    const classes = ["node"];
    if (state.selected === node.id || state.graph.selected?.node_id === node.id) classes.push("selected");
    if (isReasoningNode(node.id)) classes.push("reasoning");
    if (isNodeFiltered(node)) classes.push("filtered");
    group.setAttribute("class", classes.join(" "));
    group.setAttribute("transform", `translate(${node.x}, ${node.y})`);
    group.setAttribute("tabindex", "0");
    group.addEventListener("click", (event) => {
      event.stopPropagation();
      state.selected = node.id;
      render();
    });
    group.addEventListener("pointerdown", (event) => {
      event.stopPropagation();
      state.dragging = { node, start: pointerPoint(event), x: node.x, y: node.y };
      group.setPointerCapture(event.pointerId);
    });
    group.addEventListener("pointermove", (event) => {
      if (!state.dragging || state.dragging.node !== node) return;
      const point = pointerPoint(event);
      node.x = state.dragging.x + (point.x - state.dragging.start.x) / state.transform.scale;
      node.y = state.dragging.y + (point.y - state.dragging.start.y) / state.transform.scale;
      renderEdges();
      renderNodes();
    });
    group.addEventListener("pointerup", () => {
      state.dragging = null;
    });

    const circle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    circle.setAttribute("r", node.inferred ? "25" : "28");
    circle.setAttribute("fill", typeColor(node.type || "Entity"));
    const text = document.createElementNS("http://www.w3.org/2000/svg", "text");
    text.setAttribute("y", 45);
    text.textContent = node.label || node.id;
    const type = document.createElementNS("http://www.w3.org/2000/svg", "text");
    type.setAttribute("class", "type-label");
    type.setAttribute("y", 60);
    type.textContent = node.type || "Entity";
    group.append(circle, text, type);
    el.nodeLayer.append(group);
  });
}

function renderInspector() {
  const selected = getSelectedObject();
  el.metadata.textContent = JSON.stringify(selected || state.graph, null, 2);
}

function renderTrace() {
  const steps = state.graph.reasoning_trace || [];
  el.trace.innerHTML = "";
  if (!steps.length) {
    el.trace.textContent = "No reasoning trace in this snapshot.";
    return;
  }
  steps.forEach((step, index) => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = `trace-step${state.activeTrace === index ? " active" : ""}`;
    button.innerHTML = `
      <strong>Step ${step.step ?? index + 1}</strong>
      <span>${escapeHtml(step.description || "Inference step")}</span>
    `;
    button.addEventListener("click", () => {
      state.activeTrace = state.activeTrace === index ? null : index;
      render();
    });
    el.trace.append(button);
  });
}

function getSelectedObject() {
  if (!state.graph) return null;
  if (typeof state.selected === "object" && state.selected.edge) {
    return state.graph.edges.find((edge) => edgeKey(edge) === state.selected.edge);
  }
  return state.graph.nodes.find((node) => node.id === state.selected);
}

function activeTraceStep() {
  if (state.activeTrace == null || !state.showTrace) return null;
  return (state.graph.reasoning_trace || [])[state.activeTrace] || null;
}

function isReasoningNode(nodeId) {
  const step = activeTraceStep();
  return Boolean(step?.nodes?.includes(nodeId));
}

function isReasoningEdge(id) {
  const step = activeTraceStep();
  return Boolean(step?.edges?.includes(id));
}

function isNodeFiltered(node) {
  const text = `${node.id} ${node.label || ""} ${node.type || ""}`.toLowerCase();
  return state.hiddenNodeTypes.has(node.type || "Entity") || (state.search && !text.includes(state.search));
}

function isEdgeFiltered(edge) {
  const type = edge.type || edge.label || "related_to";
  const text = `${edge.id || ""} ${edge.label || ""} ${type}`.toLowerCase();
  return state.hiddenEdgeTypes.has(type) || (!state.showInferred && edge.inferred) || (state.search && !text.includes(state.search));
}

function countBy(items, fn) {
  const counts = new Map();
  items.forEach((item) => {
    const key = fn(item);
    counts.set(key, (counts.get(key) || 0) + 1);
  });
  return counts;
}

function resetView() {
  if (!state.graph?.nodes?.length) {
    state.transform = { x: 0, y: 0, scale: 1 };
    applyTransform();
    return;
  }
  const rect = el.svg.getBoundingClientRect();
  const width = Math.max(760, rect.width || el.svg.clientWidth || 900);
  const height = Math.max(520, rect.height || el.svg.clientHeight || 650);
  const xs = state.graph.nodes.map((node) => node.x);
  const ys = state.graph.nodes.map((node) => node.y);
  const minX = Math.min(...xs) - 140;
  const maxX = Math.max(...xs) + 140;
  const minY = Math.min(...ys) - 120;
  const maxY = Math.max(...ys) + 120;
  const graphWidth = Math.max(1, maxX - minX);
  const graphHeight = Math.max(1, maxY - minY);
  const scale = Math.min(1, (width - 56) / graphWidth, (height - 56) / graphHeight);
  state.transform = {
    x: (width - graphWidth * scale) / 2 - minX * scale,
    y: (height - graphHeight * scale) / 2 - minY * scale,
    scale,
  };
  applyTransform();
}

function applyTransform() {
  const { x, y, scale } = state.transform;
  el.viewport.setAttribute("transform", `translate(${x}, ${y}) scale(${scale})`);
}

function pointerPoint(event) {
  const rect = el.svg.getBoundingClientRect();
  return { x: event.clientX - rect.left, y: event.clientY - rect.top };
}

function setStatus(message, isError = false) {
  el.status.textContent = message;
  el.status.classList.toggle("error", isError);
}

function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, (char) => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    '"': "&quot;",
    "'": "&#039;",
  }[char]));
}

el.load.addEventListener("click", () => {
  stopLive();
  stopPolling();
  const source = el.source.value.trim();
  const snapshotSource = source.includes("/events") ? source.replace("/events", "/snapshot") : source;
  loadGraph(snapshotSource || "data/demo_graph.json").catch((error) => setStatus(error.message, true));
});

el.live.addEventListener("click", () => {
  if (state.eventSource) {
    stopLive();
    return;
  }
  startLive();
});

function startLive() {
  stopPolling();
  if (state.eventSource) return;
  state.liveWanted = true;
  const source = liveSource();
  state.eventSource = new EventSource(source);
  el.live.classList.add("active");
  setStatus(`waiting for live ${source}`);
  state.eventSource.onopen = () => setStatus(`live connected ${source}`);
  state.eventSource.addEventListener("graph", (event) => {
    setGraph(JSON.parse(event.data), `live update ${new Date().toLocaleTimeString()}`, {
      preserveFilters: true,
      preserveTrace: true,
      preserveView: true,
    });
  });
  state.eventSource.onerror = () => {
    if (state.liveWanted) setStatus(`waiting for live ${source}`);
  };
}

el.poll.addEventListener("click", () => {
  if (state.pollTimer) {
    stopPolling();
    return;
  }
  stopLive();
  pollGraph();
  state.pollTimer = window.setInterval(pollGraph, 1000);
  el.poll.classList.add("active");
});

function pollGraph() {
  const configured = el.source.value.trim();
  const source = configured.includes("/events")
    ? configured.replace("/events", "/snapshot")
    : (configured || "http://127.0.0.1:8765/snapshot");
  fetch(source, { cache: "no-store" })
    .then((response) => {
      if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
      return response.json();
    })
    .then((graph) => setGraph(graph, `polled ${new Date().toLocaleTimeString()}`, {
      preserveFilters: true,
      preserveTrace: true,
      preserveView: true,
    }))
    .catch((error) => setStatus(error.message, true));
}

function stopLive() {
  state.liveWanted = false;
  if (state.eventSource) {
    state.eventSource.close();
    state.eventSource = null;
  }
  el.live.classList.remove("active");
  setStatus("live disconnected");
}

function stopPolling() {
  if (!state.pollTimer) return;
  window.clearInterval(state.pollTimer);
  state.pollTimer = null;
  el.poll.classList.remove("active");
  setStatus("polling stopped");
}

window.addEventListener("beforeunload", () => {
  stopLive();
  stopPolling();
});

el.reset.addEventListener("click", () => {
  layoutGraph();
  resetView();
  render();
});

el.search.addEventListener("input", () => {
  state.search = el.search.value.trim().toLowerCase();
  render();
});

el.inferred.addEventListener("change", () => {
  state.showInferred = el.inferred.checked;
  render();
});

el.traceToggle.addEventListener("change", () => {
  state.showTrace = el.traceToggle.checked;
  render();
});

el.svg.addEventListener("click", () => {
  state.selected = null;
  render();
});

el.svg.addEventListener("pointerdown", (event) => {
  if (event.target !== el.svg) return;
  state.panning = { start: pointerPoint(event), x: state.transform.x, y: state.transform.y };
  el.svg.classList.add("dragging");
});

el.svg.addEventListener("pointermove", (event) => {
  if (!state.panning) return;
  const point = pointerPoint(event);
  state.transform.x = state.panning.x + point.x - state.panning.start.x;
  state.transform.y = state.panning.y + point.y - state.panning.start.y;
  applyTransform();
});

window.addEventListener("pointerup", () => {
  state.panning = null;
  el.svg.classList.remove("dragging");
});

el.svg.addEventListener("wheel", (event) => {
  event.preventDefault();
  const point = pointerPoint(event);
  const oldScale = state.transform.scale;
  const newScale = Math.max(0.25, Math.min(3, oldScale * (event.deltaY < 0 ? 1.1 : 0.9)));
  state.transform.x = point.x - ((point.x - state.transform.x) / oldScale) * newScale;
  state.transform.y = point.y - ((point.y - state.transform.y) / oldScale) * newScale;
  state.transform.scale = newScale;
  applyTransform();
}, { passive: false });

loadGraph("data/demo_graph.json")
  .catch((error) => setStatus(error.message, true))
  .finally(() => startLive());
