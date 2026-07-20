/* ACS 교통관제 검증 웹 — 맵 렌더링.
 *
 * 왜 SVG 인가:
 *   ① 통로 25개가 각각 DOM 요소라, 예약 상태가 바뀌면 그 선의 class 만 바꾸면 된다
 *      (Canvas 는 매번 전체를 다시 그려야 한다).
 *   ② 5단계에서 "통로를 클릭해 막기"를 붙일 때 onclick 을 그냥 달면 된다.
 *   ③ viewBox 에 실제 좌표 범위를 넣으면 픽셀 변환을 브라우저가 대신 해준다.
 *
 * 레이어는 아래에서 위로: 배경(베드·로봇팔) → 시설(충전/수확/예냉) → 통로 → 노드 → 로봇.
 * 아래일수록 안 변하고 위일수록 자주 변한다. 로봇 레이어는 4단계에서 채운다.
 */
const SVG_NS = "http://www.w3.org/2000/svg";

// ?theme=dark / ?theme=light 로 테마를 고정할 수 있다(없으면 OS 설정을 따른다).
const themeParam = new URLSearchParams(location.search).get("theme");
if (themeParam === "dark" || themeParam === "light") {
  document.documentElement.setAttribute("data-theme", themeParam);
}

/** SVG 요소 생성 헬퍼. attrs 를 그대로 setAttribute 한다. */
function el(name, attrs = {}, text = null) {
  const node = document.createElementNS(SVG_NS, name);
  for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
  if (text !== null) node.textContent = text;
  return node;
}

/**
 * 좌표 변환 — 여기가 맵 방향을 결정하는 유일한 지점이다.
 *
 * DB 좌표(m)를 화면 좌표(cm)로 바꾼다. 두 축 모두 뒤집힌다:
 *   DB x 가 클수록 → 화면 왼쪽   (그래서 room.x1 에서 뺀다)
 *   DB y 가 클수록 → 화면 아래   (SVG 는 y 가 아래로 증가하므로 원점만 옮긴다)
 * 근거: 로봇 전용 충전소(22·23·24, x 0.47~0.77 / y≈0.92)가 실제 도면에서 '왼쪽 아래'다.
 *
 * ×100 은 m→cm. viewBox 가 0~1 범위면 선 굵기를 0.005 같은 소수로 써야 해 다루기 나쁘다.
 * cm 로 두면 "선 굵기 0.9", "방울토마토 반지름 1.0(=지름 2cm)" 처럼 읽히는 값이 된다.
 */
function makeTransform(room) {
  const S = 100;
  return {
    x: (x) => (room.x1 - x) * S,
    y: (y) => (y - room.y0) * S,
    w: (room.x1 - room.x0) * S,
    h: (room.y1 - room.y0) * S,
  };
}

/**
 * 재배 베드용 방울토마토 패턴.
 *
 * 타일 단위가 cm 라 열매 반지름 0.7 = 지름 1.4cm — 실제 방울토마토 크기다.
 * 일부러 작고 성기게 깐다: 베드는 '배경'이라 데이터(통로·노드·로봇)보다 튀면 안 된다.
 * 무늬가 크고 촘촘하면 시선이 베드에 먼저 가서 정작 봐야 할 예약 상태를 놓친다.
 */
function buildDefs() {
  const defs = el("defs");
  const TILE = 14;
  const p = el("pattern", {
    id: "bed-tomato", width: TILE, height: TILE, patternUnits: "userSpaceOnUse",
  });
  p.appendChild(el("rect", { width: TILE, height: TILE, class: "bed-soil" }));
  // 잎(뭉치) → 그 위에 열매 → 하이라이트. 2송이를 엇갈리게 둬 반복이 티나지 않게.
  for (const [cx, cy, r] of [[3.6, 4.0, 2.4], [10.2, 10.4, 2.1]]) {
    p.appendChild(el("circle", { cx, cy, r, class: "bed-leaf" }));
  }
  for (const [cx, cy] of [[3.0, 4.4], [4.3, 3.5], [9.7, 11.0], [10.9, 10.0]]) {
    p.appendChild(el("circle", { cx, cy, r: 0.7, class: "tomato" }));
    p.appendChild(el("circle", { cx: cx - 0.22, cy: cy - 0.25, r: 0.2, class: "tomato-hi" }));
  }
  defs.appendChild(p);
  return defs;
}

/** 배경 레이어: 방 외곽 + 구조물(재배 베드 / 로봇팔). */
function renderBackground(layer, layout, T) {
  layer.appendChild(el("rect", {
    x: T.x(layout.room.x1), y: T.y(layout.room.y0),
    width: T.w, height: T.h, rx: 3, class: "room-wall",
  }));

  for (const s of layout.structures) {
    // x 축이 뒤집혀 있어 x1 이 화면 왼쪽(=rect 의 시작점)이 된다.
    const left = T.x(s.x1);
    const top = T.y(s.y0);
    const w = T.x(s.x0) - left;
    const h = T.y(s.y1) - top;
    const isBed = s.kind === "bed";
    layer.appendChild(el("rect", {
      x: left, y: top, width: w, height: h, rx: 1.5,
      class: isBed ? "struct-bed" : "struct-arm",
    }));
    // 베드는 이름을 안 쓴다 — 방울토마토 무늬로 이미 무엇인지 읽히고,
    // 글자를 얹으면 그 위를 지나는 통로·노드와 겹쳐 지저분해진다.
    if (!isBed) {
      layer.appendChild(el("text", {
        x: left + w / 2, y: top + h / 2 + 1.1, class: "struct-label",
      }, s.label));
    }
  }
}

const FACILITY_LABEL = { CHARGE: "충전", HARVEST: "수확대", PRECOOL: "예냉실" };

/**
 * 시설 레이어: task_points(충전소·수확대·예냉실)를 DB 좌표에 그린다.
 *
 * 박스를 노드 위에 겹치지 않고 '바깥쪽'(벽 방향)으로 밀어 그린다. 노드가 박스 안쪽
 * 모서리에 걸리게 되어, 박스 = 시설 공간 / 노드 = 그 시설의 진입 지점으로 읽힌다.
 * 겹쳐 그리면 노드 원이 라벨 위에 올라앉아 글자가 안 보인다.
 */
function renderFacilities(layer, graph, T, room) {
  const W = 0.135, H = 0.105;         // 시설 박스 크기(m)
  const midY = (room.y0 + room.y1) / 2;
  for (const t of graph.task_points) {
    const outward = t.y > midY ? 1 : -1;     // 아래쪽 시설은 아래로, 위쪽 시설은 위로
    const yNear = t.y;                       // 노드가 걸리는 모서리
    const yFar = t.y + outward * H;          // 벽 쪽 모서리
    const left = T.x(t.x + W / 2);
    const w = T.x(t.x - W / 2) - left;
    const top = Math.min(T.y(yNear), T.y(yFar));
    const h = Math.abs(T.y(yFar) - T.y(yNear));

    layer.appendChild(el("rect", {
      x: left, y: top, width: w, height: h, rx: 1.2,
      class: `facility facility-${t.point_type.toLowerCase()}`,
    }));
    const cx = left + w / 2;
    const cy = top + h / 2;
    const name = FACILITY_LABEL[t.point_type] || t.point_type;
    // 충전소는 어느 로봇 자리인지가 핵심 정보다(순찰 출발 노드가 여기서 정해진다).
    const sub = t.robots.length ? t.robots.join(",") : `wp${t.waypoint_id}`;
    // 글자를 노드 반대쪽(벽 쪽)으로 밀어 노드 원에 가리지 않게 한다.
    // outward=+1 이면 노드가 박스 위 모서리에 있으므로 글자는 아래로, -1 이면 반대.
    const shift = outward * 1.4;
    layer.appendChild(el("text", { x: cx, y: cy - 0.6 + shift, class: "facility-name" }, name));
    layer.appendChild(el("text", { x: cx, y: cy + 2.9 + shift, class: "facility-sub" }, sub));
  }
}

/**
 * 통로 레이어: 간선 하나당 보이는 <line> + 투명한 히트 <line> 한 쌍.
 *
 * 히트 라인이 왜 필요한가: 통로 선은 굵기가 1(=1cm)이라 그대로는 클릭이 거의 불가능하다.
 * 같은 좌표에 굵기 4 짜리 투명 선을 겹쳐 두면 눌리는 범위만 넓어진다(보이는 선은 그대로).
 */
function renderCorridors(layer, graph, T, byId) {
  for (const c of graph.corridors) {
    const a = byId.get(c.a);
    const b = byId.get(c.b);
    if (!a || !b) continue;
    const pts = { x1: T.x(a.x), y1: T.y(a.y), x2: T.x(b.x), y2: T.y(b.y) };
    layer.appendChild(el("line", {
      ...pts, id: `corridor-${c.corridor_id}`, class: "corridor",
    }));
    const hit = el("line", { ...pts, class: "corridor-hit" });
    hit.addEventListener("click", () => toggleCorridor(c.corridor_id));
    const t = document.createElementNS(SVG_NS, "title");
    t.textContent = `통로 ${c.corridor_id} (${c.a}-${c.b}) · ${(c.length * 100).toFixed(0)}cm`
      + " — 클릭하면 막기/해제";
    hit.appendChild(t);
    layer.appendChild(hit);
  }
}

/** LIVE 는 관측 전용 — 맵 클릭도 막는다(서버도 거절하지만 사유를 여기서 바로 알린다). */
function liveBlocked() {
  if (VIEW.mode !== "LIVE") return false;
  flash("LIVE(실물 관측) 모드에서는 조작할 수 없습니다");
  return true;
}

/** 통로 클릭 → 막힘 토글. 지금 막혀 있는지는 마지막 상태(VIEW.blocked)로 판단한다. */
function toggleCorridor(cid) {
  if (liveBlocked()) return;
  const action = VIEW.blocked && VIEW.blocked.has(String(cid)) ? "unblock" : "block";
  fetch(`/api/corridor/${cid}/${action}`, { method: "POST" });
}

/** 노드 클릭 → 선택된 로봇을 그 지점으로 이동시킨다. */
function gotoNode(wpId) {
  if (liveBlocked()) return;
  const sel = document.getElementById("sel-robot");
  if (!sel || !sel.value) return;
  fetch(`/api/goto/${sel.value}/${wpId}`, { method: "POST" });
}

/**
 * 노드 레이어.
 *
 * 짝(pair)은 별도 점으로 그리지 않는다 — 부모와 x·y 가 '완전히 같아서' 점을 두 개 찍으면
 * 정확히 겹쳐 "왜 하나만 보이지?"가 된다. 대신 부모 옆에 ↻ 배지를 달아
 * "여기서 제자리 회전해 반대 방향으로 한 번 더 촬영한다"를 보여준다.
 */
function renderNodes(layer, graph, T, opts) {
  const { routing, pairByParent, facilityWps, startWps } = opts;
  for (const w of graph.waypoints) {
    if (!routing.has(w.waypoint_id)) continue;      // 짝은 건너뜀
    const cx = T.x(w.x);
    const cy = T.y(w.y);
    const isFacility = facilityWps.has(w.waypoint_id);
    const isStart = startWps.has(w.waypoint_id);

    let cls = "node-via";
    let r = 1.5;
    if (isStart) { cls = "node-start"; r = 1.9; }
    else if (isFacility) { cls = "node-facility"; r = 1.4; }
    else if (w.is_patrol_point) { cls = "node-patrol"; r = 1.9; }

    layer.appendChild(el("circle", { cx, cy, r, class: cls }));

    // 시설 노드는 시설 박스가 이미 이름을 달고 있어 번호를 또 쓰면 겹친다 → 생략.
    if (!isFacility) {
      // 맨 윗줄(y<-0.3)은 위쪽에 로봇팔·수확대가 있어 라벨을 아래로 내린다.
      const below = w.y < -0.3;
      layer.appendChild(el("text", {
        x: cx, y: below ? cy + r + 3.2 : cy - r - 1.3, class: "node-id",
      }, String(w.waypoint_id)));
    }

    // 순찰 순번 배지
    if (w.patrol_order !== null && w.patrol_order !== undefined) {
      layer.appendChild(el("circle", { cx: cx - 3.0, cy: cy + 3.0, r: 1.9, class: "order-badge" }));
      layer.appendChild(el("text", {
        x: cx - 3.0, y: cy + 3.75, class: "order-text",
      }, String(w.patrol_order)));
    }

    // 짝 배지(↻) — 이 지점에서 제자리 회전 촬영이 한 번 더 있다는 표시
    const pair = pairByParent.get(w.waypoint_id);
    if (pair) {
      layer.appendChild(el("text", {
        x: cx + 3.2, y: cy - 2.0, class: "pair-badge",
      }, `↻${pair.pair}`));
    }

    // 투명한 넓은 히트 영역 — 노드 원이 작아(반지름 1.5cm) 그대로는 누르기 어렵다.
    const hit = el("circle", { cx, cy, r: 3.6, class: "node-hit" });
    hit.addEventListener("click", () => gotoNode(w.waypoint_id));
    const t = document.createElementNS(SVG_NS, "title");
    t.textContent = `waypoint ${w.waypoint_id}` +
      (w.patrol_order != null ? ` · 순찰 ${w.patrol_order}번` : "") +
      " — 클릭하면 선택한 로봇을 여기로";
    hit.appendChild(t);
    layer.appendChild(hit);
  }
}

function render(graph, layout) {
  const T = makeTransform(layout.room);
  const svg = document.getElementById("map");
  svg.setAttribute("viewBox", `0 0 ${T.w} ${T.h}`);
  svg.replaceChildren();
  svg.appendChild(buildDefs());

  const byId = new Map(graph.waypoints.map((w) => [w.waypoint_id, w]));
  const routing = new Set(graph.routing_node_ids);
  const pairByParent = new Map(graph.pairs.map((p) => [p.parent, p]));
  const facilityWps = new Set(graph.task_points.map((t) => t.waypoint_id));
  const startWps = new Set(Object.values(graph.patrol_start));

  const layers = {};
  for (const name of ["bg", "facilities", "corridors", "nodes", "robots"]) {
    layers[name] = el("g", { id: `layer-${name}` });
    svg.appendChild(layers[name]);      // 먼저 붙인 것이 아래에 깔린다
  }

  renderBackground(layers.bg, layout, T);
  renderFacilities(layers.facilities, graph, T, layout.room);
  renderCorridors(layers.corridors, graph, T, byId);
  renderNodes(layers.nodes, graph, T, { routing, pairByParent, facilityWps, startWps });

  document.getElementById("graph-summary").textContent =
    `노드 ${graph.waypoints.length}(라우팅 ${routing.size} · 짝 ${graph.pairs.length}) · `
    + `통로 ${graph.corridors.length} · 시설 ${graph.task_points.length}`;

  // 순찰 순서 — 짝은 독립 순찰 지점이 아니므로 부모 뒤에 ↻로 표기한다.
  const order = graph.waypoints
    .filter((w) => routing.has(w.waypoint_id)
      && w.patrol_order !== null && w.patrol_order !== undefined)
    .sort((a, b) => a.patrol_order - b.patrol_order)
    .map((w) => {
      const p = pairByParent.get(w.waypoint_id);
      return p ? `${w.waypoint_id}↻${p.pair}` : String(w.waypoint_id);
    });
  document.getElementById("patrol-order").textContent = order.join(" → ");

  const starts = Object.entries(graph.patrol_start)
    .map(([rid, wp]) => `${rid}→${wp}`).join(" · ");
  document.getElementById("robot-start").textContent = starts || "(등록된 충전소 없음)";

  // 실시간 갱신에서 다시 쓸 것들을 보관한다.
  // 통로 <line> 을 미리 찾아두는 이유: 10Hz 로 갱신할 때마다 DOM 을 뒤지지 않게 하려고.
  VIEW.T = T;
  VIEW.graph = graph;
  VIEW.robotsLayer = layers.robots;
  VIEW.corridorEls = new Map(
    graph.corridors.map((c) => [String(c.corridor_id),
      document.getElementById(`corridor-${c.corridor_id}`)]));
  VIEW.robotColor = new Map(
    Object.keys(graph.patrol_start).sort().map((rid, i) => [rid, (i % 3) + 1]));
  buildControls(graph);
}

/* ------------------------------------------------------------------ *
 * 실시간 상태 (WebSocket)
 * ------------------------------------------------------------------ */
const VIEW = {};

/** 조작 막대 구성: 순찰 버튼 · 이동 대상 로봇 선택 · 시나리오 · 초기화. */
function buildControls(graph) {
  const ids = Object.keys(graph.patrol_start).sort();

  const box = document.getElementById("patrol-buttons");
  box.replaceChildren();
  for (const rid of ids) {
    const b = document.createElement("button");
    b.textContent = rid;
    b.className = `run r${VIEW.robotColor.get(rid)}`;
    // 순찰은 전역 1건 제한이 있어 거절될 수 있다 → 사유를 화면에 알려준다.
    b.onclick = async () => {
      const res = await fetch(`/api/patrol/${rid}`, { method: "POST" });
      if (!res.ok) {
        const j = await res.json().catch(() => ({}));
        flash(`순찰 거절: ${j.reason || res.status}`);
      }
    };
    box.appendChild(b);
  }

  const sel = document.getElementById("sel-robot");
  sel.replaceChildren();
  for (const rid of ids) {
    const o = document.createElement("option");
    o.value = rid;
    o.textContent = rid;
    sel.appendChild(o);
  }

  document.getElementById("btn-deadlock").onclick =
    () => fetch("/api/scenario/deadlock", { method: "POST" });
  document.getElementById("btn-reset").onclick =
    () => fetch("/api/reset", { method: "POST" });
  document.getElementById("mode-sim").onclick = () => setMode("SIM");
  document.getElementById("mode-live").onclick = () => setMode("LIVE");
}

/* ------------------------------------------------------------------ *
 * SIM / LIVE 전환
 *
 * 바뀌는 건 '데이터가 어디서 오는가' 뿐이다. 맵도, 통로 색칠도, 로봇 렌더도
 * 그대로 재사용한다 — 서버가 두 소스를 같은 모양의 스냅샷으로 맞춰 주기 때문.
 * 화면이 해야 할 일은 두 가지다: (1) 지금 어느 쪽인지 분명히 보이기,
 * (2) LIVE 에서 조작 UI 를 잠그기. LIVE 는 실물을 들여다보기만 하므로,
 * 여기서 순찰 버튼을 눌러 봐야 화면엔 아무 변화도 없어 사용자만 헷갈린다.
 * ------------------------------------------------------------------ */
function applyMode(mode) {
  if (VIEW.mode === mode) return;
  VIEW.mode = mode;
  const live = mode === "LIVE";

  document.getElementById("mode-sim").className = live ? "" : "on";
  document.getElementById("mode-live").className = live ? "on" : "";
  document.body.classList.toggle("live", live);

  // 조작 막대 전체를 비활성화한다(개별 버튼을 일일이 챙기지 않아도 되게).
  for (const el of document.querySelectorAll("#controls button, #controls select")) {
    el.disabled = live;
  }
  document.getElementById("ctl-hint").innerHTML = live
    ? "<b>LIVE(실물 관측)</b> — 조작할 수 없습니다. "
      + "통로 색은 실제 ACS 의 예약표, 로봇 위치는 텔레메트리 좌표입니다."
    : "<b>노드 클릭</b> → 선택한 로봇을 그곳으로 이동 · "
      + "<b>통로 클릭</b> → 막기/해제(로봇이 그 구간에서 막힘을 보고해 우회한다)";

  // 두 소스는 이벤트 번호 체계가 달라 섞이면 시간축이 뒤죽박죽이 된다 → 비운다.
  document.getElementById("events").replaceChildren();
}

async function setMode(mode) {
  const res = await fetch(`/api/mode/${mode}`, { method: "POST" });
  const j = await res.json().catch(() => ({}));
  if (!res.ok) flash(`모드 전환 실패: ${j.reason || res.status}`);
}

/** 거절 사유 등을 잠깐 띄운다(조작이 씹힌 것처럼 보이지 않게). */
function flash(text) {
  const c = document.getElementById("conn");
  const keep = c.textContent;
  const keepCls = c.className;
  c.textContent = text;
  c.className = "conn off";
  setTimeout(() => { c.textContent = keep; c.className = keepCls; }, 2500);
}

/**
 * 통로 색칠 — 이 화면의 핵심.
 * RoutingEngine 의 예약표가 그대로 선 색이 된다. 우선순위는
 *   막힘(사용자가 만든 물리적 장애) > 예약(누가 쥐고 있나) > 회피(블랙리스트) > 빈 통로.
 * 색만으로 구분하지 않는다 — 굵기와 점선이 함께 바뀐다(적록색맹 대응).
 */
function updateCorridors(msg) {
  const avoiding = new Set(msg.avoiding.map(String));
  const blocked = new Set(msg.blocked.map(String));
  VIEW.blocked = blocked;          // 통로 클릭 시 막기/해제 판단에 쓴다
  for (const [cid, elLine] of VIEW.corridorEls) {
    if (!elLine) continue;
    const holder = msg.reservations[cid];
    let cls = "corridor";
    if (blocked.has(cid)) cls += " c-blocked";
    else if (holder) cls += ` c-res r${VIEW.robotColor.get(holder) || 1}`;
    else if (avoiding.has(cid)) cls += " c-avoid";
    elLine.setAttribute("class", cls);
  }
}

/** 로봇 레이어: 3대뿐이라 매 틱 통째로 다시 그린다(간단하고 충분히 빠르다). */
function renderRobots(msg) {
  const T = VIEW.T;
  const layer = VIEW.robotsLayer;
  layer.replaceChildren();
  for (const r of msg.robots) {
    // LIVE 에서 한 번도 텔레메트리를 못 받은 로봇은 좌표가 없다. 0,0 으로 채워
    // 그리면 '있지도 않은 로봇'이 맵 구석에 생기므로 아예 안 그린다(목록엔 남는다).
    if (r.x === null || r.y === null || r.x === undefined) continue;
    const cx = T.x(r.x);
    const cy = T.y(r.y);
    const n = VIEW.robotColor.get(r.robot_id) || 1;
    const active = r.status === "RUNNING";
    // LIVE 에서 텔레메트리가 끊긴 로봇은 '마지막으로 본 자리'일 뿐 지금 거기 있다는
    // 보장이 없다. 목록에서만 흐려지면 맵에서는 여전히 살아있는 것처럼 보이므로
    // 점선 테두리로 바꿔 '확인되지 않은 위치'임을 맵에서도 드러낸다.
    const stale = VIEW.mode === "LIVE" && r.online === false ? " stale" : "";

    // 진행 방향 표시. 화면 x 축이 뒤집혀 있으므로 x 성분만 부호를 바꾼다.
    const L = 4.2;
    layer.appendChild(el("line", {
      x1: cx, y1: cy,
      x2: cx - Math.cos(r.yaw) * L, y2: cy + Math.sin(r.yaw) * L,
      class: `robot-heading r${n}`,
    }));
    layer.appendChild(el("circle", {
      cx, cy, r: 2.6,
      class: `robot r${n}${active ? "" : " idle"}${stale}`,
    }));
    // 회전 중이면 테두리 링을 하나 더 — '안 움직이는데 통로를 쥐고 있는' 이유가 보이게.
    if (r.spinning) {
      layer.appendChild(el("circle", { cx, cy, r: 4.0, class: `robot-spin r${n}` }));
    }
    layer.appendChild(el("text", {
      x: cx, y: cy - 4.6, class: `robot-label r${n}${stale}`,
    }, r.robot_id));
  }
}

const STATUS_KO = { IDLE: "대기", RUNNING: "순찰 중", DONE: "종료" };

function renderRobotList(msg) {
  const ul = document.getElementById("robot-list");
  ul.replaceChildren();
  for (const r of msg.robots) {
    const n = VIEW.robotColor.get(r.robot_id) || 1;
    const li = document.createElement("li");
    const live = VIEW.mode === "LIVE";
    // LIVE 는 로봇이 자기 노드 번호를 모른다 — 좌표에서 역추정한 값이라 '~' 를 붙여
    // 추정임을 드러낸다. 통로 한복판이면 서버가 null 을 주므로 '주행 중'이 된다.
    const where = (live && !r.online) ? "마지막 위치"
      : r.spinning ? "회전 중"
      : r.moving ? "주행 중"
      : `${live ? "~" : ""}wp${r.waypoint_id ?? "-"}`;
    let state = r.result || STATUS_KO[r.status] || r.status;
    if (live) {
      // 실물에서 의미 있는 건 '접속 여부와 배터리'다(순찰 결과는 ACS 가 DB 에 쓴다).
      if (!r.online) state = `미수신${r.age_sec != null ? ` ${r.age_sec}s` : ""}`;
      else if (r.battery_percent != null) state += ` · ${Math.round(r.battery_percent)}%`;
    }
    li.className = live && !r.online ? "offline" : "";
    li.innerHTML = `<span class="chip r${n}"></span>`
      + `<b>${r.robot_id}</b><span class="rstate">${state} · ${where}</span>`;
    ul.appendChild(li);
  }
}

function renderEvents(events) {
  if (!events.length) return;
  const box = document.getElementById("events");
  for (const e of events) {
    const div = document.createElement("div");
    div.className = `ev ${e.level === "WARN" ? "warn" : ""}`;
    div.textContent = `${e.t.toFixed(1)}s  ${e.msg}`;
    box.appendChild(div);
  }
  while (box.childElementCount > 120) box.removeChild(box.firstChild);
  box.scrollTop = box.scrollHeight;
  document.getElementById("event-count").textContent = `(${box.childElementCount})`;
}

function applyState(msg) {
  applyMode(msg.mode || "SIM");
  updateConn(msg);
  updateCorridors(msg);
  renderRobots(msg);
  renderRobotList(msg);
  renderEvents(msg.events || []);
}

/**
 * 접속 표시등. SIM 은 WebSocket 만 살아있으면 끝이지만, LIVE 는 사슬이 하나 더 있다:
 *   브라우저 ──WS── verify_web ──HTTP── ACS
 * 뒤쪽이 끊겼는데 앞쪽만 보고 '연결됨'이라 하면 멈춘 화면을 실시간이라 착각한다.
 */
function updateConn(msg) {
  const c = document.getElementById("conn");
  if (msg.mode !== "LIVE") {
    c.textContent = "● 연결됨";
    c.className = "conn ok";
    return;
  }
  if (msg.connecting) {
    c.textContent = "◌ ACS 연결 확인 중…";
    c.className = "conn";
    c.title = "";
  } else if (!msg.connected) {
    c.textContent = "○ ACS 미연결";
    c.className = "conn off";
    c.title = msg.error || "";
  } else if (!msg.engine_ready) {
    // 예약표는 첫 순찰 때 생긴다 — '비어 있음'과 '아직 없음'은 다른 상태다.
    c.textContent = "● ACS 연결 · 순찰 대기";
    c.className = "conn warn";
    c.title = "ACS 의 RoutingEngine 은 첫 순찰 때 만들어진다(아직 예약표 없음)";
  } else {
    c.textContent = "● ACS 연결 · 교통관제 가동";
    c.className = "conn ok";
    c.title = "";
  }
}

/** WebSocket 연결. 끊기면 자동 재접속한다(서버 재시작 후에도 화면이 살아나게). */
function connectWS() {
  const conn = document.getElementById("conn");
  const ws = new WebSocket(`ws://${location.host}/ws/state`);
  ws.onopen = () => { conn.textContent = "● 연결됨"; conn.className = "conn ok"; };
  ws.onmessage = (ev) => {
    try {
      applyState(JSON.parse(ev.data));
    } catch (err) {
      console.error("상태 처리 실패", err);
    }
  };
  ws.onclose = () => {
    conn.textContent = "○ 끊김 — 재연결 중";
    conn.className = "conn off";
    setTimeout(connectWS, 1500);
  };
  ws.onerror = () => ws.close();
}

async function main() {
  try {
    const [graph, layout] = await Promise.all([
      fetch("/api/graph").then((r) => r.json()),
      fetch("/api/layout").then((r) => r.json()),
    ]);
    if (graph.error) throw new Error(graph.message || graph.error);
    render(graph, layout);
    // LIVE 를 쓸 수 있는지(=서버가 ACS 주소를 들고 있는지) 미리 물어본다.
    // 없으면 버튼을 눌러도 실패할 뿐이니 아예 비활성화하고 이유를 툴팁에 적는다.
    fetch("/api/mode").then((r) => r.json()).then((m) => {
      const b = document.getElementById("mode-live");
      b.disabled = !m.live_available;
      b.title = m.live_available
        ? `실물 ACS 관측 — ${m.acs_base}`
        : "LIVE 소스가 준비되지 않았습니다";
    }).catch(() => {});
    connectWS();          // 맵을 다 그린 뒤에 상태 스트림을 붙인다
  } catch (err) {
    document.getElementById("graph-summary").textContent = `맵 로드 실패: ${err.message}`;
  }
}

main();
