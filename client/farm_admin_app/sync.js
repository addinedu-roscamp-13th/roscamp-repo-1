/* ============================================================
 * 실시간 공유 상태 동기화 레이어
 *  - 서버(/api/state, /api/action)가 mode/follow/zones/todayKg 보관
 *  - 한 사람이 바꾸면 모든 접속자가 ~1초 내 같은 상태로 갱신
 *  - 원격 변경은 토스트로 알리고, 접속자 수를 배지에 표시
 * ============================================================ */
(function () {
  var ver = -1;
  var cid = "c" + Math.random().toString(36).slice(2) + Date.now().toString(36);
  var prev = null; // 직전 상태(원격 변경 비교용)

  function t(msg) {
    try { if (typeof toast === "function") toast(msg); } catch (e) {}
  }

  // 현재 보고 있는 화면만 다시 그린다 (각자 보던 화면은 유지)
  function rerenderActive() {
    var secs = ["admin", "farm", "worker"];
    var sec = secs.find(function (s) {
      var el = document.getElementById(s);
      return el && !el.classList.contains("hidden");
    });
    try {
      if (sec === "admin") {
        var ab = document.querySelector("#adminNav button.active");
        renderAdmin(ab ? ab.dataset.v : "a_dash");
      } else if (sec === "farm") {
        var fb = document.querySelector("#farmNav button.active");
        renderFarm(fb ? fb.dataset.v : "f_dash");
      } else if (sec === "worker") {
        var wb = document.querySelector("#workerNav button.active");
        var v = wb ? wb.dataset.v : "w_home";
        // 입력 중 화면(수동등록/원격)은 덮어쓰지 않음
        if (v !== "w_manual" && v !== "w_teleop") renderWorker(v);
      }
    } catch (e) {}
  }

  // 원격 변경을 사람이 읽을 수 있게 알림
  function announce(o, n) {
    var msgs = [];
    if (o.mode !== n.mode)
      msgs.push("다른 팀원이 " + (n.mode === "night" ? "야간" : "주간") + " 모드로 전환했습니다");
    if (o.follow !== n.follow)
      msgs.push("다른 팀원이 추종 모드를 " + (n.follow ? "다시 시작" : "해제") + "했습니다");
    (n.zones || []).forEach(function (z, i) {
      if (o.zones && o.zones[i] !== z.st && z.st === "수확 진행") {
        var zn = (S.zones[i] && S.zones[i].z) || ("구역" + (i + 1));
        msgs.push("다른 팀원이 " + zn + " 수확을 승인했습니다");
      }
    });
    if (typeof o.todayKg === "number" && n.todayKg > o.todayKg)
      msgs.push("수확량 +" + (n.todayKg - o.todayKg).toFixed(1) + "kg 합산되었습니다");
    if (msgs.length) t("🔔 " + msgs.join(" · "));
  }

  function applyRemote(d, fromRemote) {
    if (typeof S === "undefined" || !d) return;
    if (fromRemote && prev) { try { announce(prev, d); } catch (e) {} }
    S.mode = d.mode;
    S.follow = d.follow;
    S.todayKg = d.todayKg;
    (d.zones || []).forEach(function (z, i) {
      if (S.zones[i]) S.zones[i].st = z.st;
    });
    prev = {
      mode: d.mode, follow: d.follow, todayKg: d.todayKg,
      zones: (d.zones || []).map(function (z) { return z.st; }),
    };
    rerenderActive();
  }

  async function pull() {
    try {
      var r = await fetch("/api/state?cid=" + cid, { cache: "no-store" });
      var d = await r.json();
      updateBadge(d.viewers, d.mode);
      if (d.version !== ver) { ver = d.version; applyRemote(d, true); }
    } catch (e) {}
  }

  async function send(type, payload) {
    try {
      var r = await fetch("/api/action", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ type: type, payload: payload }),
      });
      var d = await r.json();
      ver = d.version;
      updateBadge(d.viewers, d.mode);
      applyRemote(d, false);
    } catch (e) {}
  }

  function updateBadge(viewers, mode) {
    var b = document.getElementById("liveBadge");
    if (!b) return;
    var v = typeof viewers === "number" ? viewers : 1;
    b.innerHTML = "👥 " + v + "명 접속";   // 주간/야간은 상단 서버시계가 담당(중복 제거)
  }

  function addLiveBadge() {
    try {
      var b = document.createElement("div");
      b.id = "liveBadge";
      b.innerHTML = "● 실시간 공유 연결 중…";
      b.style.cssText =
        "position:fixed;right:14px;bottom:14px;z-index:80;background:#1f7a4d;color:#fff;" +
        "font:600 12px/1 'Pretendard',system-ui,sans-serif;padding:8px 12px;border-radius:999px;" +
        "box-shadow:0 6px 20px rgba(16,35,26,.3)";
      document.body.appendChild(b);
    } catch (e) {}
  }

  function wire() {
    if (typeof S === "undefined") { setTimeout(wire, 120); return; }

    window.toggleMode = function () { send("toggleMode"); t("운영 모드 전환 · 전체 팀에 공유됨"); };
    window.toggleFollow = function () { send("toggleFollow"); };
    window.approve = function (i) {
      var z = S.zones[i];
      send("approve", { i: i });
      t("✅ " + (z ? z.z : "") + " 즉시 수확 승인 · 전체 팀에 공유됨");
    };
    window.submitManual = function () {
      var el = document.getElementById("manW");
      var kg = el ? parseFloat(el.textContent) || 0 : 0;
      var g = "수확분";
      try { if (typeof manGrade !== "undefined") g = manGrade; } catch (e) {}
      if (kg > 0) {
        send("addHarvest", { kg: kg });
        t("✅ " + g + " " + kg.toFixed(1) + "kg 등록 · 전체 합계에 반영됨");
      } else {
        t("무게를 입력하세요");
      }
    };
    window.resetDemo = function () {
      send("reset");
      t("데모 상태를 초기화했습니다 · 전체 공유");
    };

    addLiveBadge();
    pull();
    setInterval(pull, 1000);
    // 탭을 다시 보거나 창에 포커스가 오면 즉시 동기화 (백그라운드 throttle 보완)
    document.addEventListener("visibilitychange", function () {
      if (!document.hidden) pull();
    });
    window.addEventListener("focus", pull);
  }

  if (document.readyState === "loading")
    document.addEventListener("DOMContentLoaded", wire);
  else wire();
})();
