/* =========================================================================
   キャラクター操作UI (second edition) — 直接編集 + 補助としての対話的進化計算
   - 四辺形の胴体 + 手足 + 頭（固定形状・固定長）
   - 自由度: 胴体位置(x,y)・胴体回転 + 手足8関節 + 頭・腰2関節
   - 右パネル: 「重心と胴体中心のズレ(不安定度)が小さい姿勢」を n 個提示。
              クリックで自動的に次イテレーションへ。変更幅は毎回小さくなり微調整可能。
   - 左キャンバス: 手足=IK / 頭=向き / 胴体=移動・回転 の直接ドラッグ編集
   - すべての変更を履歴に保持し、Tスタンスまで巻き戻し可能
   ========================================================================= */

/* ------------------------- 角度・幾何ヘルパ ------------------------- */
const DEG = Math.PI / 180;
// 角度は「真上=0, 時計回り正（画面上）」で統一。方向ベクトル: (sinθ, -cosθ)
function dir(a) { const r = a * DEG; return { x: Math.sin(r), y: -Math.cos(r) }; }
function pointFrom(o, a, len) { const d = dir(a); return { x: o.x + d.x * len, y: o.y + d.y * len }; }
function angleOf(dx, dy) { return Math.atan2(dx, -dy) / DEG; } // 上=0基準の角度(deg)
function norm(a) { a = a % 360; if (a > 180) a -= 360; if (a < -180) a += 360; return a; }
function dist2(a, b) { return (a.x - b.x) ** 2 + (a.y - b.y) ** 2; }

/* ------------------------- 骨格の固定寸法 ------------------------- */
const L = {
  torso: 128,     // 骨盤→首（背骨）
  halfW: 36,      // 胴体の半幅（四辺形）
  head: 44,       // 頭の長さ
  upperArm: 66, lowerArm: 60,
  upperLeg: 78, lowerLeg: 72,
};

/* ------------------------- 姿勢モデル -------------------------
   関節角はすべて「Tスタンスで0」になるようFK側でオフセットを吸収する。 */
function makeTPose(w, h) {
  return {
    rootX: w * 0.5, rootY: h * 0.62, rootAngle: 0,
    waist: 0, head: 0,
    shoulderL: 0, elbowL: 0,
    shoulderR: 0, elbowR: 0,
    hipL: 0, kneeL: 0,
    hipR: 0, kneeR: 0,
  };
}
function clonePose(p) { return Object.assign({}, p); }

/* ------------------------- 順運動学(FK) -------------------------
   胴体(四辺形)は骨盤の2点と肩の2点を頂点に持つ。 */
function computeSkeleton(pose) {
  const P = {};
  const pelvis = { x: pose.rootX, y: pose.rootY };
  P.pelvis = pelvis;

  // 胴体(四辺形)は剛体。四隅すべてを torsoAng 基準で計算し形状を固定する。
  // 腰(waist)は骨盤を軸に胴体・腕・頭ごと回転させる（＝下半身に対する上半身の傾き）。
  const torsoAng = pose.rootAngle + pose.waist;
  const spineAngle = torsoAng; // 腕・頭の基準角
  const neck = pointFrom(pelvis, torsoAng, L.torso);
  P.neck = neck;

  // 胴体四辺形の頂点（上辺=肩, 下辺=胴体底。すべて torsoAng なので剛体を保つ）
  P.shoulderR = pointFrom(neck, torsoAng + 90, L.halfW);
  P.shoulderL = pointFrom(neck, torsoAng - 90, L.halfW);
  P.torsoBR = pointFrom(pelvis, torsoAng + 90, L.halfW);
  P.torsoBL = pointFrom(pelvis, torsoAng - 90, L.halfW);

  // 脚の付け根は胴体（四辺形）の底辺の頂点に固定する
  P.hipR = P.torsoBR;
  P.hipL = P.torsoBL;

  // 頭
  const headAngle = spineAngle + pose.head;
  P.headTop = pointFrom(neck, headAngle, L.head);
  P.headCenter = pointFrom(neck, headAngle, L.head * 0.55);

  // 腕（Tスタンスで水平: 右+90, 左-90 をFKで吸収）
  const uaR = spineAngle + 90 + pose.shoulderR;
  P.elbowR = pointFrom(P.shoulderR, uaR, L.upperArm);
  P.handR = pointFrom(P.elbowR, uaR + pose.elbowR, L.lowerArm);

  const uaL = spineAngle - 90 + pose.shoulderL;
  P.elbowL = pointFrom(P.shoulderL, uaL, L.upperArm);
  P.handL = pointFrom(P.elbowL, uaL + pose.elbowL, L.lowerArm);

  // 脚（Tスタンスで真下: +180 をFKで吸収）
  const ulR = pose.rootAngle + 180 + pose.hipR;
  P.kneeR = pointFrom(P.hipR, ulR, L.upperLeg);
  P.footR = pointFrom(P.kneeR, ulR + pose.kneeR, L.lowerLeg);

  const ulL = pose.rootAngle + 180 + pose.hipL;
  P.kneeL = pointFrom(P.hipL, ulL, L.upperLeg);
  P.footL = pointFrom(P.kneeL, ulL + pose.kneeL, L.lowerLeg);

  return P;
}

/* ------------------------- 逆運動学(2ボーンIK) ------------------------- */
function solve2Bone(base, target, L1, L2, prefElbow) {
  let dx = target.x - base.x, dy = target.y - base.y;
  let d = Math.hypot(dx, dy);
  d = Math.max(Math.abs(L1 - L2) + 0.01, Math.min(L1 + L2 - 0.01, d));
  const baseAng = angleOf(dx, dy);
  const eff = pointFrom(base, baseAng, d); // 到達可能距離に丸めた実効ターゲット
  let cosA = (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d);
  cosA = Math.max(-1, Math.min(1, cosA));
  const A = Math.acos(cosA) / DEG;
  const cands = [baseAng + A, baseAng - A];
  // 現在の肘位置に近い解を選び、姿勢の連続性を保つ
  let best = cands[0];
  if (prefElbow) {
    let bd = Infinity;
    for (const ua of cands) {
      const e = pointFrom(base, ua, L1);
      const dd = dist2(e, prefElbow);
      if (dd < bd) { bd = dd; best = ua; }
    }
  }
  const upper = best;
  const elbowPt = pointFrom(base, upper, L1);
  const lower = angleOf(eff.x - elbowPt.x, eff.y - elbowPt.y);
  return { upper, lower };
}

/* ------------------------- 描画 ------------------------- */
function fitTransform(pose, cw, ch, pad) {
  const P = computeSkeleton(pose);
  const pts = Object.values(P);
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const p of pts) {
    minX = Math.min(minX, p.x); minY = Math.min(minY, p.y);
    maxX = Math.max(maxX, p.x); maxY = Math.max(maxY, p.y);
  }
  const w = Math.max(1, maxX - minX), h = Math.max(1, maxY - minY);
  const s = Math.min((cw - pad * 2) / w, (ch - pad * 2) / h);
  const ox = (cw - w * s) / 2 - minX * s;
  const oy = (ch - h * s) / 2 - minY * s;
  return { s, ox, oy };
}

function drawSkeleton(ctx, pose, opts = {}) {
  const t = opts.transform || { s: 1, ox: 0, oy: 0 };
  const tx = p => ({ x: p.x * t.s + t.ox, y: p.y * t.s + t.oy });
  const P = computeSkeleton(pose);
  const scale = t.s;

  const col = opts.colors || {
    torso: '#3a4a63', torsoEdge: '#5c7fb0',
    limb: '#c9d6e6', head: '#e6ebf1', joint: '#4da3ff',
  };
  const alpha = opts.alpha != null ? opts.alpha : 1;
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';

  const bone = (a, b, width) => {
    const A = tx(a), B = tx(b);
    ctx.beginPath();
    ctx.moveTo(A.x, A.y); ctx.lineTo(B.x, B.y);
    ctx.lineWidth = width * scale;
    ctx.strokeStyle = col.limb;
    ctx.stroke();
  };

  // 四肢（胴体の後ろ）
  const lw = 15;
  bone(P.hipL, P.kneeL, lw); bone(P.kneeL, P.footL, lw);
  bone(P.hipR, P.kneeR, lw); bone(P.kneeR, P.footR, lw);
  bone(P.shoulderL, P.elbowL, lw * 0.85); bone(P.elbowL, P.handL, lw * 0.85);
  bone(P.shoulderR, P.elbowR, lw * 0.85); bone(P.elbowR, P.handR, lw * 0.85);

  // 胴体（剛体の四辺形）
  const quad = [P.torsoBL, P.torsoBR, P.shoulderR, P.shoulderL].map(tx);
  ctx.beginPath();
  ctx.moveTo(quad[0].x, quad[0].y);
  for (let i = 1; i < quad.length; i++) ctx.lineTo(quad[i].x, quad[i].y);
  ctx.closePath();
  ctx.fillStyle = col.torso;
  ctx.fill();
  ctx.lineWidth = 2 * scale;
  ctx.strokeStyle = col.torsoEdge;
  ctx.stroke();

  // 首
  bone(P.neck, P.headCenter, 9);

  // 頭
  const hc = tx(P.headCenter);
  ctx.beginPath();
  ctx.arc(hc.x, hc.y, (L.head * 0.5) * scale, 0, Math.PI * 2);
  ctx.fillStyle = col.head;
  ctx.fill();

  // 関節ドット
  if (opts.showJoints !== false) {
    const joints = [P.shoulderL, P.elbowL, P.shoulderR, P.elbowR,
      P.hipL, P.kneeL, P.hipR, P.kneeR, P.neck, P.pelvis];
    ctx.fillStyle = col.joint;
    for (const j of joints) {
      const J = tx(j);
      ctx.beginPath(); ctx.arc(J.x, J.y, 3.2 * scale, 0, Math.PI * 2); ctx.fill();
    }
  }

  // ドラッグ用ハンドル強調（メインキャンバスのみ）
  if (opts.handles) {
    for (const h of opts.handles) {
      const H = tx(P[h.key]);
      ctx.beginPath(); ctx.arc(H.x, H.y, 8, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(77,163,255,0.18)';
      ctx.fill();
      ctx.lineWidth = 1.5; ctx.strokeStyle = 'rgba(77,163,255,0.7)';
      ctx.stroke();
    }
  }
  ctx.restore();
}

/* ------------------------- 進化計算：候補生成 -------------------------
   安定度（モーメント）による並び替えは一時的に不使用。
   代わりに以下の優先順で候補を並べる:
     1. ユーザが最も最近操作した部位（変更量は次第に小さく＝stepFor(refineLevel)）
     2. その部位と左右対称の部位（腕・脚。同じ値 or 左右対称になる変更量）
     3. ランダムな部位
   体の回転（腰）は候補から除外し、手足と頭のみを対象にする。 */
const IEC_JOINTS = ['head', 'shoulderL', 'elbowL', 'shoulderR', 'elbowR',
  'hipL', 'kneeL', 'hipR', 'kneeR'];
const JNAME = {
  waist: '腰', head: '頭', shoulderL: '左肩', elbowL: '左肘', shoulderR: '右肩',
  elbowR: '右肘', hipL: '左股', kneeL: '左膝', hipR: '右股', kneeR: '右膝',
};
// 部位（symmetry 対象を含む）
const PARTS = {
  head: { joints: ['head'], mirror: null },
  armL: { joints: ['shoulderL', 'elbowL'], mirror: 'armR' },
  armR: { joints: ['shoulderR', 'elbowR'], mirror: 'armL' },
  legL: { joints: ['hipL', 'kneeL'], mirror: 'legR' },
  legR: { joints: ['hipR', 'kneeR'], mirror: 'legL' },
};
const PART_NAME = { head: '頭', armL: '左腕', armR: '右腕', legL: '左脚', legR: '右脚' };
const ALL_PARTS = Object.keys(PARTS);
const RANDOM_STEP = 40;

const STEP_BASE = 50, STEP_SHRINK = 0.62, STEP_MIN = 4;
function stepFor(level) {
  return Math.round(Math.max(STEP_MIN, STEP_BASE * Math.pow(STEP_SHRINK, level)));
}
function applyChanges(pose, changes) {
  const p = clonePose(pose);
  for (const k in changes) p[k] = norm(p[k] + changes[k]);
  return p;
}
function poseDist(a, b) {
  let d = 0;
  for (const k of IEC_JOINTS) d += Math.abs(norm(a[k] - b[k]));
  return d;
}
function changeLabel(changes) {
  return Object.keys(changes)
    .map(k => JNAME[k] + (changes[k] > 0 ? '↻' : '↺'))
    .join(' + ');
}
function shuffle(a) { return a.slice().sort(() => Math.random() - 0.5); }

function mkCand(pose, part, changes, category, note) {
  return { part, category, note: note || '', changes, pose: applyChanges(pose, changes) };
}
// 1. 対象部位の各関節を ±step 回転させた候補
function partVariations(pose, part, step) {
  const js = PARTS[part].joints;
  let cs;
  if (js.length === 1) {
    cs = [{ [js[0]]: +step }, { [js[0]]: -step }];
  } else {
    const [a, b] = js;
    cs = [
      { [a]: +step }, { [a]: -step }, { [b]: +step }, { [b]: -step },
      { [a]: +step, [b]: +step }, { [a]: -step, [b]: -step },
    ];
  }
  return cs.map(ch => mkCand(pose, part, ch, 'recent'));
}
// 2. 左右対称部位を「左右対称」または「同じ値」にする候補（変更量＝目標-現在）
function mirrorCands(pose, part) {
  const m = PARTS[part].mirror;
  if (!m) return [];
  const src = PARTS[part].joints, dst = PARTS[m].joints;
  const sym = {}, same = {};
  for (let i = 0; i < src.length; i++) {
    sym[dst[i]] = norm(-pose[src[i]] - pose[dst[i]]);  // dst -> -src（左右対称）
    same[dst[i]] = norm(pose[src[i]] - pose[dst[i]]);  // dst -> +src（同じ値）
  }
  return [
    mkCand(pose, m, sym, 'mirror', '左右対称'),
    mkCand(pose, m, same, 'mirror', '同じ角度'),
  ];
}
// 3. ランダムな部位を ±step
function randomCands(pose, count, step) {
  const out = [];
  let guard = 0;
  while (out.length < count && guard++ < count * 8 + 8) {
    const part = ALL_PARTS[Math.floor(Math.random() * ALL_PARTS.length)];
    const js = PARTS[part].joints;
    const j = js[Math.floor(Math.random() * js.length)];
    out.push(mkCand(pose, part, { [j]: (Math.random() < 0.5 ? -1 : 1) * step }, 'random'));
  }
  return out;
}
function takeDistinct(src, k, existing) {
  const res = [];
  for (const c of src) {
    if (res.length >= k) break;
    if ([...existing, ...res].every(q => poseDist(c.pose, q.pose) > 2)) res.push(c);
  }
  return res;
}
// 優先順（最近部位→対称部位→ランダム）に候補を n 個並べる
function generateCandidates(pose, n) {
  const out = [];
  if (lastPart && PARTS[lastPart]) {
    const step = stepFor(refineLevel);
    const hasMirror = !!PARTS[lastPart].mirror;
    // ② 左右対称部位は「左右対称」と「同じ角度」の2つを同じ優先度で提示（最近部位の枠は必ず1つ残す）
    const nMirror = hasMirror ? Math.min(2, Math.max(0, n - 1)) : 0;
    const nRandom = Math.min(1, Math.max(0, n - 1 - nMirror));
    const nLast = n - nMirror - nRandom;
    out.push(...takeDistinct(shuffle(partVariations(pose, lastPart, step)), nLast, out));
    if (nMirror) out.push(...takeDistinct(mirrorCands(pose, lastPart), nMirror, out));
  }
  // 残りはランダム部位で埋める（優先順の3番目、または初期状態）
  let guard = 0;
  while (out.length < n && guard++ < 30) {
    const need = n - out.length;
    out.push(...takeDistinct(randomCands(pose, need * 4, RANDOM_STEP), need, out));
  }
  return out.slice(0, n);
}
// モーメントの表示用整形（値が大きいので 1/1000 スケール）
function fmtInst(v) { return (v / 1000).toFixed(1); }

/* ------------------------- 状態・履歴 ------------------------- */
const canvas = document.getElementById('main-canvas');
const ctx = canvas.getContext('2d');

let history = [];       // [{ pose, lastPart, refineLevel }]
let histIndex = -1;
let currentPose = null;
let lastPart = null;    // ユーザが最も最近操作した部位（'armR' など）
let refineLevel = 0;    // その部位を連続操作した回数（変更幅を次第に小さくする）
let optionCount = 6;    // 提示する選択肢数 n（デフォルト3）
let candidates = [];  // 現在の候補
let hoverPose = null; // 候補ホバー時のゴースト

// ドラッグ可能ハンドル定義（近い順に優先）
const HANDLE_DEFS = [
  { key: 'handR', type: 'ikArmR' },
  { key: 'handL', type: 'ikArmL' },
  { key: 'footR', type: 'ikLegR' },
  { key: 'footL', type: 'ikLegL' },
  { key: 'headTop', type: 'head' },
  { key: 'neck', type: 'rotate' },
  { key: 'pelvis', type: 'move' },
];

let cssW = 0, cssH = 0;
function resizeCanvas() {
  const wrap = document.getElementById('canvas-wrap');
  const dpr = window.devicePixelRatio || 1;
  const w = wrap.clientWidth, h = wrap.clientHeight;
  canvas.width = w * dpr; canvas.height = h * dpr;
  canvas.style.width = w + 'px'; canvas.style.height = h + 'px';
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  cssW = w; cssH = h;
}

function init() {
  resizeCanvas();
  const t = makeTPose(cssW, cssH);
  history = [{ pose: clonePose(t), lastPart: null, refineLevel: 0 }];
  histIndex = 0;
  currentPose = clonePose(t);
  lastPart = null; refineLevel = 0;
  bindEvents();
  refreshAll();
}

/* ------------------------- 履歴操作 ------------------------- */
function pushHistory(pose, lp, rl) {
  history = history.slice(0, histIndex + 1);
  history.push({ pose: clonePose(pose), lastPart: lp, refineLevel: rl });
  histIndex = history.length - 1;
  currentPose = clonePose(pose);
  lastPart = lp; refineLevel = rl;
}
function undo() {
  if (histIndex <= 0) return;
  histIndex--; applyHistEntry(); refreshAll();
}
function redo() {
  if (histIndex >= history.length - 1) return;
  histIndex++; applyHistEntry(); refreshAll();
}
function applyHistEntry() {
  const e = history[histIndex];
  currentPose = clonePose(e.pose);
  lastPart = e.lastPart; refineLevel = e.refineLevel;
}
function resetT() { pushHistory(makeTPose(cssW, cssH), null, 0); refreshAll(); }

// 操作した部位から次のフォーカス（注目部位・変更幅レベル）を決める
function nextFocus(operatedPart) {
  if (!operatedPart) return { lp: lastPart, rl: refineLevel };  // 体の移動・回転は不変
  if (operatedPart === lastPart) return { lp: lastPart, rl: refineLevel + 1 }; // 連続操作→漸減
  return { lp: operatedPart, rl: 0 }; // 別部位→大きな変更幅から
}

// 候補を選択 → 適用して自動的に次のイテレーションへ
function selectCandidate(c) {
  const f = nextFocus(c.part);
  pushHistory(c.pose, f.lp, f.rl);
  hoverPose = null;
  refreshAll();
}

/* ------------------------- リフレッシュ ------------------------- */
function refreshMain() {
  ctx.clearRect(0, 0, cssW, cssH);
  // 背景グリッド
  ctx.save();
  ctx.strokeStyle = 'rgba(255,255,255,0.03)';
  ctx.lineWidth = 1;
  for (let x = 0; x < cssW; x += 40) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, cssH); ctx.stroke(); }
  for (let y = 0; y < cssH; y += 40) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(cssW, y); ctx.stroke(); }
  ctx.restore();

  // ホバー中の候補をゴースト表示
  if (hoverPose) {
    drawSkeleton(ctx, hoverPose, {
      alpha: 0.35, showJoints: false,
      colors: { torso: '#2a4a3a', torsoEdge: '#46d39a', limb: '#8fe6c0', head: '#bff0d8', joint: '#46d39a' },
    });
  }
  drawSkeleton(ctx, currentPose, { handles: HANDLE_DEFS, showCOG: true });
}

const CAT_TEXT = { recent: '① 最近操作した部位', mirror: '② 左右対称', random: '③ ランダム' };
function refreshCandidates() {
  candidates = generateCandidates(currentPose, optionCount);

  const box = document.getElementById('candidates');
  box.innerHTML = '';
  candidates.forEach((c, i) => {
    const cell = document.createElement('div');
    cell.className = 'cand cat-' + c.category;
    const cv = document.createElement('canvas');
    cv.width = 128; cv.height = 116;
    cell.appendChild(cv);
    const lbl = document.createElement('div');
    lbl.className = 'lbl';
    cell.appendChild(lbl);

    cell.addEventListener('click', () => selectCandidate(c));
    cell.addEventListener('mouseenter', () => { hoverPose = c.pose; refreshMain(); });
    cell.addEventListener('mouseleave', () => { hoverPose = null; refreshMain(); });
    box.appendChild(cell);

    // 元姿勢を薄く重ねて変化を分かりやすく
    const t0 = fitTransform(c.pose, cv.width, cv.height, 12);
    const cctx = cv.getContext('2d');
    drawSkeleton(cctx, currentPose, {
      transform: t0, alpha: 0.16, showJoints: false,
      colors: { torso: '#333', torsoEdge: '#555', limb: '#777', head: '#888', joint: '#777' },
    });
    drawSkeleton(cctx, c.pose, { transform: t0, showJoints: false });
  });
}


function refreshAll() {
  hoverPose = null;
  refreshCandidates();
  refreshMain();
}

/* ------------------------- ドラッグ編集 ------------------------- */
let drag = null;
function canvasPos(e) {
  const r = canvas.getBoundingClientRect();
  return { x: e.clientX - r.left, y: e.clientY - r.top };
}
function pointInQuad(p, q) {
  let inside = false;
  for (let i = 0, j = q.length - 1; i < q.length; j = i++) {
    const a = q[i], b = q[j];
    if (((a.y > p.y) !== (b.y > p.y)) &&
      (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) inside = !inside;
  }
  return inside;
}
function pickHandle(pos) {
  const P = computeSkeleton(currentPose);
  let best = null, bestD = 22 * 22;
  for (const h of HANDLE_DEFS) {
    const d = dist2(pos, P[h.key]);
    if (d < bestD) { bestD = d; best = h; }
  }
  if (!best && pointInQuad(pos, [P.torsoBL, P.torsoBR, P.shoulderR, P.shoulderL])) {
    best = { key: 'pelvis', type: 'move' };
  }
  return best;
}

function onDragMove(pos) {
  const P = computeSkeleton(currentPose);
  const spineAngle = currentPose.rootAngle + currentPose.waist;
  switch (drag.type) {
    case 'ikArmR': {
      const s = solve2Bone(P.shoulderR, pos, L.upperArm, L.lowerArm, P.elbowR);
      currentPose.shoulderR = norm(s.upper - (spineAngle + 90));
      currentPose.elbowR = norm(s.lower - s.upper);
      break;
    }
    case 'ikArmL': {
      const s = solve2Bone(P.shoulderL, pos, L.upperArm, L.lowerArm, P.elbowL);
      currentPose.shoulderL = norm(s.upper - (spineAngle - 90));
      currentPose.elbowL = norm(s.lower - s.upper);
      break;
    }
    case 'ikLegR': {
      const s = solve2Bone(P.hipR, pos, L.upperLeg, L.lowerLeg, P.kneeR);
      currentPose.hipR = norm(s.upper - (currentPose.rootAngle + 180));
      currentPose.kneeR = norm(s.lower - s.upper);
      break;
    }
    case 'ikLegL': {
      const s = solve2Bone(P.hipL, pos, L.upperLeg, L.lowerLeg, P.kneeL);
      currentPose.hipL = norm(s.upper - (currentPose.rootAngle + 180));
      currentPose.kneeL = norm(s.lower - s.upper);
      break;
    }
    case 'head':
      currentPose.head = norm(angleOf(pos.x - P.neck.x, pos.y - P.neck.y) - spineAngle);
      break;
    case 'rotate': // 首をドラッグ→胴体全体を回転（骨盤中心）
      currentPose.rootAngle = norm(angleOf(pos.x - P.pelvis.x, pos.y - P.pelvis.y) - currentPose.waist);
      break;
    case 'move': // 胴体を移動、接続部位はFKで追従
      currentPose.rootX = pos.x - drag.offx;
      currentPose.rootY = pos.y - drag.offy;
      break;
  }
  // ドラッグ中は現在の姿勢プレビュー・不安定度を即時更新（右画面へ反映）
  refreshMain();
}

function startDrag(pos) {
  const h = pickHandle(pos);
  if (!h) return false;
  const P = computeSkeleton(currentPose);
  drag = { type: h.type };
  if (h.type === 'move') { drag.offx = pos.x - P.pelvis.x; drag.offy = pos.y - P.pelvis.y; }
  canvas.classList.add('grabbing');
  return true;
}
// ドラッグ種別 → 操作した部位（体の移動・回転は部位なし=null）
const DRAG_PART = { ikArmR: 'armR', ikArmL: 'armL', ikLegR: 'legR', ikLegL: 'legL', head: 'head' };
function endDrag() {
  if (!drag) return;
  const part = DRAG_PART[drag.type] || null;
  drag = null;
  canvas.classList.remove('grabbing');
  // 直接編集の結果を履歴に確定 → 操作部位を最近部位として候補を再生成
  const f = nextFocus(part);
  pushHistory(currentPose, f.lp, f.rl);
  refreshAll();
}

function bindEvents() {
  window.addEventListener('resize', () => {
    const relX = currentPose.rootX / (cssW || 1), relY = currentPose.rootY / (cssH || 1);
    resizeCanvas();
    currentPose.rootX = relX * cssW; currentPose.rootY = relY * cssH;
    refreshMain();
  });

  canvas.addEventListener('mousedown', (e) => { if (startDrag(canvasPos(e))) e.preventDefault(); });
  window.addEventListener('mousemove', (e) => { if (drag) onDragMove(canvasPos(e)); });
  window.addEventListener('mouseup', endDrag);

  canvas.addEventListener('touchstart', (e) => {
    if (startDrag(canvasPos(e.touches[0]))) e.preventDefault();
  }, { passive: false });
  canvas.addEventListener('touchmove', (e) => {
    if (drag) { e.preventDefault(); onDragMove(canvasPos(e.touches[0])); }
  }, { passive: false });
  canvas.addEventListener('touchend', endDrag);

  document.getElementById('btn-undo').addEventListener('click', undo);
  document.getElementById('btn-redo').addEventListener('click', redo);
  document.getElementById('btn-reset').addEventListener('click', resetT);

  // サイコロ: 候補を再抽選（姿勢・履歴は変えない）
  document.getElementById('btn-dice').addEventListener('click', () => {
    hoverPose = null;
    refreshCandidates();
    refreshMain();
  });

  // パネル収納
  const toggle = document.getElementById('panel-toggle');
  toggle.addEventListener('click', () => {
    const app = document.getElementById('app');
    app.classList.toggle('panel-collapsed');
    toggle.textContent = app.classList.contains('panel-collapsed') ? '▶' : '◀';
    setTimeout(() => {
      const relX = currentPose.rootX / (cssW || 1), relY = currentPose.rootY / (cssH || 1);
      resizeCanvas();
      currentPose.rootX = relX * cssW; currentPose.rootY = relY * cssH;
      refreshMain();
    }, 300);
  });

  window.addEventListener('keydown', (e) => {
    if (e.ctrlKey && e.key === 'z') { e.preventDefault(); undo(); }
    else if (e.ctrlKey && (e.key === 'y' || (e.shiftKey && e.key === 'Z'))) { e.preventDefault(); redo(); }
  });
}

init();
