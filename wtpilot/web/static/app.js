'use strict';

const $ = (id) => document.getElementById(id);
const api = async (path, options) => {
  const res = await fetch(path, options);
  if (!res.ok) throw new Error(`${path}: HTTP ${res.status}`);
  return res.json();
};
const postJSON = (path, body) =>
  api(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body || {}),
  });

const MODE_LABELS = {
  off: '待机',
  stabilize: '保持稳定',
  alt_hold: '确定高度',
  navigate: '飞往目标点',
};

const SOURCE_LABELS = {
  http: '战争雷霆',
  powerbroker: 'PowerBroker库',
  sim: '离线模拟器',
  fixture: '录制回放',
};

const PHASE_LABELS = {
  idle: '待机',
  running: '运行中',
  waiting: '等待中',
};

// Chinese names for the rebindable actions; the identifier stays visible so it
// can be matched against the .blk file and the REST API.
const ACTION_LABELS = {
  elevator_up: '抬头',
  elevator_down: '低头',
  aileron_left: '左滚',
  aileron_right: '右滚',
  rudder_left: '方向舵左',
  rudder_right: '方向舵右',
  throttle_up: '加油门',
  throttle_down: '收油门',
};

let paramsBuilt = false;
let bindingsBuilt = false;
const PARAM_SPEC = [
  ['control_strategy', '航向控制策略', 'select', {options: ['segmented', 'pd'], labels: ['分段控制（航向）', 'PD 连续（航向）']}],
  ['target_heading_deg', '目标航向 (°，留空 = 保持当前)'],
  ['arrive_radius_m', '到达半径 (m)'],
  ['max_bank_deg', '最大坡度 (°)'],
  ['max_pitch_deg', '最大俯仰 (°)'],
  ['tick_hz', '循环频率 (Hz)'],
  ['heading_kp', '航向增益'],
  ['roll_kp', '滚转 P'],
  ['roll_kd', '滚转 D'],
  ['pitch_kp', '俯仰 P'],
  ['pitch_kd', '俯仰 D'],
  ['alt_kp', '高度 P'],
  ['vario_kd', '爬升率阻尼'],
  ['axis_deadband', '轴死区'],
  ['min_altitude_m', '最低高度 (m)'],
  ['use_throttle', '油门控制', 'bool'],
  ['target_speed_kmh', '目标速度 (km/h)'],
  ['use_rudder', '方向舵控制', 'bool'],
];

function metric(label, value) {
  return `<div class="metric"><div class="k">${label}</div><div class="v">${value}</div></div>`;
}

const fmt = (v, digits = 0, suffix = '') =>
  v === null || v === undefined || Number.isNaN(v) ? '—' : `${Number(v).toFixed(digits)}${suffix}`;

function renderTelemetry(t) {
  if (!t) {
    $('readout').innerHTML = metric('状态', '无数据');
    return;
  }
  $('readout').innerHTML = [
    metric('高度', fmt(t.altitude_m, 0, ' m')),
    metric('表速', fmt(t.ias_kmh, 0, ' km/h')),
    metric('航向', fmt(t.heading_deg, 1, '°')),
    metric('爬升率', fmt(t.vario_ms, 1, ' m/s')),
    metric('滚转 (原始)', fmt(t.roll_raw_deg, 1, '°')),
    metric('俯仰 (原始)', fmt(t.pitch_raw_deg, 1, '°')),
    metric('油门', fmt((t.throttle ?? 0) * 100, 0, ' %')),
    metric('敌机数', fmt(t.enemy_count, 0)),
  ].join('');
}

function renderNav(st) {
  const nav = st.nav || {};
  if (nav.distance_m === undefined) {
    $('nav-line').textContent = '地图数据里没有目标点 (point_of_interest)。';
    return;
  }
  const bearing = nav.bearing === null ? '—' : `${nav.bearing.toFixed(0)}°`;
  $('nav-line').innerHTML =
    `目标方位 <b>${bearing}</b> · 距离 <b>${(nav.distance_m / 1000).toFixed(2)} km</b> · ` +
    `航向误差 <b>${fmt(nav.heading_error, 1, '°')}</b> · 地图符号 <b>${nav.map_sign}</b>` +
    (nav.arrived ? ' · <b>已到达</b>' : '');
}

function barRow(label, value) {
  const v = value ?? 0;
  const width = Math.min(50, Math.abs(v) * 50);
  const left = v >= 0 ? 50 : 50 - width;
  return `<tr>
    <td class="name">${label}</td>
    <td><div class="bar"><div class="mid"></div>
      <div class="fill" style="left:${left}%;width:${width}%"></div></div></td>
    <td class="val">${v.toFixed(2)}</td>
  </tr>`;
}

function renderBars(cmd) {
  $('bars').innerHTML = [
    barRow('副翼', cmd.aileron),
    barRow('升降舵', cmd.elevator),
    barRow('方向舵', cmd.rudder),
    barRow('油门', cmd.throttle),
  ].join('') + `
    <tr><td class="name">目标坡度</td><td colspan="2">${fmt(cmd.roll_target, 1, '°')}</td></tr>
    <tr><td class="name">目标俯仰</td><td colspan="2">${fmt(cmd.pitch_target, 1, '°')}</td></tr>
    <tr><td class="name">航向误差</td><td colspan="2">${fmt(cmd.heading_error, 1, '°')}</td></tr>
    <tr><td class="name">高度误差</td><td colspan="2">${fmt(cmd.altitude_error, 0, ' m')}</td></tr>`;
}

function buildParams(params) {
  $('params-form').innerHTML = PARAM_SPEC.map(([key, label, kind, opts]) => {
    if (kind === 'bool') {
      return `<label class="field checkbox"><input type="checkbox" id="p-${key}"
        ${params[key] ? 'checked' : ''}><span>${label}</span></label>`;
    }
    if (kind === 'select' && opts && opts.options) {
      const optionsHtml = opts.options.map((opt, i) => {
        const label = opts.labels ? opts.labels[i] : opt;
        return `<option value="${opt}" ${params[key] === opt ? 'selected' : ''}>${label}</option>`;
      }).join('');
      return `<label class="field"><span>${label}</span>
        <select id="p-${key}">${optionsHtml}</select></label>`;
    }
    const value = params[key] === null ? '' : params[key];
    return `<label class="field"><span>${label}</span>
      <input type="text" id="p-${key}" value="${value}"></label>`;
  }).join('');
  paramsBuilt = true;
}

function readParams() {
  const out = {};
  for (const [key, , kind] of PARAM_SPEC) {
    const el = $(`p-${key}`);
    if (!el) continue;
    if (kind === 'bool') {
      out[key] = el.checked;
    } else if (kind === 'select') {
      out[key] = el.value;
    } else {
      const raw = el.value.trim();
      out[key] = raw === '' ? null : Number(raw);
    }
  }
  return out;
}

function buildBindings(bindings, overrides, offered) {
  offered = offered || {};
  $('bindings').innerHTML = window.WT.bindable.map((action) => {
    const detected = bindings[action] || '-';
    const current = (overrides[action] || []).join(', ');
    const all = offered[action];
    const extra = all && all !== detected
      ? `<br><span class="offered">文件中还有: ${all}</span>` : '';
    const label = ACTION_LABELS[action] || action;
    return `<tr>
      <td class="name">${label}<span class="ident">${action}</span></td>
      <td class="detected">${detected}${extra}</td>
      <td><input type="text" id="b-${action}" value="${current}"
        placeholder="自动"></td>
    </tr>`;
  }).join('');
  bindingsBuilt = true;
}

function readBindings() {
  const overrides = {};
  for (const action of window.WT.bindable) {
    const el = $(`b-${action}`);
    if (!el) continue;
    const names = el.value.split(',').map((s) => s.trim()).filter(Boolean);
    if (names.length) overrides[action] = names;
  }
  return overrides;
}

function setChips(st) {
  const t = st.telemetry;
  const source = SOURCE_LABELS[st.source] || st.source;
  $('chip-source').textContent =
    `数据源: ${source}` + (st.dry_run ? '（不输出按键）' : '');
  const conn = $('chip-conn');
  if (t && t.valid) {
    conn.textContent = '8111端口正常';
    conn.className = 'chip chip-good';
  } else {
    conn.textContent = '无8111数据';
    conn.className = 'chip chip-bad';
  }
  const focus = $('chip-focus');
  if (!st.injecting_keys) {
    focus.textContent = '焦点: 不适用';
    focus.className = 'chip';
  } else if (st.focused) {
    focus.textContent = '焦点: 游戏窗口';
    focus.className = 'chip chip-good';
  } else {
    focus.textContent = '焦点: 其他窗口';
    focus.className = 'chip chip-warn';
  }
  const mode = $('chip-mode');
  mode.textContent = MODE_LABELS[st.mode] || st.mode;
  mode.className = 'chip' + (st.engaged ? ' chip-good' : '');

  $('message').textContent =
    st.message +
    (st.last_error ? `  ·  最近错误: ${st.last_error}` : '') +
    (st.keys_held && st.keys_held.length ? `  ·  按住: ${st.keys_held.join(', ')}` : '');

  $('loop-info').textContent =
    `帧 ${st.ticks} · 循环 ${st.loop_hz} Hz · 时间倍率 ${st.time_scale}× · ` +
    `阶段 ${PHASE_LABELS[st.phase] || st.phase}`;

  $('calib-line').textContent =
    `滚转符号=${st.calibration.roll_sign} 俯仰符号=${st.calibration.pitch_sign} ` +
    `地图符号=${st.calibration.map_sign}`;
}

async function poll() {
  try {
    const st = await api('/api/status');
    setChips(st);
    renderTelemetry(st.telemetry);
    renderNav(st);
    renderBars(st.command);
    if (!paramsBuilt) buildParams(st.params);
    if (!bindingsBuilt) {
      const b = await api('/api/bindings');
      buildBindings(b.bindings, b.overrides, b.offered);
      $('blk-path').textContent = b.blk_path;
    }
  } catch (err) {
    $('message').textContent = `出错: ${err.message}`;
  } finally {
    setTimeout(poll, 400);
  }
}

function selectedMode() {
  const el = document.querySelector('input[name="mode"]:checked');
  return el ? el.value : 'stabilize';
}

// The target altitude belongs to "确定高度" alone.  A blank field means "hold
// whatever altitude we are at when engaging".
function readTargetAltitude() {
  const raw = $('mode-altitude').value.trim();
  return raw === '' ? null : Number(raw);
}

function syncAltitudeField() {
  const active = selectedMode() === 'alt_hold';
  const input = $('mode-altitude');
  input.disabled = !active;
  input.closest('.mode-target').classList.toggle('inactive', !active);
}

$('btn-engage').onclick = () => {
  const mode = selectedMode();
  const body = { mode, ...readParams() };
  if (mode === 'alt_hold') body.target_altitude_m = readTargetAltitude();
  postJSON('/api/engage', body)
    .then(() => {})
    .catch((e) => ($('message').textContent = e.message));
};

$('btn-disengage').onclick = () => postJSON('/api/disengage').catch(() => {});
$('btn-focus').onclick = () => postJSON('/api/focus').catch(() => {});

$('btn-save-params').onclick = () =>
  postJSON('/api/params', readParams())
    .then(() => ($('message').textContent = '参数已应用'))
    .catch((e) => ($('message').textContent = e.message));

$('btn-save-bindings').onclick = () =>
  postJSON('/api/bindings', { overrides: readBindings() })
    .then((r) => {
      buildBindings(r.bindings, readBindings(), r.offered);
      $('message').textContent = '按键覆盖已保存';
    })
    .catch((e) => ($('message').textContent = e.message));

$('btn-save-settings').onclick = () =>
  postJSON('/api/settings', {
    source: $('source-select').value,
    base_url: $('base-url').value.trim(),
    require_focus: $('require-focus').checked,
    focus_title: $('focus-title').value.trim(),
  })
    .then((r) => {
      const name = SOURCE_LABELS[r.settings.source] || r.settings.source;
      $('message').textContent = `连接已更新（数据源: ${name}）`;
      bindingsBuilt = false;
    })
    .catch((e) => ($('message').textContent = e.message));

const drop = $('blk-drop');
drop.onclick = () => $('blk-file').click();
drop.ondragover = (e) => { e.preventDefault(); drop.classList.add('over'); };
drop.ondragleave = () => drop.classList.remove('over');
drop.ondrop = (e) => {
  e.preventDefault();
  drop.classList.remove('over');
  if (e.dataTransfer.files.length) uploadBlk(e.dataTransfer.files[0]);
};
$('blk-file').onchange = (e) => {
  if (e.target.files.length) uploadBlk(e.target.files[0]);
};

async function uploadBlk(file) {
  const data = new FormData();
  data.append('file', file);
  $('message').textContent = `正在上传 ${file.name}…`;
  try {
    const res = await fetch('/api/upload_blk', { method: 'POST', body: data });
    const body = await res.json();
    if (!body.ok) throw new Error(body.error || '上传失败');
    $('blk-path').textContent = body.path;
    buildBindings(body.bindings, {}, body.offered);
    $('message').textContent = `已载入 ${body.filename}（${body.blocks} 个绑定块）`;
  } catch (err) {
    $('message').textContent = `上传失败: ${err.message}`;
  }
}

api('/api/config')
  .then((cfg) => {
    $('source-select').value = cfg.settings.source;
    $('base-url').value = cfg.settings.base_url;
    $('require-focus').checked = cfg.settings.require_focus;
    $('focus-title').value = cfg.settings.focus_title;
    $('mode-altitude').value = cfg.params.target_altitude_m ?? '';
    buildParams(cfg.params);
  })
  .catch(() => {});

$('key-names').textContent = window.WT.keyNames.join(' · ');

document.querySelectorAll('input[name="mode"]').forEach((el) =>
  el.addEventListener('change', syncAltitudeField));

// The radio is the user's intent, so the poll loop never overwrites it.
document.querySelector('input[name="mode"][value="stabilize"]').checked = true;
syncAltitudeField();
poll();