#!/usr/bin/env python3
import json
import os
import time
import threading
from flask import Flask, render_template_string, request, jsonify, Response
from adafruit_servokit import ServoKit
import cv2

kit = ServoKit(channels=16)

# Webcam setup
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
camera_lock = threading.Lock()

PRESETS_FILE = "/home/burf2000/servo-control/presets.json"

CONFIG_FILE = "/home/burf2000/servo-control/config.json"

# Joint names and default safe limits for robot channels
CHANNEL_DEFAULTS = {
    2:  {"name": "Head Rotation",    "safe_min": 0,   "safe_max": 180, "reversed": False},
    3:  {"name": "Head Tilt",        "safe_min": 50,  "safe_max": 140, "reversed": False},
    4:  {"name": "R Bicep",          "safe_min": 90,  "safe_max": 180, "reversed": False},
    5:  {"name": "R Arm Rotation",   "safe_min": 70,  "safe_max": 110, "reversed": True},
    6:  {"name": "R Shoulder Lift",  "safe_min": 70,  "safe_max": 110, "reversed": True},
    7:  {"name": "R Shoulder Rot",   "safe_min": 70,  "safe_max": 150, "reversed": True},
    8:  {"name": "L Bicep",          "safe_min": 90,  "safe_max": 180, "reversed": True},
    9:  {"name": "L Arm Rotation",   "safe_min": 70,  "safe_max": 110, "reversed": False},
    10: {"name": "L Shoulder Lift",  "safe_min": 70,  "safe_max": 110, "reversed": False},
    11: {"name": "L Shoulder Rot",   "safe_min": 70,  "safe_max": 150, "reversed": False},
}


def load_config():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE) as f:
            return json.load(f)
    return {}


def save_config(config):
    with open(CONFIG_FILE, "w") as f:
        json.dump(config, f, indent=2)


# Channel state: mode (servo/continuous/pwm), value, enabled, pulse range
saved_config = load_config()
channels = {}
for i in range(16):
    defaults = CHANNEL_DEFAULTS.get(i, {})
    saved = saved_config.get(str(i), {})
    channels[i] = {
        "mode": "servo",
        "value": 90,
        "enabled": True,
        "reversed": defaults.get("reversed", False),
        "min_pulse": 400,
        "max_pulse": 2600,
        "name": saved.get("name", defaults.get("name", "")),
        "safe_min": saved.get("safe_min", defaults.get("safe_min", 0)),
        "safe_max": saved.get("safe_max", defaults.get("safe_max", 180)),
    }
    try:
        kit.servo[i].set_pulse_width_range(400, 2600)
        kit.servo[i].angle = 90
    except Exception:
        pass

pwm_frequency = 50


def load_presets():
    if os.path.exists(PRESETS_FILE):
        with open(PRESETS_FILE) as f:
            return json.load(f)
    return {}


def save_presets(presets):
    with open(PRESETS_FILE, "w") as f:
        json.dump(presets, f, indent=2)


def apply_reverse(ch, angle):
    """If channel is reversed, flip the angle (180 - angle)."""
    if channels[ch].get("reversed", False):
        return 180 - angle
    return angle


app = Flask(__name__)

HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>PiBob Servo Control</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; background: #1a1a2e; color: #eee; padding: 16px; }
  h1 { text-align: center; margin-bottom: 16px; color: #e94560; font-size: 22px; }

  .top-bar { max-width: 1200px; margin: 0 auto 16px; display: flex; flex-wrap: wrap; gap: 12px; align-items: center; justify-content: center; }
  .top-section { background: #16213e; border-radius: 10px; padding: 12px 16px; border: 1px solid #0f3460; display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
  .top-section label { font-size: 12px; color: #888; font-weight: bold; text-transform: uppercase; }
  .top-section input, .top-section select { background: #0f3460; border: 1px solid #1a3a6e; color: #fff; padding: 6px 10px; border-radius: 6px; font-size: 13px; }
  .top-section input[type=number] { width: 70px; }

  .btn { padding: 7px 14px; border: none; border-radius: 6px; cursor: pointer; font-size: 12px; font-weight: bold; background: #e94560; color: #fff; }
  .btn:hover { background: #c73e54; }
  .btn:active { opacity: 0.7; }
  .btn-sm { padding: 5px 10px; font-size: 11px; }
  .btn-outline { background: transparent; border: 1px solid #e94560; color: #e94560; }
  .btn-outline:hover { background: #e94560; color: #fff; }
  .btn-blue { background: #0f3460; color: #eee; }
  .btn-blue:hover { background: #1a4a80; }
  .btn-green { background: #1b9e4b; }
  .btn-green:hover { background: #178a42; }
  .btn-danger { background: #a01030; }
  .btn-danger:hover { background: #c01040; }

  .grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 12px; max-width: 1400px; margin: 0 auto; }
  .card { background: #16213e; border-radius: 10px; padding: 14px; border: 1px solid #0f3460; transition: opacity 0.2s; }
  .card.disabled { opacity: 0.4; }
  .card-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; }
  .card-header h3 { color: #e94560; font-size: 13px; }
  .card-header .controls { display: flex; gap: 6px; align-items: center; }

  .mode-select { background: #0f3460; border: 1px solid #1a3a6e; color: #fff; padding: 4px 8px; border-radius: 6px; font-size: 11px; }
  .toggle { position: relative; width: 36px; height: 20px; cursor: pointer; }
  .toggle input { display: none; }
  .toggle span { position: absolute; inset: 0; background: #555; border-radius: 10px; transition: 0.2s; }
  .toggle input:checked + span { background: #1b9e4b; }
  .toggle span::after { content: ''; position: absolute; left: 2px; top: 2px; width: 16px; height: 16px; background: #fff; border-radius: 50%; transition: 0.2s; }
  .toggle input:checked + span::after { left: 18px; }

  .slider-row { display: flex; align-items: center; gap: 8px; margin-bottom: 6px; }
  input[type=range] { flex: 1; accent-color: #e94560; height: 6px; }
  .val { font-size: 18px; font-weight: bold; min-width: 50px; text-align: right; color: #fff; }
  .unit { font-size: 11px; color: #888; margin-left: 2px; }

  .btn-row { display: flex; gap: 4px; margin-bottom: 6px; }
  .btn-row button { flex: 1; }

  .rev-row { display: flex; align-items: center; gap: 6px; margin-bottom: 6px; font-size: 11px; color: #888; }
  .rev-row span { font-weight: bold; }
  .pulse-row { display: flex; align-items: center; gap: 6px; font-size: 11px; color: #888; }
  .pulse-row input { width: 60px; background: #0f3460; border: 1px solid #1a3a6e; color: #fff; padding: 3px 6px; border-radius: 4px; font-size: 11px; text-align: center; }

  .presets-section { max-width: 1200px; margin: 16px auto 0; background: #16213e; border-radius: 10px; padding: 14px; border: 1px solid #0f3460; }
  .presets-section h2 { font-size: 14px; color: #e94560; margin-bottom: 10px; }
  .preset-list { display: flex; flex-wrap: wrap; gap: 6px; }
  .preset-item { display: flex; align-items: center; gap: 4px; }
  .save-row { display: flex; gap: 8px; align-items: center; margin-bottom: 10px; }
  .save-row input { background: #0f3460; border: 1px solid #1a3a6e; color: #fff; padding: 6px 10px; border-radius: 6px; font-size: 13px; flex: 1; max-width: 200px; }
</style>
</head>
<body>
<h1>PiBob Servo Control</h1>

<div style="max-width:640px; margin:0 auto 16px; text-align:center;">
  <img id="webcam" src="/video_feed" style="width:100%; border-radius:10px; border:1px solid #0f3460; background:#000;">
  <div style="font-size:11px; color:#888; margin-top:4px;">Live Camera Feed</div>
</div>

<div class="top-bar">
  <div class="top-section">
    <label>PWM Freq</label>
    <input type="number" id="freq" value="{{ frequency }}" min="24" max="1526" step="1">
    <span style="font-size:12px;color:#888">Hz</span>
    <button class="btn btn-sm" onclick="setFreq()">Set</button>
  </div>
  <div class="top-section">
    <label>All Servos</label>
    <button class="btn btn-sm btn-blue" onclick="setAllServo(0)">0&deg;</button>
    <button class="btn btn-sm" onclick="setAllServo(90)">90&deg;</button>
    <button class="btn btn-sm btn-blue" onclick="setAllServo(180)">180&deg;</button>
  </div>
  <div class="top-section">
    <label>All Channels</label>
    <button class="btn btn-sm btn-green" onclick="enableAll(true)">Enable</button>
    <button class="btn btn-sm btn-danger" onclick="enableAll(false)">Disable</button>
  </div>
  <div class="top-section">
    <label>Test</label>
    <button class="btn btn-sm" id="sweep-btn" onclick="toggleSweep()">Sweep</button>
    <button class="btn btn-sm btn-green" id="demo-btn" onclick="runDemo()">Demo</button>
    <button class="btn btn-sm btn-blue" id="run-btn" onclick="runGesture('run')">Run</button>
    <button class="btn btn-sm btn-blue" id="point-btn" onclick="runGesture('point')">Point</button>
    <button class="btn btn-sm btn-blue" id="scared-btn" onclick="runGesture('scared')">Scared</button>
    <button class="btn btn-sm btn-blue" id="wave-btn" onclick="runGesture('wave')">Wave</button>
    <button class="btn btn-sm btn-blue" id="angry-btn" onclick="runGesture('angry')">Angry</button>
    <button class="btn btn-sm btn-blue" id="celebrate-btn" onclick="runGesture('celebrate')">Celebrate</button>
    <button class="btn btn-sm btn-blue" id="shrug-btn" onclick="runGesture('shrug')">Shrug</button>
    <button class="btn btn-sm btn-blue" id="dab-btn" onclick="runGesture('dab')">Dab</button>
    <button class="btn btn-sm btn-blue" id="fight-btn" onclick="runGesture('fight')">Fight</button>
    <button class="btn btn-sm btn-blue" id="dance-btn" onclick="runGesture('dance')">Dance</button>
    <button class="btn btn-sm btn-green" id="runall-btn" onclick="runGesture('run_all')">Run All</button>
  </div>
</div>

<div class="grid" id="grid"></div>

<div class="presets-section">
  <h2>Presets</h2>
  <div class="save-row">
    <input type="text" id="preset-name" placeholder="Preset name...">
    <button class="btn btn-sm btn-green" onclick="savePreset()">Save Current</button>
  </div>
  <div class="preset-list" id="preset-list"></div>
</div>

<script>
const state = {{ state | tojson }};
let presets = {{ presets | tojson }};

function createCard(ch) {
  const s = state[ch];
  const disabledClass = s.enabled ? '' : 'disabled';
  const checked = s.enabled ? 'checked' : '';

  let sliderHtml = '';
  let btnHtml = '';
  let unitText = '';

  if (s.mode === 'servo') {
    unitText = '&deg;';
    const sMin = s.safe_min != null ? s.safe_min : 0;
    const sMax = s.safe_max != null ? s.safe_max : 180;
    sliderHtml = `<input type="range" min="${sMin}" max="${sMax}" value="${s.value}" id="slider-${ch}" oninput="sendVal(${ch}, this.value)" ${s.enabled?'':'disabled'}>`;
    btnHtml = `<div class="btn-row">
      <button class="btn btn-sm btn-blue" onclick="quickSet(${ch},${sMin})" ${s.enabled?'':'disabled'}>${sMin}&deg;</button>
      <button class="btn btn-sm" onclick="quickSet(${ch},90)" ${s.enabled?'':'disabled'}>90&deg;</button>
      <button class="btn btn-sm btn-blue" onclick="quickSet(${ch},${sMax})" ${s.enabled?'':'disabled'}>${sMax}&deg;</button>
    </div>`;
  } else if (s.mode === 'continuous') {
    unitText = '';
    sliderHtml = `<input type="range" min="-100" max="100" value="${s.value}" id="slider-${ch}" oninput="sendVal(${ch}, this.value)" ${s.enabled?'':'disabled'}>`;
    btnHtml = `<div class="btn-row">
      <button class="btn btn-sm btn-blue" onclick="quickSet(${ch},-100)" ${s.enabled?'':'disabled'}>-100%</button>
      <button class="btn btn-sm btn-danger" onclick="quickSet(${ch},0)" ${s.enabled?'':'disabled'}>Stop</button>
      <button class="btn btn-sm btn-blue" onclick="quickSet(${ch},100)" ${s.enabled?'':'disabled'}>100%</button>
    </div>`;
  } else {
    unitText = '%';
    sliderHtml = `<input type="range" min="0" max="100" value="${s.value}" id="slider-${ch}" oninput="sendVal(${ch}, this.value)" ${s.enabled?'':'disabled'}>`;
    btnHtml = `<div class="btn-row">
      <button class="btn btn-sm btn-blue" onclick="quickSet(${ch},0)" ${s.enabled?'':'disabled'}>0%</button>
      <button class="btn btn-sm" onclick="quickSet(${ch},50)" ${s.enabled?'':'disabled'}>50%</button>
      <button class="btn btn-sm btn-blue" onclick="quickSet(${ch},100)" ${s.enabled?'':'disabled'}>100%</button>
    </div>`;
  }

  let revHtml = '';
  if (s.mode === 'servo') {
    const revChecked = s.reversed ? 'checked' : '';
    revHtml = `<div class="rev-row">
      <span>Reverse</span>
      <label class="toggle">
        <input type="checkbox" id="rev-${ch}" ${revChecked} onchange="toggleReverse(${ch}, this.checked)">
        <span></span>
      </label>
    </div>`;
  }

  let pulseHtml = '';
  if (s.mode === 'servo' || s.mode === 'continuous') {
    pulseHtml = `<div class="pulse-row">
      <span>Pulse:</span>
      <input type="number" id="pmin-${ch}" value="${s.min_pulse}" min="500" max="2500" step="50" onchange="setPulse(${ch})">
      <span>-</span>
      <input type="number" id="pmax-${ch}" value="${s.max_pulse}" min="500" max="2500" step="50" onchange="setPulse(${ch})">
      <span>&micro;s</span>
    </div>`;
  }

  const displayVal = s.mode === 'continuous' ? (s.value > 0 ? '+' + s.value : s.value) : s.value;

  const nameLabel = s.name ? ` <span style="color:#888;font-size:11px">${s.name}</span>` : '';

  let limitsHtml = '';
  if (s.mode === 'servo') {
    limitsHtml = `<div class="pulse-row" style="margin-top:4px">
      <span>Limits:</span>
      <input type="number" id="smin-${ch}" value="${s.safe_min}" min="0" max="180" onchange="setLimits(${ch})">
      <span>-</span>
      <input type="number" id="smax-${ch}" value="${s.safe_max}" min="0" max="180" onchange="setLimits(${ch})">
      <span>&deg;</span>
    </div>`;
  }

  return `<div class="card ${disabledClass}" id="card-${ch}">
    <div class="card-header">
      <h3>CH ${ch}${nameLabel}</h3>
      <div class="controls">
        <select class="mode-select" id="mode-${ch}" onchange="setMode(${ch}, this.value)">
          <option value="servo" ${s.mode==='servo'?'selected':''}>Servo</option>
          <option value="continuous" ${s.mode==='continuous'?'selected':''}>Continuous</option>
          <option value="pwm" ${s.mode==='pwm'?'selected':''}>PWM</option>
        </select>
        <label class="toggle">
          <input type="checkbox" id="en-${ch}" ${checked} onchange="toggleCh(${ch}, this.checked)">
          <span></span>
        </label>
      </div>
    </div>
    <div class="slider-row">
      ${sliderHtml}
      <span class="val" id="val-${ch}">${displayVal}</span><span class="unit" id="unit-${ch}">${unitText}</span>
    </div>
    ${btnHtml}
    ${revHtml}
    ${pulseHtml}
    ${limitsHtml}
  </div>`;
}

function renderGrid() {
  const grid = document.getElementById('grid');
  grid.innerHTML = '';
  for (let i = 0; i < 16; i++) grid.innerHTML += createCard(i);
}

function sendVal(ch, val) {
  val = parseInt(val);
  state[ch].value = val;
  const display = state[ch].mode === 'continuous' ? (val > 0 ? '+' + val : val) : val;
  document.getElementById('val-' + ch).textContent = display;
  document.getElementById('slider-' + ch).value = val;
  fetch('/set', { method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({channel: ch, value: val, mode: state[ch].mode}) });
}

function quickSet(ch, val) {
  document.getElementById('slider-' + ch).value = val;
  sendVal(ch, val);
}

function setMode(ch, mode) {
  let defaultVal = mode === 'servo' ? 90 : 0;
  state[ch].mode = mode;
  state[ch].value = defaultVal;
  fetch('/mode', { method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({channel: ch, mode: mode}) }).then(() => renderGrid());
}

function toggleCh(ch, enabled) {
  state[ch].enabled = enabled;
  document.getElementById('card-' + ch).className = 'card' + (enabled ? '' : ' disabled');
  fetch('/enable', { method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({channel: ch, enabled: enabled}) }).then(() => renderGrid());
}

function toggleReverse(ch, reversed) {
  state[ch].reversed = reversed;
  fetch('/reverse', { method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({channel: ch, reversed: reversed}) });
}

function setPulse(ch) {
  const min = parseInt(document.getElementById('pmin-' + ch).value);
  const max = parseInt(document.getElementById('pmax-' + ch).value);
  state[ch].min_pulse = min;
  state[ch].max_pulse = max;
  fetch('/pulse', { method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({channel: ch, min_pulse: min, max_pulse: max}) });
}

function setLimits(ch) {
  const smin = parseInt(document.getElementById('smin-' + ch).value);
  const smax = parseInt(document.getElementById('smax-' + ch).value);
  state[ch].safe_min = smin;
  state[ch].safe_max = smax;
  // Clamp current value to new limits
  if (state[ch].value < smin) quickSet(ch, smin);
  if (state[ch].value > smax) quickSet(ch, smax);
  fetch('/config', { method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({channel: ch, safe_min: smin, safe_max: smax}) }).then(() => renderGrid());
}

function setFreq() {
  const freq = parseInt(document.getElementById('freq').value);
  fetch('/frequency', { method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({frequency: freq}) });
}

function setAllServo(val) {
  for (let i = 0; i < 16; i++) {
    if (state[i].mode === 'servo' && state[i].enabled) quickSet(i, val);
  }
}

function enableAll(en) {
  for (let i = 0; i < 16; i++) toggleCh(i, en);
}

function renderPresets() {
  const list = document.getElementById('preset-list');
  list.innerHTML = '';
  for (const name in presets) {
    list.innerHTML += `<div class="preset-item">
      <button class="btn btn-sm btn-outline" onclick="loadPreset('${name}')">${name}</button>
      <button class="btn btn-sm btn-danger" onclick="deletePreset('${name}')">&times;</button>
    </div>`;
  }
}

function savePreset() {
  const name = document.getElementById('preset-name').value.trim();
  if (!name) return;
  fetch('/presets/save', { method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({name: name, state: JSON.parse(JSON.stringify(state))})
  }).then(r => r.json()).then(d => { presets[name] = d.state; renderPresets(); });
  document.getElementById('preset-name').value = '';
}

function loadPreset(name) {
  fetch('/presets/load', { method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({name: name})
  }).then(r => r.json()).then(d => {
    for (let ch in d.state) {
      const s = d.state[ch];
      state[ch].mode = s.mode;
      state[ch].value = s.value;
      state[ch].enabled = s.enabled;
      state[ch].reversed = s.reversed || false;
      state[ch].min_pulse = s.min_pulse;
      state[ch].max_pulse = s.max_pulse;
    }
    renderGrid();
  });
}

function deletePreset(name) {
  fetch('/presets/delete', { method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({name: name})
  }).then(() => { delete presets[name]; renderPresets(); });
}

let sweeping = false;
let sweepPoll = null;

function toggleSweep() {
  sweeping = !sweeping;
  const btn = document.getElementById('sweep-btn');
  btn.textContent = sweeping ? 'Stop Sweep' : 'Sweep';
  btn.className = sweeping ? 'btn btn-sm btn-danger' : 'btn btn-sm';
  fetch('/sweep', { method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({active: sweeping}) });
  if (sweeping) {
    sweepPoll = setInterval(pollState, 200);
  } else {
    clearInterval(sweepPoll);
    sweepPoll = null;
  }
}

function pollState() {
  fetch('/state').then(r => r.json()).then(d => {
    for (let ch in d.state) {
      const s = d.state[ch];
      state[ch].value = s.value;
      const slider = document.getElementById('slider-' + ch);
      const val = document.getElementById('val-' + ch);
      if (slider && s.mode === 'servo') { slider.value = s.value; }
      if (val && s.mode === 'servo') { val.textContent = s.value; }
    }
    if (!d.sweeping && sweeping) {
      sweeping = false;
      const btn = document.getElementById('sweep-btn');
      btn.textContent = 'Sweep';
      btn.className = 'btn btn-sm';
      clearInterval(sweepPoll);
      sweepPoll = null;
    }
  });
}

function runDemo() { runGesture('demo'); }

const gestureLabels = {demo:'Demo', run:'Run', point:'Point', scared:'Scared', wave:'Wave', angry:'Angry', celebrate:'Celebrate', shrug:'Shrug', dab:'Dab', fight:'Fight', dance:'Dance', run_all:'Run All'};

function runGesture(name) {
  // Disable all gesture buttons
  const btns = ['demo-btn','run-btn','point-btn','scared-btn','wave-btn','angry-btn','celebrate-btn','shrug-btn','dab-btn','fight-btn','dance-btn','runall-btn'];
  btns.forEach(id => { const b = document.getElementById(id); if (b) { b.disabled = true; } });
  const btn = document.getElementById(name + '-btn');
  if (btn) { btn.textContent = 'Running...'; btn.className = 'btn btn-sm btn-danger'; }
  fetch('/gesture', { method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({name: name}) }).then(() => {
    const poll = setInterval(() => {
      fetch('/state').then(r => r.json()).then(d => {
        for (let ch in d.state) {
          const s = d.state[ch];
          state[ch].value = s.value;
          const slider = document.getElementById('slider-' + ch);
          const val = document.getElementById('val-' + ch);
          if (slider && s.mode === 'servo') { slider.value = s.value; }
          if (val && s.mode === 'servo') { val.textContent = s.value; }
        }
        if (!d.demo_running) {
          clearInterval(poll);
          btns.forEach(id => { const b = document.getElementById(id); if (b) { b.disabled = false; } });
          if (btn) { btn.textContent = gestureLabels[name] || name; btn.className = name === 'demo' ? 'btn btn-sm btn-green' : 'btn btn-sm btn-blue'; }
        }
      });
    }, 200);
  });
}

renderGrid();
renderPresets();
</script>
</body>
</html>"""


@app.route("/")
def index():
    presets = load_presets()
    return render_template_string(HTML, state=channels, frequency=pwm_frequency, presets=presets)


@app.route("/set", methods=["POST"])
def set_value():
    data = request.get_json()
    ch = int(data["channel"])
    val = int(data["value"])
    mode = data.get("mode", channels[ch]["mode"])

    channels[ch]["value"] = val
    try:
        if not channels[ch]["enabled"]:
            return jsonify({"ok": True})
        if mode == "servo":
            val = max(0, min(180, val))
            kit.servo[ch].angle = apply_reverse(ch, val)
        elif mode == "continuous":
            throttle = max(-100, min(100, val)) / 100.0
            kit.continuous_servo[ch].throttle = throttle
        elif mode == "pwm":
            duty = max(0, min(100, val))
            kit._pca.channels[ch].duty_cycle = int(duty / 100.0 * 0xFFFF)
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500
    return jsonify({"ok": True})


@app.route("/mode", methods=["POST"])
def set_mode():
    data = request.get_json()
    ch = int(data["channel"])
    mode = data["mode"]
    channels[ch]["mode"] = mode

    if mode == "servo":
        channels[ch]["value"] = 90
        try:
            kit.servo[ch].set_pulse_width_range(channels[ch]["min_pulse"], channels[ch]["max_pulse"])
            if channels[ch]["enabled"]:
                kit.servo[ch].angle = apply_reverse(ch, 90)
        except Exception:
            pass
    elif mode == "continuous":
        channels[ch]["value"] = 0
        try:
            kit.continuous_servo[ch].set_pulse_width_range(channels[ch]["min_pulse"], channels[ch]["max_pulse"])
            if channels[ch]["enabled"]:
                kit.continuous_servo[ch].throttle = 0
        except Exception:
            pass
    elif mode == "pwm":
        channels[ch]["value"] = 0
        try:
            kit._pca.channels[ch].duty_cycle = 0
        except Exception:
            pass
    return jsonify({"ok": True})


@app.route("/enable", methods=["POST"])
def enable_channel():
    data = request.get_json()
    ch = int(data["channel"])
    enabled = data["enabled"]
    channels[ch]["enabled"] = enabled
    try:
        if not enabled:
            kit._pca.channels[ch].duty_cycle = 0
        else:
            mode = channels[ch]["mode"]
            val = channels[ch]["value"]
            if mode == "servo":
                kit.servo[ch].angle = apply_reverse(ch, max(0, min(180, val)))
            elif mode == "continuous":
                kit.continuous_servo[ch].throttle = max(-100, min(100, val)) / 100.0
            elif mode == "pwm":
                kit._pca.channels[ch].duty_cycle = int(max(0, min(100, val)) / 100.0 * 0xFFFF)
    except Exception:
        pass
    return jsonify({"ok": True})


@app.route("/pulse", methods=["POST"])
def set_pulse():
    data = request.get_json()
    ch = int(data["channel"])
    min_p = int(data["min_pulse"])
    max_p = int(data["max_pulse"])
    channels[ch]["min_pulse"] = min_p
    channels[ch]["max_pulse"] = max_p
    try:
        if channels[ch]["mode"] == "servo":
            kit.servo[ch].set_pulse_width_range(min_p, max_p)
        elif channels[ch]["mode"] == "continuous":
            kit.continuous_servo[ch].set_pulse_width_range(min_p, max_p)
    except Exception:
        pass
    return jsonify({"ok": True})


@app.route("/reverse", methods=["POST"])
def set_reverse():
    data = request.get_json()
    ch = int(data["channel"])
    channels[ch]["reversed"] = data["reversed"]
    # Re-apply current value with new direction
    try:
        if channels[ch]["mode"] == "servo" and channels[ch]["enabled"]:
            hw_angle = apply_reverse(ch, channels[ch]["value"])
            kit.servo[ch].angle = hw_angle
    except Exception:
        pass
    return jsonify({"ok": True})


@app.route("/config", methods=["POST"])
def set_config():
    data = request.get_json()
    ch = int(data["channel"])
    if "name" in data:
        channels[ch]["name"] = data["name"]
    if "safe_min" in data:
        channels[ch]["safe_min"] = int(data["safe_min"])
    if "safe_max" in data:
        channels[ch]["safe_max"] = int(data["safe_max"])
    # Persist to config file
    config = load_config()
    config[str(ch)] = {
        "name": channels[ch]["name"],
        "safe_min": channels[ch]["safe_min"],
        "safe_max": channels[ch]["safe_max"],
    }
    save_config(config)
    return jsonify({"ok": True})


@app.route("/frequency", methods=["POST"])
def set_frequency():
    global pwm_frequency
    data = request.get_json()
    freq = int(data["frequency"])
    freq = max(24, min(1526, freq))
    pwm_frequency = freq
    try:
        kit._pca.frequency = freq
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500
    return jsonify({"ok": True})


@app.route("/presets/save", methods=["POST"])
def preset_save():
    data = request.get_json()
    name = data["name"]
    presets = load_presets()
    presets[name] = {str(k): v for k, v in channels.items()}
    save_presets(presets)
    return jsonify({"ok": True, "state": presets[name]})


@app.route("/presets/load", methods=["POST"])
def preset_load():
    data = request.get_json()
    name = data["name"]
    presets = load_presets()
    if name not in presets:
        return jsonify({"ok": False, "error": "not found"}), 404
    preset = presets[name]
    for ch_str, s in preset.items():
        ch = int(ch_str)
        channels[ch].update(s)
        try:
            if channels[ch]["mode"] in ("servo", "continuous"):
                kit.servo[ch].set_pulse_width_range(s["min_pulse"], s["max_pulse"])
            if not channels[ch]["enabled"]:
                kit._pca.channels[ch].duty_cycle = 0
            elif channels[ch]["mode"] == "servo":
                kit.servo[ch].angle = apply_reverse(ch, max(0, min(180, s["value"])))
            elif channels[ch]["mode"] == "continuous":
                kit.continuous_servo[ch].throttle = max(-100, min(100, s["value"])) / 100.0
            elif channels[ch]["mode"] == "pwm":
                kit._pca.channels[ch].duty_cycle = int(max(0, min(100, s["value"])) / 100.0 * 0xFFFF)
        except Exception:
            pass
    return jsonify({"ok": True, "state": preset})


@app.route("/presets/delete", methods=["POST"])
def preset_delete():
    data = request.get_json()
    name = data["name"]
    presets = load_presets()
    presets.pop(name, None)
    save_presets(presets)
    return jsonify({"ok": True})


sweep_active = False
sweep_lock = threading.Lock()


def sweep_worker():
    global sweep_active
    step = 2
    delay = 0.02
    while True:
        with sweep_lock:
            if not sweep_active:
                return
        # Sweep 0 -> 180
        for angle in range(0, 181, step):
            with sweep_lock:
                if not sweep_active:
                    return
            for ch in range(16):
                if channels[ch]["mode"] == "servo" and channels[ch]["enabled"]:
                    channels[ch]["value"] = angle
                    try:
                        kit.servo[ch].angle = apply_reverse(ch, angle)
                    except Exception:
                        pass
            time.sleep(delay)
        # Sweep 180 -> 0
        for angle in range(180, -1, -step):
            with sweep_lock:
                if not sweep_active:
                    return
            for ch in range(16):
                if channels[ch]["mode"] == "servo" and channels[ch]["enabled"]:
                    channels[ch]["value"] = angle
                    try:
                        kit.servo[ch].angle = apply_reverse(ch, angle)
                    except Exception:
                        pass
            time.sleep(delay)


@app.route("/sweep", methods=["POST"])
def toggle_sweep():
    global sweep_active
    data = request.get_json()
    active = data.get("active", False)
    with sweep_lock:
        sweep_active = active
    if active:
        t = threading.Thread(target=sweep_worker, daemon=True)
        t.start()
    return jsonify({"ok": True, "active": active})


gesture_active = False
gesture_lock = threading.Lock()


def clamp_to_limits(ch, val):
    """Clamp a value to the channel's safe limits."""
    safe_min = channels[ch].get("safe_min", 0)
    safe_max = channels[ch].get("safe_max", 180)
    return max(safe_min, min(safe_max, val))


def move_to(targets, steps=20, delay=0.03):
    """Smoothly move channels to target positions. targets = {ch: angle}"""
    starts = {ch: channels[ch]["value"] for ch in targets}
    clamped = {ch: clamp_to_limits(ch, val) for ch, val in targets.items()}
    for i in range(1, steps + 1):
        with gesture_lock:
            if not gesture_active:
                return
        for ch, target in clamped.items():
            val = int(starts[ch] + (target - starts[ch]) * i / steps)
            channels[ch]["value"] = val
            try:
                if channels[ch]["enabled"]:
                    kit.servo[ch].angle = apply_reverse(ch, val)
            except Exception:
                pass
        time.sleep(delay)


def move_to_center(chs):
    """Return channels to 90 center, clamped to limits."""
    targets = {ch: clamp_to_limits(ch, 90) for ch in chs}
    move_to(targets)


HEAD_CHS = [2, 3]
ALL_ARM_CHS = [4, 5, 6, 7, 8, 9, 10, 11]
ALL_BODY_CHS = HEAD_CHS + ALL_ARM_CHS


def gesture_demo():
    """Demo: test each joint pair through its range."""
    # Head rotation left-right
    move_to({2: 30})
    move_to({2: 150})
    move_to_center(HEAD_CHS)
    # Head tilt up-down
    move_to({3: 50})
    move_to({3: 140})
    move_to_center(HEAD_CHS)
    # Arms
    biceps = [4, 8]
    arm_rot = [5, 9]
    shoulder_lift = [6, 10]
    shoulder_rot = [7, 11]
    for _ in range(2):
        move_to({4: 160, 8: 160})
        move_to_center(biceps)
    for _ in range(2):
        move_to({5: 70, 9: 70})
        move_to_center(arm_rot)
        move_to({5: 110, 9: 110})
        move_to_center(arm_rot)
    for _ in range(2):
        move_to({6: 70, 10: 70})
        move_to_center(shoulder_lift)
        move_to({6: 110, 10: 110})
        move_to_center(shoulder_lift)
    for _ in range(2):
        move_to({7: 70, 11: 70})
        move_to_center(shoulder_rot)
        move_to({7: 150, 11: 150})
        move_to_center(shoulder_rot)


def gesture_run():
    """Running: arms swing alternately with slight bicep bend, head bobs."""
    for _ in range(6):
        # Right arm forward, left arm back, head bobs down
        move_to({7: 120, 11: 70, 4: 120, 8: 100, 3: 100}, steps=15, delay=0.02)
        # Left arm forward, right arm back, head bobs up
        move_to({7: 70, 11: 120, 4: 100, 8: 120, 3: 80}, steps=15, delay=0.02)
    move_to_center(ALL_BODY_CHS)


def gesture_point():
    """Point with right arm: raise shoulder, extend arm forward, head looks at target."""
    # Raise right arm forward and straighten bicep, head looks right
    move_to({7: 140, 4: 90, 5: 90, 6: 90, 2: 40}, steps=25)
    time.sleep(1.0)
    move_to_center([2, 3, 4, 5, 6, 7])


def gesture_scared():
    """Scared: bring both arms up to face level, biceps bent, head ducks."""
    # Arms up in front, bent at bicep, rotated inward, head tilts down and trembles
    move_to({7: 130, 11: 130, 4: 180, 8: 180, 6: 70, 10: 70, 5: 100, 9: 80, 3: 130}, steps=10, delay=0.02)
    # Shake with arm rotation trembling inward, head shakes
    for _ in range(3):
        move_to({5: 105, 9: 75, 2: 80}, steps=5, delay=0.02)
        move_to({5: 95, 9: 85, 2: 100}, steps=5, delay=0.02)
    time.sleep(0.3)
    move_to_center(ALL_BODY_CHS)


def gesture_wave():
    """Wave: raise right arm and wave side to side, head follows hand."""
    # Raise right arm up
    move_to({7: 140, 4: 130}, steps=25)
    # Wave by rotating arm side to side, head tracks the wave
    for _ in range(4):
        move_to({5: 75, 6: 80, 2: 70}, steps=10, delay=0.02)
        move_to({5: 105, 6: 100, 2: 110}, steps=10, delay=0.02)
    move_to_center([2, 3, 4, 5, 6, 7])


def gesture_angry():
    """Angry: punch forward aggressively, head thrusts forward."""
    # Head tilts down aggressively, raise arm slightly
    move_to({7: 115, 3: 120}, steps=15)
    for _ in range(3):
        # Punch: extend bicep fast, head jabs forward
        move_to({4: 155, 3: 135}, steps=8, delay=0.02)
        time.sleep(0.15)
        # Pull back
        move_to({4: 100, 3: 115}, steps=8, delay=0.02)
        time.sleep(0.1)
    move_to_center([2, 3, 4, 7])


def gesture_celebrate():
    """Celebrate: both arms up, shake them, head looks up and around."""
    # Both arms up, head tilts up
    move_to({7: 145, 11: 145, 4: 130, 8: 130, 3: 55}, steps=20)
    # Shake arms, head sways side to side
    for _ in range(4):
        move_to({5: 80, 9: 100, 2: 70}, steps=6, delay=0.02)
        move_to({5: 100, 9: 80, 2: 110}, steps=6, delay=0.02)
    move_to_center(ALL_BODY_CHS)


def gesture_shrug():
    """Shrug: lift shoulders up, bend arms out, head tilts to side."""
    # Shoulders up, arms out to sides, biceps bent, head tilts
    move_to({6: 105, 10: 105, 7: 110, 11: 110, 4: 130, 8: 130, 2: 120, 3: 70}, steps=15)
    time.sleep(0.8)
    move_to_center(ALL_BODY_CHS)


def gesture_dab():
    """Dab: right arm out to side, left arm bent across face, head dips into elbow."""
    # Left arm up across face, right arm extended, head dips down and right
    move_to({8: 150, 11: 120, 10: 100, 4: 90, 7: 90, 6: 105, 2: 40, 3: 120}, steps=20)
    time.sleep(1.0)
    move_to_center(ALL_BODY_CHS)


def gesture_fight():
    """Fight: alternating punches, uppercuts, and combos (~5 seconds)."""
    # Guard stance - arms up, biceps bent, head down ready
    move_to({7: 115, 11: 115, 4: 150, 8: 150, 3: 115}, steps=15)
    time.sleep(0.2)
    # Right jab - head tracks right
    move_to({7: 135, 4: 100, 2: 70}, steps=8, delay=0.02)
    move_to({7: 115, 4: 150, 2: 90}, steps=8, delay=0.02)
    # Left jab - head tracks left
    move_to({11: 135, 8: 100, 2: 110}, steps=8, delay=0.02)
    move_to({11: 115, 8: 150, 2: 90}, steps=8, delay=0.02)
    # Right-left combo (fast)
    move_to({7: 140, 4: 95, 2: 70}, steps=6, delay=0.02)
    move_to({7: 115, 4: 150}, steps=6, delay=0.02)
    move_to({11: 140, 8: 95, 2: 110}, steps=6, delay=0.02)
    move_to({11: 115, 8: 150, 2: 90}, steps=6, delay=0.02)
    # Right uppercut - shoulder rotation up with bicep extend
    move_to({7: 145, 4: 100, 6: 80, 3: 130}, steps=8, delay=0.02)
    move_to({7: 115, 4: 150, 6: 90, 3: 115}, steps=10, delay=0.02)
    # Left uppercut
    move_to({11: 145, 8: 100, 10: 80, 3: 130}, steps=8, delay=0.02)
    move_to({11: 115, 8: 150, 10: 90, 3: 115}, steps=10, delay=0.02)
    # Flurry - rapid alternating punches, head bobs
    for _ in range(4):
        move_to({7: 135, 4: 100, 11: 110, 8: 160, 2: 75}, steps=5, delay=0.02)
        move_to({7: 110, 4: 160, 11: 135, 8: 100, 2: 105}, steps=5, delay=0.02)
    # Final big right hook - arm rotation + shoulder, head follows
    move_to({5: 75, 7: 140, 4: 100, 2: 50}, steps=10, delay=0.02)
    time.sleep(0.3)
    move_to_center(ALL_BODY_CHS)


def gesture_dance():
    """Dance: rhythmic arm movements, shimmies, and grooves (~10 seconds)."""
    # Warm up - arms out to sides, head bobs
    move_to({6: 105, 10: 105, 7: 110, 11: 110, 3: 70}, steps=15)
    # Disco point - right arm up, left down, head follows pointing arm
    for _ in range(3):
        move_to({7: 140, 4: 100, 11: 80, 8: 150, 2: 60, 3: 55}, steps=10, delay=0.02)
        move_to({7: 80, 4: 150, 11: 140, 8: 100, 2: 120, 3: 55}, steps=10, delay=0.02)
    # Shoulder shimmy - alternate shoulder lifts, head bobs
    for _ in range(4):
        move_to({6: 75, 10: 105, 3: 75}, steps=6, delay=0.02)
        move_to({6: 105, 10: 75, 3: 65}, steps=6, delay=0.02)
    # Arm wave - both arms sweep side to side, head sways
    for _ in range(3):
        move_to({5: 75, 9: 75, 7: 120, 11: 120, 2: 60}, steps=10, delay=0.02)
        move_to({5: 105, 9: 105, 7: 120, 11: 120, 2: 120}, steps=10, delay=0.02)
    # Robot groove - arms bent, pump up and down, head nods
    move_to({4: 150, 8: 150}, steps=10)
    for _ in range(4):
        move_to({7: 130, 11: 130, 3: 75}, steps=8, delay=0.02)
        move_to({7: 100, 11: 100, 3: 60}, steps=8, delay=0.02)
    # Funky chicken - shoulder lifts with arm rotation, head bobs
    for _ in range(3):
        move_to({6: 105, 10: 105, 5: 80, 9: 100, 4: 170, 8: 170, 3: 80}, steps=8, delay=0.02)
        move_to({6: 80, 10: 80, 5: 100, 9: 80, 4: 130, 8: 130, 3: 60}, steps=8, delay=0.02)
    # Finish with a dab
    move_to({8: 150, 11: 120, 10: 100, 4: 90, 7: 90, 6: 105, 2: 40, 3: 120}, steps=15)
    time.sleep(0.5)
    move_to_center(ALL_BODY_CHS)


def gesture_run_all():
    """Run all gestures in sequence with delays."""
    time.sleep(5)
    all_gestures = [
        gesture_run, gesture_point, gesture_scared, gesture_wave,
        gesture_angry, gesture_celebrate, gesture_shrug, gesture_dab,
        gesture_fight, gesture_dance,
    ]
    for i, func in enumerate(all_gestures):
        with gesture_lock:
            if not gesture_active:
                return
        func()
        move_to_center(ALL_BODY_CHS)
        if i < len(all_gestures) - 1:
            time.sleep(1)


GESTURES = {
    "demo": gesture_demo,
    "run": gesture_run,
    "point": gesture_point,
    "scared": gesture_scared,
    "wave": gesture_wave,
    "angry": gesture_angry,
    "celebrate": gesture_celebrate,
    "shrug": gesture_shrug,
    "dab": gesture_dab,
    "fight": gesture_fight,
    "dance": gesture_dance,
    "run_all": gesture_run_all,
}


def gesture_worker(name):
    global gesture_active
    try:
        func = GESTURES.get(name)
        if func:
            func()
    finally:
        move_to_center(ALL_BODY_CHS)
        with gesture_lock:
            gesture_active = False


@app.route("/gesture", methods=["POST"])
def run_gesture():
    global gesture_active
    data = request.get_json()
    name = data.get("name", "demo")
    with gesture_lock:
        if gesture_active:
            return jsonify({"ok": False, "error": "gesture running"})
        gesture_active = True
    t = threading.Thread(target=gesture_worker, args=(name,), daemon=True)
    t.start()
    return jsonify({"ok": True, "gesture": name})


@app.route("/demo", methods=["POST"])
def toggle_demo():
    """Backwards compat."""
    global gesture_active
    with gesture_lock:
        if gesture_active:
            return jsonify({"ok": True, "active": True})
        gesture_active = True
    t = threading.Thread(target=gesture_worker, args=("demo",), daemon=True)
    t.start()
    return jsonify({"ok": True, "active": True})


@app.route("/state", methods=["GET"])
def get_state():
    return jsonify({"state": {str(k): v for k, v in channels.items()}, "sweeping": sweep_active, "demo_running": gesture_active})


def generate_frames():
    while True:
        with camera_lock:
            success, frame = camera.read()
        if not success:
            continue
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ret:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=80)
    finally:
        camera.release()
