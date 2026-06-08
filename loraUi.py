from flask import Flask, render_template_string, jsonify, request
import threading
import time
import re
from collections import deque

import serial
from serial.tools import list_ports

app = Flask(__name__)

SERIAL_PORT = 'COM18'          # Change this to your STM32 CDC port, e.g. COM3 or /dev/ttyACM0
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 1
MAX_POINTS = 120
lock = threading.Lock()

state = {
    "time": deque(maxlen=MAX_POINTS),
    "alt": deque(maxlen=MAX_POINTS),
    "vel": deque(maxlen=MAX_POINTS),
    "temp": deque(maxlen=MAX_POINTS),
    "pressure": deque(maxlen=MAX_POINTS),
    "ax_hist": deque(maxlen=MAX_POINTS),
    "ay_hist": deque(maxlen=MAX_POINTS),
    "az_hist": deque(maxlen=MAX_POINTS),
    "gx": 0.0,
    "gy": 0.0,
    "gz": 0.0,
    "ax": 0.0,
    "ay": 0.0,
    "az": 0.0,
    "lat": 18.5204,
    "lon": 73.8567,
    "rssi": -70,
    "status": "WAITING",
    "phase": "No data",
    "packet": 0,
    "apogee": 0.0,
    "max_temp": 0.0,
    "min_rssi": -70,
    "last_update": 0.0,
    "sat": 0,
    "fix": 0,
    "port": SERIAL_PORT,
    "serial_ok": False,
    "last_raw": ""
}

telemetry_pattern = re.compile(
    r"PKT:(?P<pkt>\d+)\s+"
    r"ALT:(?P<alt>-?\d+(?:\.\d+)?)m\s+"
    r"VEL:(?P<vel>-?\d+(?:\.\d+)?)m/s\s+"
    r"P:(?P<pres>-?\d+(?:\.\d+)?)hPa\s+"
    r"T:(?P<temp>-?\d+(?:\.\d+)?)C\s+"
    r"SAT:(?P<sat>\d+)\s+"
    r"FIX:(?P<fix>\d+)\s+"
    r"GX:(?P<gx>-?\d+(?:\.\d+)?)\s+"
    r"GY:(?P<gy>-?\d+(?:\.\d+)?)\s+"
    r"GZ:(?P<gz>-?\d+(?:\.\d+)?)\s+"
    r"AX:(?P<ax>-?\d+(?:\.\d+)?)\s+"
    r"AY:(?P<ay>-?\d+(?:\.\d+)?)\s+"
    r"AZ:(?P<az>-?\d+(?:\.\d+)?)\s+"
    r"LAT:(?P<lat>-?\d+(?:\.\d+)?)\s+"
    r"LON:(?P<lon>-?\d+(?:\.\d+)?)\s+"
    r"RSSI:(?P<rssi>-?\d+)"
)


def available_ports():
    return [p.device for p in list_ports.comports()]


def to_list_payload():
    with lock:
        return {
            "time": list(state["time"]),
            "alt": list(state["alt"]),
            "vel": list(state["vel"]),
            "temp": list(state["temp"]),
            "pressure": list(state["pressure"]),
            "ax_hist": list(state["ax_hist"]),
            "ay_hist": list(state["ay_hist"]),
            "az_hist": list(state["az_hist"]),
            "gx": state["gx"],
            "gy": state["gy"],
            "gz": state["gz"],
            "ax": state["ax"],
            "ay": state["ay"],
            "az": state["az"],
            "lat": state["lat"],
            "lon": state["lon"],
            "rssi": state["rssi"],
            "status": state["status"],
            "phase": state["phase"],
            "packet": state["packet"],
            "apogee": state["apogee"],
            "max_temp": state["max_temp"],
            "min_rssi": state["min_rssi"],
            "last_update": state["last_update"],
            "sat": state["sat"],
            "fix": state["fix"],
            "port": state["port"],
            "serial_ok": state["serial_ok"],
            "last_raw": state["last_raw"]
        }


def derive_phase(alt, vel, fix):
    if fix == 0:
        return "No GPS fix"
    if alt < 5 and abs(vel) < 1:
        return "Idle"
    if vel > 3:
        return "Ascent"
    if vel < -3:
        return "Descent"
    return "Cruise"


def update_from_line(line, start_time):
    m = telemetry_pattern.search(line)
    if not m:
        with lock:
            state["last_raw"] = line.strip()
        return False

    pkt = int(m.group('pkt'))
    alt = float(m.group('alt'))
    vel = float(m.group('vel'))
    pressure = float(m.group('pres'))
    temp = float(m.group('temp'))
    sat = int(m.group('sat'))
    fix = int(m.group('fix'))
    gx = float(m.group('gx'))
    gy = float(m.group('gy'))
    gz = float(m.group('gz'))
    ax = float(m.group('ax'))
    ay = float(m.group('ay'))
    az = float(m.group('az'))
    lat = float(m.group('lat'))
    lon = float(m.group('lon'))
    rssi = int(m.group('rssi'))

    t = round(time.time() - start_time, 1)
    phase = derive_phase(alt, vel, fix)
    status = "LIVE" if fix else "NO_FIX"

    with lock:
        state["time"].append(t)
        state["alt"].append(round(alt, 2))
        state["vel"].append(round(vel, 2))
        state["temp"].append(round(temp, 2))
        state["pressure"].append(round(pressure, 2))
        state["ax_hist"].append(round(ax, 2))
        state["ay_hist"].append(round(ay, 2))
        state["az_hist"].append(round(az, 2))
        state["gx"] = round(gx, 2)
        state["gy"] = round(gy, 2)
        state["gz"] = round(gz, 2)
        state["ax"] = round(ax, 2)
        state["ay"] = round(ay, 2)
        state["az"] = round(az, 2)
        state["lat"] = round(lat, 6)
        state["lon"] = round(lon, 6)
        state["rssi"] = rssi
        state["status"] = status
        state["phase"] = phase
        state["packet"] = pkt
        state["sat"] = sat
        state["fix"] = fix
        state["apogee"] = round(max(state["apogee"], alt), 2)
        state["max_temp"] = round(max(state["max_temp"], temp), 2)
        state["min_rssi"] = min(state["min_rssi"], rssi)
        state["last_update"] = time.time()
        state["serial_ok"] = True
        state["last_raw"] = line.strip()
    return True


def serial_reader():
    start_time = time.time()
    while True:
        try:
            with lock:
                state["port"] = SERIAL_PORT
                state["status"] = "CONNECTING"
                state["phase"] = "Opening serial"
                state["serial_ok"] = False

            ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=SERIAL_TIMEOUT)
            time.sleep(2)

            with lock:
                state["status"] = "CONNECTED"
                state["phase"] = "Waiting packets"
                state["serial_ok"] = True

            while True:
                raw = ser.readline()
                if not raw:
                    with lock:
                        if state["last_update"] and (time.time() - state["last_update"] > 3):
                            state["status"] = "TIMEOUT"
                            state["phase"] = "No recent packet"
                    continue

                try:
                    line = raw.decode('utf-8', errors='ignore').strip()
                except Exception:
                    line = ''

                if line:
                    update_from_line(line, start_time)

        except serial.SerialException:
            with lock:
                state["status"] = "SERIAL ERROR"
                state["phase"] = "Check COM port"
                state["serial_ok"] = False
            time.sleep(2)
        except Exception:
            with lock:
                state["status"] = "ERROR"
                state["phase"] = "Reader exception"
                state["serial_ok"] = False
            time.sleep(2)


threading.Thread(target=serial_reader, daemon=True).start()

BASE_HTML = """
<!DOCTYPE html>
<html lang="en" data-theme="dark">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>{{ title }}</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700;800&display=swap" rel="stylesheet">
  {% if plotly %}<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>{% endif %}
  {% if three %}<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>{% endif %}
  <style>
    :root, [data-theme='light'] {
      --bg: #f7f6f2;
      --surface: #fcfbf8;
      --surface-2: #f1efe9;
      --border: rgba(40,37,29,0.10);
      --text: #1f2937;
      --muted: #6b7280;
      --primary: #0f766e;
      --primary-2: #14b8a6;
      --danger: #e11d48;
      --success: #16a34a;
      --shadow: 0 10px 30px rgba(17,24,39,0.08);
    }
    [data-theme='dark'] {
      --bg: #0b1020;
      --surface: #121a2b;
      --surface-2: #192338;
      --border: rgba(255,255,255,0.08);
      --text: #e5eefc;
      --muted: #94a3b8;
      --primary: #2dd4bf;
      --primary-2: #22c55e;
      --danger: #fb7185;
      --success: #34d399;
      --shadow: 0 14px 40px rgba(0,0,0,0.34);
    }
    * { box-sizing: border-box; }
    html, body { margin: 0; padding: 0; font-family: 'Inter', sans-serif; background: var(--bg); color: var(--text); }
    body { min-height: 100vh; }
    a { color: inherit; text-decoration: none; }
    button { font: inherit; }
    .shell { min-height: 100vh; display: grid; grid-template-rows: auto 1fr; }
    .topbar {
      position: sticky; top: 0; z-index: 50;
      display: flex; align-items: center; justify-content: space-between;
      padding: 14px 20px; background: color-mix(in srgb, var(--surface) 88%, transparent);
      backdrop-filter: blur(12px); border-bottom: 1px solid var(--border);
    }
    .brand { display: flex; align-items: center; gap: 12px; font-weight: 700; letter-spacing: 0.02em; }
    .logo {
      width: 34px; height: 34px; border-radius: 10px; display: grid; place-items: center;
      background: linear-gradient(135deg, var(--primary), var(--primary-2)); color: white; box-shadow: var(--shadow);
      font-size: 18px;
    }
    .nav { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
    .nav a, .theme-toggle {
      padding: 10px 14px; border: 1px solid var(--border); border-radius: 12px; color: var(--muted);
      background: var(--surface);
    }
    .nav a.active { color: var(--text); border-color: color-mix(in srgb, var(--primary) 35%, var(--border)); }
    .theme-toggle { cursor: pointer; }
    .page { padding: 20px; max-width: 1440px; width: 100%; margin: 0 auto; }
    .hero { display: flex; justify-content: space-between; gap: 16px; align-items: center; margin-bottom: 18px; flex-wrap: wrap; }
    .hero h1 { margin: 0; font-size: clamp(1.5rem, 2vw, 2rem); }
    .hero p { margin: 6px 0 0; color: var(--muted); }
    .statusbar { display: flex; gap: 10px; flex-wrap: wrap; }
    .pill {
      padding: 10px 12px; border-radius: 999px; background: var(--surface); border: 1px solid var(--border);
      font-size: 0.92rem; color: var(--muted);
    }
    .pill strong { color: var(--text); }
    .grid { display: grid; gap: 16px; }
    .metrics { grid-template-columns: repeat(5, minmax(0, 1fr)); margin-bottom: 16px; }
    .main-grid { grid-template-columns: 1.15fr 0.85fr; align-items: stretch; }
    .charts-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); margin-top: 16px; }
    .card {
      background: linear-gradient(180deg, var(--surface), color-mix(in srgb, var(--surface) 76%, var(--surface-2)));
      border: 1px solid var(--border); border-radius: 20px; box-shadow: var(--shadow);
      overflow: hidden;
    }
    .card-head { display: flex; align-items: center; justify-content: space-between; gap: 8px; padding: 16px 18px 0; }
    .card-title { margin: 0; font-size: 1rem; }
    .card-sub { margin: 4px 0 0; color: var(--muted); font-size: 0.9rem; }
    .card-body { padding: 18px; }
    .metric-card { padding: 18px; }
    .metric-label { color: var(--muted); font-size: 0.88rem; margin-bottom: 8px; }
    .metric-value { font-size: clamp(1.6rem, 3vw, 2.1rem); font-weight: 800; letter-spacing: -0.03em; }
    .metric-foot { margin-top: 8px; color: var(--muted); font-size: 0.9rem; }
    .telemetry-list { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 12px; }
    .telemetry-item { padding: 14px; border-radius: 14px; background: var(--surface-2); border: 1px solid var(--border); }
    .telemetry-item span { display: block; color: var(--muted); font-size: 0.84rem; margin-bottom: 6px; }
    .telemetry-item strong { font-size: 1rem; }
    .rawbox { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; white-space: pre-wrap; word-break: break-word; font-size: 0.82rem; }
    #threeCanvas { width: 100%; aspect-ratio: 1 / 1; min-height: 360px; }
    .chart { height: 300px; }
    .map-box {
      height: 220px; border-radius: 16px; background:
        radial-gradient(circle at 30% 40%, rgba(45,212,191,0.22), transparent 26%),
        radial-gradient(circle at 68% 60%, rgba(34,197,94,0.18), transparent 24%),
        linear-gradient(180deg, color-mix(in srgb, var(--surface-2) 88%, transparent), var(--surface));
      border: 1px solid var(--border); position: relative; overflow: hidden;
    }
    .map-grid {
      position: absolute; inset: 0;
      background-image:
        linear-gradient(to right, rgba(148,163,184,0.10) 1px, transparent 1px),
        linear-gradient(to bottom, rgba(148,163,184,0.10) 1px, transparent 1px);
      background-size: 28px 28px;
    }
    .map-dot {
      position: absolute; width: 14px; height: 14px; border-radius: 999px; background: var(--danger);
      box-shadow: 0 0 0 8px rgba(251,113,133,0.16);
      transform: translate(-50%, -50%);
    }
    .footer-note { margin-top: 16px; color: var(--muted); font-size: 0.88rem; }
    @media (max-width: 1180px) {
      .metrics { grid-template-columns: repeat(3, minmax(0, 1fr)); }
      .main-grid, .charts-grid { grid-template-columns: 1fr; }
    }
    @media (max-width: 720px) {
      .page { padding: 14px; }
      .metrics { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .telemetry-list { grid-template-columns: 1fr; }
      .topbar { padding: 12px 14px; }
      .nav { width: 100%; justify-content: flex-start; }
    }
  </style>
</head>
<body>
<div class="shell">
  <header class="topbar">
    <div class="brand">
      <div class="logo">⦿</div>
      <div>CanSat Mission Control</div>
    </div>
    <nav class="nav">
      <a href="/" class="{% if active == 'dashboard' %}active{% endif %}">Dashboard</a>
      <a href="/graphs" class="{% if active == 'graphs' %}active{% endif %}">Graphs</a>
      <button class="theme-toggle" id="themeToggle" aria-label="Toggle theme">Theme</button>
    </nav>
  </header>
  <main class="page">
    {{ body|safe }}
  </main>
</div>
<script>
  (function(){
    const root = document.documentElement;
    let mode = window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
    root.setAttribute('data-theme', mode);
    const btn = document.getElementById('themeToggle');
    if (btn) btn.addEventListener('click', () => {
      mode = mode === 'dark' ? 'light' : 'dark';
      root.setAttribute('data-theme', mode);
    });
  })();
  const fmt = {
    num(v, d=2){ return Number.isFinite(v) ? Number(v).toFixed(d) : '--'; },
    signed(v, d=2){ return Number.isFinite(v) ? `${v >= 0 ? '+' : ''}${Number(v).toFixed(d)}` : '--'; },
    last(arr){ return Array.isArray(arr) && arr.length ? arr[arr.length - 1] : null; }
  };
</script>
{{ script|safe }}
</body>
</html>
"""


@app.route('/data')
def get_data():
    return jsonify(to_list_payload())


@app.route('/ports')
def ports():
    return jsonify({"ports": available_ports(), "selected": SERIAL_PORT})


@app.route('/')
def index():
    body = """
    <section class="hero">
      <div>
        <h1>Flight telemetry dashboard</h1>
        <p>Real-time serial telemetry from STM32 USB CDC into a live web dashboard.</p>
      </div>
      <div class="statusbar">
        <div class="pill">Mode: <strong>Live Serial</strong></div>
        <div class="pill">Port: <strong id="port">--</strong></div>
        <div class="pill">Packet: <strong id="packet">--</strong></div>
        <div class="pill">Phase: <strong id="phase">--</strong></div>
        <div class="pill">Status: <strong id="status">--</strong></div>
      </div>
    </section>

    <section class="grid metrics">
      <article class="card metric-card"><div class="metric-label">Altitude</div><div class="metric-value" id="alt">--</div><div class="metric-foot">Apogee <span id="apogee">--</span></div></article>
      <article class="card metric-card"><div class="metric-label">Velocity</div><div class="metric-value" id="vel">--</div><div class="metric-foot">Latest vertical speed</div></article>
      <article class="card metric-card"><div class="metric-label">Temperature</div><div class="metric-value" id="temp">--</div><div class="metric-foot">Max <span id="max_temp">--</span></div></article>
      <article class="card metric-card"><div class="metric-label">Signal RSSI</div><div class="metric-value" id="rssi">--</div><div class="metric-foot">Min <span id="min_rssi">--</span></div></article>
      <article class="card metric-card"><div class="metric-label">Pressure</div><div class="metric-value" id="pressure">--</div><div class="metric-foot">Barometric reading</div></article>
    </section>

    <section class="grid main-grid">
      <article class="card">
        <div class="card-head">
          <div>
            <h2 class="card-title">3D attitude</h2>
            <p class="card-sub">Live gyroscope-driven orientation</p>
          </div>
        </div>
        <div class="card-body"><div id="threeCanvas"></div></div>
      </article>

      <article class="card">
        <div class="card-head">
          <div>
            <h2 class="card-title">Live telemetry</h2>
            <p class="card-sub">GPS, IMU, link, and raw packet monitor</p>
          </div>
        </div>
        <div class="card-body">
          <div class="telemetry-list">
            <div class="telemetry-item"><span>Latitude</span><strong id="lat">--</strong></div>
            <div class="telemetry-item"><span>Longitude</span><strong id="lon">--</strong></div>
            <div class="telemetry-item"><span>Gyro X / Y / Z</span><strong id="gyro">--</strong></div>
            <div class="telemetry-item"><span>Accel X / Y / Z</span><strong id="accel">--</strong></div>
            <div class="telemetry-item"><span>GPS Sat / Fix</span><strong id="gps_meta">--</strong></div>
            <div class="telemetry-item"><span>Updated</span><strong id="updated">--</strong></div>
          </div>
          <div style="height:14px"></div>
          <div class="map-box">
            <div class="map-grid"></div>
            <div class="map-dot" id="mapDot" style="left:50%;top:50%"></div>
          </div>
          <div style="height:14px"></div>
          <div class="telemetry-item rawbox"><span>Last raw line</span><strong id="last_raw">--</strong></div>
        </div>
      </article>
    </section>

    <p class="footer-note">Set the correct COM port in the Python file before running. Open /ports to list detected serial ports.</p>
    """

    script = """
    <script>
      let scene, camera, renderer, bodyMesh;
      function init3D(){
        const mount = document.getElementById('threeCanvas');
        const w = mount.clientWidth || 420;
        const h = mount.clientHeight || 420;
        scene = new THREE.Scene();
        camera = new THREE.PerspectiveCamera(48, w / h, 0.1, 1000);
        renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        renderer.setSize(w, h);
        mount.appendChild(renderer.domElement);

        scene.add(new THREE.AmbientLight(0xffffff, 0.9));
        const keyLight = new THREE.DirectionalLight(0xffffff, 1.2);
        keyLight.position.set(5, 6, 7);
        scene.add(keyLight);

        const bodyGeo = new THREE.CylinderGeometry(0.7, 0.7, 2.8, 48);
        const bodyMat = new THREE.MeshStandardMaterial({ color: 0x2dd4bf, metalness: 0.55, roughness: 0.32 });
        bodyMesh = new THREE.Mesh(bodyGeo, bodyMat);
        scene.add(bodyMesh);

        const nose = new THREE.Mesh(
          new THREE.ConeGeometry(0.72, 0.9, 48),
          new THREE.MeshStandardMaterial({ color: 0xe5eefc, metalness: 0.2, roughness: 0.45 })
        );
        nose.position.y = 1.85;
        bodyMesh.add(nose);

        const finMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.25, roughness: 0.5 });
        for (let i = 0; i < 4; i++) {
          const fin = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.7, 0.65), finMat);
          const angle = i * Math.PI / 2;
          fin.position.set(Math.cos(angle) * 0.72, -1.1, Math.sin(angle) * 0.72);
          fin.rotation.y = angle;
          bodyMesh.add(fin);
        }

        camera.position.set(0, 0.6, 6.2);
        function animate(){
          requestAnimationFrame(animate);
          renderer.render(scene, camera);
        }
        animate();
        window.addEventListener('resize', () => {
          const nw = mount.clientWidth || 420;
          const nh = mount.clientHeight || 420;
          camera.aspect = nw / nh;
          camera.updateProjectionMatrix();
          renderer.setSize(nw, nh);
        });
      }

      function setText(id, value){
        const el = document.getElementById(id);
        if (el) el.textContent = value;
      }

      function updateMap(lat, lon){
        const dot = document.getElementById('mapDot');
        const x = 50 + ((lon - 73.8567) * 7000);
        const y = 50 - ((lat - 18.5204) * 7000);
        dot.style.left = Math.max(8, Math.min(92, x)) + '%';
        dot.style.top = Math.max(8, Math.min(92, y)) + '%';
      }

      async function poll(){
        try {
          const res = await fetch('/data');
          const d = await res.json();
          const alt = fmt.last(d.alt), vel = fmt.last(d.vel), temp = fmt.last(d.temp), pressure = fmt.last(d.pressure);

          setText('port', d.port || '--');
          setText('packet', d.packet);
          setText('phase', d.phase);
          setText('status', d.status);
          setText('alt', `${fmt.num(alt)} m`);
          setText('vel', `${fmt.signed(vel)} m/s`);
          setText('temp', `${fmt.num(temp)} °C`);
          setText('rssi', `${d.rssi} dBm`);
          setText('pressure', `${fmt.num(pressure)} hPa`);
          setText('apogee', `${fmt.num(d.apogee)} m`);
          setText('max_temp', `${fmt.num(d.max_temp)} °C`);
          setText('min_rssi', `${d.min_rssi} dBm`);
          setText('lat', fmt.num(d.lat, 6));
          setText('lon', fmt.num(d.lon, 6));
          setText('gyro', `${fmt.signed(d.gx)} / ${fmt.signed(d.gy)} / ${fmt.signed(d.gz)} deg/s`);
          setText('accel', `${fmt.signed(d.ax,2)} / ${fmt.signed(d.ay,2)} / ${fmt.num(d.az,2)}`);
          setText('gps_meta', `${d.sat} sats / fix ${d.fix}`);
          setText('updated', d.last_update ? new Date(d.last_update * 1000).toLocaleTimeString() : '--');
          setText('last_raw', d.last_raw || '--');
          updateMap(d.lat, d.lon);

          if (bodyMesh) {
            bodyMesh.rotation.x = d.gx * 0.012;
            bodyMesh.rotation.y = d.gy * 0.012;
            bodyMesh.rotation.z = d.gz * 0.012;
          }
        } catch (e) {
          setText('status', 'FETCH ERROR');
        }
      }

      init3D();
      poll();
      setInterval(poll, 400);
    </script>
    """
    return render_template_string(BASE_HTML, title="Mission Control Dashboard", active="dashboard", body=body, script=script, plotly=False, three=True)


@app.route('/graphs')
def graphs():
    body = """
    <section class="hero">
      <div>
        <h1>Telemetry plots</h1>
        <p>Rolling history from live serial packets.</p>
      </div>
      <div class="statusbar">
        <div class="pill">Window: <strong>Last 120 samples</strong></div>
        <div class="pill">Source: <strong>STM32 USB CDC</strong></div>
      </div>
    </section>

    <section class="grid charts-grid">
      <article class="card"><div class="card-head"><div><h2 class="card-title">Altitude</h2><p class="card-sub">Meters</p></div></div><div class="card-body"><div id="altGraph" class="chart"></div></div></article>
      <article class="card"><div class="card-head"><div><h2 class="card-title">Velocity</h2><p class="card-sub">m/s</p></div></div><div class="card-body"><div id="velGraph" class="chart"></div></div></article>
      <article class="card"><div class="card-head"><div><h2 class="card-title">Temperature</h2><p class="card-sub">Degrees C</p></div></div><div class="card-body"><div id="tempGraph" class="chart"></div></div></article>
      <article class="card"><div class="card-head"><div><h2 class="card-title">Pressure</h2><p class="card-sub">hPa</p></div></div><div class="card-body"><div id="pressureGraph" class="chart"></div></div></article>
      <article class="card" style="grid-column: 1 / -1;"><div class="card-head"><div><h2 class="card-title">Acceleration</h2><p class="card-sub">Ax, Ay, Az</p></div></div><div class="card-body"><div id="accGraph" class="chart" style="height:340px"></div></div></article>
    </section>
    """

    script = """
    <script>
      function plotLayout(title, ytitle){
        const dark = document.documentElement.getAttribute('data-theme') === 'dark';
        return {
          title: { text: title, font: { size: 16 } },
          paper_bgcolor: 'transparent',
          plot_bgcolor: 'transparent',
          margin: { l: 48, r: 18, t: 42, b: 40 },
          xaxis: {
            title: 'Time (s)',
            gridcolor: dark ? 'rgba(255,255,255,0.08)' : 'rgba(0,0,0,0.08)',
            zerolinecolor: dark ? 'rgba(255,255,255,0.10)' : 'rgba(0,0,0,0.10)'
          },
          yaxis: {
            title: ytitle,
            gridcolor: dark ? 'rgba(255,255,255,0.08)' : 'rgba(0,0,0,0.08)',
            zerolinecolor: dark ? 'rgba(255,255,255,0.10)' : 'rgba(0,0,0,0.10)'
          },
          font: {
            family: 'Inter, sans-serif',
            color: getComputedStyle(document.documentElement).getPropertyValue('--text').trim()
          },
          legend: { orientation: 'h' }
        };
      }

      function lineTrace(x, y, name, color){
        return {
          x, y, name, type: 'scatter', mode: 'lines',
          line: { width: 3, color },
          hovertemplate: '%{y}<extra>' + name + '</extra>'
        };
      }

      function draw(d){
        Plotly.react('altGraph', [lineTrace(d.time, d.alt, 'Altitude', '#2dd4bf')], plotLayout('Altitude', 'm'), {displayModeBar:false, responsive:true});
        Plotly.react('velGraph', [lineTrace(d.time, d.vel, 'Velocity', '#38bdf8')], plotLayout('Velocity', 'm/s'), {displayModeBar:false, responsive:true});
        Plotly.react('tempGraph', [lineTrace(d.time, d.temp, 'Temperature', '#fb923c')], plotLayout('Temperature', '°C'), {displayModeBar:false, responsive:true});
        Plotly.react('pressureGraph', [lineTrace(d.time, d.pressure, 'Pressure', '#a78bfa')], plotLayout('Pressure', 'hPa'), {displayModeBar:false, responsive:true});
        Plotly.react('accGraph', [
          lineTrace(d.time, d.ax_hist, 'Ax', '#22c55e'),
          lineTrace(d.time, d.ay_hist, 'Ay', '#f59e0b'),
          lineTrace(d.time, d.az_hist, 'Az', '#f43f5e')
        ], plotLayout('Acceleration', 'm/s² or scaled unit'), {displayModeBar:false, responsive:true});
      }

      async function refresh(){
        try {
          const res = await fetch('/data');
          const d = await res.json();
          draw(d);
        } catch (e) {}
      }
      refresh();
      setInterval(refresh, 400);
      document.getElementById('themeToggle')?.addEventListener('click', () => setTimeout(refresh, 50));
    </script>
    """
    return render_template_string(BASE_HTML, title="Mission Control Graphs", active="graphs", body=body, script=script, plotly=True, three=False)


if __name__ == '__main__':
    print('Available serial ports:', available_ports())
    print(f'Opening serial port: {SERIAL_PORT} @ {SERIAL_BAUD}')
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)