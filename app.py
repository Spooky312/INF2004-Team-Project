from flask import Flask, render_template_string, jsonify
from flask_socketio import SocketIO
import json
from collections import deque
import paho.mqtt.client as mqtt
import os

# Broker config can be overridden with environment variables MQTT_BROKER / MQTT_PORT
# Default keeps localhost for simple local tests, but set MQTT_BROKER=192.168.1.64 if
# your broker is bound to the LAN address (common on Windows when mosquitto is started
# with a 0.0.0.0 listener).
BROKER_HOST = os.environ.get("MQTT_BROKER", "127.0.0.1")
BROKER_PORT = int(os.environ.get("MQTT_PORT", 1883))
TOPIC       = os.environ.get("MQTT_TOPIC", "pico/telemetry")

app = Flask(__name__)
socketio = SocketIO(app, async_mode="threading", cors_allowed_origins="*")
history = deque(maxlen=300)

HTML = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>Pico MQTT Dashboard – Live Telemetry</title>
  <script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
  <style>
    body { font-family: system-ui, Arial, sans-serif; margin: 32px; background:#f8fafc; color:#111827; }
    h1 { margin-bottom: 8px; }
    .grid { display: grid; grid-template-columns: repeat(3, minmax(220px, 1fr)); gap: 12px; margin-top: 16px;}
    .card { background: white; border-radius: 14px; padding: 16px; box-shadow: 0 6px 20px rgba(0,0,0,.08); }
    .label { font-size: 12px; color:#6b7280; text-transform: uppercase; letter-spacing: .08em; }
    .value { font-size: 28px; font-weight: 700; margin-top: 4px; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, "Liberation Mono", monospace; }
    footer { margin-top: 24px; color:#6b7280; font-size: 12px; }
    #logs { margin-top: 16px; background: white; border-radius: 12px; padding: 16px; box-shadow: 0 4px 12px rgba(0,0,0,.05); max-height: 260px; overflow-y: auto; }
    #logs pre { font-size: 13px; margin: 0; white-space: pre-wrap; }
  </style>
</head>
<body>
  <h1>Pico Robot – Live Telemetry</h1>

  <div class="grid">
    <div class="card"><div class="label">RPM Left</div><div class="value" id="rpm_l">–</div></div>
    <div class="card"><div class="label">RPM Right</div><div class="value" id="rpm_r">–</div></div>
    <div class="card"><div class="label">Speed (m/s)</div><div class="value" id="speed">–</div></div>
    <div class="card"><div class="label">Distance (m)</div><div class="value" id="dist_m">–</div></div>

    <div class="card"><div class="label">Heading Raw (°)</div><div class="value" id="heading_raw">–</div></div>
    <div class="card"><div class="label">Heading Filtered (°)</div><div class="value" id="heading_filt">–</div></div>
    <div class="card"><div class="label">Target Heading (°)</div><div class="value" id="target_heading">–</div></div>

    <div class="card"><div class="label">Ticks Left</div><div class="value" id="ticks_l">–</div></div>
    <div class="card"><div class="label">Ticks Right</div><div class="value" id="ticks_r">–</div></div>
    <div class="card"><div class="label">Obstacle distance (cm)</div><div class="value" id="obstacle_distance_cm">–</div></div>
    <div class="card"><div class="label">Obstacle Width (cm)</div><div class="value" id="obstacle_width_cm">–</div></div>
    <div class="card"><div class="label">Obstacle Count</div><div class="value" id="obstacle_count">–</div></div>
    <div class="card"><div class="label">Line Event</div><div class="value" id="line_event">–</div></div>
    <div class="card"><div class="label">Barcode Command</div><div class="value" id="barcode_command">–</div></div>
    <div class="card"><div class="label">Current State</div><div class="value" id="current_state">–</div></div>
  </div>

  <div id="logs">
    <div class="label">Logs / Last Payload</div>
    <pre class="mono" id="last_json">(waiting…)</pre>
    <hr style="border:none; border-top:1px solid #e5e7eb; margin:12px 0;">
    <pre class="mono" id="log_feed"></pre>
  </div>

  <footer>Topic: <span class="mono">pico/telemetry</span></footer>

<script>
  const el = id => document.getElementById(id);
  const s = io();
  const MAX_LINES = 200;

  function appendLog(dataObj) {
    const feed = el('log_feed');
    const lines = (feed.textContent ? feed.textContent.split('\\n') : []);

    const now = new Date();
    const timestamp = now.toLocaleString('en-SG', {
      year: 'numeric', month: '2-digit', day: '2-digit',
      hour: '2-digit', minute: '2-digit', second: '2-digit',
      hour12: false
    });

    const formatted = `[${timestamp}] ${JSON.stringify(dataObj)}`;
    lines.push(formatted);
    while (lines.length > MAX_LINES) lines.shift();
    feed.textContent = lines.join('\\n');
    feed.scrollTop = feed.scrollHeight;
  }

  function renderTelemetry(d) {
    try {
      const dist_val = ('dist_m' in d) ? d.dist_m : (('dist' in d) ? d.dist : undefined);
      if ('rpm_l' in d)              el('rpm_l').textContent              = Number(d.rpm_l).toFixed(2);
      if ('rpm_r' in d)              el('rpm_r').textContent              = Number(d.rpm_r).toFixed(2);
      if ('speed' in d)              el('speed').textContent              = Number(d.speed).toFixed(3);
      if (dist_val !== undefined)    el('dist_m').textContent             = Number(dist_val).toFixed(3);
      if ('heading_raw' in d)        el('heading_raw').textContent        = Number(d.heading_raw).toFixed(2);
      if ('heading_filt' in d)       el('heading_filt').textContent       = Number(d.heading_filt).toFixed(2);
      if ('target_heading' in d)     el('target_heading').textContent     = Number(d.target_heading).toFixed(2);
      if ('ticks_l' in d)            el('ticks_l').textContent            = d.ticks_l;
      if ('ticks_r' in d)            el('ticks_r').textContent            = d.ticks_r;

      // 🆕 Handle obstacle distance
      if ('obstacle_distance_cm' in d) el('obstacle_distance_cm').textContent = Number(d.obstacle_distance_cm).toFixed(2);
      if ('obstacle_width_cm' in d)    el('obstacle_width_cm').textContent    = Number(d.obstacle_width_cm).toFixed(2);
      if ('obstacle_count' in d)       el('obstacle_count').textContent       = d.obstacle_count;
      if ('line_event' in d)           el('line_event').textContent           = d.line_event;
      if ('barcode_command' in d)      el('barcode_command').textContent      = d.barcode_command;
      if ('current_state' in d)        el('current_state').textContent        = d.current_state;

      const pretty = JSON.stringify(d, null, 2);
      el('last_json').textContent = pretty;
      appendLog(d);
    } catch (e) {
      console.error("Error updating telemetry:", e);
    }
  }

  s.on('telemetry', (d) => {
    renderTelemetry(d);
  });
</script>

</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML)

def start_mqtt():
    client = mqtt.Client()
    # track connection state for /status (best-effort; used for debug)
    mqtt_state = {"connected": False}

    def on_connect(c, userdata, flags, rc):
        if rc == 0:
            print("✅ MQTT connected (rc=0)")
            mqtt_state["connected"] = True
            c.subscribe(TOPIC)
        else:
            print(f"⚠️ MQTT connect returned rc={rc}")

    def on_message(c, userdata, msg):
        try:
            payload = msg.payload.decode("utf-8", "ignore")
            print("📡 Received:", payload)
            data = json.loads(payload)

            # 🆕 Normalize obstacle distance
            # Some firmware might send 'dist' or 'front_distance' instead of obstacle_distance_cm
            if "obstacle_distance_cm" not in data:
            # Accept alternative names if your firmware ever uses them
                if "front_distance_cm" in data:
                    data["obstacle_distance_cm"] = float(data["front_distance_cm"])
                elif "front_distance_m" in data:
                    data["obstacle_distance_cm"] = float(data["front_distance_m"]) * 100.0
                else:
                    data["obstacle_distance_cm"] = None  # unknown / not provided

            history.append(data)
            socketio.emit("telemetry", data)
        except Exception as e:
            print("❌ Bad payload:", e)

    client.on_connect = on_connect
    client.on_message = on_message
    try:
        print(f"[MQTT] Connecting to broker {BROKER_HOST}:{BROKER_PORT} ...")
        client.connect(BROKER_HOST, BROKER_PORT, 60)
        client.loop_start()
    except Exception as e:
        print("⚠️ Broker not found or not running:", e)
    return client


@app.route("/status")
def status():
    # lightweight health page: is the web server up and (best-effort) mqtt reachable?
    info = {"mqtt_broker": BROKER_HOST, "mqtt_port": BROKER_PORT}
    # not an authoritative connected flag (paho client state is inside start_mqtt),
    # but presence of the broker host/port is useful for debugging from browser.
    return jsonify(info)

if __name__ == "__main__":
    mqtt_client = start_mqtt()
    socketio.run(app, host="0.0.0.0", port=5000)
