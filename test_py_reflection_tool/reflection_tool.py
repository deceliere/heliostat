#!/usr/bin/env python3
"""
Local reflection visualization tool.

This script starts a tiny local HTTP server and serves a self-contained HTML/JS
tool for exploring:
  - solar azimuth / altitude from GPS + local date/time
  - mirror normal orientation
  - reflected ray direction

No third-party Python dependencies are required.
"""

from __future__ import annotations

import argparse
import contextlib
import http.server
import socket
import socketserver
import sys
import threading
import webbrowser


HTML_PAGE = """<!doctype html>
<html lang="fr">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Reflection Tool</title>
  <style>
    :root {
      --bg: #f5f7fb;
      --panel: #ffffff;
      --line: #d8dee9;
      --text: #0f172a;
      --muted: #64748b;
      --accent: #2563eb;
      --sun: #f59e0b;
      --dark: #111827;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: ui-sans-serif, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      color: var(--text);
      background: linear-gradient(135deg, #eef2ff 0%, var(--bg) 45%, #ffffff 100%);
    }
    .page {
      max-width: 1400px;
      margin: 0 auto;
      padding: 24px;
    }
    .hero {
      margin-bottom: 20px;
    }
    h1 {
      margin: 0 0 8px;
      font-size: 32px;
      line-height: 1.1;
    }
    .sub {
      max-width: 900px;
      color: var(--muted);
      line-height: 1.5;
    }
    .layout {
      display: grid;
      grid-template-columns: 360px 1fr;
      gap: 20px;
      align-items: start;
    }
    .card {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 20px;
      box-shadow: 0 12px 32px rgba(15, 23, 42, 0.06);
      overflow: hidden;
    }
    .card-header {
      padding: 18px 20px 0;
      font-size: 20px;
      font-weight: 700;
    }
    .card-body {
      padding: 20px;
    }
    .grid {
      display: grid;
      gap: 14px;
    }
    .grid.two {
      grid-template-columns: repeat(2, minmax(0, 1fr));
    }
    .fields {
      display: grid;
      gap: 16px;
    }
    label {
      display: grid;
      gap: 8px;
      font-size: 14px;
      font-weight: 600;
    }
    input[type="number"],
    input[type="date"],
    input[type="time"] {
      width: 100%;
      padding: 10px 12px;
      border: 1px solid var(--line);
      border-radius: 12px;
      font: inherit;
      background: #fff;
      color: var(--text);
    }
    input[type="range"] {
      width: 100%;
    }
    .hint {
      margin-top: -2px;
      color: var(--muted);
      font-size: 12px;
      line-height: 1.4;
    }
    .stat-grid {
      display: grid;
      gap: 14px;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      margin-bottom: 14px;
    }
    .stat {
      border: 1px solid var(--line);
      background: rgba(255,255,255,0.85);
      border-radius: 16px;
      padding: 14px;
      min-height: 82px;
    }
    .stat .k {
      color: var(--muted);
      font-size: 12px;
      margin-bottom: 6px;
    }
    .stat .v {
      font-size: 24px;
      font-weight: 700;
      letter-spacing: -0.03em;
    }
    .views {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 18px;
      margin-top: 18px;
    }
    .vector-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 14px;
      margin-top: 18px;
    }
    .vector-box {
      border: 1px solid var(--line);
      border-radius: 16px;
      padding: 14px;
      background: rgba(255,255,255,0.85);
    }
    .vector-title {
      font-size: 13px;
      font-weight: 700;
      margin-bottom: 8px;
    }
    .vector-title.sun { color: #b45309; }
    .vector-title.normal { color: #0f172a; }
    .vector-title.refl { color: #1d4ed8; }
    .mono {
      font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
      font-size: 13px;
      color: #334155;
      line-height: 1.45;
      word-break: break-all;
    }
    .note {
      margin-top: 18px;
      padding: 14px 16px;
      border-radius: 16px;
      border: 1px solid #fcd34d;
      background: #fffbeb;
      color: #92400e;
      line-height: 1.5;
      font-size: 14px;
    }
    .badge-row {
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      margin-top: 10px;
    }
    .badge {
      padding: 6px 10px;
      border-radius: 999px;
      background: #e2e8f0;
      color: #334155;
      font-size: 12px;
      font-weight: 600;
    }
    svg {
      width: 100%;
      height: auto;
      border: 1px solid var(--line);
      border-radius: 16px;
      background: #fff;
    }
    @media (max-width: 1100px) {
      .layout { grid-template-columns: 1fr; }
      .stat-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .views { grid-template-columns: 1fr; }
      .vector-grid { grid-template-columns: 1fr; }
    }
    @media (max-width: 680px) {
      .page { padding: 16px; }
      .grid.two, .stat-grid { grid-template-columns: 1fr; }
      h1 { font-size: 28px; }
    }
  </style>
</head>
<body>
  <div class="page">
    <div class="hero">
      <h1>Réflexion d’un rayon solaire sur un miroir</h1>
      <div class="sub">
        Entre un lieu GPS, une date/heure locale et l’orientation du miroir. L’outil calcule la position du soleil,
        la normale au miroir et la direction du rayon réfléchi. Il est volontairement autonome: une seule page servie localement.
      </div>
      <div class="badge-row">
        <div class="badge">Approximation solaire type NOAA</div>
        <div class="badge">Repère: azimut depuis le nord</div>
        <div class="badge">Lancé via Python</div>
      </div>
    </div>

    <div class="layout">
      <section class="card">
        <div class="card-header">Paramètres</div>
        <div class="card-body fields">
          <div class="grid two">
            <label>
              Latitude
              <input id="lat" type="number" step="0.0001" value="46.2044">
            </label>
            <label>
              Longitude
              <input id="lon" type="number" step="0.0001" value="6.1432">
            </label>
          </div>

          <div class="grid two">
            <label>
              Date locale
              <input id="date" type="date">
            </label>
            <label>
              Heure locale
              <input id="time" type="time">
            </label>
          </div>

          <label>
            Décalage UTC (heures)
            <input id="utcOffset" type="number" step="1" value="2">
            <div class="hint">Suisse: généralement 1 en hiver, 2 en été.</div>
          </label>

          <label>
            Azimut du miroir (direction horizontale de la normale)
            <input id="mirrorAz" type="range" min="0" max="360" step="1" value="180">
            <div class="hint"><span id="mirrorAzValue">180</span>°. 0° = nord, 90° = est, 180° = sud, 270° = ouest.</div>
          </label>

          <label>
            Inclinaison du miroir
            <input id="mirrorTilt" type="range" min="-89" max="89" step="1" value="45">
            <div class="hint"><span id="mirrorTiltValue">45</span>°. 0° = miroir vertical, 90° ≈ miroir horizontal face au ciel. Cette valeur correspond à l’altitude de la normale du miroir.</div>
          </label>
        </div>
      </section>

      <section>
        <div class="stat-grid">
          <div class="stat"><div class="k">Azimut du soleil</div><div class="v" id="sunAz">—</div></div>
          <div class="stat"><div class="k">Altitude du soleil</div><div class="v" id="sunAlt">—</div></div>
          <div class="stat"><div class="k">Azimut du rayon réfléchi</div><div class="v" id="reflAz">—</div></div>
          <div class="stat"><div class="k">Altitude du rayon réfléchi</div><div class="v" id="reflAlt">—</div></div>
        </div>

        <div class="stat-grid">
          <div class="stat"><div class="k">Azimut de la normale</div><div class="v" id="normalAz">—</div></div>
          <div class="stat"><div class="k">Altitude de la normale</div><div class="v" id="normalAlt">—</div></div>
          <div class="stat"><div class="k">Angle incidence/réflexion</div><div class="v" id="incidenceAngle">—</div></div>
          <div class="stat"><div class="k">Soleil au-dessus de l’horizon</div><div class="v" id="sunVisible">—</div></div>
        </div>

        <div class="views">
          <section class="card">
            <div class="card-header">Vue polaire</div>
            <div class="card-body"><div id="polarView"></div></div>
          </section>

          <section class="card">
            <div class="card-header">Coupe verticale</div>
            <div class="card-body"><div id="sideView"></div></div>
          </section>
        </div>

        <section class="card" style="margin-top: 18px;">
          <div class="card-header">Vecteurs utiles</div>
          <div class="card-body">
            <div class="vector-grid">
              <div class="vector-box">
                <div class="vector-title sun">Rayon incident</div>
                <div class="mono" id="sunVec">—</div>
              </div>
              <div class="vector-box">
                <div class="vector-title normal">Normale du miroir</div>
                <div class="mono" id="normalVec">—</div>
              </div>
              <div class="vector-box">
                <div class="vector-title refl">Rayon réfléchi</div>
                <div class="mono" id="reflVec">—</div>
              </div>
            </div>

            <div class="note">
              <strong>Limites réelles :</strong> cet outil est utile pour le prototypage géométrique, mais il ne modélise pas la taille apparente du soleil,
              les imperfections du miroir, les ombres, ni la photométrie. Il faut donc le considérer comme un calculateur d’orientation.
            </div>
          </div>
        </section>
      </section>
    </div>
  </div>

  <script>
    function deg2rad(d) { return (d * Math.PI) / 180; }
    function rad2deg(r) { return (r * 180) / Math.PI; }
    function clamp(v, min, max) { return Math.max(min, Math.min(max, v)); }
    function normalize(v) {
      const m = Math.hypot(v[0], v[1], v[2]) || 1;
      return [v[0] / m, v[1] / m, v[2] / m];
    }
    function dot(a, b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
    function reflect(i, n) {
      const d = dot(i, n);
      return normalize([i[0] - 2*d*n[0], i[1] - 2*d*n[1], i[2] - 2*d*n[2]]);
    }
    function vecFromAzAlt(azDeg, altDeg) {
      const az = deg2rad(azDeg);
      const alt = deg2rad(altDeg);
      const x = Math.cos(alt) * Math.sin(az);
      const y = Math.cos(alt) * Math.cos(az);
      const z = Math.sin(alt);
      return normalize([x, y, z]);
    }
    function azAltFromVec(v) {
      const n = normalize(v);
      const alt = rad2deg(Math.asin(clamp(n[2], -1, 1)));
      let az = rad2deg(Math.atan2(n[0], n[1]));
      if (az < 0) az += 360;
      return { azimuth: az, altitude: alt };
    }
    function fmt(n, digits = 2) {
      return Number.isFinite(n) ? n.toFixed(digits) : "—";
    }
    function fmtVec(v) {
      return "[" + v.map((x) => fmt(x, 4)).join(", ") + "]";
    }
    function parseLocalToUTC(dateStr, timeStr, utcOffsetHours) {
      const [y, m, d] = dateStr.split("-").map(Number);
      const [hh, mm] = timeStr.split(":").map(Number);
      return new Date(Date.UTC(y, (m || 1) - 1, d || 1, (hh || 0) - utcOffsetHours, mm || 0, 0));
    }
    function solarPosition(date, latDeg, lonDeg) {
      const year = date.getUTCFullYear();
      const start = Date.UTC(year, 0, 0, 0, 0, 0);
      const now = date.getTime();
      const dayOfYear = (now - start) / 86400000;

      const hours = date.getUTCHours() + date.getUTCMinutes() / 60 + date.getUTCSeconds() / 3600;
      const gamma = (2 * Math.PI / 365) * (dayOfYear - 1 + (hours - 12) / 24);

      const eqtime = 229.18 * (
        0.000075 +
        0.001868 * Math.cos(gamma) -
        0.032077 * Math.sin(gamma) -
        0.014615 * Math.cos(2 * gamma) -
        0.040849 * Math.sin(2 * gamma)
      );

      const decl =
        0.006918 -
        0.399912 * Math.cos(gamma) +
        0.070257 * Math.sin(gamma) -
        0.006758 * Math.cos(2 * gamma) +
        0.000907 * Math.sin(2 * gamma) -
        0.002697 * Math.cos(3 * gamma) +
        0.00148 * Math.sin(3 * gamma);

      const minutesUTC = date.getUTCHours() * 60 + date.getUTCMinutes() + date.getUTCSeconds() / 60;
      const trueSolarTime = (minutesUTC + eqtime + 4 * lonDeg + 1440) % 1440;

      let hourAngleDeg = trueSolarTime / 4 - 180;
      if (hourAngleDeg < -180) hourAngleDeg += 360;

      const ha = deg2rad(hourAngleDeg);
      const lat = deg2rad(latDeg);
      const cosZenith = clamp(
        Math.sin(lat) * Math.sin(decl) + Math.cos(lat) * Math.cos(decl) * Math.cos(ha),
        -1, 1
      );
      const zenith = Math.acos(cosZenith);
      const altitude = 90 - rad2deg(zenith);

      const azRad = Math.atan2(
        Math.sin(ha),
        Math.cos(ha) * Math.sin(lat) - Math.tan(decl) * Math.cos(lat)
      );
      let azimuth = (rad2deg(azRad) + 180) % 360;
      if (azimuth < 0) azimuth += 360;

      return { azimuth, altitude };
    }
    function polarPoint(az, alt, c, r) {
      const rr = r * (1 - clamp(alt, 0, 90) / 90);
      const a = deg2rad(az);
      return {
        x: c + rr * Math.sin(a),
        y: c - rr * Math.cos(a),
      };
    }
    function renderPolarView(sunAz, sunAlt, normalAz, normalAlt, reflAz, reflAlt) {
      const size = 360;
      const c = size / 2;
      const r = 140;
      const sun = polarPoint(sunAz, sunAlt, c, r);
      const normal = polarPoint(normalAz, normalAlt, c, r);
      const refl = polarPoint(reflAz, reflAlt, c, r);
      return `
        <svg viewBox="0 0 ${size} ${size}">
          <circle cx="${c}" cy="${c}" r="${r}" fill="none" stroke="#cbd5e1" stroke-width="1.5"/>
          <circle cx="${c}" cy="${c}" r="${r * 2/3}" fill="none" stroke="#e2e8f0" stroke-width="1"/>
          <circle cx="${c}" cy="${c}" r="${r / 3}" fill="none" stroke="#e2e8f0" stroke-width="1"/>
          <line x1="${c}" y1="${c-r}" x2="${c}" y2="${c+r}" stroke="#e2e8f0"/>
          <line x1="${c-r}" y1="${c}" x2="${c+r}" y2="${c}" stroke="#e2e8f0"/>
          <text x="${c}" y="24" text-anchor="middle" fill="#64748b" font-size="12">N</text>
          <text x="${c}" y="${size-12}" text-anchor="middle" fill="#64748b" font-size="12">S</text>
          <text x="24" y="${c+4}" text-anchor="middle" fill="#64748b" font-size="12">W</text>
          <text x="${size-24}" y="${c+4}" text-anchor="middle" fill="#64748b" font-size="12">E</text>

          <line x1="${c}" y1="${c}" x2="${sun.x}" y2="${sun.y}" stroke="#f59e0b" stroke-width="3"/>
          <line x1="${c}" y1="${c}" x2="${normal.x}" y2="${normal.y}" stroke="#0f172a" stroke-width="3"/>
          <line x1="${c}" y1="${c}" x2="${refl.x}" y2="${refl.y}" stroke="#2563eb" stroke-width="3"/>

          <circle cx="${sun.x}" cy="${sun.y}" r="7" fill="#f59e0b"/>
          <circle cx="${normal.x}" cy="${normal.y}" r="7" fill="#0f172a"/>
          <circle cx="${refl.x}" cy="${refl.y}" r="7" fill="#2563eb"/>

          <text x="${sun.x + 10}" y="${sun.y - 8}" fill="#b45309" font-size="12">Soleil</text>
          <text x="${normal.x + 10}" y="${normal.y - 8}" fill="#334155" font-size="12">Normale</text>
          <text x="${refl.x + 10}" y="${refl.y - 8}" fill="#1d4ed8" font-size="12">Réfléchi</text>
        </svg>`;
    }
    function lineEnd(cx, cy, angleDeg, length) {
      const a = deg2rad(angleDeg);
      return { x: cx + length * Math.cos(a), y: cy - length * Math.sin(a) };
    }
    function projectToMirrorVerticalPlane(v, mirrorAz) {
      const horiz = vecFromAzAlt(mirrorAz, 0);
      const up = [0, 0, 1];
      const x = dot(v, horiz);
      const z = dot(v, up);
      const m = Math.hypot(x, z) || 1;
      return { x: x / m, z: z / m, rawX: x, rawZ: z };
    }
    function renderSideView(sunToMirror, normalVec, reflectedVec, mirrorAz) {
      const w = 500, h = 240, cx = 200, cy = 170, len = 95;
      const normal2 = projectToMirrorVerticalPlane(normalVec, mirrorAz);
      const incident2 = projectToMirrorVerticalPlane(sunToMirror, mirrorAz);
      const reflected2 = projectToMirrorVerticalPlane(reflectedVec, mirrorAz);

      const normalAngle = rad2deg(Math.atan2(normal2.z, normal2.x));
      const mirrorA = lineEnd(cx, cy, normalAngle - 90, len);
      const mirrorB = lineEnd(cx, cy, normalAngle + 90, len);
      const normal = lineEnd(cx, cy, normalAngle, 70);
      const sunStart = { x: cx - incident2.x * 120, y: cy - incident2.z * -120 };
      const reflEnd = { x: cx + reflected2.x * 120, y: cy - reflected2.z * 120 };
      return `
        <svg viewBox="0 0 ${w} ${h}">
          <line x1="30" y1="${cy}" x2="470" y2="${cy}" stroke="#e2e8f0" stroke-width="1.5"/>
          <text x="438" y="${cy - 8}" fill="#94a3b8" font-size="12">horizon</text>

          <line x1="${mirrorA.x}" y1="${mirrorA.y}" x2="${mirrorB.x}" y2="${mirrorB.y}" stroke="#111827" stroke-width="5" stroke-linecap="round"/>
          <line x1="${cx}" y1="${cy}" x2="${normal.x}" y2="${normal.y}" stroke="#0f172a" stroke-dasharray="5 4" stroke-width="2"/>
          <line x1="${sunStart.x}" y1="${sunStart.y}" x2="${cx}" y2="${cy}" stroke="#f59e0b" stroke-width="3"/>
          <line x1="${cx}" y1="${cy}" x2="${reflEnd.x}" y2="${reflEnd.y}" stroke="#2563eb" stroke-width="3"/>

          <circle cx="${cx}" cy="${cy}" r="4" fill="#111827"/>
          <text x="${mirrorB.x + 10}" y="${mirrorB.y}" fill="#334155" font-size="12">miroir</text>
          <text x="${normal.x + 8}" y="${normal.y - 6}" fill="#334155" font-size="12">normale</text>
          <text x="${sunStart.x - 50}" y="${sunStart.y - 6}" fill="#b45309" font-size="12">rayon incident</text>
          <text x="${reflEnd.x + 8}" y="${reflEnd.y - 6}" fill="#1d4ed8" font-size="12">rayon réfléchi</text>
        </svg>`;
    }

    function update() {
      const lat = parseFloat(document.getElementById("lat").value) || 0;
      const lon = parseFloat(document.getElementById("lon").value) || 0;
      const date = document.getElementById("date").value;
      const time = document.getElementById("time").value;
      const utcOffset = parseFloat(document.getElementById("utcOffset").value) || 0;
      const mirrorAz = parseFloat(document.getElementById("mirrorAz").value) || 0;
      const mirrorTilt = parseFloat(document.getElementById("mirrorTilt").value) || 0;

      document.getElementById("mirrorAzValue").textContent = fmt(mirrorAz, 0);
      document.getElementById("mirrorTiltValue").textContent = fmt(mirrorTilt, 0);

      const dt = parseLocalToUTC(date, time, utcOffset);
      const sun = solarPosition(dt, lat, lon);
      const sunToMirror = normalize(vecFromAzAlt(sun.azimuth, sun.altitude).map((v) => -v));
      const normal = vecFromAzAlt(mirrorAz, mirrorTilt);
      const reflected = reflect(sunToMirror, normal);

      const sunAzAlt = { azimuth: sun.azimuth, altitude: sun.altitude };
      const reflAzAlt = azAltFromVec(reflected);
      const normalAzAlt = azAltFromVec(normal);
      const incidenceAngle = rad2deg(Math.acos(clamp(Math.abs(dot(sunToMirror, normal)), -1, 1)));

      document.getElementById("sunAz").textContent = fmt(sun.azimuth) + "°";
      document.getElementById("sunAlt").textContent = fmt(sun.altitude) + "°";
      document.getElementById("reflAz").textContent = fmt(reflAzAlt.azimuth) + "°";
      document.getElementById("reflAlt").textContent = fmt(reflAzAlt.altitude) + "°";
      document.getElementById("normalAz").textContent = fmt(normalAzAlt.azimuth) + "°";
      document.getElementById("normalAlt").textContent = fmt(normalAzAlt.altitude) + "°";
      document.getElementById("incidenceAngle").textContent = fmt(incidenceAngle) + "°";
      document.getElementById("sunVisible").textContent = sun.altitude > 0 ? "oui" : "non";

      document.getElementById("sunVec").textContent = fmtVec(sunToMirror);
      document.getElementById("normalVec").textContent = fmtVec(normal);
      document.getElementById("reflVec").textContent = fmtVec(reflected);

      document.getElementById("polarView").innerHTML = renderPolarView(
        sunAzAlt.azimuth,
        sunAzAlt.altitude,
        normalAzAlt.azimuth,
        normalAzAlt.altitude,
        reflAzAlt.azimuth,
        reflAzAlt.altitude
      );

      document.getElementById("sideView").innerHTML = renderSideView(
        sunToMirror,
        normal,
        reflected,
        mirrorAz
      );
    }

    function initDefaults() {
      const now = new Date();
      const dateDefault = [
        now.getFullYear(),
        String(now.getMonth() + 1).padStart(2, "0"),
        String(now.getDate()).padStart(2, "0"),
      ].join("-");
      const timeDefault = [
        String(now.getHours()).padStart(2, "0"),
        String(now.getMinutes()).padStart(2, "0"),
      ].join(":");
      document.getElementById("date").value = dateDefault;
      document.getElementById("time").value = timeDefault;
    }

    initDefaults();
    for (const el of document.querySelectorAll("input")) {
      el.addEventListener("input", update);
      el.addEventListener("change", update);
    }
    update();
  </script>
</body>
</html>
"""


class ReflectionToolHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path not in ("/", "/index.html"):
            self.send_error(404, "Not found")
            return

        body = HTML_PAGE.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args) -> None:  # noqa: A003
        return


class ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True


def choose_port(preferred_port: int) -> int:
    if preferred_port != 0:
        return preferred_port
    with contextlib.closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Launch the local reflection visualization tool.")
    parser.add_argument("--host", default="127.0.0.1", help="Host interface to bind. Default: 127.0.0.1")
    parser.add_argument("--port", type=int, default=0, help="Port to bind. Default: choose a free port")
    parser.add_argument("--no-browser", action="store_true", help="Do not auto-open a browser tab")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    port = choose_port(args.port)
    server = ThreadingHTTPServer((args.host, port), ReflectionToolHandler)
    actual_host, actual_port = server.server_address[:2]
    url = f"http://{actual_host}:{actual_port}/"

    print(f"Reflection tool available at {url}")
    print("Press Ctrl+C to stop.")

    if not args.no_browser:
        webbrowser.open(url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping server.")
    finally:
        server.server_close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
