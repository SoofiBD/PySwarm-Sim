/**
 * realtime_map.js — WebSocket-driven Google Maps drone visualiser.
 *
 * Architecture
 * ------------
 * 1. `initMap()` is called by the Google Maps loader callback.  It creates
 *    the map, opens the WebSocket to /ws/telemetry, and registers the
 *    message handler.
 *
 * 2. Every message from the server is a JSON object:
 *      { type: "telemetry", drones: [...], targets: [...] }
 *
 * 3. When a message arrives the *target* positions are recorded and a
 *    requestAnimationFrame loop linearly interpolates markers between the
 *    previous and current positions.  This gives visually smooth animation
 *    even when the server tick rate (20 Hz) is slower than the display
 *    refresh rate (60 Hz).
 *
 * Smooth interpolation maths
 * --------------------------
 * Let t_last = timestamp of last server message
 *     t_next = t_last + TICK_MS       (estimated next message)
 *     t_now  = performance.now()
 *     alpha  = clamp((t_now - t_last) / TICK_MS, 0, 1)
 *
 *     pos(t_now) = lerp(pos_prev, pos_target, alpha)
 */

"use strict";

// ── Configuration ────────────────────────────────────────────────────────────
const WS_URL   = `ws://${location.host}/ws/telemetry`;
const TICK_MS  = 50;   // expected server interval — used for lerp smoothing
const COMM_RANGE_M = 500;   // metres — link drawn if drones within this range
const WARN_RATIO   = 0.80;  // link turns red above WARN_RATIO × COMM_RANGE

// ── State ────────────────────────────────────────────────────────────────────
let gmap;                         // google.maps.Map instance
let ws;                           // WebSocket

const droneMarkers  = {};         // id → { marker, prev, target, ts }
const targetMarkers = {};         // id → marker
const commLines     = [];         // active polylines

// ── Icons (cached) ───────────────────────────────────────────────────────────
const ICONS = {
    drone: {
        path: "M12 2a10 10 0 1 0 0 20A10 10 0 0 0 12 2z",  // SVG fallback
        url: "https://maps.google.com/mapfiles/kml/shapes/airports.png",
        scaledSize: () => new google.maps.Size(32, 32),
        anchor:     () => new google.maps.Point(16, 16),
    },
    Free: {
        url: "https://maps.google.com/mapfiles/ms/icons/green-dot.png",
        scaledSize: () => new google.maps.Size(36, 36),
        anchor:     () => new google.maps.Point(18, 36),
    },
    Visiting: {
        url: "https://maps.google.com/mapfiles/ms/icons/yellow-dot.png",
        scaledSize: () => new google.maps.Size(36, 36),
        anchor:     () => new google.maps.Point(18, 36),
    },
    Visited: {
        url: "https://maps.google.com/mapfiles/ms/icons/red-dot.png",
        scaledSize: () => new google.maps.Size(36, 36),
        anchor:     () => new google.maps.Point(18, 36),
    },
};

function droneIcon() {
    return {
        url: ICONS.drone.url,
        scaledSize: ICONS.drone.scaledSize(),
        anchor:     ICONS.drone.anchor(),
    };
}

function targetIcon(state) {
    const cfg = ICONS[state] || ICONS.Free;
    return { url: cfg.url, scaledSize: cfg.scaledSize(), anchor: cfg.anchor() };
}

// ── Google Maps initialisation (called by Maps API callback) ─────────────────
function initMap() {
    gmap = new google.maps.Map(document.getElementById("map"), {
        center:    { lat: 41.015137, lng: 28.979530 },
        zoom:      16,
        mapTypeId: "roadmap",
        styles: [{ featureType: "poi", stylers: [{ visibility: "off" }] }],
    });

    _connectWebSocket();
    requestAnimationFrame(_animationLoop);
}

// ── WebSocket ────────────────────────────────────────────────────────────────
function _connectWebSocket() {
    ws = new WebSocket(WS_URL);
    ws.onopen    = () => { console.log("[WS] connected"); _updateStatus("Connected"); };
    ws.onclose   = () => { console.warn("[WS] closed — retrying in 3 s"); _updateStatus("Reconnecting…"); setTimeout(_connectWebSocket, 3000); };
    ws.onerror   = (e) => console.error("[WS] error", e);
    ws.onmessage = _onMessage;

    // Send a ping every 20 s to keep the server-side receive loop alive
    setInterval(() => { if (ws.readyState === WebSocket.OPEN) ws.send("ping"); }, 20_000);
}

function _onMessage(event) {
    const msg = JSON.parse(event.data);
    if (msg.type !== "telemetry") return;

    const now = performance.now();
    _processDrones(msg.drones, now);
    _processTargets(msg.targets);
    _rebuildCommLinks(msg.drones);
}

// ── Drone marker management ───────────────────────────────────────────────────
function _processDrones(drones, now) {
    const seen = new Set();

    for (const d of drones) {
        seen.add(d.id);
        const latLng = new google.maps.LatLng(d.lat, d.lng);

        if (!droneMarkers[d.id]) {
            // First appearance — create marker
            const marker = new google.maps.Marker({
                position: latLng,
                map:      gmap,
                icon:     droneIcon(),
                title:    `UAV ${d.id}`,
                zIndex:   10,
            });
            // Info window with live telemetry
            const infoWin = new google.maps.InfoWindow();
            marker.addListener("click", () => {
                infoWin.setContent(_droneInfoHtml(d));
                infoWin.open(gmap, marker);
            });
            droneMarkers[d.id] = { marker, infoWin, prev: latLng, target: latLng, ts: now, data: d };
        } else {
            // Subsequent update — record prev & target for interpolation
            const entry = droneMarkers[d.id];
            entry.prev   = entry.marker.getPosition();
            entry.target = latLng;
            entry.ts     = now;
            entry.data   = d;
        }
    }

    // Remove markers for drones no longer in the feed
    for (const id of Object.keys(droneMarkers)) {
        if (!seen.has(Number(id))) {
            droneMarkers[id].marker.setMap(null);
            delete droneMarkers[id];
        }
    }
}

// ── Target marker management ─────────────────────────────────────────────────
function _processTargets(targets) {
    const seen = new Set();
    for (const t of targets) {
        seen.add(t.id);
        const latLng = new google.maps.LatLng(t.lat, t.lng);
        if (!targetMarkers[t.id]) {
            targetMarkers[t.id] = new google.maps.Marker({
                position: latLng,
                map:      gmap,
                icon:     targetIcon(t.state),
                title:    `Target ${t.id}`,
                zIndex:   5,
            });
        } else {
            targetMarkers[t.id].setPosition(latLng);
            targetMarkers[t.id].setIcon(targetIcon(t.state));
        }
    }
    for (const id of Object.keys(targetMarkers)) {
        if (!seen.has(Number(id))) {
            targetMarkers[id].setMap(null);
            delete targetMarkers[id];
        }
    }
}

// ── Communication link lines ─────────────────────────────────────────────────
function _rebuildCommLinks(drones) {
    // Clear old lines
    for (const line of commLines) line.setMap(null);
    commLines.length = 0;

    for (let i = 0; i < drones.length; i++) {
        for (let j = i + 1; j < drones.length; j++) {
            const dist = _haversineM(drones[i], drones[j]);
            if (dist > COMM_RANGE_M) continue;
            const ratio = dist / COMM_RANGE_M;
            const color = ratio > WARN_RATIO ? "#FF4444" : "#00CC44";
            const line = new google.maps.Polyline({
                path:          [{ lat: drones[i].lat, lng: drones[i].lng },
                                 { lat: drones[j].lat, lng: drones[j].lng }],
                geodesic:      true,
                strokeColor:   color,
                strokeOpacity: 0.8,
                strokeWeight:  1.5,
            });
            line.setMap(gmap);
            commLines.push(line);
        }
    }
}

// ── Smooth animation loop ────────────────────────────────────────────────────
/**
 * Linear interpolation between marker's previous position and target.
 *
 * alpha = (now - message_timestamp) / TICK_MS  clamped [0, 1]
 *
 * Because requestAnimationFrame fires at ~60 fps and the server sends at
 * 20 fps, each 50 ms server message drives ~3 rAF frames of smooth motion.
 */
function _animationLoop() {
    const now = performance.now();
    for (const entry of Object.values(droneMarkers)) {
        const alpha  = Math.min(1.0, (now - entry.ts) / TICK_MS);
        const prev   = entry.prev;
        const target = entry.target;
        const lat = prev.lat() + (target.lat() - prev.lat()) * alpha;
        const lng = prev.lng() + (target.lng() - prev.lng()) * alpha;
        entry.marker.setPosition(new google.maps.LatLng(lat, lng));
    }
    requestAnimationFrame(_animationLoop);
}

// ── Utilities ─────────────────────────────────────────────────────────────────
function _haversineM(p1, p2) {
    const R    = 6_378_137;
    const dLat = (p2.lat - p1.lat) * Math.PI / 180;
    const dLng = (p2.lng - p1.lng) * Math.PI / 180;
    const a    = Math.sin(dLat/2)**2 +
                 Math.cos(p1.lat * Math.PI/180) *
                 Math.cos(p2.lat * Math.PI/180) *
                 Math.sin(dLng/2)**2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
}

function _droneInfoHtml(d) {
    return `<div style="font-family:monospace;font-size:12px">
        <b>UAV ${d.id}</b><br>
        State : ${d.state}<br>
        Alt   : ${d.alt} m<br>
        Speed : ${d.speed} m/s<br>
        Vel   : (${d.vx}, ${d.vy}, ${d.vz})
    </div>`;
}

function _updateStatus(msg) {
    const el = document.getElementById("ws-status");
    if (el) el.textContent = msg;
}
