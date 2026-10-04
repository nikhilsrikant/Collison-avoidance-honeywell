#!/usr/bin/env python3
"""Local hub for the collision avoidance demo. Standard library only.

  python server.py                 then open http://localhost:8765

What it does
  /                 serves airspace3d.html (the 3D escape display)
  /api/traffic      live ADS-B near a point. Uses your own RTL-SDR receiver if you pass --dump1090,
                    otherwise free public feeds (airplanes.live, then adsb.lol)
  /api/runway       runway_guard.py posts the runway station state here; the 3D display reads it
  /api/state        the 3D display posts whether its airplane is on final; runway_guard.py reads it
"""
import argparse
import json
import math
import os
import socket
import threading
import time
import urllib.parse
import urllib.request
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
VERSION = "1.0"

PUBLIC_FEEDS = [
    ("airplanes.live", "https://api.airplanes.live/v2/point/{lat:.4f}/{lon:.4f}/{dist}"),
    ("adsb.lol", "https://api.adsb.lol/v2/point/{lat:.4f}/{lon:.4f}/{dist}"),
]

# Other scripts (tabletop.py) can add their own endpoints here: path -> function(query_or_body) -> (obj, http_code)
EXTRA_GET = {}
EXTRA_POST = {}
PING_INFO = {}

state_lock = threading.Lock()
shared = {"runway": None, "runway_ts": None, "display": None, "display_ts": None}
cache = {}            # key -> (time, payload)
last_public_call = [0.0]


def haversine_nm(lat1, lon1, lat2, lon2):
    r = 3440.065
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def norm(a):
    """Normalize one aircraft from readsb / dump1090 / ADSBx-v2 style JSON."""
    alt = a.get("alt_baro")
    on_ground = alt == "ground"
    if alt is None or on_ground:
        alt = a.get("alt_geom")
    if on_ground:
        alt = 0
    vs = a.get("baro_rate")
    if vs is None:
        vs = a.get("geom_rate")
    flight = (a.get("flight") or "").strip() or a.get("r") or a.get("hex")
    return {
        "hex": a.get("hex"),
        "flight": flight,
        "lat": a.get("lat"),
        "lon": a.get("lon"),
        "alt_ft": alt,
        "gs_kt": a.get("gs"),
        "track": a.get("track", a.get("true_heading")),
        "vs_fpm": vs,
        "category": a.get("category"),
        "type": a.get("t"),
        "on_ground": bool(on_ground),
        "seen": a.get("seen_pos", a.get("seen")),
    }


def fetch_json(url, timeout=6):
    req = urllib.request.Request(url, headers={"User-Agent": "collision-avoidance-hackathon-demo/1.0"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def get_traffic(lat, lon, dist, dump1090_url):
    key = (round(lat, 3), round(lon, 3), int(dist))
    hit = cache.get(key)
    if hit and time.time() - hit[0] < 1.5:
        return hit[1]
    errors = []
    if dump1090_url:
        try:
            d = fetch_json(dump1090_url, timeout=3)
            ac = []
            for a in d.get("aircraft", []):
                if a.get("lat") is None or a.get("lon") is None:
                    continue
                if haversine_nm(lat, lon, a["lat"], a["lon"]) <= dist:
                    ac.append(norm(a))
            payload = {"source": "your receiver", "now": time.time(), "ac": ac}
            cache[key] = (time.time(), payload)
            return payload
        except Exception as e:  # receiver not running: fall back to public feeds
            errors.append("receiver: %s" % e)
    for name, tpl in PUBLIC_FEEDS:
        wait = 1.05 - (time.time() - last_public_call[0])   # be polite: about 1 request per second
        if wait > 0:
            time.sleep(wait)
        last_public_call[0] = time.time()
        try:
            d = fetch_json(tpl.format(lat=lat, lon=lon, dist=int(dist)))
            ac = [norm(a) for a in (d.get("ac") or d.get("aircraft") or []) if a.get("lat") is not None]
            payload = {"source": name, "now": time.time(), "ac": ac}
            cache[key] = (time.time(), payload)
            return payload
        except Exception as e:
            errors.append("%s: %s" % (name, e))
    raise RuntimeError("; ".join(errors) or "no feed available")


class Handler(SimpleHTTPRequestHandler):
    dump1090_url = None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=HERE, **kwargs)

    def log_message(self, fmt, *args):
        if args and isinstance(args[1], str) and args[1].startswith(("4", "5")) and "/favicon" not in str(args[0]):
            super().log_message(fmt, *args)

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def send_json(self, obj, code=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.end_headers()

    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        q = urllib.parse.parse_qs(u.query)
        if u.path == "/":
            self.path = "/airspace3d.html"
            return super().do_GET()
        if u.path == "/api/ping":
            info = {"ok": True, "version": VERSION}
            info.update(PING_INFO)
            return self.send_json(info)
        if u.path in EXTRA_GET:
            obj, code = EXTRA_GET[u.path](q)
            return self.send_json(obj, code)
        if u.path == "/api/traffic":
            try:
                lat = float(q.get("lat", ["33.4343"])[0])
                lon = float(q.get("lon", ["-112.0116"])[0])
                dist = max(1, min(100, float(q.get("dist", ["25"])[0])))
                return self.send_json(get_traffic(lat, lon, dist, self.dump1090_url))
            except Exception as e:
                return self.send_json({"error": str(e), "ac": []}, 502)
        if u.path in ("/api/runway", "/api/state"):
            k = "runway" if u.path == "/api/runway" else "display"
            with state_lock:
                stored = shared[k]
                ts = shared[k + "_ts"]
            if stored is None:
                stored = {"state": "OFFLINE"} if k == "runway" else {}
            data = dict(stored)
            data["age"] = None if ts is None else round(time.time() - ts, 2)
            return self.send_json(data)
        return super().do_GET()

    def do_POST(self):
        u = urllib.parse.urlparse(self.path)
        if u.path not in ("/api/runway", "/api/state") and u.path not in EXTRA_POST:
            return self.send_json({"error": "not found"}, 404)
        try:
            n = int(self.headers.get("Content-Length", "0"))
            data = json.loads(self.rfile.read(n).decode("utf-8") or "{}")
        except Exception as e:
            return self.send_json({"error": "bad json: %s" % e}, 400)
        if u.path in EXTRA_POST:
            obj, code = EXTRA_POST[u.path](data)
            return self.send_json(obj, code)
        k = "runway" if u.path == "/api/runway" else "display"
        with state_lock:
            shared[k] = data
            shared[k + "_ts"] = time.time()
        return self.send_json({"ok": True})


def lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def serve(port=8765, dump1090=None, title="Collision avoidance demo hub", show_adsb=True):
    Handler.dump1090_url = dump1090
    httpd = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    print("%s v%s" % (title, VERSION))
    print("  3D display:      http://localhost:%d" % port)
    print("  other devices:   http://%s:%d  (same Wi-Fi)" % (lan_ip(), port))
    if show_adsb:
        print("  ADS-B source:    %s" % (dump1090 or "public feeds (airplanes.live, adsb.lol)"))
    print("Press Ctrl+C to stop.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--dump1090", default=None,
                    help="your receiver's aircraft.json, e.g. http://127.0.0.1:8080/data/aircraft.json")
    args = ap.parse_args()
    serve(args.port, args.dump1090)


if __name__ == "__main__":
    main()
