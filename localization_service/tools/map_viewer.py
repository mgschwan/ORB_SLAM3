"""
map_viewer.py — Three.js HTTP viewer for sparse_map.json files.

Called by offline_mapper.py's `visualize` subcommand; also usable standalone:
  python3 map_viewer.py <sparse_map.json> [--port 8765]
"""

import http.server
import json
import webbrowser
from pathlib import Path

_HTML_TEMPLATE = """\
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Sparse Map Viewer</title>
<style>
  body { margin:0; overflow:hidden; background:#111; font-family:monospace; }
  #panel {
    position:absolute; top:12px; left:12px;
    background:rgba(0,0,0,0.7); color:#ddd;
    padding:12px 16px; border-radius:8px;
    font-size:12px; line-height:1.6; min-width:160px;
    pointer-events:none;
    user-select:none;
  }
  #panel b { color:#fff; }
  .dot-yellow { color:#ffdd44; }
  .dot-red    { color:#ff5533; }
  .dot-green  { color:#44ff88; }
  #btn-cameras {
    display:block; width:100%; margin-top:10px;
    background:#2a2a3e; color:#ccc; border:1px solid #555;
    border-radius:4px; padding:5px 0; font-size:11px;
    font-family:monospace; cursor:pointer; pointer-events:auto;
  }
  #btn-cameras:hover { background:#3a3a5e; color:#fff; }
</style>
<script>__THREE_JS__</script>
<script>__ORBIT_JS__</script>
</head>
<body>
<div id="panel"><b>Loading...</b></div>
<script>
// ── Scene setup ────────────────────────────────────────────────────────────
const scene    = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a2e);

const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setSize(innerWidth, innerHeight);
document.body.appendChild(renderer.domElement);

const camera = new THREE.PerspectiveCamera(60, innerWidth/innerHeight, 0.001, 2000);
camera.position.set(0, 2, 5);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping  = true;
controls.dampingFactor  = 0.05;

scene.add(new THREE.AmbientLight(0xffffff, 1.0));
// Grid and axes are added after data loads so they can be sized to the scene.

window.addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

// ── Coord conversion: ORB-SLAM3 (Y-down, Z-fwd) → Three.js (Y-up, Z-back) ─
function orb2three(x, y, z) { return new THREE.Vector3(x, -y, -z); }

function camPtToThree(cx, cy, cz, twc) {
  const x = twc[0][0]*cx + twc[0][1]*cy + twc[0][2]*cz + twc[0][3];
  const y = twc[1][0]*cx + twc[1][1]*cy + twc[1][2]*cz + twc[1][3];
  const z = twc[2][0]*cx + twc[2][1]*cy + twc[2][2]*cz + twc[2][3];
  return orb2three(x, y, z);
}

// ── Draw one camera frustum as a LineSegments ───────────────────────────────
function makeFrustum(kf, cam, depth, color) {
  const {fx, fy, cx, cy, width, height} = cam;
  const twc = kf.Twc;

  const origin  = camPtToThree(0, 0, 0, twc);
  const corners = [
    camPtToThree((-cx)/fx*depth,        (-cy)/fy*depth,        depth, twc),
    camPtToThree((width-cx)/fx*depth,   (-cy)/fy*depth,        depth, twc),
    camPtToThree((width-cx)/fx*depth,   (height-cy)/fy*depth,  depth, twc),
    camPtToThree((-cx)/fx*depth,        (height-cy)/fy*depth,  depth, twc),
  ];

  const pts = [origin, ...corners].flatMap(v => [v.x, v.y, v.z]);
  const idx = [0,1, 0,2, 0,3, 0,4,  1,2, 2,3, 3,4, 4,1];

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  geo.setIndex(idx);
  return new THREE.LineSegments(geo, new THREE.LineBasicMaterial({color}));
}

// ── Load and build scene ────────────────────────────────────────────────────
fetch('/map.json').then(r => r.json()).then(data => {
  const cam = data.camera;
  const kfs = data.keyframes;
  const mps = data.mappoints;

  // ── Compute scene scale first so everything else can reference it ──────────
  const allPts = mps.length > 0 ? mps.map(mp => orb2three(...mp.pos))
                                 : kfs.map(kf => camPtToThree(0,0,0,kf.Twc));
  const bbox   = new THREE.Box3().setFromPoints(allPts);
  const center = bbox.getCenter(new THREE.Vector3());
  const extent = Math.max(bbox.getSize(new THREE.Vector3()).length(), 0.5);

  // ── Grid and axes sized to scene ───────────────────────────────────────────
  const gridSize = extent * 2.5;
  const gridDivs = Math.max(10, Math.min(80, Math.round(gridSize * 4)));
  const grid = new THREE.GridHelper(gridSize, gridDivs, 0x333333, 0x222222);
  grid.position.set(center.x, bbox.min.y, center.z);
  scene.add(grid);
  scene.add(new THREE.AxesHelper(extent * 0.08));

  // ── Camera clip planes and orbit controls scaled to scene ──────────────────
  camera.near = extent * 0.001;
  camera.far  = extent * 100;
  camera.updateProjectionMatrix();
  controls.minDistance = extent * 0.01;
  controls.maxDistance = extent * 20;
  controls.zoomSpeed   = 0.8;
  controls.panSpeed    = 0.6;

  // ── MapPoints ─────────────────────────────────────────────────────────────
  if (mps.length > 0) {
    const pos = new Float32Array(mps.length * 3);
    mps.forEach((mp, i) => {
      const v = orb2three(...mp.pos);
      pos[i*3]=v.x; pos[i*3+1]=v.y; pos[i*3+2]=v.z;
    });
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    scene.add(new THREE.Points(geo,
      new THREE.PointsMaterial({color:0xffdd44, size:extent * 0.003})));
  }

  // ── Camera frustums + trajectory ───────────────────────────────────────────
  const frustumDepth = extent * 0.06;
  const cameraGroup  = new THREE.Group();
  scene.add(cameraGroup);

  const camPts = [];
  kfs.forEach((kf, i) => {
    const t = i / Math.max(kfs.length - 1, 1);
    const color = new THREE.Color(1-t, 0.2+t*0.8, 0.2);
    cameraGroup.add(makeFrustum(kf, cam, frustumDepth, color));
    camPts.push(camPtToThree(0, 0, 0, kf.Twc));
  });

  if (camPts.length >= 2) {
    const pos = new Float32Array(camPts.length * 3);
    camPts.forEach((v,i) => { pos[i*3]=v.x; pos[i*3+1]=v.y; pos[i*3+2]=v.z; });
    const idx = [];
    for(let i=0;i<camPts.length-1;i++) idx.push(i, i+1);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setIndex(idx);
    cameraGroup.add(new THREE.LineSegments(geo,
      new THREE.LineBasicMaterial({color:0x888888})));
  }

  // ── Fit initial camera view to scene ──────────────────────────────────────
  controls.target.copy(center);
  camera.position.copy(center).add(
    new THREE.Vector3(0, extent * 0.5, extent * 1.2));
  controls.update();

  document.getElementById('panel').innerHTML =
    '<b>Sparse Map</b><br>' +
    '<span class="dot-yellow">&#9632;</span> MapPoints: ' + mps.length + '<br>' +
    '<span class="dot-red">&#9632;</span> First cam &nbsp;' +
    '<span class="dot-green">&#9632;</span> Last cam<br>' +
    'Keyframes: ' + kfs.length + '<br>' +
    '<br><span style="color:#666;font-size:11px">Drag to orbit · Scroll to zoom<br>Right-drag to pan</span>' +
    '<button id="btn-cameras">Hide cameras</button>';

  document.getElementById('btn-cameras').addEventListener('click', () => {
    cameraGroup.visible = !cameraGroup.visible;
    document.getElementById('btn-cameras').textContent =
      cameraGroup.visible ? 'Hide cameras' : 'Show cameras';
  });
});

// ── Render loop ─────────────────────────────────────────────────────────────
function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();
</script>
</body>
</html>
"""


def _build_page(html_dir: Path) -> bytes:
    three_js = (html_dir / "three.min.js").read_text(encoding="utf-8")
    orbit_js = (html_dir / "orbitcontrols.js").read_text(encoding="utf-8")
    return (_HTML_TEMPLATE
            .replace("__THREE_JS__", three_js)
            .replace("__ORBIT_JS__", orbit_js)
            .encode("utf-8"))


def serve(map_data: dict, port: int, html_dir: Path) -> None:
    """Serve map_data as an interactive Three.js viewer on the given port."""
    page_bytes = _build_page(html_dir)
    map_json   = json.dumps(map_data, separators=(",", ":")).encode("utf-8")

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            if self.path == "/":
                self._send(200, "text/html; charset=utf-8", page_bytes)
            elif self.path == "/map.json":
                self._send(200, "application/json", map_json)
            else:
                self.send_response(404)
                self.end_headers()

        do_HEAD = do_GET

        def _send(self, code, mime, body: bytes):
            self.send_response(code)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)

    server = http.server.HTTPServer(("", port), Handler)
    url = f"http://localhost:{port}"
    print(f"Map viewer : {url}  (Ctrl+C to stop)")
    webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser(description="Sparse map Three.js viewer")
    p.add_argument("map",  help="sparse_map.json to display")
    p.add_argument("--port", type=int, default=8765)
    a = p.parse_args()
    with open(a.map) as f:
        data = json.load(f)
    html_dir = Path(__file__).parent.parent / "html"
    serve(data, a.port, html_dir)
