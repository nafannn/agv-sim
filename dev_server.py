"""Server lokal untuk menguji tampilan sebelum dideploy ke Vercel.

Menjalankan index.html sebagai berkas statis dan /api/simulate memakai
fungsi Python yang sama dengan yang akan dijalankan Vercel:

    python dev_server.py           # http://127.0.0.1:8099
"""

from __future__ import annotations

import json
import os
import sys
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(ROOT, "api"))

from simulate import run_sim  # noqa: E402

PORT = int(os.environ.get("PORT", "8099"))
ALLOWED = ("skenario", "kontroler", "m_scale", "mu_scale")


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=ROOT, **kw)

    def do_GET(self):
        if self.path.startswith("/api/simulate"):
            q = parse_qs(urlparse(self.path).query)
            kw = {k: q[k][0] for k in ALLOWED if k in q}
            try:
                body = json.dumps(run_sim(**kw)).encode()
                self.send_response(200)
                code = 200
            except Exception as exc:  # noqa: BLE001
                body = json.dumps({"error": str(exc)}).encode()
                self.send_response(400)
                code = 400
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            print(f"  /api/simulate {kw} -> {code} ({len(body)/1024:.0f} KB)")
            return
        super().do_GET()

    def log_message(self, fmt, *args):
        pass


if __name__ == "__main__":
    print(f"server lokal: http://127.0.0.1:{PORT}")
    ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()
