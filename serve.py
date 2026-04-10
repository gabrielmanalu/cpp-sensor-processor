#!/usr/bin/env python3
"""
Minimal HTTP server for the point-cloud visualizer.

Usage (from project root):
    python serve.py          # serves on http://localhost:8080
    python serve.py 9000     # custom port
"""
import http.server
import socketserver
import webbrowser
import sys
import os

DEFAULT_PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080

# Serve from the project root so /vis/index.html and /vis/viz_data.json both resolve.
os.chdir(os.path.dirname(os.path.abspath(__file__)))

class Handler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, fmt, *args):
        # Suppress noisy access logs; only print errors.
        if args[1] not in ("200", "304"):
            super().log_message(fmt, *args)

# Try DEFAULT_PORT, then increment until a free one is found.
port = DEFAULT_PORT
while True:
    try:
        httpd = socketserver.TCPServer(("", port), Handler)
        break
    except OSError:
        print(f"Port {port} is in use, trying {port + 1}...")
        port += 1

url = f"http://localhost:{port}/vis/"
with httpd:
    print(f"Serving at {url}")
    print("Press Ctrl+C to stop.\n")
    webbrowser.open(url)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped.")
