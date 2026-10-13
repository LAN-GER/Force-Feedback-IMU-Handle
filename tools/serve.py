"""Serve the USB workbench on loopback with predictable browser module MIME types."""

import argparse
import socket
import webbrowser
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class WorkbenchServer(ThreadingHTTPServer):
    # Windows SO_REUSEADDR can allow multiple servers to bind the same address.
    allow_reuse_address = False
    allow_reuse_port = False

    def server_bind(self):
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


class WorkbenchHandler(SimpleHTTPRequestHandler):
    # Windows registry MIME associations can otherwise mark .js/.mjs as text/plain.
    extensions_map = {
        **SimpleHTTPRequestHandler.extensions_map,
        ".html": "text/html",
        ".css": "text/css",
        ".js": "text/javascript",
        ".mjs": "text/javascript",
        ".svg": "image/svg+xml",
    }

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


def main():
    parser = argparse.ArgumentParser(description="Launch the force-feedback handle USB web panel")
    parser.add_argument("--port", type=int, default=8776)
    parser.add_argument("--open", action="store_true", help="Open the panel in your default browser")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    directory = Path(__file__).resolve().parent / "web"
    handler = partial(WorkbenchHandler, directory=str(directory))
    try:
        with WorkbenchServer(("127.0.0.1", args.port), handler) as server:
            url = f"http://127.0.0.1:{args.port}/"
            print(f"USB workbench: {url}", flush=True)
            print("Open in Chrome / Edge. Press Ctrl+C to stop.", flush=True)
            if args.open:
                webbrowser.open(url)
            try:
                server.serve_forever()
            except KeyboardInterrupt:
                pass
    except OSError as error:
        parser.exit(1, f"Cannot start local server on http://127.0.0.1:{args.port}/: {error}\n"
                    "Close the previous serve.py terminal, or try --port 8777.\n")


if __name__ == "__main__":
    main()
