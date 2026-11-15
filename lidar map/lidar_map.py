#!/usr/bin/env python3
import json
import math
import os
import queue
import subprocess
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import numpy as np
    from scipy.spatial import cKDTree
except ImportError:
    np = None
    cKDTree = None

ROOT = os.path.dirname(os.path.abspath(__file__))
BRIDGE = os.path.join(ROOT, "bin", "lidar_bridge")


def detect_port():
    configured = os.environ.get("RPLIDAR_PORT")
    if configured:
        return configured
    candidates = sorted(path for path in ("/dev/cu.usbserial-110", "/dev/cu.usbserial-10") if os.path.exists(path))
    return candidates[0] if candidates else "/dev/cu.usbserial-110"


PORT = detect_port()
BAUD = "460800"


class LidarMap(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("RPLIDAR C1 · Live 2D Map")
        self.geometry("1180x780")
        self.minsize(900, 620)
        self.configure(bg="#0b1020")
        self.proc = None
        self.events = queue.Queue()
        self.errors = queue.Queue()
        self.points = []
        self.map_points = []
        self.mapping_enabled = False
        self.pose = [0.0, 0.0, 0.0]
        self.map_index = {}
        self.total_points = 0
        self.scans = 0
        self.scale = 90.0
        self.pan_x = 0
        self.pan_y = 0
        self.drag_origin = None
        self.capture_note = ""
        self.protocol("WM_DELETE_WINDOW", self.close)
        self._style()
        self._build()
        self.after(40, self._tick)

    def _style(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TButton", background="#202a46", foreground="#e8eefc", borderwidth=0, padding=(13, 8))
        style.map("TButton", background=[("active", "#2b3a63")])

    def _build(self):
        top = tk.Frame(self, bg="#111a31", height=72)
        top.pack(fill="x")
        tk.Label(top, text="RPLIDAR C1", bg="#111a31", fg="#f3f6ff", font=("Helvetica", 20, "bold")).pack(side="left", padx=(24, 5), pady=18)
        tk.Label(top, text="LIVE 2D MAPPING", bg="#111a31", fg="#7384aa", font=("Helvetica", 10, "bold")).pack(side="left", pady=22)
        self.status = tk.Label(top, text="● DISCONNECTED", bg="#111a31", fg="#ff7188", font=("Helvetica", 10, "bold"))
        self.status.pack(side="right", padx=24)

        side = tk.Frame(self, bg="#111a31", width=245)
        side.pack(side="left", fill="y")
        side.pack_propagate(False)
        self._section(side, "CONNECTION")
        self._info(side, "PORT", PORT)
        self._info(side, "BAUD RATE", BAUD)
        self._info(side, "PROTOCOL", "SDK native HQ nodes")
        self.start_btn = ttk.Button(side, text="▶  START SCAN", command=self.start)
        self.start_btn.pack(fill="x", padx=18, pady=(24, 8))
        ttk.Button(side, text="■  STOP SCAN", command=self.stop).pack(fill="x", padx=18)
        ttk.Button(side, text="▣  CAPTURE MAP", command=self.capture_map).pack(fill="x", padx=18, pady=(8, 0))
        self.mapping_btn = ttk.Button(side, text="＋  START MAPPING", command=self.toggle_mapping)
        self.mapping_btn.pack(fill="x", padx=18, pady=(8, 0))
        ttk.Button(side, text="⌫  CLEAR STORED MAP", command=self.clear_map).pack(fill="x", padx=18, pady=(8, 0))
        self._section(side, "SCAN TELEMETRY")
        self.stat_points = self._stat(side, "POINTS / FRAME", "—")
        self.stat_scans = self._stat(side, "FRAMES RECEIVED", "0")
        self.stat_range = self._stat(side, "MAX RANGE", "—")
        self.stat_rate = self._stat(side, "UPDATE RATE", "—")
        self.stat_map = self._stat(side, "STORED MAP POINTS", "0")
        self._section(side, "CONTROLS")
        tk.Label(side, text="Scroll to zoom  ·  Drag to pan\nDouble-click to reset view", justify="left", bg="#111a31", fg="#7384aa", font=("Helvetica", 10)).pack(anchor="w", padx=18)
        tk.Label(side, text="COORDINATE SYSTEM", bg="#111a31", fg="#5c6c91", font=("Helvetica", 9, "bold")).pack(anchor="w", padx=18, pady=(24, 6))
        tk.Label(side, text="X / Y metres (2D)\nZ / height: unavailable", justify="left", bg="#111a31", fg="#c0cbe3", font=("Helvetica", 10)).pack(anchor="w", padx=18)

        main = tk.Frame(self, bg="#0b1020")
        main.pack(side="left", fill="both", expand=True)
        self.canvas = tk.Canvas(main, bg="#0b1020", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=16, pady=16)
        self.canvas.bind("<MouseWheel>", self.zoom)
        self.canvas.bind("<Button-4>", self.zoom)
        self.canvas.bind("<Button-5>", self.zoom)
        self.canvas.bind("<ButtonPress-1>", self.pan_start)
        self.canvas.bind("<B1-Motion>", self.pan_move)
        self.canvas.bind("<Double-Button-1>", self.reset_view)

    def _section(self, parent, text):
        tk.Label(parent, text=text, bg="#111a31", fg="#5c6c91", font=("Helvetica", 9, "bold")).pack(anchor="w", padx=18, pady=(26, 10))

    def _info(self, parent, label, value):
        row = tk.Frame(parent, bg="#111a31")
        row.pack(fill="x", padx=18, pady=3)
        tk.Label(row, text=label, bg="#111a31", fg="#7787a9", font=("Helvetica", 9)).pack(side="left")
        tk.Label(row, text=value, bg="#111a31", fg="#e6ecfb", font=("Helvetica", 9, "bold")).pack(side="right")

    def _stat(self, parent, label, value):
        box = tk.Frame(parent, bg="#18223c")
        box.pack(fill="x", padx=18, pady=3)
        tk.Label(box, text=label, bg="#18223c", fg="#8291b3", font=("Helvetica", 8)).pack(anchor="w", padx=10, pady=(7, 0))
        out = tk.Label(box, text=value, bg="#18223c", fg="#f0f4ff", font=("Helvetica", 14, "bold"))
        out.pack(anchor="w", padx=10, pady=(1, 7))
        return out

    def start(self):
        if self.proc and self.proc.poll() is None:
            return
        if not os.path.exists(BRIDGE):
            self.status.config(text="● BUILD REQUIRED", fg="#ffbd69")
            return
        self.points = []
        self.status.config(text="● CONNECTING…", fg="#ffbd69")
        self.proc = subprocess.Popen([BRIDGE, PORT, BAUD], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._error_reader, daemon=True).start()

    def _reader(self):
        for line in self.proc.stdout:
            try: self.events.put(json.loads(line))
            except json.JSONDecodeError: pass
        self.events.put({"type": "exited", "code": self.proc.poll()})

    def _error_reader(self):
        for line in self.proc.stderr:
            message = line.strip()
            if message:
                self.errors.put(message)

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
        self.status.config(text="● STOPPED", fg="#ffbd69")

    def _tick(self):
        try:
            while True:
                error = self.errors.get_nowait()
                self.status.config(text="● CONNECTION ERROR", fg="#ff7188")
                self.capture_note = error.replace("bridge: ", "")
        except queue.Empty:
            pass
        try:
            while True:
                event = self.events.get_nowait()
                typ = event.get("type")
                if typ == "connected": self.status.config(text="● CONNECTED", fg="#5be3a4")
                elif typ == "scanning": self.status.config(text="● SCANNING", fg="#5be3a4")
                elif typ == "scan":
                    self.points = event.get("points", [])
                    self.scans += 1
                    self.total_points += len(self.points)
                    self.stat_points.config(text=str(len(self.points)))
                    self.stat_scans.config(text=str(self.scans))
                    if self.points: self.stat_range.config(text=f"{max(p[1] for p in self.points):.1f} m")
                    if self.mapping_enabled:
                        self.update_map()
                elif typ == "exited" and self.status.cget("text") == "● SCANNING": self.status.config(text="● DISCONNECTED", fg="#ff7188")
        except queue.Empty: pass
        self.draw()
        self.after(40, self._tick)

    def _origin(self):
        return self.canvas.winfo_width()/2 + self.pan_x, self.canvas.winfo_height()/2 + self.pan_y

    def draw(self):
        c = self.canvas; c.delete("all")
        ox, oy = self._origin()
        render_points = self.map_points if self.mapping_enabled and self.map_points else self.points
        max_distance = max((math.hypot(p[0], p[1]) for p in render_points), default=4) if render_points and self.mapping_enabled else max((p[1] for p in self.points), default=4)
        max_m = max(4, math.ceil(max_distance / 2) * 2)
        for radius_m in range(1, max_m + 1):
            r = radius_m * self.scale
            if r > max(c.winfo_width(), c.winfo_height()): break
            c.create_oval(ox-r, oy-r, ox+r, oy+r, outline="#233052", width=1)
            c.create_text(ox+r+5, oy-3, text=f"{radius_m}m", fill="#526184", anchor="w", font=("Helvetica", 9))
        edge = max(c.winfo_width(), c.winfo_height())
        c.create_line(ox-edge, oy, ox+edge, oy, fill="#263457", arrow="last")
        c.create_line(ox, oy+edge, ox, oy-edge, fill="#263457", arrow="last")
        c.create_text(ox+edge-12, oy+15, text="X (m)", fill="#8190b2", anchor="e", font=("Helvetica", 10, "bold"))
        c.create_text(ox+12, oy-edge+12, text="Y (m)", fill="#8190b2", anchor="w", font=("Helvetica", 10, "bold"))
        c.create_text(ox+10, oy+10, text="(0, 0)", fill="#f5b95b", anchor="nw", font=("Helvetica", 9))
        if self.mapping_enabled:
            mapped_points = [(x, y, 15) for x, y in self.map_points]
        else:
            mapped_points = []
            for angle, dist, quality in self.points:
                rad = math.radians(angle)
                mapped_points.append((math.sin(rad) * dist, math.cos(rad) * dist, quality))
        for x_m, y_m, quality in mapped_points:
            x, y = ox + x_m*self.scale, oy - y_m*self.scale
            color = "#64e5b0" if quality >= 10 else "#3fb8b3"
            r = 2.4 if quality >= 10 else 1.8
            c.create_oval(x-r, y-r, x+r, y+r, fill=color, outline="")
        c.create_oval(ox-5, oy-5, ox+5, oy+5, fill="#f5b95b", outline="")
        c.create_text(ox+12, oy+13, text="C1", fill="#f5b95b", anchor="w", font=("Helvetica", 9, "bold"))
        if self.capture_note:
            c.create_text(18, c.winfo_height()-16, text=self.capture_note, fill="#64e5b0", anchor="w", font=("Helvetica", 10))

    def toggle_mapping(self):
        self.mapping_enabled = not self.mapping_enabled
        if self.mapping_enabled:
            self.map_points = []
            self.map_index = {}
            self.pose = [0.0, 0.0, 0.0]
            self.mapping_btn.config(text="■  STOP MAPPING")
            self.capture_note = "Mapping active — move slowly for scan matching"
        else:
            self.mapping_btn.config(text="＋  START MAPPING")
            self.capture_note = "Mapping paused — stored map retained"
        self.draw()

    def clear_map(self):
        self.map_points = []
        self.map_index = {}
        self.pose = [0.0, 0.0, 0.0]
        self.stat_map.config(text="0")
        self.capture_note = "Stored map cleared"
        self.draw()

    @staticmethod
    def _local_xy(scan):
        result = []
        for angle, distance, quality in scan[::2]:
            if distance <= 0.02:
                continue
            rad = math.radians(angle)
            result.append((math.sin(rad) * distance, math.cos(rad) * distance))
        return result

    def _rebuild_map_index(self):
        self.map_index = {}
        cell = 0.20
        for x, y in self.map_points:
            key = (math.floor(x / cell), math.floor(y / cell))
            self.map_index.setdefault(key, []).append((x, y))

    def _match_pose(self, local_points):
        if not self.map_points or np is None or cKDTree is None:
            return self.pose[:]
        source = np.asarray(local_points[::max(1, len(local_points) // 500)], dtype=np.float64)
        target = np.asarray(self.map_points, dtype=np.float64)
        tree = cKDTree(target)
        pose = np.asarray(self.pose, dtype=np.float64)
        for _ in range(8):
            c, s = math.cos(pose[2]), math.sin(pose[2])
            rotation = np.asarray([[c, -s], [s, c]])
            world = source @ rotation.T + pose[:2]
            distances, indices = tree.query(world, distance_upper_bound=0.65)
            valid = np.isfinite(distances) & (distances < 0.65)
            if int(valid.sum()) < 20:
                break
            src = world[valid]
            dst = target[indices[valid]]
            src_center = src.mean(axis=0)
            dst_center = dst.mean(axis=0)
            covariance = (src - src_center).T @ (dst - dst_center)
            u, _, vt = np.linalg.svd(covariance)
            delta_rotation = vt.T @ u.T
            if np.linalg.det(delta_rotation) < 0:
                vt[-1, :] *= -1
                delta_rotation = vt.T @ u.T
            delta_translation = dst_center - delta_rotation @ src_center
            updated_rotation = delta_rotation @ rotation
            pose[:2] = delta_rotation @ pose[:2] + delta_translation
            pose[2] = math.atan2(updated_rotation[1, 0], updated_rotation[0, 0])
            if float(np.linalg.norm(delta_translation)) < 0.002 and abs(math.atan2(delta_rotation[1, 0], delta_rotation[0, 0])) < math.radians(0.15):
                break
        return pose.tolist()

    def update_map(self):
        local_points = self._local_xy(self.points)
        if len(local_points) < 10:
            return
        if self.map_points:
            self.pose = self._match_pose(local_points)
        px, py, pa = self.pose
        ca, sa = math.cos(pa), math.sin(pa)
        for lx, ly in local_points:
            self.map_points.append((px + ca * lx - sa * ly, py + sa * lx + ca * ly))
        if len(self.map_points) > 18000:
            self.map_points = self.map_points[-18000:]
        self._rebuild_map_index()
        self.stat_map.config(text=f"{len(self.map_points):,}")

    def capture_map(self):
        capture_points = self.map_points if self.mapping_enabled and self.map_points else self.points
        if not capture_points:
            messagebox.showinfo("Capture map", "No scan points are available yet. Start the scanner first.")
            return
        stamp = time.strftime("%Y%m%d-%H%M%S")
        path = filedialog.asksaveasfilename(
            title="Capture current 2D map",
            initialfile=f"rplidar-map-{stamp}.svg",
            defaultextension=".svg",
            filetypes=[("SVG map", "*.svg"), ("All files", "*.*")],
        )
        if not path:
            return
        width, height = max(1, self.canvas.winfo_width()), max(1, self.canvas.winfo_height())
        ox, oy = self._origin()
        parts = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            '<rect width="100%" height="100%" fill="#0b1020"/>',
            '<g fill="none" stroke="#233052" stroke-width="1">',
        ]
        if self.mapping_enabled and self.map_points:
            max_m = max(4, math.ceil(max((math.hypot(x, y) for x, y in self.map_points), default=4) / 2) * 2)
        else:
            max_m = max(4, math.ceil(max((p[1] for p in capture_points), default=4) / 2) * 2)
        for radius_m in range(1, max_m + 1):
            r = radius_m * self.scale
            if r > max(width, height):
                break
            parts.append(f'<circle cx="{ox:.1f}" cy="{oy:.1f}" r="{r:.1f}"/>')
        parts += ['</g>', f'<g stroke="#263457"><line x1="0" y1="{oy:.1f}" x2="{width}" y2="{oy:.1f}"/><line x1="{ox:.1f}" y1="0" x2="{ox:.1f}" y2="{height}"/></g>', '<g fill="#64e5b0">']
        csv_rows = ["angle_deg,distance_m,quality,x_m,y_m"]
        if self.mapping_enabled and self.map_points:
            export_points = [(0.0, 0.0, 15, x, y) for x, y in capture_points]
        else:
            export_points = []
            for angle, dist, quality in capture_points:
                rad = math.radians(angle)
                export_points.append((angle, dist, quality, math.sin(rad) * dist, math.cos(rad) * dist))
        for angle, dist, quality, x_m, y_m in export_points:
            x, y = ox + x_m * self.scale, oy - y_m * self.scale
            parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="2"/>')
            csv_rows.append(f"{angle:.3f},{math.hypot(x_m, y_m):.4f},{quality},{x_m:.4f},{y_m:.4f}")
        parts += ['</g>', f'<g fill="#f5b95b" font-family="Helvetica" font-size="12"><text x="{ox+12:.1f}" y="{oy-12:.1f}">X / Y metres · Z unavailable</text></g>', '</svg>']
        with open(path, "w", encoding="utf-8") as output:
            output.write("\n".join(parts))
        csv_path = os.path.splitext(path)[0] + ".csv"
        with open(csv_path, "w", encoding="utf-8") as output:
            output.write("\n".join(csv_rows) + "\n")
        self.capture_note = f"Captured {len(capture_points)} points → {os.path.basename(path)} + CSV"
        self.draw()

    def zoom(self, event):
        direction = 1 if getattr(event, "delta", 0) > 0 or getattr(event, "num", 0) == 4 else -1
        self.scale = max(25, min(260, self.scale * (1.12 if direction > 0 else 0.89)))
        self.draw()

    def pan_start(self, event): self.drag_origin = (event.x, event.y, self.pan_x, self.pan_y)
    def pan_move(self, event):
        if self.drag_origin:
            x, y, px, py = self.drag_origin; self.pan_x, self.pan_y = px + event.x-x, py + event.y-y; self.draw()
    def reset_view(self, _=None): self.scale, self.pan_x, self.pan_y = 90.0, 0, 0; self.draw()
    def close(self): self.stop(); self.destroy()


if __name__ == "__main__":
    LidarMap().mainloop()
