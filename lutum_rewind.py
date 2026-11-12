#!/usr/bin/env python3
"""
lutum_rewind.py - rescue a clay print on a Duet/RepRapFirmware LUTUM after a
clay-feed dropout by restarting it from where extrusion actually stopped.

Workflow
--------
  1. Pause the print (DWC or PanelDue).
  2. Fix the clay feed and prime the nozzle by hand.
  3. Jog the nozzle so it sits on the END of the last good bead, just touching
     its top surface (that point is where the new print will start).
  4. Run:   python3 lutum_rewind.py --host <printer-ip> [--gcode <local copy>]
     The script reads the paused job and the nozzle position, finds that point
     in the G-code, and writes a new file that:
       - lifts, travels above the start point, lowers onto it,
       - restores extrusion mode / feedrate / tool / pressure advance etc.,
       - finishes the interrupted move from the exact point, then
       - continues with the rest of the original file verbatim.
  5. It offers to upload that file, then to cancel the paused job and start it.

Offline / dry run (no printer):
     python3 lutum_rewind.py --gcode part.gcode --touch 120.5,88.2,31.5

No third-party packages needed (Python 3.8+).
"""

import argparse
import json
import math
import os
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

CACHE_DIR = os.path.expanduser("~/.cache/lutum_rewind")
WORD_RE = re.compile(r"([A-Z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")

# Modal commands whose last value before the cut is re-issued in the header,
# keyed so that per-extruder settings (D/P parameter) are kept separately.
MODAL_CMDS = {"G20", "G21", "M572", "M221", "M220", "M204", "M201", "M203", "M566"}


# --------------------------------------------------------------------------
# Duet HTTP API (standalone RRF, the "rr_" interface used by DWC)
# --------------------------------------------------------------------------
class Duet:
    def __init__(self, host, password, timeout=15):
        if not host.startswith("http"):
            host = "http://" + host
        self.base = host.rstrip("/")
        self.password = password
        self.timeout = timeout
        self.session_key = None

    def _open(self, path, params=None, data=None, timeout=None, retry=True):
        url = self.base + path
        if params:
            url += "?" + urllib.parse.urlencode(params)
        req = urllib.request.Request(url, data=data, method="POST" if data is not None else "GET")
        if self.session_key:
            req.add_header("X-Session-Key", str(self.session_key))
        if data is not None:
            req.add_header("Content-Type", "application/octet-stream")
            req.add_header("Content-Length", str(len(data)))
        try:
            return urllib.request.urlopen(req, timeout=timeout or self.timeout)
        except urllib.error.HTTPError as e:
            if e.code == 401 and retry and path != "/rr_connect":
                self.connect()
                return self._open(path, params, data, timeout, retry=False)
            raise

    def _json(self, path, params=None, **kw):
        with self._open(path, params, **kw) as r:
            return json.loads(r.read().decode("utf-8", "replace"))

    def connect(self):
        t = time.strftime("%Y-%m-%dT%H:%M:%S")
        r = self._json("/rr_connect", {"password": self.password, "time": t})
        if r.get("err"):
            raise SystemExit("rr_connect failed (err=%s): wrong password?" % r.get("err"))
        self.session_key = r.get("sessionKey")
        return r

    def model(self, key):
        r = self._json("/rr_model", {"key": key, "flags": "d99vn"})
        return r.get("result")

    def status(self):
        return self.model("state.status")

    def gcode(self, cmd):
        self._json("/rr_gcode", {"gcode": cmd})
        time.sleep(0.3)
        with self._open("/rr_reply") as r:
            return r.read().decode("utf-8", "replace").strip()

    def download(self, name, dest, expected_size=None):
        tmp = dest + ".part"
        with self._open("/rr_download", {"name": name}, timeout=120) as r, open(tmp, "wb") as f:
            total = expected_size or int(r.headers.get("Content-Length") or 0)
            got, t0 = 0, time.time()
            while True:
                chunk = r.read(1 << 16)
                if not chunk:
                    break
                f.write(chunk)
                got += len(chunk)
                if total:
                    rate = got / max(time.time() - t0, 1e-3) / 1024
                    print("\r  downloading %5.1f%%  (%d kB/s)" % (100.0 * got / total, rate), end="", flush=True)
        print()
        if expected_size and got != expected_size:
            os.remove(tmp)
            raise SystemExit("download incomplete: %d of %d bytes" % (got, expected_size))
        os.replace(tmp, dest)

    def upload(self, name, src):
        with open(src, "rb") as f:
            data = f.read()
        print("  uploading %d kB to %s ..." % (len(data) // 1024, name))
        t = time.strftime("%Y-%m-%dT%H:%M:%S")
        r = self._json("/rr_upload", {"name": name, "time": t}, data=data, timeout=900)
        if r.get("err"):
            raise SystemExit("upload failed: %r" % r)


# --------------------------------------------------------------------------
# G-code scanning
# --------------------------------------------------------------------------
def parse_words(code):
    """Return (command, {letter: float}) for one line of code (comment stripped)."""
    words = WORD_RE.findall(code.upper())
    if words and words[0][0] == "N":
        words = words[1:]
    if not words:
        return None, {}
    letter, num = words[0]
    if letter in "GMT":
        try:
            cmd = letter + str(int(float(num)))
        except ValueError:
            cmd = letter + num
        params = {}
        for l, v in words[1:]:
            params.setdefault(l, float(v))
        return cmd, params
    return None, {}


def seg_xy_project(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L2))
    qx, qy = x0 + t * dx, y0 + t * dy
    return t, math.hypot(px - qx, py - qy)


def scan(path, touch, limit=None, tol=4.0):
    """
    Walk the file up to byte offset `limit`, tracking machine state, and
    collect 'passes': runs of consecutive extruding moves whose XY path comes
    within `tol` mm of the touch point.  Each pass keeps its best point
    together with a snapshot of the modal state needed to resume there.
    """
    tx, ty, tz = touch
    st = dict(x=None, y=None, z=None, e=0.0, f=None, abs_xyz=True, abs_e=False,
              tool=None, layer=0, inches=False)
    modal = {}
    passes = []
    cur = None          # pass currently being extended
    last_cand_move = -10
    move_idx = 0
    nearest = None      # global 3-D nearest, fallback
    warnings = set()

    off = 0
    line_no = 0
    with open(path, "rb") as fh:
        for raw in fh:
            line_no += 1
            start = off
            off += len(raw)
            if limit is not None and start >= limit:
                break
            text = raw.decode("ascii", "replace")
            code, _, comment = text.partition(";")
            c = comment.strip().upper()
            if c.startswith("LAYER_CHANGE") or c.startswith("LAYER:"):
                st["layer"] += 1
            if not code.strip():
                continue
            cmd, p = parse_words(code)
            if cmd is None:
                continue

            if cmd in ("G0", "G1", "G2", "G3"):
                move_idx += 1
                if "F" in p:
                    st["f"] = p["F"]
                x0, y0, z0, e0 = st["x"], st["y"], st["z"], st["e"]
                nxt = {}
                for a in "XYZ":
                    k = a.lower()
                    if a in p:
                        if st["abs_xyz"]:
                            nxt[k] = p[a]
                        else:
                            nxt[k] = None if st[k] is None else st[k] + p[a]
                    else:
                        nxt[k] = st[k]
                de = 0.0
                if "E" in p:
                    if st["abs_e"]:
                        de = p["E"] - e0
                        st["e"] = p["E"]
                    else:
                        de = p["E"]
                        st["e"] = e0 + de
                x1, y1, z1 = nxt["x"], nxt["y"], nxt["z"]
                st["x"], st["y"], st["z"] = x1, y1, z1

                known = None not in (x0, y0, z0, x1, y1, z1)
                moved = known and (abs(x1 - x0) + abs(y1 - y0) + abs(z1 - z0) > 1e-9)
                if not (de > 1e-9 and moved):
                    continue
                if cmd in ("G2", "G3"):
                    warnings.add("arc moves (G2/G3) present: matched as straight chords")

                t, dxy = seg_xy_project(tx, ty, x0, y0, x1, y1)
                zt = z0 + t * (z1 - z0)
                d3 = math.hypot(dxy, zt - tz)
                if nearest is None or d3 < nearest[0]:
                    nearest = (d3, dict(line_no=line_no, start=start, end=off, t=t, dxy=dxy,
                                        a=(x0, y0, z0), b=(x1, y1, z1), e0=e0, de=de, cmd=cmd,
                                        f=st["f"], abs_e=st["abs_e"], abs_xyz=st["abs_xyz"],
                                        tool=st["tool"], layer=st["layer"], modal=dict(modal),
                                        z_at=zt, d3=d3, text=text.rstrip("\r\n")))
                if dxy > tol:
                    continue
                cand = dict(line_no=line_no, start=start, end=off, t=t, dxy=dxy,
                            a=(x0, y0, z0), b=(x1, y1, z1), e0=e0, de=de, cmd=cmd,
                            f=st["f"], abs_e=st["abs_e"], abs_xyz=st["abs_xyz"],
                            tool=st["tool"], layer=st["layer"], modal=None,
                            z_at=zt, d3=d3, text=text.rstrip("\r\n"))
                if cur is not None and last_cand_move == move_idx - 1:
                    if d3 < cur["d3"]:
                        cand["modal"] = dict(modal)
                        cur.update(cand)
                else:
                    cand["modal"] = dict(modal)
                    cur = cand
                    passes.append(cur)
                last_cand_move = move_idx
                continue

            if cmd == "G90":
                st["abs_xyz"] = True
            elif cmd == "G91":
                st["abs_xyz"] = False
            elif cmd == "M82":
                st["abs_e"] = True
            elif cmd == "M83":
                st["abs_e"] = False
            elif cmd == "G92":
                if not p:
                    st["x"] = st["y"] = st["z"] = 0.0
                    st["e"] = 0.0
                for a in "XYZE":
                    if a in p:
                        st[a.lower()] = p[a]
            elif cmd == "G28":
                axes = [a for a in "XYZ" if a in p] or list("XYZ")
                for a in axes:
                    st[a.lower()] = None
            elif cmd == "G20":
                warnings.add("file uses inches (G20): not supported, results will be wrong")
            elif cmd.startswith("T") and cmd[1:].lstrip("-").isdigit():
                st["tool"] = cmd
            if cmd in MODAL_CMDS:
                key = cmd if cmd not in ("G20", "G21") else "units"
                if "D" in p:
                    key += "D%g" % p["D"]
                modal[key] = code.strip()
    return passes, nearest[1] if nearest else None, sorted(warnings), off


# --------------------------------------------------------------------------
# Building the resume file
# --------------------------------------------------------------------------
def fmt(v):
    s = "%.4f" % v
    s = s.rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def build_resume(cand, opts, source_name, touch):
    """Return (header_lines, resume_byte_offset, start_xyz, zshift, mode)."""
    if not cand["abs_xyz"]:
        raise SystemExit("The chosen point lies inside a G91 (relative XYZ) block; cannot resume there.")
    (x0, y0, z0), (x1, y1, z1) = cand["a"], cand["b"]
    t = cand["t"]
    if cand["cmd"] in ("G2", "G3") or t < 0.01:
        mode, t = "whole", 0.0
    elif t > 0.99:
        mode, t = "skip", 1.0
    else:
        mode = "split"
    sx, sy, sz = x0 + t * (x1 - x0), y0 + t * (y1 - y0), z0 + t * (z1 - z0)
    e_start = cand["e0"] + t * cand["de"]
    f = cand["f"]

    zshift = opts.z_shift
    if opts.z_from_touch:
        zshift += touch[2] - sz
    lift = opts.lift

    H = []
    H.append("; ==== LUTUM rewind resume file ====")
    H.append("; source       : %s" % source_name)
    H.append("; resumes at   : line %d (byte %d), layer %s, %s of that move"
             % (cand["line_no"], cand["start"], cand["layer"] or "?", "%.0f%%" % (t * 100)))
    H.append("; original line: %s" % cand["text"].strip())
    H.append("; gcode point  : X%s Y%s Z%s" % (fmt(sx), fmt(sy), fmt(sz)))
    H.append("; nozzle touch : X%s Y%s Z%s" % tuple(fmt(v) for v in touch))
    H.append("; z shift      : %s mm" % fmt(zshift))
    H.append("; generated    : %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    H.append("G21")
    for k in sorted(cand["modal"] or {}):
        if k != "units":
            H.append(cand["modal"][k] + "   ; restored modal setting")
    if cand["tool"]:
        H.append(cand["tool"] + "   ; restore tool")
    H.append("G90")
    H.append("M83")
    H.append("G91")
    H.append("G1 Z%s F%s   ; lift clear of the print" % (fmt(lift), fmt(opts.z_feed)))
    H.append("G90")
    H.append("G1 X%s Y%s F%s   ; travel above the resume point" % (fmt(sx), fmt(sy), fmt(opts.travel_feed)))
    H.append("G1 Z%s F%s   ; lower onto the resume point" % (fmt(sz + zshift), fmt(opts.z_feed)))
    if abs(zshift) > 1e-6:
        H.append("G92 Z%s   ; shift Z so the rest of the file lines up with the actual print" % fmt(sz))
    if opts.prime > 0:
        H.append("G1 E%s F%s   ; prime" % (fmt(opts.prime), fmt(opts.prime_feed)))
    if opts.dwell > 0:
        H.append("G4 S%s" % fmt(opts.dwell))
    if cand["abs_e"]:
        H.append("M82")
        H.append("G92 E%s   ; extruder position at the resume point" % fmt(e_start))
    if f:
        H.append("G1 F%s   ; print feedrate in effect at the resume point" % fmt(f))
    H.append("; ---- continuing original file ----")

    if mode == "split":
        e_word = cand["e0"] + cand["de"] if cand["abs_e"] else cand["de"] * (1 - t)
        H.append("G1 X%s Y%s Z%s E%s%s   ; remainder of the interrupted move"
                 % (fmt(x1), fmt(y1), fmt(z1), fmt(e_word), (" F" + fmt(f)) if f else ""))
        resume_off = cand["end"]
    elif mode == "skip":
        resume_off = cand["end"]
    else:
        resume_off = cand["start"]
    return H, resume_off, (sx, sy, sz), zshift, mode


def write_full_file(src, H, resume_off, out_path):
    """Header + the rest of the original file: a standalone job to print."""
    with open(src, "rb") as fi, open(out_path, "wb") as fo:
        fo.write(("\n".join(H) + "\n").encode("ascii"))
        fi.seek(resume_off)
        while True:
            b = fi.read(1 << 20)
            if not b:
                break
            fo.write(b)


def write_resurrect(H, resume_off, job_name, out_path):
    """
    A small macro (0:/sys/lutum-rewind.g, run with M98) in the same shape as
    the resurrect.g RRF writes after a pause or power loss: set up, select the
    ORIGINAL file, seek to the byte offset and resume it.  Only a few hundred
    bytes go over WiFi.  Not run via M916, which requires resurrect-prologue.g
    to exist and would overwrite the firmware's power-loss file.
    """
    lines = list(H)
    lines.insert(1, '; run with M98 P"0:/sys/lutum-rewind.g" after cancelling the paused job')
    lines.append('M23 "%s"' % job_name)
    lines.append("M26 S%d" % resume_off)
    lines.append("M24")
    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")


def show_context(path, line_no, before=3, after=3):
    lo, hi = line_no - before, line_no + after
    with open(path, "rb") as f:
        for i, raw in enumerate(f, 1):
            if i < lo:
                continue
            if i > hi:
                break
            mark = ">>" if i == line_no else "  "
            print("   %s %7d  %s" % (mark, i, raw.decode("ascii", "replace").rstrip()))


# --------------------------------------------------------------------------
def ask(prompt, default=False, assume_yes=False):
    if assume_yes:
        print(prompt + " yes (--yes)")
        return True
    d = "Y/n" if default else "y/N"
    a = input("%s [%s] " % (prompt, d)).strip().lower()
    return default if not a else a.startswith("y")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=os.environ.get("LUTUM_HOST"), help="printer IP / hostname (or env LUTUM_HOST)")
    ap.add_argument("--password", default=os.environ.get("LUTUM_PASSWORD", "reprap"),
                    help="DWC password (or env LUTUM_PASSWORD)")
    ap.add_argument("--gcode", help="local copy of the job file (skips the download; must be identical)")
    ap.add_argument("--touch", help="offline: X,Y,Z nozzle position instead of asking the printer")
    ap.add_argument("--limit", type=int, help="offline: only search before this byte offset")
    ap.add_argument("--tol", type=float, default=4.0, help="XY match tolerance in mm (default 4)")
    ap.add_argument("--lift", type=float, default=5.0, help="lift before travelling to the start point (mm, default 5)")
    ap.add_argument("--z-shift", type=float, default=0.0,
                    help="extra Z offset for the rest of the print, e.g. -0.5 to squash onto a slumped layer")
    ap.add_argument("--z-from-touch", action="store_true",
                    help="use the measured nozzle Z instead of the G-code Z (shifts the rest of the print)")
    ap.add_argument("--prime", type=float, default=0.0, help="extruder mm to push before moving (default 0)")
    ap.add_argument("--prime-feed", type=float, default=300.0)
    ap.add_argument("--dwell", type=float, default=0.0, help="seconds to wait after priming")
    ap.add_argument("--travel-feed", type=float, default=3000.0)
    ap.add_argument("--z-feed", type=float, default=600.0)
    ap.add_argument("--method", choices=["macro", "file"], default="macro",
                    help="macro (default): upload a tiny 0:/sys/lutum-rewind.g that seeks into the original "
                         "file and run it with M98.  file: upload the whole trimmed remainder as a new job.")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "out"))
    ap.add_argument("--yes", action="store_true", help="answer yes to every question (careful)")
    ap.add_argument("--install-page", action="store_true",
                    help="upload rewind.html to the printer's 0:/www/ so it is at http://<printer>/rewind.html")
    opts = ap.parse_args()

    if opts.install_page:
        if not opts.host:
            raise SystemExit("--install-page needs --host")
        page = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rewind.html")
        duet = Duet(opts.host, opts.password)
        duet.connect()
        duet.upload("0:/www/rewind.html", page)
        host = opts.host if opts.host.startswith("http") else "http://" + opts.host
        print("installed. Open %s/rewind.html while on the printer's WiFi." % host.rstrip("/"))
        return

    duet = None
    job_name = None
    limit = opts.limit
    if opts.touch:
        touch = tuple(float(v) for v in opts.touch.split(","))
        if len(touch) != 3:
            raise SystemExit("--touch needs X,Y,Z")
        if not opts.gcode:
            raise SystemExit("offline mode needs --gcode")
        src = opts.gcode
        job_name = os.path.basename(src)
    else:
        if not opts.host:
            raise SystemExit("give --host <printer ip> (or --touch for an offline dry run)")
        duet = Duet(opts.host, opts.password)
        info = duet.connect()
        print("connected to %s (%s)" % (opts.host, info.get("boardType", "?")))
        status = duet.status()
        print("printer status:", status)
        if status != "paused" and not ask("Printer is not paused. Continue anyway?", False, opts.yes):
            return
        job = duet.model("job") or {}
        jf = job.get("file") or {}
        job_name = jf.get("fileName")
        size = jf.get("size")
        limit = job.get("filePosition")
        if not job_name:
            raise SystemExit("no job file reported by the printer")
        axes = duet.model("move.axes") or []
        pos = {a["letter"]: a.get("userPosition") for a in axes}
        touch = (pos["X"], pos["Y"], pos["Z"])
        print("job: %s  (%s bytes, read up to byte %s, layer %s)" % (job_name, size, limit, job.get("layer")))

        if opts.gcode:
            src = opts.gcode
            lsize = os.path.getsize(src)
            if size and lsize != size:
                print("WARNING: local file is %d bytes, printer's is %d bytes; they are not the same file." % (lsize, size))
                if not ask("Use the local file anyway?", False, opts.yes):
                    return
        else:
            os.makedirs(CACHE_DIR, exist_ok=True)
            base = os.path.basename(job_name.split(":", 1)[-1])
            src = os.path.join(CACHE_DIR, "%s.%s" % (size, base))
            if os.path.exists(src) and os.path.getsize(src) == size:
                print("using cached copy", src)
            else:
                print("downloading job file from printer (use --gcode to skip this) ...")
                duet.download(job_name, src, size)

    print("nozzle position: X%.3f Y%.3f Z%.3f" % touch)
    t0 = time.time()
    passes, nearest, warnings, scanned = scan(src, touch, limit, opts.tol)
    print("scanned %d kB in %.1fs" % (scanned // 1024, time.time() - t0))
    for w in warnings:
        print("WARNING:", w)

    if not passes:
        if not nearest:
            raise SystemExit("no extrusion moves found before the current file position")
        print("No toolpath passes within %.1f mm in XY. Nearest extrusion is %.1f mm away (line %d)."
              % (opts.tol, nearest["d3"], nearest["line_no"]))
        print("Jog closer to the end of the bead, or raise --tol.")
        return

    ranked = sorted(passes, key=lambda c: c["d3"])[:8]
    print()
    print("Toolpath passes under the nozzle (best first):")
    print("   #   dist   dXY    gcodeZ  touch-gcode  layer     line   file%")
    total = os.path.getsize(src)
    for i, c in enumerate(ranked, 1):
        print("  %2d  %5.2f  %5.2f  %8.3f  %+9.3f   %5s  %8d  %5.1f"
              % (i, c["d3"], c["dxy"], c["z_at"], touch[2] - c["z_at"], c["layer"] or "-",
                 c["line_no"], 100.0 * c["start"] / total))
    if len(ranked) > 1 and abs(ranked[0]["d3"] - ranked[1]["d3"]) < 0.25:
        print("  NOTE: the top two are nearly tied; check the layer before accepting.")
    choice = 1
    if not opts.yes:
        a = input("\nResume from which pass? [1] ").strip()
        if a:
            choice = int(a)
    cand = ranked[choice - 1]
    print("\nCut point in the original file:")
    show_context(src, cand["line_no"])

    os.makedirs(opts.out, exist_ok=True)
    H, resume_off, (sx, sy, sz), zshift, mode = build_resume(cand, opts, job_name, touch)
    base = os.path.basename(job_name.split(":", 1)[-1])
    if opts.method == "file":
        out_name = "rewind_" + base
        out_path = os.path.join(opts.out, out_name)
        write_full_file(src, H, resume_off, out_path)
    else:
        out_name = "lutum-rewind.g"
        out_path = os.path.join(opts.out, "lutum-rewind_" + base + ".g")
        write_resurrect(H, resume_off, job_name if ":" in job_name else "0:/gcodes/" + job_name, out_path)
    print("\nresume point X%.3f Y%.3f Z%.3f  (Z shift %+.3f, %s), file offset %d"
          % (sx, sy, sz, zshift, mode, resume_off))
    print("wrote", out_path, "(%d bytes)" % os.path.getsize(out_path))
    if abs(touch[2] - sz) > 0.5 and not opts.z_from_touch:
        print("NOTE: nozzle Z differs from the G-code Z by %+.2f mm. Consider --z-from-touch or --z-shift."
              % (touch[2] - sz))

    if duet is None:
        return
    if opts.method == "file":
        remote_dir = job_name.rsplit("/", 1)[0] if "/" in job_name else "0:/gcodes"
        remote = remote_dir + "/" + out_name
    else:
        remote = "0:/sys/lutum-rewind.g"
    if not ask("\nUpload to printer as %s ?" % remote, True, opts.yes):
        return
    duet.connect()
    duet.upload(remote, out_path)
    print("  uploaded.")

    start_cmd = ('M32 "%s"' if opts.method == "file" else 'M98 P"%s"') % remote
    if not ask("Cancel the paused job and resume from the chosen point now (%s)?" % start_cmd, False, opts.yes):
        print("OK. To start it later: cancel the paused job in DWC, then send %s from the console." % start_cmd)
        print("Do not move or home the axes in between.")
        return
    duet.connect()
    if duet.status() == "paused":
        duet.gcode("M0")
        for _ in range(60):
            time.sleep(0.5)
            if duet.status() == "idle":
                break
        else:
            raise SystemExit("printer did not go idle after cancelling; send %s manually." % start_cmd)
    reply = duet.gcode(start_cmd)
    if reply:
        print("  printer replied:", reply)
    if reply.lower().startswith("error"):
        raise SystemExit("The printer refused %s. The job is cancelled; if nothing has moved, fix the cause "
                         "and send %s from the console." % (start_cmd, start_cmd))
    time.sleep(1.5)
    print("printer status:", duet.status())


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\naborted")
