# LUTUM Rewind

Rescue a clay 3D print after a clay-feed dropout. When the auger runs dry for a moment you lose a layer or two:
the printer keeps moving, but nothing comes out. This tool restarts the print from the point where the bead
actually stopped.

It is written for LUTUM printers, but it should work with any printer running a Duet board with
RepRapFirmware 3.x (standalone, not an SBC). **It has not yet been tested on real hardware. Try it on a scrap
print first.**

## How it works

1. Pause the print. Fix the clay feed and prime the nozzle by hand.
2. Jog the nozzle onto the **end of the last good bead**, just touching its top.
3. The tool reads the paused job and the nozzle position from the printer, finds that point in the G-code
   (every pass of the toolpath under the nozzle, ranked by distance; the lost layers show up as passes
   above the nozzle), and writes a small macro, `0:/sys/lutum-rewind.g`, that:
   - lifts, travels above the point and lowers onto it,
   - restores extrusion mode, extruder position, feedrate, tool and pressure advance,
   - finishes the interrupted move from that exact point,
   - seeks into the original file (`M23` / `M26 S<byte offset>` / `M24`) and continues.
4. It uploads that file, cancels the paused job and runs it with `M98`. The firmware's own power-loss file
   `resurrect.g` is left alone, and `resurrect-prologue.g` isn't needed (unlike with `M916`).

Only a few hundred bytes are uploaded, so you aren't waiting minutes for a big file over the Duet's WiFi
while the clay dries.

## Web page (recommended)

`rewind.html` is a single self-contained page. Put it **on the printer** so it works on the printer's own
WiFi with no internet:

- copy it to the `www` folder on the printer's SD card, or
- `python3 lutum_rewind.py --host <printer-ip> --install-page`

Then open `http://<printer-ip>/rewind.html`.

The copy on GitHub Pages can't talk to a printer, because browsers block https pages from calling plain-http
devices. It still works offline: load G-code, type a position, and generate the resume file to copy over yourself.

## Command line

```
python3 lutum_rewind.py --host <printer-ip> [--password <pw>] [--gcode <local copy of the job>]
```

To try it with no printer: `python3 lutum_rewind.py --gcode part.gcode --touch X,Y,Z`.
Run `--help` for the options (Z shift, priming, lift height, full-file mode). No dependencies beyond Python 3.8+.

## Caveats

- After cancelling, don't home or move the axes before resuming.
- A Z shift is applied with `G92 Z` and stays in effect until the axes are homed again.
- If you touch exactly at a perimeter's seam, the loop's start and end score the same. Check the line number.
- Arc moves (`G2`/`G3`) are matched as straight chords and resumed from the start of the arc.
