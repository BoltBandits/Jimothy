#!/usr/bin/env python3
"""Transfer the two edited intake cubics into the existing autonomous controller.

Only control-point edits are supported: endpoints, headings, other movements,
and waits are checked against the reviewed mechanism sequence before writing.
The production robot keeps its existing local odometry origin and calibration.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "visualizations/working/main-bezier.pp.json"
OUTPUT = ROOT / "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MainAutoCurves.java"
CURVE_IDS = ("first-intake-bezier", "second-intake-bezier")
# Local targets from the reviewed runOpMode, rounded to match the preview file.
TARGETS = {
    "first-intake-bezier": (-9.7, -47.38, 2.7),
    "main-nominal-8": (2.286678, -46.814723, 2.7),
    "main-nominal-9": (-2.990410, -41.056916, 0),
    "main-nominal-10": (-2.990410, -41.056916, 90),
    "main-nominal-11": (-2.990410, -41.056916, 180),
    "second-intake-bezier": (-121.07, -15.38, -170),
    "main-nominal-14": (-121.417296, -13.410384, -170),
    "main-nominal-17": (-111.569219, -11.673903, -170),
    "main-nominal-19": (-121.243648, -14.395192, -170),
    "main-nominal-21": (-118.289225, -13.874248, -170),
    "main-nominal-22": (-121.243648, -14.395192, -170),
    "main-nominal-24": (-109.93, -47.73, 90),
    "main-nominal-25": (-109.93, -52.73, 90),
}
SEQUENCE = [
    ("wait", "main-nominal-1", 0), ("wait", "main-nominal-2", 4000),
    ("path", CURVE_IDS[0]), ("wait", "main-nominal-7", 250),
    *[("path", f"main-nominal-{i}") for i in (8, 9, 10, 11)],
    ("path", CURVE_IDS[1]), ("path", "main-nominal-14"),
    ("wait", "main-nominal-15", 0), ("wait", "main-nominal-16", 4000),
    ("path", "main-nominal-17"), ("wait", "main-nominal-18", 250),
    ("path", "main-nominal-19"), ("wait", "main-nominal-20", 500),
    ("path", "main-nominal-21"), ("path", "main-nominal-22"),
    ("wait", "main-nominal-23", 4000),
    ("path", "main-nominal-24"), ("path", "main-nominal-25"),
]


def require(condition, message):
    if not condition:
        raise ValueError(message + "; review the autonomous actions before importing")


def local_point(point, start):
    dx, dy = point["x"] - start["x"], point["y"] - start["y"]
    angle = math.radians(start["headingDeg"])
    return (dx * math.cos(angle) + dy * math.sin(angle),
            -dx * math.sin(angle) + dy * math.cos(angle))


def angle_difference(a, b):
    return (a - b + 180) % 360 - 180


def read_project(path=PROJECT):
    doc = json.loads(path.read_text())
    require(doc["version"] == "2.0.0", "Unsupported project version")
    start = doc["startPoint"]
    require(all(math.isfinite(start[k]) for k in ("x", "y", "headingDeg")), "Invalid start")
    require(abs(angle_difference(start["headingDeg"], 270)) < 1e-6, "Changed starting orientation")
    require(not doc.get("activePaths"), "Parallel paths are unsupported")
    lines = doc["lines"]
    require([line["id"] for line in lines] == list(TARGETS), "Changed motion order")
    sequence = [((s["kind"], s["lineId"]) if s["kind"] == "path" else
                 (s["kind"], s.get("id"), s.get("durationMs"))) for s in doc["sequence"]]
    require(sequence == SEQUENCE, "Changed sequence or wait timing")
    previous, curves = start, {}
    for line in lines:
        identifier = line["id"]
        require(line["kind"] == "atomic" and not line.get("throughPoints"), "Unsupported curve type")
        require(not line.get("waitBeforeMs", 0) and not line.get("waitAfterMs", 0), "Changed path wait")
        points = [previous, *line["controlPoints"], line["endPoint"]]
        require(all(math.isfinite(p[k]) for p in points for k in ("x", "y")), "Non-finite coordinate")
        expected = TARGETS[identifier]
        end = local_point(line["endPoint"], start)
        require(math.dist(end, expected[:2]) < 1e-5, f"Changed endpoint: {line['name']}")
        heading = line["heading"]
        if identifier == CURVE_IDS[0]:
            require(heading == {"type": "linear", "startDeg": 270, "endDeg": 272.7},
                    "Changed first-approach heading")
            start_h, end_h = 0.0, 2.7
        else:
            require(heading["type"] == "constant"
                    and abs(angle_difference(heading["degrees"] - start["headingDeg"], expected[2])) < 1e-6,
                    f"Changed heading: {line['name']}")
            start_h = end_h = expected[2]
        require(len(line["controlPoints"]) == (2 if identifier in CURVE_IDS else 0),
                f"Changed control-point count: {line['name']}")
        if identifier in CURVE_IDS:
            curves[identifier] = dict(points=[local_point(p, start) for p in points],
                                      startHeading=start_h, endHeading=end_h)
        previous = line["endPoint"]
    return doc, curves


def render(path=PROJECT):
    doc, curves = read_project(path)
    # Ignore presentation preferences and include exactly the imported geometry.
    geometry = dict(start={key: doc["startPoint"][key] for key in ("x", "y", "headingDeg")},
                    curves=curves)
    digest = hashlib.sha256(json.dumps(geometry, sort_keys=True).encode()).hexdigest()
    def number(value):
        rounded = round(value, 12)
        return repr(0.0 if rounded == 0 else rounded)
    parts = ["package org.firstinspires.ftc.teamcode;\n\n",
             "/** Generated from visualizations/working/main-bezier.pp.json.\n",
             " * Run python3 scripts/import_main_auto_curves.py after editing control points.\n",
             " * Other motion/heading/timing edits require reviewing the mechanism sequence.\n",
             " */\nfinal class MainAutoCurves {\n",
             f'    static final String GEOMETRY_SHA256 = "{digest}";\n']
    for key, name in (("x", "X"), ("y", "Y"), ("headingDeg", "HEADING_DEGREES")):
        parts.append(f"    static final double FIELD_START_{name} = {number(doc['startPoint'][key])};\n")
    for identifier, name in zip(CURVE_IDS, ("FIRST_INTAKE", "SECOND_INTAKE")):
        curve = curves[identifier]
        parts.append(f"\n    static final AutoBezierCurve {name} = new AutoBezierCurve(\n")
        for x, y in curve["points"]:
            parts.append(f"            {number(x)}, {number(y)},\n")
        parts.append(f"            {number(curve['startHeading'])}, {number(curve['endHeading'])});\n")
    parts.append("\n    private MainAutoCurves() { }\n}\n")
    return "".join(parts)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=PROJECT)
    parser.add_argument("--check", action="store_true", help="Fail if Java is out of sync; write nothing")
    args = parser.parse_args()
    generated = render(args.project)
    if args.check:
        require(OUTPUT.exists() and OUTPUT.read_text() == generated, "Java curves need regeneration")
        print("Autonomous intake curves match the visualizer project.")
    else:
        OUTPUT.write_text(generated)
        print(f"Updated {OUTPUT.relative_to(ROOT)} from {args.project}")
