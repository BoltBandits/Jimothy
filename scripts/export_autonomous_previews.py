#!/usr/bin/env python3
"""Export reviewed autonomous command flows to official Pedro visualizer projects.

This is a set of adapters for Jimothy's current Java routines, not a Java
interpreter or a drivetrain simulator. Numeric targets come from source;
changes to command flow require reviewing the adapters and their tests.
"""

import argparse
import ast
import hashlib
import json
import math
import operator
from pathlib import Path
import re
import subprocess

import import_main_auto_curves as intake_curves


ROOT = Path(__file__).resolve().parents[1]
JAVA = ROOT / "TeamCode/src/main/java/org/firstinspires/ftc/teamcode"
COLORS = ["#ffb45d", "#5fe0cf", "#c4a1ff", "#ff7698", "#92db72"]


def require(condition, message):
    if not condition:
        raise ValueError(message + "; review the preview adapter before exporting")


def arguments(text):
    """Split Java arguments without splitting nested calls or quoted strings."""
    result, start, depth, quoted, escaped = [], 0, 0, False, False
    for i, char in enumerate(text):
        if quoted:
            if char == '"' and not escaped:
                quoted = False
            escaped = char == "\\" and not escaped
        elif char == '"':
            quoted = True
        elif char in "([{":
            depth += 1
        elif char in ")]}":
            depth -= 1
        elif char == "," and depth == 0:
            result.append(text[start:i].strip())
            start = i + 1
    if text.strip():
        result.append(text[start:].strip())
    return result


def calls(text, name):
    result = []
    for match in re.finditer(r"\b" + re.escape(name) + r"\s*\(", text):
        start, depth, quoted, escaped = match.end(), 1, False, False
        for i in range(start, len(text)):
            char = text[i]
            if quoted:
                if char == '"' and not escaped:
                    quoted = False
                escaped = char == "\\" and not escaped
            elif char == '"':
                quoted = True
            elif char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
                if depth == 0:
                    result.append(arguments(text[start:i]))
                    break
        else:
            raise ValueError(f"Unbalanced call to {name}")
    return result


class Source:
    def __init__(self, relative, root=JAVA):
        self.path = root / relative
        self.raw = self.path.read_text()
        # Preserve line positions; comments must not look like active calls.
        self.text = re.sub(r'"(?:\\.|[^"\\])*"|//[^\n]*|/\*[\s\S]*?\*/',
                           lambda m: m[0] if m[0].startswith('"') else
                           re.sub(r"[^\n]", " ", m[0]), self.raw)
        self.constants = dict(re.findall(
            r"static\s+final\s+(?:double|long|int)\s+(\w+)\s*=\s*([^;]+);", self.text))

    def method(self, name):
        match = re.search(r"(?:public|protected|private)\s+(?:static\s+)?\w+\s+"
                          + re.escape(name) + r"\s*\([^)]*\)\s*(?:throws [^{]+)?\{", self.text)
        require(match is not None, f"Missing method {self.path.name}:{name}")
        # All reviewed method bodies have balanced braces outside strings.
        depth = 1
        for token in re.finditer(r'"(?:\\.|[^"\\])*"|[{}]', self.text[match.end():]):
            if token[0] == "{":
                depth += 1
            elif token[0] == "}":
                depth -= 1
                if depth == 0:
                    return self.text[match.end():match.end() + token.start()]
        raise ValueError(f"Unbalanced method {name}")

    def number(self, expression):
        operations = {ast.Add: operator.add, ast.Sub: operator.sub,
                      ast.Mult: operator.mul, ast.Div: operator.truediv,
                      ast.USub: operator.neg, ast.UAdd: operator.pos}

        def visit(node):
            if isinstance(node, ast.Constant) and type(node.value) in (int, float):
                return float(node.value)
            if isinstance(node, ast.Name) and node.id in self.constants:
                return self.number(self.constants[node.id])
            if isinstance(node, ast.BinOp) and type(node.op) in operations:
                return operations[type(node.op)](visit(node.left), visit(node.right))
            if isinstance(node, ast.UnaryOp) and type(node.op) in operations:
                return operations[type(node.op)](visit(node.operand))
            raise ValueError(f"Unsupported numeric expression: {expression}")

        return visit(ast.parse(expression.strip(), mode="eval").body)

    def nums(self, args):
        return tuple(self.number(arg) for arg in args)

    def reference(self, method):
        offset = self.text.index(self.method(method))
        return f"{self.path.relative_to(ROOT)}:{self.text[:offset].count(chr(10)) + 1} ({method})"

    def contains(self, method, expression):
        require(re.sub(r"\s+", "", expression) in
                re.sub(r"\s+", "", self.method(method)),
                f"Changed {self.path.name}:{method}: {expression}")


def relative(pose, forward, left, heading=None):
    x, y, h = pose
    angle = math.radians(h)
    return (x + forward * math.cos(angle) - left * math.sin(angle),
            y + forward * math.sin(angle) + left * math.cos(angle),
            h if heading is None else heading)


def field_pose(pose, start):
    x, y, h = relative(start, pose[0], pose[1])
    return tuple(round(value, 6) for value in (x, y, (start[2] + pose[2]) % 360))


class Route:
    def __init__(self, key, opmodes, start, note):
        self.key, self.opmodes, self.start, self.note = key, opmodes, start, note
        self.pose, self.events = (0.0, 0.0, 0.0), []

    def move(self, name, pose, source, kind="target"):
        self.pose = tuple(pose)
        self.events.append(dict(kind=kind, name=name, localPose=list(self.pose),
                                fieldPose=list(field_pose(self.pose, self.start)), source=source))

    def offset(self, name, args, source, nudge=False):
        pose = relative(self.pose, args[0], 0 if nudge else args[1],
                        None if nudge else args[2])
        self.move(name, pose, source, "encoder-nudge" if nudge else "relative-target")

    def wait(self, name, milliseconds, source, variable=False):
        self.events.append(dict(kind="wait", name=name, durationMs=round(milliseconds),
                                variable=variable, source=source))

    def project(self):
        doc = dict(version="2.0.0", startPoint=dict(x=self.start[0], y=self.start[1],
                   headingDeg=self.start[2], locked=False), lines=[], shapes=[], sequence=[],
                   fieldPoints=[], activePaths=[])
        for i, event in enumerate(self.events):
            identifier = f"{self.key}-{i + 1}"
            if event["kind"] == "wait":
                doc["sequence"].append(dict(kind="wait", id=identifier,
                                           name=event["name"], durationMs=event["durationMs"]))
            else:
                x, y, h = event["fieldPose"]
                doc["lines"].append(dict(kind="atomic", id=identifier, name=event["name"],
                    endPoint=dict(x=x, y=y), controlPoints=event.get("controlPoints", []),
                    heading=event.get("heading", dict(type="constant", degrees=h)), color=COLORS[i % len(COLORS)],
                    locked=False, waitBeforeMs=0, waitAfterMs=0,
                    waitBeforeName="", waitAfterName=""))
                doc["sequence"].append(dict(kind="path", lineId=identifier))
        return doc

    def outside(self):
        return [e["name"] for e in self.events if "fieldPose" in e
                and any(not 0 <= c <= 144 for c in e["fieldPose"][:2])]


def main_route(source, start, early=False):
    body = source.method("runOpMode")
    direct = calls(body, "driveToPose")
    offsets = calls(body, "driveRobotRelative")
    nudges = calls(body, "driveIntakeNudge")
    curves = calls(body, "followCurve")
    require(not direct and len(offsets) == 4 and len(nudges) == 3,
            "Changed main motion call counts")
    require([c[0] for c in curves] == ["MainAutoCurves.FIRST_INTAKE", "MainAutoCurves.SECOND_INTAKE"],
            "Changed main intake curve sequence")
    source.contains("turnInPlaceToHeading", "return turnCounterClockwiseBy(180.0, step, timeoutSeconds);")
    require(intake_curves.OUTPUT.read_text() == intake_curves.render(),
            "MainAutoCurves.java is out of sync with the editable project")
    project, geometry = intake_curves.read_project()
    start = tuple(project["startPoint"][key] for key in ("x", "y", "headingDeg"))
    imported = {line["id"]: line for line in project["lines"]}

    def add_curve(route, identifier):
        curve = geometry[identifier]
        line = imported[identifier]
        route.move(line["name"], (*curve["points"][-1], curve["endHeading"]),
                   "MainAutoCurves.java; " + ref, "bezier")
        route.events[-1].update(controlPoints=line["controlPoints"], heading=line["heading"])

    ref = source.reference("runOpMode")
    route = Route("main-early-park" if early else "main-nominal",
                  ["BioBuzzForward30Auto", "BlueAutoMode"], start,
                  "Tag missing / shooter spin-up failure branch after second-intake shift." if early else
                  "Full successful command sequence, assuming the 25-second forced park does not interrupt it.")
    route.wait("Opening spin-up: variable, omitted", 0, ref, True)
    route.wait("Opening shooting", source.number("SHOOTING_SECONDS") * 1000, ref)
    add_curve(route, intake_curves.CURVE_IDS[0])
    route.wait("Arm lower settle", source.number("ARM_LOWER_SETTLE_MS"), ref)
    route.offset("Intake forward nudge (nominal)", source.nums(nudges[0][:1]), ref, True)
    route.offset("Clearance: back and left", source.nums(offsets[0][:3]), ref)
    turn = source.nums(calls(source.method("turnInPlaceToHeading"), "turnCounterClockwiseBy")[0][:1])[0]
    # Split the half-turn to preserve CCW direction in a shortest-turn simulator.
    for part in range(2):
        route.move(f"CCW half-turn: part {part + 1}/2 (in place)",
                   (*route.pose[:2], route.pose[2] + turn / 2),
                   source.reference("turnInPlaceToHeading"), "turn-preview")
    add_curve(route, intake_curves.CURVE_IDS[1])
    route.offset("Shift right for second shot", source.nums(offsets[1][:3]), ref)
    route.wait("Second-shot tag / spin-up: variable, omitted", 0, ref, True)
    if not early:
        shooting = calls(body, "runShooterAndWindmill")
        require(len(shooting) == 3 and shooting[0] == [] and shooting[1][0] == "true"
                and shooting[2] == ["false", "SHOOTING_SECONDS"], "Changed main shooting sequence")
        route.wait("Second shooting", source.number(shooting[1][1]) * 1000, ref)
        route.offset("Back away after second shot", source.nums(offsets[2][:3]), ref)
        route.wait("Arm lower settle", source.number("ARM_LOWER_SETTLE_MS"), ref)
        route.offset("Collect: forward and left", source.nums(offsets[3][:3]), ref)
        sleeps = calls(body, "sleep")
        require(len(sleeps) == 2, "Changed main settling waits")
        route.wait("Intake settle", source.number(sleeps[0][0]), ref)
        route.offset("Final intake reverse nudge (nominal)", source.nums(nudges[1][:1]), ref, True)
        route.offset("Final intake forward nudge (nominal)", source.nums(nudges[2][:1]), ref, True)
        route.wait("Final shooting", source.number("SHOOTING_SECONDS") * 1000, ref)
    park_ref = source.reference("parkTwoFeetRobotLeft")
    park = source.method("parkTwoFeetRobotLeft")
    park_lines, park_offsets = calls(park, "followLineSegment"), calls(park, "driveRobotRelative")
    require(len(park_lines) == len(park_offsets) == 1, "Changed park helper")
    route.move("Absolute final park target", source.nums(park_lines[0][:3]), park_ref)
    route.offset("Back away after park", source.nums(park_offsets[0][:3]), park_ref)
    return route


def op_side_route(source, start):
    ref = source.reference("runOpMode")
    route = Route("op-side-OUTSIDE-FIELD", ["RedAutoOpSide", "BlueAutoOpSide"], start,
                  "Outside field from Tara start; square waypoint stops; final move conditional on tag and time.")
    direct = calls(source.method("runOpMode"), "driveToPose")
    require(len(direct) == 3, "Changed Op Side waypoint count")
    for i, call in enumerate(direct):
        route.move(f"Stopped waypoint {i + 1}", source.nums(call[:3]), ref)
    route.wait("Upright tag / spin-up: variable, omitted", 0, ref, True)
    route.wait("Shooting", source.number("SHOOTING_SECONDS") * 1000, ref)
    departure = source.number("AUTO_PERIOD_SECONDS") - source.number("FINAL_MOVE_REMAINING_SECONDS")
    route.wait(f"Hold until auto t={departure:g}s: variable, omitted", 0, ref, True)
    body = source.method("recordFinalPathReference")
    # This reference frame is calculated, not visited by the robot.
    x_match = re.search(r"finalPathOriginX\s*=\s*originX\s*\+\s*\(([^)]+)\)\s*\*\s*cos\s*-\s*\(([^)]+)\)\s*\*\s*sin", body)
    h_match = calls(body, "Math.toRadians")
    require(x_match is not None and len(h_match) == 1, "Changed Op Side reference frame")
    dx, dy = source.nums(x_match.groups())
    source.contains("recordFinalPathReference",
                    f"finalPathOriginY = originY + ({x_match[1]}) * sin + ({x_match[2]}) * cos;")
    reference = relative(route.pose, dx, dy, route.pose[2] + source.number(h_match[0][0]))
    final = calls(source.method("followFinalPath"), "driveToRelativePose")
    require(len(final) == 1 and final[0][:3] == ["finalPathOriginX", "finalPathOriginY", "finalPathOriginHeading"],
            "Changed Op Side final move")
    fx, fy, fh = source.nums(final[0][3:6])
    route.move("Direct final move (no stop at reference frame)",
               relative(reference, fx, fy, reference[2] + fh), source.reference("followFinalPath"))
    return route


def tara_project(source):
    poses = {name: source.nums(arguments(args)) for name, args in
             re.findall(r"final Pose (\w+) = poseFactory\.of\(([^)]+)\);", source.text)}
    require("start" in poses, "Missing Tara start pose")
    start = poses["start"]
    doc = Route("tara", ["TaraPedroPath"], start, "").project()
    curves = re.findall(r"Paths\.curve\(([^)]+)\)\.(constant|linear)\(([^)]+)\)", source.text)
    require(len(curves) == len(calls(source.text, "Paths.curve")) > 0, "Unsupported Tara curve")
    previous = start
    for i, (point_args, mode, heading_args) in enumerate(curves):
        points = [poses[name] for name in arguments(point_args)]
        require(points[0][:2] == previous[:2], "Discontinuous Tara curve position")
        heading_poses = [poses[name] for name in arguments(heading_args)]
        heading = (dict(type="constant", degrees=heading_poses[0][2]) if mode == "constant"
                   else dict(type="linear", startDeg=heading_poses[0][2], endDeg=heading_poses[1][2]))
        identifier = f"tara-curve-{i + 1}"
        doc["lines"].append(dict(kind="atomic", id=identifier, name=f"Tara curve {i + 1}",
            endPoint=dict(zip(("x", "y"), points[-1][:2])),
            controlPoints=[dict(zip(("x", "y"), p[:2])) for p in points[1:-1]],
            heading=heading, color=COLORS[i % len(COLORS)], locked=False,
            waitBeforeMs=0, waitAfterMs=0, waitBeforeName="", waitAfterName=""))
        doc["sequence"].append(dict(kind="path", lineId=identifier))
        previous = points[-1]
    return start, doc


def build(root=JAVA):
    names = ["BioBuzzForward30Auto", "BlueAutoMode", "RedAutoOpSide", "BlueAutoOpSide",
             "OppositeSideMainAutoBase", "RedOppositeSideMainAuto", "BlueOppositeSideMainAuto",
             "JimothyAutoPark", "JimothySimplestPark"]
    sources = {name: Source(name + ".java", root) for name in names}
    tara = Source("students/TaraPedroPath.java", root)
    start, tara_doc = tara_project(tara)
    # Blue subclasses currently change tag gates only, without mirroring geometry.
    for name, parent, methods in [
        ("BlueAutoMode", "BioBuzzForward30Auto", {"secondShotTargetName", "isSecondShotTargetSingleTag"}),
        ("BlueAutoOpSide", "RedAutoOpSide", {"targetColorLabel", "isTargetCluster", "isTargetSingleTag"}),
        ("BlueOppositeSideMainAuto", "OppositeSideMainAutoBase", {"targetTagName", "isTargetSingleTag"}),
        ("RedOppositeSideMainAuto", "OppositeSideMainAutoBase",
         {"targetTagName", "isTargetSingleTag", "driveToRecordedNextPositionAfterShooting"})]:
        text = sources[name].text
        require(f"extends {parent}" in text and set(re.findall(
            r"protected\s+\w+\s+(\w+)\s*\(", text)) == methods,
            f"Changed subclass {name}")
    routes = [main_route(sources["BioBuzzForward30Auto"], start),
              main_route(sources["BioBuzzForward30Auto"], start, early=True),
              op_side_route(sources["RedAutoOpSide"], start)]
    base = sources["OppositeSideMainAutoBase"]
    base.contains("driveToRecordedNextPositionAfterShooting", "return false;")
    sources["RedOppositeSideMainAuto"].contains("driveToRecordedNextPositionAfterShooting", "return true;")
    target = calls(base.method("runOpMode"), "driveFastToPose")
    require(len(target) == 1, "Changed Opposite Side motion")
    for color in ("Red", "Blue"):
        route = Route(color.lower() + "-opposite-side", [color + "OppositeSideMainAuto"], start,
                      "Move after successful tag/shoot sequence." if color == "Red" else
                      "Stationary: this subclass does not enable the post-shot move.")
        route.wait("Tag / spin-up: variable, omitted", 0, base.reference("runOpMode"), True)
        route.wait("Shooting", base.number("SHOOTING_SECONDS") * 1000, base.reference("runOpMode"))
        if color == "Red":
            route.move("Post-shot recorded target", base.nums(target[0]), base.reference("runOpMode"))
        routes.append(route)
    source = sources["JimothyAutoPark"]
    route = Route("jimothy-auto-park-OUTSIDE-FIELD", ["JimothyAutoPark"], start,
                  "Outside field from Tara start; two stopped pose targets.")
    targets = calls(source.method("runOpMode"), "driveToPose")
    require(len(targets) == 2, "Changed Jimothy Auto Park")
    for name, call in zip(("Corner stop", "Park stop"), targets):
        route.move(name, source.nums(call[:3]), source.reference("runOpMode"))
    routes.append(route)
    source = sources["JimothySimplestPark"]
    source.contains("runOpMode", "setDrivePower(DRIVE_POWER);")
    route = Route("jimothy-simplest-park", ["JimothySimplestPark"], start,
                  "Nominal encoder-controlled straight drive; timeout/slip can shorten it.")
    route.move("Encoder park (nominal)",
               (math.copysign(source.number("DISTANCE_INCHES"), source.number("DRIVE_POWER")), 0, 0),
               source.reference("runOpMode"), "encoder-nudge")
    routes.append(route)
    represented = {mode for route in routes for mode in route.opmodes} | {"TaraPedroPath"}
    registered = {p.stem for p in root.rglob("*.java")
                  if re.search(r"^\s*@Autonomous\b", p.read_text(), re.M)}
    require(represented == registered, f"Autonomous coverage changed: {registered ^ represented}")
    return routes, tara_doc, list(sources.values()) + [tara]


def write(output):
    routes, tara_doc, sources = build()
    output.mkdir(parents=True, exist_ok=True)
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    manifest = dict(sourceCommit=commit, defaultStartPose=tara_doc["startPoint"],
                    transform="fieldXY = routeStartXY + R(routeStartHeading) * localXY; fieldHeading = routeStartHeading + localHeading",
                    units="inches and degrees", sources={}, routes=[])
    for source in sources:
        manifest["sources"][str(source.path.relative_to(ROOT))] = hashlib.sha256(source.raw.encode()).hexdigest()
    for path in (intake_curves.PROJECT, intake_curves.OUTPUT, JAVA / "AutoBezierCurve.java"):
        manifest["sources"][str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    for route in routes:
        filename = route.key + ".pp.json"
        (output / filename).write_text(json.dumps(route.project(), indent=2) + "\n")
        manifest["routes"].append(dict(file=filename, opmodes=route.opmodes, note=route.note,
                                       startPose=dict(zip(("x", "y", "headingDeg"), route.start)),
                                       centerTargetsOutsideField=route.outside(), events=route.events))
    (output / "tara-current.pp.json").write_text(json.dumps(tara_doc, indent=2) + "\n")
    manifest["routes"].append(dict(file="tara-current.pp.json", opmodes=["TaraPedroPath"],
                                   note="Actual Pedro Bezier curves, copied from source without transforming."))
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    table = "\n".join(f"| {', '.join(r.opmodes)} | [{r.key}.pp.json]({r.key}.pp.json) | "
                       f"{'OUTSIDE FIELD' if r.outside() else 'Center targets inside field'} |" for r in routes)
    readme = f"""# Autonomous previews in the official Pedro visualizer

Source commit: `{commit[:7]}`. Generated by `python3 scripts/export_autonomous_previews.py`.

Open https://visualizer.pedropathing.com/ and use **File → Open from computer…** to load a `.pp.json` file. Use the BioBuzz field map. Imported projects also appear under **File → Open…** in that browser. Select a path to edit its points or heading; the play control animates the preview. Keep edited copies under a new name because regeneration replaces these exports.

The main Red/Blue routes use the corrected start in `working/main-bezier.pp.json` (one inch +field X). Their two intake Béziers are imported into the existing Java controller. Other autonomous previews place local origin `(0, 0, 0°)` at Tara's current start: **({tara_doc['startPoint']['x']}, {tara_doc['startPoint']['y']}, {tara_doc['startPoint']['headingDeg']}°)**. This uses the starting orientation as well as the position. At 270°, `fieldX = {tara_doc['startPoint']['x']} + localY`, `fieldY = {tara_doc['startPoint']['y']} - localX`. No scaling, mirroring, or recentering is applied. Blue subclasses currently inherit the Red geometry and change tag checks only.

| Autonomous class | Project | Center target bounds (0–144 in) |
| --- | --- | --- |
{table}
| TaraPedroPath | [tara-current.pp.json](tara-current.pp.json) | Original Pedro curves |

The Op Side and older Auto Park projects have targets beyond the field from this start. Those coordinates are preserved; the visualizer can clip them. Center targets inside the field do not establish robot-footprint or obstacle clearance, including at the starting pose.

## What these previews mean

`main-nominal` shows the full successful command sequence: opening shot, first intake Bézier and forward nudge, clearance, CCW half-turn, second-intake Bézier, shooting shift, second shot, collection moves and reverse/forward nudges, final shot, and park. `main-early-park` branches after the second-intake shooting shift if the tag is missing or spin-up fails. The real main routine can force park from its **live measured pose at 25 seconds**. Its branch point depends on actual runtime, so a single static file cannot predict that interruption. The full nominal preview may not finish during a real autonomous period.

The main exports preserve the two imported Bézier curves and their headings; other movements connect **nominal command targets with straight lines**. The Java pose controllers can rotate while translating, leave a waypoint within tolerance, overshoot, or time out. Relative moves and the Op Side final reference frame use measured arrival poses on the robot; these exports assume each preceding target was reached exactly. The main intake approaches each follow one continuous cubic; Op Side and Auto Park stop at their waypoints. Op Side drives directly from its shooting position to the composed final target; the calculated reference frame is not an extra waypoint. Blue Opposite Side is stationary and deliberately has no drive path.

Heading entries record command setpoints. Pedro's preview may turn before translating; this does not reproduce the custom controller's heading trajectory. The dedicated CCW half-turn is represented by two zero-distance 90° turns to preserve its direction. Encoder nudge lengths assume successful travel without drift or slip. The Tara and main intake exports preserve the actual Bezier control points and heading interpolators.

Shooting windows and fixed settle sleeps are included in the sequence. Variable tag, spin-up, and elapsed-time holds are labeled with zero duration rather than guessed. Preview speed, acceleration, and robot dimensions come from the visualizer settings and are **not calibrated to this robot**; displayed time is not a match-time prediction. Stops, disabled-op-mode exits, failures, forced park and mechanism behavior need robot logs to replay accurately.

Editing these generated previews changes the visualizer project only. For the main intake curves, edit `visualizations/working/main-bezier.pp.json` and run `python3 scripts/import_main_auto_curves.py` to update Java. That importer validates the rest of the motion and mechanism sequence; only the two intake curves' control points can change without reviewing the remaining autonomous code. Exporting Pedro Java does not update the existing mechanism sequence automatically.

## Regenerating and checking

Run from the repository root:

```sh
python3 scripts/import_main_auto_curves.py --check
python3 scripts/export_autonomous_previews.py
python3 -m unittest discover -s scripts -p 'test_export_autonomous_previews.py'
```

The exporter reads numeric constants, movement arguments and Tara's curve definitions from Java. It has explicit adapters for the reviewed command flows, with guards for several structural changes and coverage of registered autonomous classes. It is not a general Java interpreter: review adapters whenever control flow or helper semantics change. `manifest.json` records source hashes, source methods, local/field poses, waits, branch notes, and targets outside the field.
"""
    (output / "README.md").write_text(readme)
    print(f"Exported {len(routes) + 1} projects to {output}")
    for route in routes:
        print(f"  {route.key}: {len(route.project()['lines'])} paths"
              + (f"; {len(route.outside())} center targets OUTSIDE FIELD" if route.outside() else ""))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "visualizations/autonomous")
    write(parser.parse_args().output)
