# Editable autonomous projects

`main-editable.pp.json` is a working copy of the main Red/Blue route from the
official Pedro visualizer, using Tara's start pose. It is outside the generated
`autonomous` directory so rerunning the preview exporter preserves this copy.

The first four segments are combined into **First intake approach**, a native
Pedro path group. It preserves the existing waypoint positions and individual
headings. Expand the arrow beside the group to edit its segments, or select
the group and choose **Ungroup** to separate them again. A snapshot before
grouping is retained in `backups/`.

`main-bezier.pp.json` is the current smooth project. Its starting pose is
**(55.9219, 7.7305, 270°)**: one inch right on the field image (+field X) from
the original represented start. The entire route, including every endpoint
and control point, has been translated together. Relative geometry, headings,
and waits are preserved. `field-placement.json` records this correction. The
grouped project and generated previews retain their original placement.

The first intake approach is replaced by one cubic Bézier with two editable
control points, fitted using the grouped route's start and intake destination
before the field-position correction above. The intermediate waypoints only
guide the fit; the curve is not constrained to pass through them. The largest
sampled distance between the original polyline and the curve is about
**0.65 inches**. This is a sampled estimate, not a proven maximum. Heading
interpolates linearly from 270° to 272.7°, replacing the original heading step.
Fit coordinates and measurements for the first approach are recorded in
`main-bezier-fit.json`.

Select **First intake approach (Bezier)** and drag **CP1** or **CP2**, or edit
their coordinate fields, to reshape the single curve. The grouped version
remains available as `main-editable.pp.json`.

The same `main-bezier.pp.json` project also combines **Straight first 2/3 of
distance to second intake** and **Diagonal to second intake** into **Second
intake approach (Bezier)**. This is one cubic curve with two editable control
points, preserving the original start and second-intake destination and the
constant 100° heading. The intermediate corner guides the fit without being
preserved. Its largest sampled deviation from the two original segments is
about **3.2 inches** at the time of fitting; details are in `second-intake-bezier-fit.json`.
The current second curve includes subsequent manual control-point edits, so that
historical deviation estimate does not describe the current curve. The first
approach, other movements, and all waits are retained from the current browser
project. A snapshot before this second conversion is retained in `backups/`.

In https://visualizer.pedropathing.com/:

1. Open `main-bezier.pp.json` through **File → Open from computer…**, or
   **File → Open…** if it is already in that browser's library.
2. Pause playback and select a path in the left panel. Expand a group first
   to select one of its segments.
3. Drag its endpoint on the field, or type **End X** and **End Y** in the right
   panel. Use **Control point** to add a curve, and the heading controls to
   change the robot's orientation.
4. **File → Save** (⌘S on Mac) saves in the browser's library.
   **File → Save to computer…** saves a `.pp.json` file on disk. Choose this
   working file when you want to update the local copy.

## Updating the existing autonomous

The latest two intake curves are now implemented by `BioBuzzForward30Auto`
(Red Autonomous Mode), also inherited by `BlueAutoMode`. `MainAutoCurves.java`
contains the imported cubic controls. `AutoBezierCurve.java` supplies curve
sampling, projection and heading interpolation to the existing odometry and
mecanum controller. This uses the robot's existing three-wheel calibration.

The local odometry origin remains `(0, 0, 0°)` at the actual robot starting
position. Its corrected field placement is `(55.9219, 7.7305, 270°)`:
`localX = -(fieldY - 7.7305)`, `localY = fieldX - 55.9219`.
Translating the entire field drawing one inch right therefore does not add an
extra one-inch drive command.

The opening/second/final shooting windows, intake nudges, clearance, CCW turn,
tag checks, early park and forced park at 25 seconds retain their Java sequence.
The second curve raises the arm at 72.94% of arc-length travel, matching the old
combined route's relative travel progress at 25% into its final diagonal. That
preserves a late approach event, rather than raising at 25% of the whole curve;
the physical location and elapsed time need verification on the robot.
The first curve has an 8-second timeout; the second keeps the original two
segments' combined 14-second timeout. The 25-second forced park still preempts
either approach using the measured robot pose. Parking connects that live pose
directly to the existing absolute parking target.

For subsequent edits:

1. Edit **CP1/CP2** on either intake curve in `main-bezier.pp.json`.
2. Use **File → Save to computer…** and replace this working file. Browser
   **Save** alone does not update the repository file.
3. From the repository root run:

   ```sh
   python3 scripts/import_main_auto_curves.py
   python3 scripts/import_main_auto_curves.py --check
   python3 scripts/export_autonomous_previews.py
   python3 -m unittest discover -s scripts -p 'test_*.py'
   ./gradlew :TeamCode:compileDebugJavaWithJavac
   ```

   Set `JAVA_HOME` to a working JDK and `ANDROID_HOME` to the Android SDK if
   they are not already configured. Then build/deploy through the team's
   normal Robot Controller workflow and verify the edited route on the robot.

The importer accepts changes to the two cubic curves' control points and full
field translations. It rejects endpoint, heading, other-motion and timing edits
that would require changing the mechanism sequence. Those changes need a Java
review; they are not silently ignored. Generated previews in `autonomous/` can
be regenerated without overwriting this working project. See
[the preview notes](../autonomous/README.md) for runtime assumptions.
