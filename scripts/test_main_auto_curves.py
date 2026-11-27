"""Check editor import contracts and the actual Java curve math without robot hardware."""

import bisect
import copy
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

import import_main_auto_curves as importer


class CurveImportTests(unittest.TestCase):
    def setUp(self):
        self.doc = json.loads(importer.PROJECT.read_text())

    def read_modified(self, mutate):
        doc = copy.deepcopy(self.doc)
        mutate(doc)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'edited.pp.json'
            path.write_text(json.dumps(doc))
            return importer.read_project(path)

    def test_java_is_in_sync_and_frame_round_trips(self):
        self.assertEqual(importer.OUTPUT.read_text(), importer.render())
        doc, curves = importer.read_project()
        start = doc['startPoint']
        self.assertEqual((start['x'], start['y'], start['headingDeg']), (55.9219, 7.7305, 270))
        previous = start
        for line in doc['lines']:
            if line['id'] in curves:
                field_points = [previous, *line['controlPoints'], line['endPoint']]
                for (local_x, local_y), field in zip(curves[line['id']]['points'], field_points):
                    self.assertAlmostEqual(start['x'] + local_y, field['x'])
                    self.assertAlmostEqual(start['y'] - local_x, field['y'])
            previous = line['endPoint']

    def test_control_point_edit_changes_import(self):
        _, original = importer.read_project()
        _, edited = self.read_modified(lambda d: d['lines'][5]['controlPoints'][0].update(x=30.5))
        self.assertNotEqual(original, edited)

    def test_full_field_translation_preserves_local_geometry(self):
        _, original = importer.read_project()
        def translate(d):
            points = [d['startPoint']]
            for line in d['lines']:
                points.extend([*line['controlPoints'], line['endPoint']])
            for p in points:
                p['x'] += 1
        _, shifted = self.read_modified(translate)
        for key in original:
            for a, b in zip(original[key]['points'], shifted[key]['points']):
                self.assertAlmostEqual(a[0], b[0])
                self.assertAlmostEqual(a[1], b[1])

    def test_unsupported_motion_or_timing_edits_are_rejected(self):
        changes = [
            lambda d: d['lines'][0]['endPoint'].update(x=9),
            lambda d: d['lines'][5]['heading'].update(degrees=110),
            lambda d: d['lines'][1]['controlPoints'].append({'x': 4, 'y': 5}),
            lambda d: d['lines'][5].update(throughPoints=[{'x': 4, 'y': 5}]),
            lambda d: d['sequence'][1].update(durationMs=1000),
            lambda d: d['sequence'].reverse(),
            lambda d: d['lines'][0].update(waitAfterMs=100),
            lambda d: d['startPoint'].update(headingDeg=0),
            lambda d: d['lines'][5]['controlPoints'][0].update(x=float('nan')),
        ]
        for change in changes:
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.read_modified(change)


class JavaCurveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        home = os.environ.get('JAVA_HOME')
        javac = str(Path(home) / 'bin/javac') if home else shutil.which('javac')
        java = str(Path(home) / 'bin/java') if home else shutil.which('java')
        if not javac or not java:
            raise unittest.SkipTest('Set JAVA_HOME to run the Java geometry checks')
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        source = importer.OUTPUT.parent
        harness = importer.ROOT / 'scripts/tests/AutoBezierCurveChecks.java'
        subprocess.run([javac, '-d', cls.temp.name, str(source / 'AutoBezierCurve.java'),
                        str(source / 'MainAutoCurves.java'), str(harness)], check=True, capture_output=True)
        result = subprocess.run([java, '-cp', cls.temp.name,
                                 'org.firstinspires.ftc.teamcode.AutoBezierCurveChecks'],
                                check=True, capture_output=True, text=True)
        cls.samples = json.loads(result.stdout)

    def test_java_curve_sampling_matches_independent_dense_reference(self):
        _, curves = importer.read_project()
        for identifier, curve in curves.items():
            points = curve['points']
            def point(t):
                return tuple((1-t)**3 * points[0][axis] + 3*(1-t)**2*t * points[1][axis]
                             + 3*(1-t)*t*t * points[2][axis] + t**3 * points[3][axis]
                             for axis in (0, 1))
            count = 20000
            dense = [point(i/count) for i in range(count+1)]
            distances = [0.0]
            for a, b in zip(dense, dense[1:]):
                distances.append(distances[-1] + math.dist(a, b))
            actual = self.samples[identifier]
            self.assertAlmostEqual(actual['length'], distances[-1], delta=0.002)
            for i, sample in enumerate(actual['points']):
                fraction = i / (len(actual['points'])-1)
                wanted = distances[-1] * fraction
                high = min(count, max(1, bisect.bisect_left(distances, wanted)))
                within = (wanted-distances[high-1])/(distances[high]-distances[high-1])
                expected = point((high-1+within)/count)
                self.assertLess(math.dist(sample[:2], expected), 0.002)
                self.assertAlmostEqual(sample[2], math.radians(curve['startHeading']
                                       + fraction * (curve['endHeading']-curve['startHeading'])))


if __name__ == '__main__':
    unittest.main()
