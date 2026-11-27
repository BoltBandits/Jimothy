"""Check the frame conversion and the less obvious autonomous command semantics."""

import math
import unittest

import export_autonomous_previews as previews


class AutonomousPreviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        routes, cls.tara, _ = previews.build()
        cls.routes = {route.key: route for route in routes}
        cls.start = (54.9219, 7.7305, 270)

    def assertPoseAlmostEqual(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=6)

    def test_tara_frame_rotates_translation_and_heading(self):
        self.assertEqual(previews.field_pose((0, 0, 0), self.start), self.start)
        self.assertPoseAlmostEqual(previews.field_pose((10, 3, 45), self.start),
                                   (57.9219, -2.2695, 315))
        self.assertPoseAlmostEqual(previews.field_pose((-8, 0, 0), self.start),
                                   (54.9219, 15.7305, 270))

    def test_relative_left_is_rotated_at_arrival_heading(self):
        self.assertPoseAlmostEqual(previews.relative((10, 20, 90), -5, 6, 0), (4, 15, 0))

    def test_balanced_java_call_arguments(self):
        self.assertEqual(previews.calls('drive(1, Math.toRadians(8), "a,b (c)", false)', 'drive'),
                         [['1', 'Math.toRadians(8)', '"a,b (c)"', 'false']])

    def test_main_curves_nudges_and_ccw_turn(self):
        route = self.routes['main-nominal']
        moves = [e for e in route.events if 'localPose' in e]
        self.assertEqual(len(moves), 13)
        self.assertEqual([e['kind'] for e in moves].count('encoder-nudge'), 3)
        intake = (-9.70, -47.38, 2.7)
        nudge = previews.relative(intake, 12, 0)
        clearance = previews.relative(nudge, -5, 6, 0)
        self.assertPoseAlmostEqual(moves[0]['localPose'], intake)
        self.assertPoseAlmostEqual(moves[1]['localPose'], nudge)
        self.assertPoseAlmostEqual(moves[2]['localPose'], clearance)
        self.assertPoseAlmostEqual(moves[3]['localPose'], (*clearance[:2], 90))
        self.assertPoseAlmostEqual(moves[4]['localPose'], (*clearance[:2], 180))
        self.assertPoseAlmostEqual(moves[5]['localPose'], (-121.07, -15.38, -170))
        self.assertEqual([e['kind'] for e in moves].count('bezier'), 2)
        # The reverse and forward nudge must both be retained, even though they retrace.
        self.assertPoseAlmostEqual(moves[8]['localPose'], moves[10]['localPose'])
        self.assertNotEqual(moves[9]['localPose'], moves[10]['localPose'])

    def test_park_is_absolute_then_robot_relative(self):
        for key in ('main-nominal', 'main-early-park'):
            route = self.routes[key]
            moves = [e for e in route.events if 'localPose' in e]
            self.assertPoseAlmostEqual(moves[-2]['localPose'], (-109.93, -47.73, 90))
            self.assertPoseAlmostEqual(moves[-1]['localPose'], (-109.93, -52.73, 90))
            self.assertPoseAlmostEqual(moves[-1]['fieldPose'], (3.1919, 117.6605, 0))

    def test_early_park_branches_before_second_shot(self):
        full = self.routes['main-nominal']
        early = self.routes['main-early-park']
        full_moves = [e['localPose'] for e in full.events if 'localPose' in e]
        early_moves = [e['localPose'] for e in early.events if 'localPose' in e]
        self.assertEqual(early_moves[:7], full_moves[:7])
        self.assertEqual(len(early_moves), 9)
        self.assertFalse(any(e['name'] == 'Second shooting' for e in early.events))

    def test_op_side_composes_reference_without_visiting_it(self):
        route = self.routes['op-side-OUTSIDE-FIELD']
        moves = [e for e in route.events if 'localPose' in e]
        self.assertEqual(len(moves), 4)
        a, b = math.radians(1.4), math.radians(1.9)
        expected = (123.09 - 9.95 * math.cos(a) + 21.81 * math.sin(a)
                    - 5.67 * math.cos(b) - 71.88 * math.sin(b),
                    -32.87 - 9.95 * math.sin(a) - 21.81 * math.cos(a)
                    - 5.67 * math.sin(b) + 71.88 * math.cos(b), 3.7)
        self.assertPoseAlmostEqual(moves[-1]['localPose'], expected)
        self.assertEqual(len(route.outside()), 4)

    def test_aliases_and_stationary_blue_opposite(self):
        self.assertEqual(self.routes['main-nominal'].opmodes,
                         ['BioBuzzForward30Auto', 'BlueAutoMode'])
        self.assertEqual(self.routes['blue-opposite-side'].project()['lines'], [])
        red_moves = [e for e in self.routes['red-opposite-side'].events if 'localPose' in e]
        self.assertPoseAlmostEqual(red_moves[0]['localPose'], (-8.32, 7, 0))

    def test_original_tara_beziers_are_preserved(self):
        lines = self.tara['lines']
        self.assertEqual([len(line['controlPoints']) for line in lines], [1, 2, 1])
        self.assertEqual(lines[0]['controlPoints'][0], {'x': 30.924, 'y': 30.6199})
        self.assertEqual(lines[1]['heading'], {'type': 'linear', 'startDeg': 0, 'endDeg': 90})
        self.assertEqual(lines[-1]['endPoint'], {'x': 12.0397, 'y': 115.8362})

    def test_constants_flow_into_generated_geometry_and_changed_structure_fails(self):
        source = previews.Source('BioBuzzForward30Auto.java')
        source.constants['SECOND_SHOOTING_RIGHT_OFFSET_INCHES'] = '3.0'
        route = previews.main_route(source, self.start)
        shifted = next(e for e in route.events if e['name'] == 'Shift right for second shot')
        self.assertPoseAlmostEqual(shifted['localPose'],
                                   previews.relative((-121.07, -15.38, -170), 0, -3, -170))
        source.text = source.text.replace('followCurve(MainAutoCurves.FIRST_INTAKE',
                                          'differentController(MainAutoCurves.FIRST_INTAKE')
        with self.assertRaisesRegex(ValueError, 'Changed main intake curve sequence'):
            previews.main_route(source, self.start)

    def test_main_preview_matches_edited_project_geometry(self):
        doc, _ = previews.intake_curves.read_project()
        actual = self.routes['main-nominal'].project()
        self.assertEqual(actual['startPoint'], doc['startPoint'])
        for exported, edited in zip(actual['lines'], doc['lines']):
            self.assertPoseAlmostEqual(exported['endPoint'].values(), edited['endPoint'].values())
            self.assertEqual(exported['controlPoints'], edited['controlPoints'])
            self.assertEqual(exported['heading'], edited['heading'])

    def test_bounds_are_reported_without_changing_targets(self):
        self.assertFalse(self.routes['main-nominal'].outside())
        park = self.routes['jimothy-auto-park-OUTSIDE-FIELD']
        self.assertEqual(len(park.outside()), 2)
        self.assertPoseAlmostEqual(park.events[0]['fieldPose'], (54.7119, -72.7295, 265.2))

    def test_sequence_references_and_variable_waits(self):
        for route in self.routes.values():
            doc = route.project()
            ids = {line['id'] for line in doc['lines']}
            self.assertEqual(ids, {item['lineId'] for item in doc['sequence'] if item['kind'] == 'path'})
            self.assertTrue(all(e['durationMs'] == 0 for e in route.events
                                if e['kind'] == 'wait' and e['variable']))


if __name__ == '__main__':
    unittest.main()
