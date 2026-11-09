import unittest

from navigation_core.targets import local_detour


class LocalDetourTests(unittest.TestCase):
    start = (0., 0., 1.6)
    goal = (1.8, 0., 1.6)
    box = [(i/10, j/10, 1.6) for i in range(4, 13) for j in range(-6, 7)]

    def test_clear_and_occupied_endpoints_leave_original_goal(self):
        inside = lambda p: True
        self.assertFalse(local_detour(self.start, self.goal, [], inside))
        self.assertFalse(local_detour(self.start, self.goal, [self.goal], inside))
        self.assertFalse(local_detour(self.start, self.goal, [(0.8, 0., 2.3)], inside))

    def test_blocked_corridor_stages_around_inflated_box(self):
        route = local_detour(self.start, self.goal, self.box, lambda p: abs(p[1]) < 1.5)
        self.assertEqual(route, ((0., 1., 1.6), (1.8, 1., 1.6), self.goal))

    def test_restricted_fence_uses_available_side_or_returns_no_detour(self):
        route = local_detour(self.start, self.goal, self.box, lambda p: -1.5 < p[1] <= 0.)
        self.assertEqual(route, ((0., -1., 1.6), (1.8, -1., 1.6), self.goal))
        self.assertFalse(local_detour(self.start, self.goal, self.box, lambda p: abs(p[1]) < .5))
