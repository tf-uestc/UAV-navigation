from base_detector import Detection


class DetectorFixture:
    def __init__(self, **kwargs):
        self.calls = []

    def detect(self, rgb, depth, target_text, info):
        self.calls.append(target_text)
        return Detection((0, 0, 2, 2), (0., 0., 2.), 2., 1., target_text=target_text)
