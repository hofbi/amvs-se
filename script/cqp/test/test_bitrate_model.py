"""Bitrate model tests"""

import sys
import time
import unittest
from pathlib import Path

try:
    sys.path.append(str(Path(__file__).parents[2]))
except IndexError:
    pass

from cqp.bitrate_model.model import BitrateModel, ModelParameter


class ModelTest(unittest.TestCase):
    """Bitrate model tests"""

    def setUp(self) -> None:
        self.model = BitrateModel(ModelParameter())

    @staticmethod
    def evaluate(iterations, handle, *args):
        start = time.time()
        for _ in range(0, iterations):
            handle(*args)
        end = time.time()
        return (end - start) / iterations

    def test_evaluate__runtime(self):
        print(
            f"Average runtime analytic model: "
            f"{self.evaluate(10000, self.model.evaluate, 81, 14, 24, 30, 1, '352x288', 1, 1)}s"
        )


if __name__ == "__main__":
    unittest.main()
