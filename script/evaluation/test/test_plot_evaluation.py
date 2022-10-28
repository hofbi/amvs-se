"""Plot evaluation tests"""

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

try:
    sys.path.append(str(Path(__file__).parents[2]))
except IndexError:
    pass

from evaluation.plot_evaluation import read_average_data_from_eval_path


class PlotEvaluationTest(unittest.TestCase):
    """Plot evaluation tests"""

    def test_read_average_data_from_eval_path__not_existing__raise_file_not_found_error(
        self,
    ):
        path = MagicMock()
        path.is_file.return_value = False
        path.is_dir.return_value = False
        with self.assertRaises(FileNotFoundError):
            read_average_data_from_eval_path(path)

    def test_read_average_data_from_eval_path__not_csv__raise_value_error(self):
        path = MagicMock()
        path.is_file.return_value = True
        path.suffix.return_value = ""
        with self.assertRaises(ValueError):
            read_average_data_from_eval_path(path)


if __name__ == "__main__":
    unittest.main()
