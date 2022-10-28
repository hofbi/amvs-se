"""Plot helper module tests"""

import sys
import unittest
from pathlib import Path

from pyfakefs.fake_filesystem_unittest import TestCase

try:
    sys.path.append(str(Path(__file__).parents[2]))
except IndexError:
    pass

from cqp.bitrate_model.plot import remove_duplicate_lines


class PlotTest(TestCase):
    """Plot tests"""

    def setUp(self) -> None:
        self.setUpPyfakefs()
        self.file_path = Path("test")

    def test_remove_duplicate_lines__empty_string__unchanged(self):
        self.fs.create_file(self.file_path, contents="")
        remove_duplicate_lines(self.file_path)
        self.assertEqual(self.file_path.read_text(), "")

    def test_remove_duplicate_lines__no_duplicates__unchanged(self):
        input_text = """
table{%
x  y
979 1556
1187 1963
};
                """
        self.fs.create_file(self.file_path, contents=input_text)
        remove_duplicate_lines(self.file_path)
        self.assertEqual(self.file_path.read_text(), input_text)

    def test_remove_duplicate_lines__duplicates__removed(self):
        input_text = """
table{%
x  y
979 1556
1187 1963
979 1556
979 1556
1187 1963
3 5
};
        """
        expected_result = """
table{%
x  y
979 1556
1187 1963
3 5
};
        """
        self.fs.create_file(self.file_path, contents=input_text)
        remove_duplicate_lines(self.file_path)
        self.assertEqual(self.file_path.read_text(), expected_result)


if __name__ == "__main__":
    unittest.main()
