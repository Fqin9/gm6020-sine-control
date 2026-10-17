import importlib.util
from pathlib import Path
import struct
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('export_assessment', Path(__file__).resolve().parents[1] / 'tools/export_assessment.py')
exporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exporter)


class ExportTests(unittest.TestCase):
    scratch = Path(__file__).resolve().parents[1] / 'build' / 'host-tests'

    @classmethod
    def setUpClass(cls):
        cls.scratch.mkdir(parents=True, exist_ok=True)

    def test_capture_roundtrip(self):
        with tempfile.TemporaryDirectory(dir=self.scratch) as folder:
            root = Path(folder).resolve()
            assert root.is_relative_to(self.scratch.resolve())
            binary = root / 'input.bin'
            binary.write_bytes(struct.pack('<4I', 1, 2, 4, 16000) +
                               struct.pack('<Iffffi', 0, 1, .5, 3, 3, 100) +
                               struct.pack('<Iffffi', 20, 2, 1.5, 3, 3, 200))
            result = exporter.export(binary, root / 'capture')
            self.assertAlmostEqual(result['rms_error'], .5)
            self.assertTrue((root / 'capture.csv').is_file())
            self.assertIn('Measured', (root / 'capture.svg').read_text(encoding='utf-8'))
            self.assertTrue((root / 'capture.json').is_file())

    def test_reject_incomplete_or_uninitialized(self):
        with tempfile.TemporaryDirectory(dir=self.scratch) as folder:
            root = Path(folder).resolve()
            assert root.is_relative_to(self.scratch.resolve())
            binary = root / 'bad.bin'
            for content in (b'', bytes(16), struct.pack('<4I', 1, 10, 4, 16000)):
                binary.write_bytes(content)
                with self.assertRaises(ValueError):
                    exporter.export(binary, root / 'should-not-exist')
            self.assertFalse((root / 'should-not-exist.csv').exists())


if __name__ == '__main__':
    unittest.main()
