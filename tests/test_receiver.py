import importlib.util
import random
import struct
import tempfile
import unittest
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("receiver", ROOT / "PC" / "receive_adc.py")
receiver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(receiver)


def capture(values=None, session=7):
    values = values if values is not None else [(i * 29) & 4095 for i in range(512)]
    checksum = zlib.crc32(b"".join(struct.pack("<H", value) for value in values))
    lines = [f"BEGIN,{session},{len(values)},1000000\n"]
    lines.extend(f"DATA,{session},{i},{value}\n" for i, value in enumerate(values))
    lines.append(f"END,{session},{len(values)},{checksum:08X}\n")
    return lines, checksum


class ReceiverTests(unittest.TestCase):
    def test_every_byte_and_random_chunk_boundaries(self):
        lines, crc = capture()
        payload = "".join(lines).encode()
        for max_chunk in (1, 7, 64, 512, len(payload)):
            parser = receiver.CaptureParser()
            rng = random.Random(42)
            offset = 0
            while offset < len(payload):
                step = rng.randint(1, max_chunk)
                parser.feed(payload[offset:offset + step])
                offset += step
            self.assertTrue(parser.complete)
            self.assertEqual(len(parser.samples), 512)
            self.assertEqual(parser.crc, crc)

    def test_missing_duplicate_reorder_wrong_session_and_corruption(self):
        original, _ = capture()
        variants = []
        lines = original.copy(); del lines[100]; variants.append(lines)
        lines = original.copy(); lines.insert(100, lines[100]); variants.append(lines)
        lines = original.copy(); lines[100], lines[101] = lines[101], lines[100]; variants.append(lines)
        lines = original.copy(); lines[100] = lines[100].replace("DATA,7,", "DATA,8,"); variants.append(lines)
        lines = original.copy(); lines[100] = "DATA,7,99,0\n"; variants.append(lines)
        lines = original.copy(); lines[-1] = "END,7,256,00000000\n"; variants.append(lines)
        for lines in variants:
            with self.assertRaises(receiver.CaptureError):
                receiver.CaptureParser().feed("".join(lines).encode())

    def test_partial_capture_never_saved(self):
        lines, _ = capture()
        parser = receiver.CaptureParser()
        parser.feed("".join(lines[:-1]).encode())
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "samples.csv"
            with self.assertRaises(receiver.CaptureError):
                receiver.save_csv(parser, output)
            self.assertFalse(output.exists())

    def test_csv_has_exactly_512_data_rows(self):
        lines, _ = capture()
        parser = receiver.CaptureParser()
        parser.feed("".join(lines).encode())
        with tempfile.TemporaryDirectory() as directory:
            path = receiver.save_csv(parser, Path(directory) / "samples.csv")
            self.assertEqual(len(path.read_text().splitlines()), 513)

    def test_bad_frames_and_extra_data(self):
        for payload in (b"X" * 128, b"DATA,7,0,4000\n", b"ERROR,7,5\n", b"\xff\n"):
            with self.assertRaises(receiver.CaptureError):
                receiver.CaptureParser().feed(payload)
        lines, _ = capture()
        with self.assertRaises(receiver.CaptureError):
            receiver.CaptureParser().feed(("".join(lines) + "DATA,7,512,0\n").encode())


if __name__ == "__main__":
    unittest.main()
