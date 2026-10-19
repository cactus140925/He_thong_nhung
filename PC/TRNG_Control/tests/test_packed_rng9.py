"""Regression tests for optimized type-3 packed RNG9 frames."""
import struct
import unittest
import zlib

from trng.protocol import (WireFramer, encode_binary_block, RNG_PACKED_SAMPLES,
                           BINARY_FRAME_BYTES)
from trng.rng9 import pack_words


class PackedRng9Tests(unittest.TestCase):
    def receive(self, data):
        frame = WireFramer()
        return frame.feed(b"BEGINR,9,250,1000000\n" + data)

    def test_full_packet_37_words_and_tail_for_all_counts(self):
        for count in range(1, RNG_PACKED_SAMPLES + 1):
            with self.subTest(count=count):
                samples = [((x * 123) ^ 0x1ff) & 0x1ff for x in range(count)]
                encoded = encode_binary_block(9, 0, samples, mode="rng9")
                self.assertEqual(len(encoded), BINARY_FRAME_BYTES)
                self.assertEqual(encoded[5], 3)
                result = self.receive(encoded)
                self.assertEqual(result[-1], ("capture_block_rng", (9, 0, tuple(samples))))
                self.assertEqual(encoded[16:16 + len(pack_words(samples))], pack_words(samples))
                self.assertFalse(any(encoded[16 + len(pack_words(samples)):58]))
        with self.assertRaises(ValueError):
            encode_binary_block(9, 0, [0] * (RNG_PACKED_SAMPLES + 1), mode="rng9")

    def test_padding_corruption_rejected_even_with_forged_valid_frame_crc(self):
        for count in (1, 17, 37):
            payload = bytearray(encode_binary_block(9, 0, [511] * count, mode="rng9"))
            payload[57] |= 1  # unused bit in the 42-byte payload
            struct.pack_into("<I", payload, 58, zlib.crc32(payload[4:58]))
            events = self.receive(payload)
            self.assertTrue(any(k == "parse_error" for k, _ in events))
            self.assertFalse(any(k == "capture_block_rng" for k, _ in events))

    def test_header_count_and_crc_corruption(self):
        frame = bytearray(encode_binary_block(9, 0, [0] * 37, mode="rng9"))
        frame[6] = 38
        struct.pack_into("<I", frame, 58, zlib.crc32(frame[4:58]))
        self.assertTrue(any(k == "parse_error" for k, _ in self.receive(frame)))
        frame = bytearray(encode_binary_block(9, 0, [0] * 37, mode="rng9"))
        frame[16] ^= 0x80
        self.assertTrue(any(k == "parse_error" for k, _ in self.receive(frame)))

    def test_old_rng9_type2_remains_accepted(self):
        words = [1, 0, 256, 511, 73]
        packet = encode_binary_block(9, 0, words, mode="rng9", legacy_rng9=True)
        self.assertEqual(packet[5], 2)
        self.assertEqual(self.receive(packet)[-1], ("capture_block_rng", (9, 0, tuple(words))))

    def test_overhead_improvement(self):
        samples = 50_000
        original = ((samples + 20) // 21) * 62
        packed = ((samples + 36) // 37) * 62
        self.assertEqual(original, 147622)
        self.assertEqual(packed, 83824)
        self.assertLess(packed / original, 0.57)


if __name__ == "__main__":
    unittest.main()
