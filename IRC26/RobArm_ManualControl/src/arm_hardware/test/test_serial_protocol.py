"""Serial protocol v1: framing, CRC, command encoding, telemetry parsing, error handling."""
import math
import random
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

_SRC = Path(__file__).resolve().parents[2]
for _p in ('arm_control', 'arm_hardware'):
    sys.path.insert(0, str(_SRC / _p))

from arm_hardware import serial_protocol as sp  # noqa: E402
from arm_hardware import telemetry_parser as tp  # noqa: E402


def motors(overrides=None):
    ms = [NS(channel=c, direction=0, demand=0.0) for c in range(1, 8)]
    for c, (d, x) in (overrides or {}).items():
        ms[c - 1] = NS(channel=c, direction=d, demand=x)
    return ms


def command(ctype, seq, ms=None):
    return NS(command_type=ctype, sequence=seq, motors=motors() if ms is None else ms)


def decode_all(data: bytes):
    return sp.FrameDecoder().feed(data)


def sample_telemetry():
    t = tp.TelemetryData(telemetry_seq=1000, firmware_state=3, accepted_seq_valid=True,
                         last_accepted_cmd_seq=5, command_age_ms=40)
    t.channels[0] = tp.ChannelTelemetry(valid_flags=3, applied_output=0.1, encoder_counts=1234)
    t.channels[1] = tp.ChannelTelemetry(valid_flags=5, applied_output=-0.25, joint_angle_rad=1.2345)
    for i in range(2, 6):
        t.channels[i] = tp.ChannelTelemetry(valid_flags=1, applied_output=0.0)
    t.channels[6] = tp.ChannelTelemetry(valid_flags=9, applied_output=0.5, current_a=0.75)
    return t


class TestCrcAndCobs(unittest.TestCase):
    def test_crc_check_value(self):
        self.assertEqual(sp.crc16_ccitt_false(b'123456789'), 0x29B1)

    def test_crc_empty_is_init(self):
        self.assertEqual(sp.crc16_ccitt_false(b''), 0xFFFF)

    def test_cobs_published_vectors(self):
        vectors = {b'': '01', b'\x00': '0101', b'\x00\x00': '010101', b'\x00\x11\x00': '01021101',
                   b'\x11\x22\x00\x33': '0311220233', b'\x11\x22\x33\x44': '0511223344',
                   b'\x11\x00\x00\x00': '0211010101'}
        for raw, enc in vectors.items():
            self.assertEqual(sp.cobs_encode(raw).hex(), enc, raw.hex())
            self.assertEqual(sp.cobs_decode(bytes.fromhex(enc)), raw)

    def test_cobs_roundtrip_random_and_no_zero_in_output(self):
        rng = random.Random(1234)
        for _ in range(500):
            data = bytes(rng.choice([0, 0, 1, 255, rng.randrange(256)]) for _ in range(rng.randrange(0, 300)))
            enc = sp.cobs_encode(data)
            self.assertNotIn(0, enc)
            self.assertEqual(sp.cobs_decode(enc), data)

    def test_cobs_decode_rejects_bad_input(self):
        for bad in (b'\x00', b'\x05\x01', b'\x02\x00', b'\x03\x01\x00\x01'):
            with self.assertRaises(sp.ProtocolError):
                sp.cobs_decode(bad)


class TestCommandVectors(unittest.TestCase):
    """The same vectors appear in docs/serial_protocol.md for firmware tests."""
    VECTORS = [
        (command(2, 1, []), '05010204010101035d3d00'),
        (command(3, 2, []), '050103040201010321e300'),
        (command(4, 3), '0501040403010103d45d00'),
        (command(1, 0x01020304), '08010112040302010101010101010101010101010103ca6400'),
        (command(1, 5, motors({1: (1, 0.1), 2: (-1, 0.25), 7: (1, 1.0)})),
         '0501011205010102640306ff0101010101010105e803d01b00'),
    ]

    def test_vectors_encode_exactly(self):
        for cmd, hexstr in self.VECTORS:
            self.assertEqual(sp.encode_command(cmd).hex(), hexstr)

    def test_vectors_decode_back(self):
        for cmd, hexstr in self.VECTORS:
            (frame,) = decode_all(bytes.fromhex(hexstr))
            self.assertIsInstance(frame, sp.Frame)
            dec = sp.decode_command_frame(frame)
            self.assertEqual(dec.sequence, cmd.sequence & 0xFFFFFFFF)

    def test_setpoint_wire_values(self):
        cmd = command(1, 5, motors({1: (1, 0.1), 2: (-1, 0.25), 7: (1, 1.0)}))
        (frame,) = decode_all(sp.encode_command(cmd))
        self.assertEqual(sp.decode_command_frame(frame).permille, [100, -250, 0, 0, 0, 0, 1000])

    def test_tiny_demand_never_quantizes_to_stop(self):
        cmd = command(1, 0, motors({3: (1, 0.0001)}))
        (frame,) = decode_all(sp.encode_command(cmd))
        self.assertEqual(sp.decode_command_frame(frame).permille[2], 1)

    def test_invalid_command_cannot_be_encoded(self):
        for bad in (command(0, 0), command(1, 0, []), command(1, 0, motors({1: (1, math.nan)})),
                    command(1, 0, motors({1: (1, 1.5)})), command(1, 0, motors()[:6]),
                    command(4, 0, motors({1: (1, 0.5)}))):
            with self.assertRaises(sp.ProtocolError):
                sp.encode_command(bad)

    def test_stop_and_estop_encode_even_with_garbage_motors(self):
        for t in (2, 3):
            frame_bytes = sp.encode_command(command(t, 9, 'garbage'))
            (frame,) = decode_all(frame_bytes)
            self.assertEqual(frame.msg_type, t)
            self.assertEqual(len(frame.payload), 4)   # never any motor data

    def test_sequence_wraps_to_uint32(self):
        (frame,) = decode_all(sp.encode_command(command(2, 0xFFFFFFFF, [])))
        self.assertEqual(sp.decode_command_frame(frame).sequence, 0xFFFFFFFF)

    def test_decode_command_frame_strict(self):
        with self.assertRaises(sp.ProtocolError):
            sp.decode_command_frame(sp.Frame(1, 0x55, b'\x00' * 4))
        with self.assertRaises(sp.ProtocolError):
            sp.decode_command_frame(sp.Frame(1, sp.TYPE_SETPOINT, b'\x00' * 17))
        with self.assertRaises(sp.ProtocolError):
            sp.decode_command_frame(sp.Frame(1, sp.TYPE_STOP, b'\x00' * 5))
        with self.assertRaises(sp.ProtocolError):
            sp.decode_command_frame(sp.Frame(1, sp.TYPE_SETPOINT, struct.pack('<I7h', 0, 1001, 0, 0, 0, 0, 0, 0)))


class TestFrameDecoder(unittest.TestCase):
    def setUp(self):
        self.good = sp.encode_command(command(1, 7, motors({4: (1, 0.5)})))

    def test_single_frame(self):
        (f,) = decode_all(self.good)
        self.assertIsInstance(f, sp.Frame)

    def test_byte_by_byte_partial_feeds(self):
        dec = sp.FrameDecoder()
        out = []
        for b in self.good:
            out += dec.feed(bytes([b]))
        self.assertEqual(len(out), 1)
        self.assertIsInstance(out[0], sp.Frame)

    def test_two_frames_in_one_chunk_and_split_across_chunks(self):
        a, b = self.good, sp.encode_command(command(2, 8, []))
        self.assertEqual(len(decode_all(a + b)), 2)
        dec = sp.FrameDecoder()
        out = dec.feed(a + b[:3]) + dec.feed(b[3:])
        self.assertEqual([type(x) for x in out], [sp.Frame, sp.Frame])

    def test_every_single_bit_flip_is_detected(self):
        # No single-bit corruption of an encoded frame may yield an accepted frame.
        for i in range(len(self.good) - 1):          # exclude the delimiter itself
            for bit in range(8):
                data = bytearray(self.good)
                data[i] ^= 1 << bit
                if data[i] == 0:                      # created a delimiter: splits the frame
                    pass
                results = sp.FrameDecoder().feed(bytes(data) + b'\x00')
                self.assertFalse(any(isinstance(r, sp.Frame) for r in results), (i, bit))

    def test_truncated_frame_never_accepted(self):
        for n in range(1, len(self.good) - 1):
            results = sp.FrameDecoder().feed(self.good[:n] + b'\x00')
            self.assertFalse(any(isinstance(r, sp.Frame) for r in results), n)

    def test_garbage_then_valid_frame_resynchronizes(self):
        rng = random.Random(7)
        junk = bytes(rng.randrange(1, 256) for _ in range(60))
        results = sp.FrameDecoder().feed(junk + b'\x00' + self.good)
        self.assertIsInstance(results[-1], sp.Frame)
        self.assertEqual(sum(isinstance(r, sp.Frame) for r in results), 1)

    def test_overflow_is_bounded_and_recovers(self):
        dec = sp.FrameDecoder()
        results = dec.feed(b'\x55' * 5000)
        self.assertEqual(results, [])
        results = dec.feed(b'\x00' + self.good)
        self.assertEqual([r.kind for r in results if isinstance(r, sp.FrameError)], ['overflow'])
        self.assertIsInstance(results[-1], sp.Frame)

    def test_empty_chunks_ignored(self):
        results = sp.FrameDecoder().feed(b'\x00\x00\x00' + self.good + b'\x00\x00')
        self.assertEqual(len(results), 1)

    def test_error_kinds(self):
        bad_crc = bytearray(self.good)
        bad_crc[-2] ^= 0x01 if bad_crc[-2] != 0x01 else 0x02
        kinds = {r.kind for r in sp.FrameDecoder().feed(bytes(bad_crc)) if isinstance(r, sp.FrameError)}
        self.assertTrue(kinds <= {'crc', 'cobs', 'length'} and kinds)

    def test_random_noise_never_raises(self):
        rng = random.Random(99)
        dec = sp.FrameDecoder()
        for _ in range(200):
            dec.feed(bytes(rng.randrange(256) for _ in range(rng.randrange(1, 400))))


class TestTelemetry(unittest.TestCase):
    def frame(self, payload=None, version=sp.PROTOCOL_VERSION, mtype=sp.TYPE_TELEMETRY):
        payload = tp.build_telemetry_payload(sample_telemetry()) if payload is None else payload
        (f,) = decode_all(sp.encode_frame(mtype, payload, version))
        return f

    def test_sizes(self):
        self.assertEqual((tp.HEADER_LEN, tp.CHANNEL_LEN, tp.TELEMETRY_PAYLOAD_LEN), (20, 17, 139))

    def test_roundtrip(self):
        d = tp.parse_telemetry_frame(self.frame())
        self.assertEqual((d.telemetry_seq, d.firmware_state, d.accepted_seq_valid,
                          d.last_accepted_cmd_seq, d.command_age_ms), (1000, 3, True, 5, 40))
        self.assertAlmostEqual(d.channels[0].applied_output, 0.1)
        self.assertEqual(d.channels[0].encoder_counts, 1234)
        self.assertAlmostEqual(d.channels[1].joint_angle_rad, 1.2345, places=6)
        self.assertAlmostEqual(d.channels[6].current_a, 0.75)

    def test_invalid_readings_are_nan_not_zero(self):
        d = tp.parse_telemetry_frame(self.frame())
        self.assertTrue(math.isnan(d.channels[0].joint_angle_rad))   # M1 has no angle
        self.assertTrue(math.isnan(d.channels[0].current_a))
        self.assertTrue(math.isnan(d.channels[6].joint_angle_rad))
        self.assertEqual(d.channels[6].encoder_counts, 0)

    def test_values_behind_cleared_valid_bit_are_ignored(self):
        t = sample_telemetry()
        payload = bytearray(tp.build_telemetry_payload(t))
        # Corrupt M2's angle field (offset 20 + 17 + 1+2+2+4) but leave VALID_ANGLE cleared.
        off = tp.HEADER_LEN + tp.CHANNEL_LEN
        payload[off] = 1                                      # valid_flags = OUTPUT only
        struct.pack_into('<i', payload, off + 9, 777777)
        d = tp.parse_telemetry_frame(self.frame(bytes(payload)))
        self.assertTrue(math.isnan(d.channels[1].joint_angle_rad))

    def test_version_mismatch_is_distinguished(self):
        with self.assertRaises(tp.TelemetryError) as cm:
            tp.parse_telemetry_frame(self.frame(version=2))
        self.assertTrue(getattr(cm.exception, 'version_mismatch', False))
        self.assertEqual(cm.exception.remote_version, 2)

    def test_rejects_malformed_payloads(self):
        good = bytearray(tp.build_telemetry_payload(sample_telemetry()))
        cases = {}
        cases['short'] = bytes(good[:-1])
        cases['long'] = bytes(good) + b'\x00'
        b = bytearray(good); b[4] = 7; cases['fw_state'] = bytes(b)
        b = bytearray(good); b[5] = 2; cases['accepted flag'] = bytes(b)
        b = bytearray(good); b[14:18] = struct.pack('<I', 0x100); cases['global fault bits'] = bytes(b)
        b = bytearray(good); b[tp.HEADER_LEN] = 0x10; cases['valid bits'] = bytes(b)
        b = bytearray(good); struct.pack_into('<H', b, tp.HEADER_LEN + 1, 0x10); cases['motor fault bits'] = bytes(b)
        b = bytearray(good); struct.pack_into('<h', b, tp.HEADER_LEN + 3, 1001); cases['output range'] = bytes(b)
        for name, payload in cases.items():
            with self.assertRaises(tp.TelemetryError, msg=name):
                tp.parse_telemetry_frame(self.frame(payload))

    def test_wrong_message_type(self):
        with self.assertRaises(tp.TelemetryError):
            tp.parse_telemetry_frame(self.frame(mtype=0x42))

    def test_telemetry_vector_matches_doc(self):
        vec = sp.encode_frame(sp.TYPE_TELEMETRY, tp.build_telemetry_payload(sample_telemetry()))
        self.assertEqual(len(vec), 146)
        self.assertEqual(sp.crc16_ccitt_false(b'\x01\x81\x8b' + tp.build_telemetry_payload(sample_telemetry())),
                         struct.unpack('<H', sp.cobs_decode(vec[:-1])[-2:])[0])


if __name__ == '__main__':
    unittest.main()
