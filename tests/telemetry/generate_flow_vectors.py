"""Independent wire fixtures: stdlib struct + bitwise X25 CRC, no MAVLink library packing."""
from pathlib import Path
import struct


def frame(x: float, y: float, quality: int, seq: int, version: int) -> bytes:
    payload = struct.pack('<QIfffffIfhBB', 123456789, 20000, x, y,
                          float('nan'), float('nan'), float('nan'),
                          20000, 0.777, 0, 0, quality)
    assert len(payload) == 44
    if version == 2:
        payload = payload.rstrip(b'\0') or b'\0'
        header = bytes([len(payload), 0, 0, seq, 2, 88, 106, 0, 0])
        magic = b'\xfd'
    else:
        assert version == 1
        header = bytes([len(payload), seq, 2, 88, 106])
        magic = b'\xfe'
    crc = 0xffff
    for byte in header + payload + bytes([138]):
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return magic + header + payload + struct.pack('<H', crc)


def write_vectors(destination: Path) -> None:
    # Nonzero quality, omitted zero quality with nonzero motion, screenshot-like zeros,
    # and stationary observation with valid nonzero quality.
    cases = [(0.125, -0.25, 87, 2), (0.125, -0.25, 0, 2),
             (0.0, 0.0, 0, 2), (0.0, 0.0, 255, 2),
             (0.125, -0.25, 87, 1), (0.125, -0.25, 0, 1),
             (0.0, 0.0, 0, 1), (0.0, 0.0, 255, 1),
             (0.125, -0.25, 87, 2)]
    definitions = ['/* Generated independent MAVLink v1/v2 wire test vectors. */']
    for index, (x, y, quality, version) in enumerate(cases):
        data = frame(x, y, quality, index + 10, version)
        values = ', '.join(f'0x{byte:02x}' for byte in data)
        definitions.append(f'static const uint8_t wire_case_{index}[] = {{{values}}};')
    definitions += [
        'static const struct { const uint8_t *bytes; unsigned size; float x, y; unsigned quality; } wire_cases[] = {'
    ]
    for index, (x, y, quality, version) in enumerate(cases):
        definitions.append(f'    {{wire_case_{index}, sizeof(wire_case_{index}), {x}f, {y}f, {quality}U}},')
    definitions.append('};\n')
    destination.write_text('\n'.join(definitions), encoding='utf-8')
