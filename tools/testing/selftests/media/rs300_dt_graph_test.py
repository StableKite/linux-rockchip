#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Fail-closed Mini2 graph test for the real Orange Pi 5 base/merged DTBs.

Usage: python3 rs300_dt_graph_test.py BASE.dtb MERGED.dtb
Both DTBs must be compiled/applied separately, e.g. by the Linux 6.1.172
DTC/fdtoverlay build. This checker does not build or apply the overlay.
All invariants are explicit: ``python -O`` cannot remove them.
"""

import argparse
import struct
import sys
from pathlib import Path

FDT_MAGIC = 0xD00DFEED
BEGIN_NODE, END_NODE, PROP, NOP, END = 1, 2, 3, 4, 9


class GraphError(ValueError):
    """A fatal invalid-input or invalid-graph condition."""


def require(condition, message):
    if not condition:
        raise GraphError(message)


def cell32(data, description):
    require(data is not None and len(data) == 4,
            f'{description} must be one 32-bit cell')
    return struct.unpack('>I', data)[0]


def parse_tree(path):
    try:
        data = Path(path).read_bytes()
    except OSError as exc:
        raise GraphError(f'Cannot read DTB {path}: {exc}') from exc

    require(len(data) >= 40, f'Truncated DTB header: {path}')
    (magic, total, struct_off, strings_off, reserve_off,
     version, last, _boot, strings_len, struct_len) = struct.unpack_from('>10I', data)
    require(magic == FDT_MAGIC, f'Invalid FDT magic: {path}')
    require(16 <= version <= 17 and last <= version,
            f'Unsupported/incompatible FDT header version: {path}')
    require(40 <= total <= len(data), f'Invalid FDT total size: {path}')
    require(40 <= reserve_off < total,
            f'Out-of-bounds FDT reservation section: {path}')
    require(40 <= struct_off < total and 40 <= strings_off <= total,
            f'Out-of-bounds FDT offsets: {path}')
    require(struct_len > 0 and struct_off + struct_len <= total,
            f'Out-of-bounds FDT structure block: {path}')
    require(strings_off + strings_len <= total,
            f'Out-of-bounds FDT strings block: {path}')
    struct_end = struct_off + struct_len
    require(struct_end <= strings_off or strings_off + strings_len <= struct_off,
            f'Overlapping FDT data regions: {path}')
    names = data[strings_off:strings_off + strings_len]
    pos = struct_off
    stack = []
    nodes = {}
    finished = False
    seen_root = False

    while pos < struct_end:
        require(pos + 4 <= struct_end, f'Truncated FDT token: {path}')
        token = struct.unpack_from('>I', data, pos)[0]
        pos += 4
        if token == BEGIN_NODE:
            require(pos < struct_end, f'Truncated FDT node: {path}')
            end = data.find(b'\0', pos, struct_end)
            require(end >= 0, f'Unterminated FDT node name: {path}')
            name = data[pos:end].decode('utf-8')
            pos = (end + 4) & ~3
            require(pos <= struct_end, f'Unaligned FDT node name: {path}')
            if not seen_root:
                require(name == '', f'Invalid root node: {path}')
                seen_root = True
            else:
                require(bool(stack), f'Second or detached root node: {path}')
                require(bool(name) and '/' not in name,
                        f'Invalid child node name in {path}: {name!r}')
            stack.append(name)
            node = '/' + '/'.join(n for n in stack if n)
            require(node not in nodes, f'Duplicate DTB node: {node}')
            nodes[node] = {}
        elif token == END_NODE:
            require(bool(stack), f'Unbalanced END_NODE in {path}')
            stack.pop()
        elif token == PROP:
            require(bool(stack), f'Property outside FDT node in {path}')
            require(pos + 8 <= struct_end, f'Truncated FDT property header: {path}')
            size, offset = struct.unpack_from('>2I', data, pos)
            pos += 8
            require(offset < len(names), f'Invalid FDT property name offset: {path}')
            name_end = names.find(b'\0', offset)
            require(name_end > offset, f'Invalid FDT property name: {path}')
            name = names[offset:name_end].decode('utf-8')
            require(pos + size <= struct_end, f'Truncated FDT property value: {name}')
            node = '/' + '/'.join(n for n in stack if n)
            require(name not in nodes[node], f'Duplicate FDT property {node}/{name}')
            nodes[node][name] = data[pos:pos + size]
            pos = (pos + size + 3) & ~3
            require(pos <= struct_end, f'Invalid FDT property alignment: {path}')
        elif token == NOP:
            continue
        elif token == END:
            require(seen_root and not stack, f'Unclosed FDT nodes: {path}')
            require(pos == struct_end, f'Unexpected trailing structure data: {path}')
            finished = True
            break
        else:
            raise GraphError(f'Invalid FDT token {token} in {path}')

    require(finished, f'Missing FDT_END marker: {path}')
    require('/' in nodes, f'FDT root node missing: {path}')
    return nodes


def collect_graph(nodes, label):
    owners = {}
    duplicate = []
    for path, props in nodes.items():
        handles = [cell32(props[key], f'{path}/{key}')
                   for key in ('phandle', 'linux,phandle') if key in props]
        if not handles:
            continue
        require(all(0 < h < 0xFFFFFFFF for h in handles),
                f'Invalid phandle on {path}: {handles}')
        require(len(set(handles)) == 1,
                f'phandle/linux,phandle mismatch on {path}')
        handle = handles[0]
        other = owners.get(handle)
        if other is not None and other != path:
            duplicate.append((handle, other, path))
        owners[handle] = path

    links = {}
    missing = []
    for path, props in nodes.items():
        if 'remote-endpoint' not in props:
            continue
        handle = cell32(props['remote-endpoint'], f'{path}/remote-endpoint')
        dest = owners.get(handle)
        links[path] = dest
        if dest is None:
            missing.append((path, handle))
    require(not duplicate, f'{label}: duplicate phandles: {duplicate!r}')
    require(not missing, f'{label}: dangling remote-endpoint: {missing!r}')
    return links, owners


def enabled(nodes, path):
    part = ''
    for segment in path.split('/'):
        if not segment:
            continue
        part += '/' + segment
        status = nodes.get(part, {}).get('status', b'okay\0')
        if status != b'okay\0' and status != b'ok\0':
            return False
    return True


def check(base_path, merged_path):
    old = parse_tree(base_path)
    new = parse_tree(merged_path)
    receiver = '/csi2-dphy0/ports/port@0/endpoint@0'
    sensor_node = '/i2c@fec90000/camera@3c'
    sensor = sensor_node + '/port/endpoint'
    legacy_node = '/i2c@fec90000/ov13850-1@10'
    legacy = legacy_node + '/port/endpoint'

    require(sensor_node not in old, 'Mini2 camera unexpectedly present in base DTB')
    for what, nodes in [('base', old), ('merged', new)]:
        for path in (receiver, legacy_node, legacy):
            require(path in nodes, f'{what}: required Orange Pi camera1 path missing: {path}')
    require(sensor_node in new and sensor in new, 'Mini2 node/endpoint missing from merged DTB')
    old_links, old_owners = collect_graph(old, 'base')
    new_links, new_owners = collect_graph(new, 'merged')
    require(len(new_links) == len(old_links) + 1,
            f'Unexpected remote-endpoint count: {len(old_links)} -> {len(new_links)}')
    old_receiver_id = cell32(old[receiver].get('phandle'), 'Original CSI phandle')
    new_receiver_id = cell32(new[receiver].get('phandle'), 'Merged CSI phandle')
    mini2_id = cell32(new[sensor].get('phandle'), 'Mini2 phandle')
    require(old_receiver_id == new_receiver_id,
            'Existing camera1 receiver phandle overwritten')
    require(old_links.get(legacy) == receiver, 'Base disabled OV13850 is not wired as expected')
    require(new_links.get(legacy) == receiver, 'Disabled OV13850 reference lost')
    require(new[legacy_node].get('status') == b'disabled\0',
            'OV13850 unexpectedly enabled')
    require(new_links.get(sensor) == receiver, 'Mini2 -> CSI graph edge invalid')
    require(new_links.get(receiver) == sensor, 'CSI -> Mini2 graph edge invalid')
    require(new_owners.get(mini2_id) == sensor, 'Mini2 phandle ownership invalid')
    for path, dest in new_links.items():
        if not enabled(new, path):
            require(path == legacy or new_links.get(dest) == path,
                    f'Unexpected non-reciprocal disabled endpoint: {path}')
        else:
            require(new_links.get(dest) == path,
                    f'Non-reciprocal enabled endpoint: {path} -> {dest}')
    for path, dest in old_links.items():
        if path != receiver:
            require(new_links.get(path) == dest,
                    f'Previously existing camera graph edge changed: {path}')
    require(new[legacy].get('remote-endpoint') == old[legacy].get('remote-endpoint'),
            'OV13850 reference was rewritten')
    require(new[sensor].get('data-lanes') == struct.pack('>2I', 1, 2),
            'Mini2 CSI data lanes are not 1,2')
    require(new[sensor].get('link-frequencies') == (200000000).to_bytes(8, 'big'),
            'Mini2 CSI link frequency is not 200 MHz')
    require(new[sensor].get('bus-type') == struct.pack('>I', 4),
            'Mini2 CSI bus is not D-PHY')
    require(new[sensor_node].get('compatible') == b'infisense,mini2-640\0',
            'Wrong Mini2 compatible property')
    require(new[sensor_node].get('reg') == struct.pack('>I', 0x3c),
            'Wrong Mini2 I2C address')
    # Endpoint reciprocity is not enough. A disabled I2C controller,
    # physical D-PHY, CSI host, or CIF receiver makes the path unusable
    # even when its graph links remain valid. Check every active pipeline
    # node and its ancestors using the exact R70 Orange Pi 5 node paths.
    required_pipeline = (
        '/i2c@fec90000',
        sensor_node,
        '/csi2-dphy0-hw@fedc0000',
        '/csi2-dphy0',
        '/mipi2-csi2-hw@fdd30000',
        '/mipi2-csi2',
        '/rkcif@fdce0000',
        '/rkcif-mipi-lvds2',
    )
    for path in required_pipeline:
        require(path in new, f'Required Mini2 pipeline node missing: {path}')
        require(enabled(new, path),
                f'Required Mini2 pipeline node is not enabled: {path}')
    require(new[sensor_node].get('status') == b'okay\0',
            'Mini2 sensor is missing explicit status=okay')
    require(not enabled(new, legacy), 'OV13850 legacy endpoint unexpectedly active')
    print(f'BASE_GRAPH remote={len(old_links)} dangling=0 duplicate_phandles=0')
    print(f'MERGED_GRAPH remote={len(new_links)} dangling=0 duplicate_phandles=0')
    print(f'RS300_GRAPH_GATE=PASS receiver_phandle=0x{new_receiver_id:x}')
    print('NOTE: disabled OV13850 remains non-reciprocal, but all active links are reciprocal')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('base_dtb', type=Path)
    parser.add_argument('merged_dtb', type=Path)
    args = parser.parse_args(argv)
    try:
        check(args.base_dtb, args.merged_dtb)
    except Exception as exc:
        print(f'RS300_GRAPH_GATE=FAIL {type(exc).__name__}: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
