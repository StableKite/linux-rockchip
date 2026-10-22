#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Check compiled Orange Pi 5 Max IMX415 DTBs (requires dtc's fdtget).

Usage: python3 imx415_opi5max_dt_test.py DIRECTORY_CONTAINING_THE_THREE_DTBS
This verifies topology and enabled dependencies, not camera operation.
"""

import argparse
from pathlib import Path
import subprocess
import sys


def require(condition, message):
    if not condition:
        raise ValueError(message)


class Tree:
    def __init__(self, path):
        self.path = path
        self.properties = {}

    def run(self, *args):
        result = subprocess.run(['fdtget', *args[:2], str(self.path), *args[2:]],
                                check=True, capture_output=True, text=True)
        return result.stdout.strip()

    def props(self, node):
        if node not in self.properties:
            # -p takes one option, unlike the -t TYPE calls below.
            result = subprocess.run(['fdtget', '-p', str(self.path), node],
                                    check=True, capture_output=True, text=True)
            self.properties[node] = result.stdout.splitlines()
        return self.properties[node]

    def cells(self, node, prop):
        return [int(value, 16) for value in self.run('-t', 'x', node, prop).split()]

    def string(self, node, prop):
        return self.run('-t', 's', node, prop)

    def enabled(self, node):
        for path in [node, *[str(p) for p in Path(node).parents]]:
            if 'status' in self.props(path):
                if self.string(path, 'status') not in ('ok', 'okay'):
                    return False
        return True

    def link(self, left, right):
        for source, target in ((left, right), (right, left)):
            require(self.enabled(source), f'disabled graph endpoint: {source}')
            require(self.cells(source, 'remote-endpoint') == self.cells(target, 'phandle'),
                    f'wrong graph link: {source} -> {target}')


I2C = ('/i2c@fec80000', '/i2c@feab0000', '/i2c@fec90000')
PHY = ('/csi2-dphy0', '/csi2-dphy3', '/csi2-dcphy0')
HOST = ('/mipi2-csi2', '/mipi4-csi2', '/mipi0-csi2')
CIF = ('/rkcif-mipi-lvds2', '/rkcif-mipi-lvds4', '/rkcif-mipi-lvds')
ISP = ('/rkisp0-vir2', '/rkisp1-vir1', '/rkisp0-vir0')
HW = ('/csi2-dphy0-hw@fedc0000', '/csi2-dphy1-hw@fedc8000', '/phy@feda0000')
ISP_HW = ('/rkisp@fdcb0000', '/rkisp@fdcc0000', '/rkisp@fdcb0000')
ISP_MMU = ('/iommu@fdcb7f00', '/iommu@fdcc7f00', '/iommu@fdcb7f00')
HOST_HW = ('/mipi2-csi2-hw@fdd30000', '/mipi4-csi2-hw@fdd50000',
           '/mipi0-csi2-hw@fdd10000')


def check(path, port):
    tree = Tree(path)
    sensor = I2C[port] + '/imx415@1a'
    require('rockchip,rk3588-orangepi-5-max' in tree.string('/', 'compatible').split(),
            'wrong board compatible')
    require(tree.string(sensor, 'compatible') == 'sony,imx415', 'wrong sensor')
    require(tree.cells(sensor, 'reg') == [0x1a], 'wrong I2C address')
    require(tree.cells(sensor, 'rockchip,camera-module-index') == [port], 'wrong port')
    require(tree.cells(sensor, 'assigned-clock-rates') == [37125000], 'wrong XVCLK')
    require(tree.string(sensor, 'clock-names') == 'xvclk', 'wrong clock name')
    legacy = I2C[port] + f'/ov13855-{port}@36'
    for prop in ('clocks', 'pinctrl-0'):
        require(tree.cells(sensor, prop) == tree.cells(legacy, prop),
                f'{prop} does not match connector wiring')
    require(tree.cells(sensor, 'assigned-clocks') == tree.cells(sensor, 'clocks'),
            'wrong assigned clock')
    for prop, legacy_prop in (('reset-gpios', 'reset-gpios'),
                              ('power-gpios', 'pwdn-gpios')):
        actual = tree.cells(sensor, prop)
        require(actual[:2] == tree.cells(legacy, legacy_prop)[:2] and actual[2:] == [1],
                f'{prop}: wrong GPIO or active-low flag')

    rx = PHY[port] + '/ports/port@0/endpoint@1'
    sensor_ep = sensor + '/port/endpoint'
    tree.link(sensor_ep, rx)
    for endpoint in (sensor_ep, rx):
        require(tree.cells(endpoint, 'data-lanes') == [1, 2, 3, 4], 'wrong CSI lanes')
    tree.link(PHY[port] + '/ports/port@1/endpoint@0',
              HOST[port] + '/ports/port@0/endpoint@1')
    tree.link(HOST[port] + '/ports/port@1/endpoint@0', CIF[port] + '/port/endpoint')
    tree.link(CIF[port] + '-sditf/port/endpoint', ISP[port] + '/port/endpoint@0')
    require('remote-endpoint' not in tree.props(legacy + '/port/endpoint'),
            'replaced sensor retains a stale graph reference')

    for node in (I2C[port], sensor, HW[port], HOST_HW[port], '/rkcif@fdce0000',
                 '/iommu@fdce0800', ISP_HW[port], ISP_MMU[port]):
        require(tree.enabled(node), f'disabled pipeline dependency: {node}')
    require(tree.cells(ISP_HW[port], 'iommus') == tree.cells(ISP_MMU[port], 'phandle'),
            'wrong ISP IOMMU')
    for other in range(3):
        if other != port:
            for node in (PHY[other], HOST[other], CIF[other], CIF[other] + '-sditf', ISP[other]):
                require(not tree.enabled(node), f'unselected camera path enabled: {node}')
        for name in (f'ov13850-{other}@10', f'ov13855-{other}@36'):
            require(not tree.enabled(I2C[other] + '/' + name), 'competing sensor enabled')
    print(f'Camera{port}: PASS (4 lanes, reciprocal graph, active PHY/CSI/CIF/ISP/MMU)')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dtb_directory', type=Path)
    args = parser.parse_args()
    try:
        for port in range(3):
            check(args.dtb_directory / f'rk3588-orangepi-5-max-imx415-cam{port}.dtb', port)
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f'FAIL: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
