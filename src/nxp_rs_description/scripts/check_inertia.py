#!/usr/bin/env python3
"""Check this robot's CAD-to-URDF inertia transfer and Gazebo SDF conversion.

Run after sourcing ROS 2. Requires numpy, xacro, and Gazebo Classic's gz.
The link_2 CAD rotation is an explicit assumption, not validated by this audit.
"""

from pathlib import Path
import argparse
import os
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]
COMPONENTS = ('xx', 'xy', 'xz', 'yy', 'yz', 'zz')


def tensor(values):
    xx, xy, xz, yy, yz, zz = values
    return np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live', action='store_true', help='Also compare the running nxp_rs model')
    parser.add_argument('--gazebo-port', default='11346', help='Inspection Gazebo master port')
    args = parser.parse_args()
    urdf_text = subprocess.check_output([
        'xacro', str(PACKAGE / 'urdf/nxp_rs_gazebo.xacro'),
    ], text=True)
    robot = ET.fromstring(urdf_text)
    with tempfile.TemporaryDirectory(prefix='nxp_inertia_') as directory:
        urdf_path = Path(directory) / 'robot.urdf'
        urdf_path.write_text(urdf_text)
        sdf_text = subprocess.check_output(['gz', 'sdf', '-p', str(urdf_path)], text=True)
    sdf = ET.fromstring(sdf_text)
    sdf_links = {link.get('name'): link for link in sdf.findall('model/link')}
    failures = []
    live_links = {}
    if args.live:
        env = dict(os.environ, GAZEBO_MASTER_URI='http://127.0.0.1:' + args.gazebo_port)
        live = subprocess.check_output(['gz', 'model', '-m', 'nxp_rs', '-i'],
                                       text=True, env=env, timeout=15)
        for block in re.findall(r'^link \{\n(.*?)^\}', live, re.M | re.S):
            name = re.search(r'^  name: "nxp_rs::([^"]+)"', block, re.M)[1]
            live_links[name] = re.search(r'  inertial \{\n(.*?)\n  \}', block, re.S)[1]
        print('Live model bench mount:', 'world_to_base' in live)
    total_mass = 0.0
    count = 0
    print('link           mass(kg)  min principal inertia  CAD match  SDF match')
    for link in robot.findall('link'):
        name = link.get('name')
        inertial = link.find('inertial')
        if name == 'world':
            continue
        count += 1
        if inertial is None:
            failures.append(f'{name}: missing inertia')
            continue
        mass = float(inertial.find('mass').get('value'))
        com = np.fromstring(inertial.find('origin').get('xyz'), sep=' ')
        rpy = np.fromstring(inertial.find('origin').get('rpy', '0 0 0'), sep=' ')
        values = np.array([float(inertial.find('inertia').get('i' + c)) for c in COMPONENTS])
        if not np.all(np.isfinite(np.r_[mass, com, rpy, values])):
            failures.append(f'{name}: nonfinite inertia data')
            continue
        moments = np.linalg.eigvalsh(tensor(values))
        if mass <= 0 or moments[0] <= 0 or moments[2] > moments[0] + moments[1] + 1e-12:
            failures.append(f'{name}: nonphysical mass / principal moments')
        report = (PACKAGE / 'meshes' / (name.replace('_', '-') + '-inertial.txt')).read_text()
        cad_mass = float(re.search(r'Mass \(user-overridden\) = ([\d.]+)', report)[1])
        cad_com = np.array([float(re.search(r'\b' + axis + r' = ([-\d.]+)', report)[1])
                            for axis in 'XYZ'])
        cad_values = np.array([float(re.search(r'\bL' + c + r' = ([-\d.]+)', report)[1])
                               for c in COMPONENTS])
        cad_values[[1, 2, 4]] *= -1  # Positive products -> inertia matrix entries.
        rotation = np.diag([1, -1, -1]) if name == 'link_2' else np.eye(3)
        cad_ok = (abs(mass - cad_mass) < 1e-12
                  and np.allclose(com, rotation @ cad_com, atol=1e-12, rtol=0)
                  and np.allclose(rpy, 0, atol=1e-12, rtol=0)
                  and np.allclose(tensor(values), rotation @ tensor(cad_values) @ rotation.T,
                                  atol=1e-12, rtol=0))
        sdf_link = sdf_links.get(name)
        sdf_ok = False
        if sdf_link is not None:
            sdf_mass = float(sdf_link.findtext('inertial/mass'))
            sdf_pose = np.fromstring(sdf_link.findtext('inertial/pose'), sep=' ')
            sdf_values = [float(sdf_link.findtext('inertial/inertia/i' + c)) for c in COMPONENTS]
            sdf_ok = (abs(sdf_mass - mass) < 1e-12
                      and np.allclose(sdf_pose, np.r_[com, rpy], atol=1e-12, rtol=0)
                      and np.allclose(sdf_values, values, atol=1e-12, rtol=0))
        if not cad_ok:
            failures.append(f'{name}: differs from CAD with documented frame mapping')
        if not sdf_ok:
            failures.append(f'{name}: missing or changed in SDF conversion')
        if args.live:
            live_block = live_links.get(name, '')
            fields = ('mass', 'x', 'y', 'z', 'ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')
            matches = [re.search(r'\b' + field + r': ([-+\deE.]+)', live_block) for field in fields]
            live_ok = all(matches) and np.allclose(
                [float(match[1]) for match in matches], np.r_[mass, com, values],
                atol=1e-12, rtol=0,
            )
            orientation = re.search(r'orientation \{(.*?)\}', live_block, re.S)
            if orientation:
                quaternion = [float(re.search(r'\b' + axis + r': ([-+\deE.]+)',
                                             orientation[1])[1]) for axis in 'xyzw']
                live_ok = live_ok and np.allclose(quaternion, [0, 0, 0, 1], atol=1e-12, rtol=0)
            else:
                live_ok = False
            if not live_ok:
                failures.append(f'{name}: differs from live Gazebo inertia')
        total_mass += mass
        print(f'{name:14} {mass:8.3f}  {moments[0]:21.9g}  {str(cad_ok):9}  {sdf_ok}')
    print(f'Physical links: {count}; total mass: {total_mass:.3f} kg')
    print('ASSUMPTION: link_2 uses CAD-to-link Rx(pi); all other CAD frames align with link frames.')
    print('CAD agreement does not establish real hardware mass distribution or controller stability.')
    for failure in failures:
        print('FAIL:', failure)
    if failures:
        raise SystemExit(1)
    print('PASS: positive masses, valid principal moments, CAD transfer, and SDF preservation.')
    if args.live:
        print(f'PASS: all {count} links match live Gazebo mass, COM, orientation, and inertia.')


if __name__ == '__main__':
    main()
