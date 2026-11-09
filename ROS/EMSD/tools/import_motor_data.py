"""Import the measured workbook without requiring Excel dependencies.

command is a lookup coordinate (ESC signal - 200) / 1300, not a wire command.
No zero-thrust/off measurement is manufactured.
"""
import csv
from collections import defaultdict
from pathlib import Path
from statistics import mean
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[1]
NS = {'m': 'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}


def read_workbook():
    with zipfile.ZipFile(ROOT / 'motor_data.xlsx') as archive:
        shared = [''.join(s.itertext()) for s in ET.fromstring(archive.read('xl/sharedStrings.xml')).findall('m:si', NS)] if 'xl/sharedStrings.xml' in archive.namelist() else []
        sheet = ET.fromstring(archive.read('xl/worksheets/sheet1.xml'))
        rows = []
        for row in sheet.findall('.//m:row', NS):
            cells = {}
            for cell in row.findall('m:c', NS):
                value = cell.find('m:v', NS)
                if value is not None:
                    text = value.text
                    cells[''.join(filter(str.isalpha, cell.get('r')))] = shared[int(text)] if cell.get('t') == 's' else text
            if int(row.get('r')) == 2:
                expected = {'A': 'Time (s)', 'B': 'ESC signal', 'I': 'Torque (N·m)', 'J': 'Thrust (kgf)', 'K': 'Voltage (V)', 'L': 'Current (A)'}
                if any(cells.get(k) != v for k, v in expected.items()):
                    raise ValueError('Workbook columns changed; check units before importing')
            if int(row.get('r')) >= 3 and 'B' in cells:
                rows.append([float(cells[k]) for k in ('A', 'B', 'I', 'J', 'K', 'L', 'M')])
    if not rows:
        raise ValueError('No measured data found')
    return rows


def main():
    rows = read_workbook()
    with (ROOT / 'control/motor_data.raw.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['time_s', 'esc_signal', 'torque_nm_signed', 'thrust_kgf_signed', 'voltage_v', 'current_a', 'electrical_rpm'])
        writer.writerows(rows)
    groups = defaultdict(list)
    for row in rows:
        groups[round(row[1], 6)].append(row)
    low, high = min(groups), max(groups)
    with (ROOT / 'control/motor_data.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['command', 'thrust_n', 'torque_nm', 'current_a', 'esc_signal', 'voltage_v', 'sample_count'])
        for signal, samples in sorted(groups.items()):
            writer.writerow([(signal-low)/(high-low), mean(abs(r[3])*9.80665 for r in samples),
                             mean(abs(r[2]) for r in samples), mean(r[5] for r in samples),
                             signal, mean(r[4] for r in samples), len(samples)])
    print(f'Imported {len(rows)} measurements into {len(groups)} lookup points; ESC range {low:g}–{high:g}.')


if __name__ == '__main__':
    main()
