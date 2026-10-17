"""Convert an actual g_assessment_log RAM dump to CSV, SVG and error statistics.

Run only on captured binary data; this script never generates synthetic measurements.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import struct


def export(source, destination):
    data = Path(source).read_bytes()
    if len(data) < 16:
        raise ValueError('日志缺少16字节头部')
    version, count, mode, duration = struct.unpack_from('<4I', data)
    if version != 1 or mode not in (4, 5) or not 1 <= count <= 1001:
        raise ValueError('不是有效的考核日志，或尚未记录数据')
    if len(data) < 16 + count * 24:
        raise ValueError('日志不完整')
    points = [struct.unpack_from('<Iffffi', data, 16 + i * 24) for i in range(count)]
    if any(not all(math.isfinite(x) for x in p[1:5]) for p in points):
        raise ValueError('日志中存在非有限反馈')
    if any(points[i][0] < points[i-1][0] for i in range(1, count)) or points[-1][0] > duration + 100:
        raise ValueError('日志时间不连续或超出配置时限')
    target_index, actual_index = (1, 2) if mode == 4 else (3, 4)
    errors = [p[target_index] - p[actual_index] for p in points]
    result = {
        'mode': 'speed' if mode == 4 else 'position', 'source': str(source),
        'unit': 'rad/s' if mode == 4 else 'rad', 'recorded_points': count,
        'duration_ms': duration, 'last_recorded_ms': points[-1][0],
        'mean_abs_error': sum(map(abs, errors)) / count,
        'rms_error': math.sqrt(sum(e*e for e in errors) / count),
        'max_abs_error': max(map(abs, errors)),
        'note': '20 ms抽样统计；是否完整结束及故障原因另见g_assessment_status和output_fault。未设定验收误差门槛。'
    }
    dest = Path(destination)
    dest.parent.mkdir(parents=True, exist_ok=True)
    with dest.with_suffix('.csv').open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)
        writer.writerow(['time_ms', 'speed_target_rad_s', 'speed_actual_rad_s',
                         'position_target_rad', 'position_actual_rad', 'requested_raw'])
        writer.writerows(points)
    dest.with_suffix('.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    values = [p[i] for p in points for i in (target_index, actual_index)]
    low, high = min(values), max(values)
    margin = max((high - low) * .1, .1)
    low -= margin
    high += margin
    def path(index):
        return ' '.join(f'{60 + p[0]/max(duration,1)*880:.2f},{350-(p[index]-low)/(high-low)*290:.2f}' for p in points)
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="420" viewBox="0 0 1000 420">
<rect width="1000" height="420" fill="white"/><g font-family="sans-serif" fill="#263238">
<text x="60" y="30" font-size="20">GM6020 {result['mode']} — captured feedback ({result['unit']})</text>
<path d="M60 55V350H945" fill="none" stroke="#90a4ae"/>
<text x="5" y="65">{high:.3f}</text><text x="5" y="350">{low:.3f}</text>
<text x="60" y="380">0 s</text><text x="890" y="380">{duration/1000:g} s</text>
<text x="300" y="400" fill="#d32f2f">Target</text><text x="450" y="400" fill="#1565c0">Measured</text>
<polyline points="{path(target_index)}" fill="none" stroke="#d32f2f" stroke-width="2"/>
<polyline points="{path(actual_index)}" fill="none" stroke="#1565c0" stroke-width="1.5"/></g></svg>'''
    dest.with_suffix('.svg').write_text(svg, encoding='utf-8')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path)
    parser.add_argument('--out', type=Path, default=Path('captures/assessment'))
    args = parser.parse_args()
    print(json.dumps(export(args.binary, args.out), ensure_ascii=False, indent=2))
