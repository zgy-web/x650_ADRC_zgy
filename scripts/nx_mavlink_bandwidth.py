"""Offline full-duplex 8N1 budget, using pinned maximum MAVLink payload sizes."""
import argparse
import json
from pathlib import Path

DEFAULT = Path(__file__).resolve().parents[1] / 'configs/hardware/nx_mavlink_uart_v1.json'


def calculate(profile, stage, signed=False):
    if stage not in ('bench', 'mpc'):
        raise ValueError('Unknown stage')
    overhead = profile['unsigned_overhead_bytes'] + (profile['signature_extra_bytes'] if signed else 0)
    capacity = profile['baud'] / profile['serial_bits_per_byte']
    reserve = profile['asynchronous_reserve_bytes_s_per_direction']
    result = {'stage': stage, 'signed_size_budget': signed, 'baud': profile['baud'],
              'capacity_bytes_s_per_direction': capacity, 'directions': {}, 'measured': False}
    seen = set()
    for m in profile['messages']:
        key = (m['direction'], m['id'])
        if key in seen or m['direction'] not in ('px4_to_nx', 'nx_to_px4'):
            raise ValueError('Duplicate/invalid direction and message ID')
        seen.add(key)
        rate = m[stage + '_hz']
        if not 0 <= rate <= 1000 or not 0 <= m['max_payload'] <= 255:
            raise ValueError('Invalid size/rate')
        if rate:
            d = result['directions'].setdefault(m['direction'], {'messages': [], 'periodic_bytes_s': 0})
            size = m['max_payload'] + overhead
            d['messages'].append({'name': m['name'], 'id': m['id'], 'hz': rate,
                                  'max_frame_bytes': size, 'bytes_s': size * rate,
                                  'serialization_ms': 1000 * size / capacity})
            d['periodic_bytes_s'] += size * rate
    for direction, d in result['directions'].items():
        d['reserve_bytes_s'] = reserve
        d['total_budget_bytes_s'] = d['periodic_bytes_s'] + reserve
        d['link_fraction'] = d['total_budget_bytes_s'] / capacity
        d['uart_fits'] = d['total_budget_bytes_s'] <= capacity
        d['px4_stream_budget_fits'] = (d['total_budget_bytes_s'] <= profile['px4_tx_budget_bytes_s']
                                       if direction == 'px4_to_nx' else None)
    result['budget_fits'] = all(d['uart_fits'] and d['px4_stream_budget_fits'] is not False
                                for d in result['directions'].values())
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profile', type=Path, default=DEFAULT)
    p.add_argument('--output', type=Path)
    a = p.parse_args()
    profile = json.loads(a.profile.read_text(encoding='utf-8'))
    result = [calculate(profile, stage, signed) for stage in ('bench', 'mpc') for signed in (False, True)]
    text = json.dumps(result, indent=2)
    if a.output:
        a.output.parent.mkdir(parents=True, exist_ok=True)
        with a.output.open('x', encoding='utf-8') as f:
            f.write(text + '\n')
    print(text)
    return 0 if all(r['budget_fits'] for r in result) else 2


if __name__ == '__main__':
    raise SystemExit(main())
