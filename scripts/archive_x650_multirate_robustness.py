#!/usr/bin/env python3
"""Package closed new mismatch runs, including failed audits, without overwriting evidence."""
import argparse
import json
from pathlib import Path

from archive_x650_robustness_raw import files_below, build_archives, identity, read

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, action='append', required=True)
    parser.add_argument('--stage', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    stage, out = args.stage.resolve(strict=True), args.output.resolve()
    if out.exists() or not out.is_relative_to(stage) or (ROOT/'.amsim/runtime.json').exists():
        raise ValueError('A new output under the stage and a closed simulation runtime are required')
    paths = files_below(stage)
    context = dict(scope='Fresh 24 s rigid mass/inertia mismatch runs with unchanged accepted 25/50 Hz backend; failed outcomes retained',
                   hardware_qualified=False, delay_tests_enabled=False, audits=[])
    for report_path in args.report:
        report = read(report_path)
        if report.get('kind') != 'x650_cpp_multirate_robustness_report_v1':
            raise ValueError('Unexpected report type')
        launch = read(Path(report['launch_directory'])/'launch.json')
        if launch.get('runtime_closed') is not True:
            raise ValueError('Cannot archive an open physical run')
        context['audits'].append(identity(report_path))
        for key in ('launch_directory','run_directory'):
            paths.extend(files_below(report[key]))
        paths.extend(files_below(report_path.parent))
        cfg = read(Path(report['launch_directory'])/'task.json')
        model = ROOT/'generated/imported_models'/cfg['model']
        paths.extend(p for p in files_below(model) if p.suffix.lower() in {'.sdf','.config','.urdf','.json','.yaml','.xml','.txt'})
    for pattern in ('scripts/*multirate*.py','tests/unit/*multirate*.py','scenarios/x650_cpp_multirate_mass*.yaml'):
        paths.extend(ROOT.glob(pattern))
    for folder in ROOT.glob('simulations/x650_cpp_multirate*'):
        paths.extend(folder.glob('*.py'))
    paths.append(ROOT/'scripts/archive_x650_robustness_raw.py')
    out.mkdir()
    receipt = build_archives(paths, root=ROOT, output_directory=out,
        prefix='multirate_robustness_evidence', context=context, max_bytes=48*1024*1024)
    print(json.dumps({k:receipt[k] for k in ('complete','verified_member_count','archives')},indent=2))


if __name__ == '__main__':
    main()
