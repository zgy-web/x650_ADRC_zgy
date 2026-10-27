"""Bound C++ raw-data / full-builder reproduction of existing Python solves.

No solver is generated here. Original state/history is copied without changes;
each generated C call is reset by the CLI and never permits publication. Timing
samples are observations, not a scheduling or original-issue deadline proof.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np


MAPPING_TOLERANCE=1e-10
SOLUTION_TOLERANCE=1e-6
REPETITIONS=3


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def canonical(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def write(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists():raise ValueError('Refusing to replace evidence: '+str(path))
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
def identity(path):return dict(path=str(Path(path).resolve()),sha256=sha(path))


def prepare(out,selection_paths,numerical_dir,mode):
    out.mkdir(parents=True,exist_ok=False);config=None;rows=[];seen=set()
    for selection_path in selection_paths:
        doc=read(selection_path)
        if doc.get('kind')!='standard_nmpc_bound_fixture_selection_v1':raise ValueError('Predeclared bound selection required')
        if config is None:config=doc['config']
        if canonical(config)!=canonical(doc['config']):raise ValueError('Mixed physical configuration')
        for wrapped in doc['cases']:
            key=(wrapped['source_sha256'],wrapped['source_index'])
            if key in seen:raise ValueError('Duplicate original source record')
            seen.add(key);index=len(rows);case=wrapped['case']
            if not all(k in case for k in ('id','memory','input','oracle')):raise ValueError('Incomplete original fixture')
            payload=dict(kind='x650_standard_native_bound_original_case_v1',config=config,cases=[wrapped])
            filename=out/'inputs'/f'original_{index:03d}.json';write(filename,payload)
            rows.append(dict(index=index,case_id=case['id'],source=wrapped['source'],source_sha256=wrapped['source_sha256'],
                             source_index=wrapped['source_index'],case_sha256=canonical(case),
                             original_fixture=identity(filename),original_case=case,
                             numeric_case=str((numerical_dir/f'case_{index:03d}/numeric_case.json').resolve()),
                             python_result=str((numerical_dir/f'case_{index:03d}/acados_{mode}.json').resolve())))
    if len(rows)!=16:raise ValueError('This qualification is bound to all 16 preselected original cases')
    combined=out/'inputs/original_cases.json'
    write(combined,dict(kind='x650_standard_native_bound_original_cases_v1',config=config,
                       cases=[dict(source=r['source'],source_sha256=r['source_sha256'],source_index=r['source_index'],case=r['original_case']) for r in rows]))
    manifest=dict(kind='x650_standard_native_reproduction_inputs_v1',mode=mode,count=len(rows),
                  numerical_dir=str(numerical_dir.resolve()),physical_config_sha256=canonical(config),
                  config=config,original_cases=identity(combined),selections=[identity(p) for p in selection_paths],
                  mapping_absolute_tolerance=MAPPING_TOLERANCE,solution_absolute_tolerance=SOLUTION_TOLERANCE,
                  repetitions=REPETITIONS,rows=rows)
    write(out/'inputs_manifest.json',manifest)
    return manifest


def run_cli(binary,input_path,output_path,*,claimed_sha=None,expected_rejection=False):
    if output_path.exists():raise ValueError('Output already exists: '+str(output_path))
    args=[str(binary),str(input_path),sha(input_path) if claimed_sha is None else claimed_sha,str(output_path),str(REPETITIONS)]
    started=time.monotonic();result=subprocess.run(args,capture_output=True,text=True,timeout=1800)
    evidence=dict(command=args,returncode=result.returncode,wall_s=time.monotonic()-started,
                  stdout=result.stdout,stderr=result.stderr,output_exists=output_path.exists())
    write(output_path.with_suffix('.process.json'),evidence)
    if expected_rejection:
        evidence['passed']=result.returncode==2 and not output_path.exists()
        return evidence
    if result.returncode!=0 or not output_path.exists():raise RuntimeError('Native reproduction failed: '+result.stderr)
    return read(output_path)


def max_error(actual,expected):
    a=np.asarray(actual,dtype=float).ravel();b=np.asarray(expected,dtype=float).ravel()
    if a.shape!=b.shape:raise ValueError(f'Shape mismatch: {a.shape} != {b.shape}')
    if not np.isfinite(a).all() or not np.isfinite(b).all():raise ValueError('Nonfinite compared numerical values')
    return float(np.max(abs(a-b))) if a.size else 0.


def stats(values):
    a=np.asarray(values,dtype=float)
    return dict(count=len(values),min=float(a.min()),median=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(a.max())) if len(a) else None


def verify_compiled_identity(report,mode,graph,config):
    expected='SQP' if mode=='sqp' else 'SQP_RTI'
    if report.get('compiled_solver_mode')!=expected:raise ValueError('C++ report lacks matching compiled solver mode')
    if report.get('compiled_graph_sha256')!=graph or report.get('compiled_config_sha256')!=config:
        raise ValueError('C++ report differs from bound symbolic graph/physical configuration')


def qualify(out,manifest,binary_dir):
    from x650_nmpc_standard.model import build_model, NumericalCase, rollout
    mode=manifest['mode'];numerical=Path(manifest['numerical_dir']);python_report=read(numerical/'report.json')
    if not python_report.get('complete') or python_report['selected_count']!=16:raise ValueError('Complete 16-case Python qualification required')
    if not python_report.get('source_files_unchanged'):raise ValueError('Python qualification source identity was not stable')
    if python_report['configuration_sha256']!=manifest['physical_config_sha256']:raise ValueError('Python configuration changed')
    python_delivery=read(numerical/'delivery_manifest.json')
    for item in python_delivery['files']:
        if sha(item['path'])!=item['sha256']:raise ValueError('Python numerical evidence changed: '+item['path'])
    for item in manifest['selections']+[manifest['original_cases']]+[r['original_fixture'] for r in manifest['rows']]:
        if sha(item['path'])!=item['sha256']:raise ValueError('Bound original input changed')
    generation=numerical/f'generated_{mode}';binding=read(generation/'generation_binding.json')
    if binding['mode']!=('SQP' if mode=='sqp' else 'SQP_RTI'):raise ValueError('Generation mode differs')
    graph=binding['graph_sha256'];generated_manifest=read(generation/'generated_solver_manifest.json')
    for item in generated_manifest['files']:
        if sha(item['path'])!=item['sha256']:raise ValueError('Generated model artifact changed: '+item['path'])
    raw_binary=binary_dir/'x650_standard_native_replay';full_binary=binary_dir/'x650_standard_solver_replay'
    binaries=[identity(raw_binary),identity(full_binary)]
    model=build_model(manifest['config']);full=run_cli(full_binary,Path(manifest['original_cases']['path']),out/'full_cpp.json')
    verify_compiled_identity(full,mode,graph,manifest['physical_config_sha256'])
    if len(full['records'])!=16:raise ValueError('Full C++ output coverage differs')
    rows=[];timings={k:[] for k in ('raw_setup_ms','raw_solve_ms','raw_residual_evaluation_ms',
                                  'full_builder_ms','full_setup_ms','full_solve_ms','full_postprocess_ms')}
    for bound,full_row in zip(manifest['rows'],full['records']):
        index=bound['index'];numeric=read(bound['numeric_case']);reference=read(bound['python_result'])
        py_summary=python_report['cases'][index]
        if py_summary['case_id']!=bound['case_id'] or py_summary['case_sha256']!=bound['case_sha256']:raise ValueError('Python case order/source differs')
        if sha(bound['numeric_case'])!=py_summary['numeric_case']['sha256']:raise ValueError('Python bound numeric export changed')
        source=numeric['source_identity']
        if any(source[k]!=bound[k] for k in ('source','source_sha256','source_index','case_sha256')):raise ValueError('Numeric export source identity differs')
        if numeric['graph_sha256']!=graph or numeric['config_sha256']!=manifest['physical_config_sha256']:raise ValueError('Numeric export model identity differs')
        if reference['graph_sha256']!=graph or reference['binding_sha256']!=numeric['binding_sha256']:raise ValueError('Python solve does not match numeric export')
        if reference['backend']!='acados_'+('sqp' if mode=='sqp' else 'sqp_rti'):raise ValueError('Python solve backend differs')
        if reference['accepted_for_publication'] is not False:raise ValueError('Offline Python solve unexpectedly authorizes publication')
        if full_row['id']!=bound['case_id'] or not full_row['completed']:raise ValueError('Full C++ case failed: '+str(full_row.get('error')))
        mapping={key:max_error(full_row[key],numeric[target]) for key,target in
                 [('x0','x0'),('parameters','p'),('initial_plan','guess_u'),('horizon','time_steps')]}
        raw=run_cli(raw_binary,Path(bound['numeric_case']),out/f'raw_{index:03d}.json')
        verify_compiled_identity(raw,mode,graph,manifest['physical_config_sha256'])
        if len(raw['samples'])!=REPETITIONS or len(full_row['samples'])!=REPETITIONS:raise ValueError('Missing independent reset repetitions')
        numerical_case=NumericalCase(np.array(numeric['x0']),np.array(numeric['p']),np.array(numeric['guess_u']),np.array(numeric['time_steps']),numeric['metadata'])
        comparisons=[]
        for repetition,(raw_sample,full_sample) in enumerate(zip(raw['samples'],full_row['samples'])):
            raw_plan=np.array(raw_sample['plan']).reshape(15,5);full_plan=np.array(full_sample['plan']).reshape(15,5)
            raw_rollout=rollout(model,numerical_case,raw_plan)['states']
            errors=dict(raw_plan_vs_python=max_error(raw_plan,reference['plan']),full_plan_vs_python=max_error(full_plan,reference['plan']),
                        raw_rollout_vs_python=max_error(raw_rollout,reference['states']),
                        full_generated_rollout_vs_python=max_error(full_sample['rollout_states'],reference['states']),
                        raw_vs_full_shooting_states=max_error(raw_sample['states'],full_sample['states']),
                        raw_vs_full_plan=max_error(raw_plan,full_plan))
            flags=dict(raw_status_matches=raw_sample['raw_status']==reference['raw_status'],
                       full_status_matches=full_sample['raw_status']==reference['raw_status'],
                       raw_success_matches=raw_sample['solver_success']==reference['solver_success'],
                       full_success_matches=full_sample['solver_success']==reference['solver_success'],
                       raw_no_publication=raw_sample['online_publication_approved'] is False,
                       full_no_publication=full_sample['accepted_for_publication'] is False)
            comparisons.append(dict(repetition=repetition,absolute_errors=errors,flags=flags,
                                    passed=max(errors.values())<=SOLUTION_TOLERANCE and all(flags.values())))
            for source_sample,prefix,keys in [(raw_sample,'raw_',('setup_ms','solve_ms','residual_evaluation_ms')),
                                              (full_sample,'full_',('setup_ms','solve_ms','postprocess_ms'))]:
                for key in keys:timings[prefix+key].append(float(source_sample[key]))
        timings['full_builder_ms'].append(float(full_row['case_builder_ms']))
        mapping_passed=max(mapping.values())<=MAPPING_TOLERANCE
        row=dict(index=index,case_id=bound['case_id'],mapping_absolute_errors=mapping,mapping_passed=mapping_passed,
                 numeric_case=identity(bound['numeric_case']),python_result=identity(bound['python_result']),
                 python_solver_flags={key:reference[key] for key in ('raw_status','solver_success','numerically_feasible','first_order_kkt_passed','accepted_for_publication')},
                 comparisons=comparisons,passed=mapping_passed and all(r['passed'] for r in comparisons))
        rows.append(row);write(out/f'comparison_{index:03d}.json',row)
        print(json.dumps(dict(case_id=bound['case_id'],passed=row['passed'],maximum_solution_error=max(max(c['absolute_errors'].values()) for c in comparisons))),flush=True)
    rejection_dir=out/'rejections';rejection_dir.mkdir();rejections=[]
    first_numeric=read(manifest['rows'][0]['numeric_case']);first_original=read(manifest['rows'][0]['original_fixture']['path'])
    bad_cases=[('raw_wrong_hash',raw_binary,first_numeric,'0'*64),('full_wrong_hash',full_binary,first_original,'0'*64)]
    for field in ('graph_sha256','config_sha256'):
        bad=copy.deepcopy(first_numeric);bad[field]='f'*64;bad_cases.append(('raw_wrong_'+field,raw_binary,bad,None))
    bad=copy.deepcopy(first_original);bad['config']['response'][0]*=1.01;bad_cases.append(('full_same_dimension_wrong_config',full_binary,bad,None))
    bad=copy.deepcopy(first_numeric);bad['x0']=bad['x0'][:-1];bad_cases.append(('raw_short_x0',raw_binary,bad,None))
    bad=copy.deepcopy(first_numeric);bad['p'][0]=bad['p'][0][:-1];bad_cases.append(('raw_short_stage_parameters',raw_binary,bad,None))
    for name,binary,data,claimed in bad_cases:
        source=rejection_dir/(name+'_input.json');write(source,data)
        evidence=run_cli(binary,source,rejection_dir/(name+'_forbidden_output.json'),claimed_sha=claimed,expected_rejection=True)
        rejections.append(dict(name=name,**evidence))
    unchanged=all(sha(b['path'])==b['sha256'] for b in binaries)
    result=dict(kind='x650_standard_cpp_solver_reproduction_v1',mode=mode,count=len(rows),repetitions=REPETITIONS,
                passed=all(row['passed'] for row in rows) and all(r['passed'] for r in rejections) and unchanged,
                mapping_absolute_tolerance=MAPPING_TOLERANCE,solution_absolute_tolerance=SOLUTION_TOLERANCE,
                model_graph_sha256=graph,physical_config_sha256=manifest['physical_config_sha256'],
                binaries=binaries,binaries_unchanged=unchanged,generation_binding=identity(generation/'generation_binding.json'),
                generation_manifest=identity(generation/'generated_solver_manifest.json'),python_report=identity(numerical/'report.json'),
                records=rows,rejections=rejections,observed_timing_ms={key:stats(value) for key,value in timings.items()},
                timing_samples_ms=timings,physical_acceptance=False,hard_realtime_proven=False,accepted_for_publication=False,
                limitations=['Python results retain exact rollout states, not raw returned shooting states.',
                             'Raw and full C++ shooting states are compared to each other; exact rollouts are compared to Python.',
                             'Only flags shared by both APIs are compared; Python KKT/feasibility flags are not invented for C++.',
                             'Observed setup/solve/postprocess samples do not prove original issue deadline or real-time scheduling.',
                             'Model/candidate numerical agreement does not replace original event validation or new physical feedback experiments.'])
    write(out/'report.json',result)
    print(json.dumps(dict(complete=True,passed=result['passed'],mode=mode,output=str(out/'report.json'))),flush=True)
    return 0 if result['passed'] else 2


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--selection',action='append',type=Path)
    parser.add_argument('--numerical-dir',type=Path);parser.add_argument('--mode',choices=('sqp','rti'),default='sqp')
    parser.add_argument('--output',type=Path);parser.add_argument('--binary-dir',type=Path)
    parser.add_argument('--prepare-only',action='store_true');parser.add_argument('--prepared-dir',type=Path)
    args=parser.parse_args()
    if args.prepared_dir:
        if args.output or args.selection or args.numerical_dir:parser.error('Prepared inputs cannot be rebound')
        out=args.prepared_dir.resolve();manifest=read(out/'inputs_manifest.json')
    else:
        if not args.output or not args.selection or not args.numerical_dir:parser.error('Output, numerical directory and selections required')
        out=args.output.resolve();manifest=prepare(out,args.selection,args.numerical_dir,args.mode)
    if args.prepare_only:
        print(json.dumps(dict(prepared=True,count=manifest['count'],mode=manifest['mode'],manifest=str(out/'inputs_manifest.json'))));return 0
    if args.binary_dir is None:parser.error('Binary directory required for native execution')
    if (out/'report.json').exists():raise ValueError('Use fresh native reproduction output; previous report is evidence')
    return qualify(out,manifest,args.binary_dir.resolve())


if __name__=='__main__':raise SystemExit(main())
