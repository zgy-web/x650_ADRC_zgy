#!/usr/bin/env python3
"""Summarize raw runtime evidence; never grants a flight-stage certificate."""
import argparse
import hashlib
import json
from pathlib import Path


def read(path): return json.loads(Path(path).read_text())
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--root", type=Path, required=True)
    p.add_argument("--healthy", required=True)
    p.add_argument("--fault-suffix", required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    cases = {"healthy": args.root/args.healthy}
    for case in ("worker_pause", "worker_short_pause", "source_pause", "recorder_pause"):
        cases[case] = args.root/(case+args.fault_suffix)
    reports = {case: read(path/"runtime_report.json") for case,path in cases.items()}
    args.output.mkdir(parents=True, exist_ok=False)
    manifests, checks, table = {}, {}, []
    for name, directory in cases.items():
        report = reports[name]
        source = read(directory/"worker_startup.json")
        final = read(directory/"worker_final.json")
        gc_summary = {str(g): {"count": sum(e["generation"]==g for e in final["gc_events"]),
                              "max_ms": max([(e["end_ns"]-e["start_ns"])/1e6 for e in final["gc_events"] if e["generation"]==g],default=0.)} for g in range(3)}
        checks[name] = bool(report["passed"] and report["protocol_complete"]
            and report["native_core_unchanged"] and report["sources_unchanged"]
            and not report["remaining_owned_processes"] and not report["actuator_topics"]
            and final["gc_enabled_before"] and final["gc_enabled_after"]
            and source["actually_loaded_native_optimization"]["sha256"]==report["native_core"]["binding"]["library_sha256"])
        if name=="healthy":
            checks[name] = checks[name] and not report["output_receipts"]["unknown_output"]
        if name=="source_pause": checks[name] = checks[name] and report["timing"]["first_fault"]["reason"] in (2,4)
        if name in ("worker_pause","worker_short_pause") and report["timing"]["first_fault"]:
            checks[name] = checks[name] and report["timing"]["first_fault"]["reason"] in (1,4)
        manifests[name] = {n:{"path":str(directory/n),"sha256":sha(directory/n),"bytes":(directory/n).stat().st_size}
            for n in ("runtime_report.json","dispatch_ticks.json","ingress_receipts.json","gate_events.json",
                      "worker_startup.json","worker_final.json","diagnostic_sender.json","recording/manifest.json","recording/diagnostics.jsonl")}
        if manifests[name]["recording/diagnostics.jsonl"]["sha256"] != report["recording"]["records_sha256"]:
            checks[name] = False
        table.append({"case":name,"contract_passed":checks[name],"accepted":report["timing"]["accepted_candidates"],
            "unique_shadow_output":report["output_receipts"]["unique_output_count"],
            "interval_max_ms":report["timing"]["dispatch_interval_ms"]["max"],
            "valid_command_age_max_ms":report["timing"]["valid_command_age_ms"]["max"],
            "fault":report["timing"]["first_fault"],"recording_complete":report["recording"]["complete"],
            "diagnostic_dropped":report["recording"]["dropped"],"gc":gc_summary,
            "injection":report.get("injection"),"equivalence":{k:v for k,v in report.get("equivalence",{}).items() if k!="sequences"}})
    summary = {"kind":"x650_isolated_runtime_summary_v1","software_contract_passed":all(checks.values()),
        "checks":checks,"physical_hold_qualified":False,"hardware_qualified":False,
        "scope":"same-host typed ROS software process/recording contracts only; no plant or actuators",
        "cases":table,"evidence":manifests,
        "frozen_task_sha256":reports["healthy"]["frozen_task_sha256"],
        "native_library_sha256":reports["healthy"]["native_core"]["binding"]["library_sha256"]}
    (args.output/"summary.json").write_text(json.dumps(summary,indent=2,allow_nan=False)+"\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    plt.rcParams.update({"font.size":10,"axes.grid":True,"grid.alpha":.22})
    fig, axes = plt.subplots(2, 2, figsize=(14,9), constrained_layout=True)
    healthy=reports["healthy"];start=healthy["window"]["start_ns"]
    ticks=read(cases["healthy"]/"dispatch_ticks.json")
    ticks=[t for t in ticks if start<=t["actual_monotonic_ns"]<healthy["window"]["end_ns"]]
    times=np.array([(t["actual_monotonic_ns"]-start)/1e9 for t in ticks])
    axes[0,0].plot(times[1:],np.diff([t["actual_monotonic_ns"] for t in ticks])/1e6,label="Output interval",color="#1769aa",lw=.9)
    axes[0,0].axhline(20,ls="--",color="#555",lw=1,label="20 ms schedule")
    axes[0,0].set(title="A | Full 30 s output schedule",xlabel="Elapsed wall time [s]",ylabel="Interval [ms]",ylim=(18,22))
    axes[0,0].legend(loc="upper right")
    good=[t for t in ticks if t["output_valid"]]
    axes[0,1].plot([(t["actual_monotonic_ns"]-start)/1e9 for t in good], [t["command_age_ns"]/1e6 for t in good],color="#00897b",lw=.9,label="Age of actually output candidate")
    axes[0,1].axhline(50,ls="--",color="#c62828",label="50 ms validity limit")
    gc=read(cases["healthy"]/"worker_final.json")["gc_events"]
    for e in gc:
        if e["generation"]==2:
            at=(e["start_ns"]-start)/1e9
            axes[0,1].axvline(at,color="#6a1b9a",alpha=.8,lw=1.2,label="Observed generation-2 GC")
    axes[0,1].set(title="B | Original timestamp retained across repeats",xlabel="Elapsed wall time [s]",ylabel="Command age [ms]",ylim=(0,65))
    axes[0,1].legend(loc="lower right")
    for ax,name,title in ((axes[1,0],"worker_pause","C | Worker paused for 100 ms"),
                          (axes[1,1],"recorder_pause","D | Recorder paused for 1000 ms")):
        r=reports[name];inj=r["injection"];zero=inj["stop_ns"]
        data=read(cases[name]/"dispatch_ticks.json")
        data=[t for t in data if -0.3<=(t["actual_monotonic_ns"]-zero)/1e9<=1.35]
        ax.axvspan(0,(inj["resume_ns"]-zero)/1e9,alpha=.14,color="#ff8f00",label="Injected process pause")
        ax.plot([(t["actual_monotonic_ns"]-zero)/1e9 for t in data[1:]],np.diff([t["actual_monotonic_ns"] for t in data])/1e6,color="#1769aa",lw=1,label="Output tick interval")
        valid=[t for t in data if t["output_valid"]]
        ax.plot([(t["actual_monotonic_ns"]-zero)/1e9 for t in valid],[t["command_age_ns"]/1e6 for t in valid],color="#00897b",lw=1,label="Valid command age")
        ax.axhline(50,ls="--",color="#c62828",lw=1)
        f=r["timing"]["first_fault"]
        if f:
            at=(f["actual_monotonic_ns"]-zero)/1e9
            ax.axvline(at,color="#c62828",lw=1.2,label="Output invalid / latched")
            ax.text(.55,55,"Ticks continue; no valid commands",color="#c62828",ha="center")
        else:
            ax.text(.55,55,f"{r['recording']['dropped']} diagnostic rows explicitly lost",color="#9a5700",ha="center")
        ax.set(title=title,xlabel="Time relative to pause start [s]",ylabel="Time [ms]",ylim=(0,65))
        ax.legend(loc="lower right",fontsize=8)
    fig.suptitle("X650 frozen MPC | execution isolation evidence\nSoftware messages only: no physical plant, actuator application or flight precision claim",fontsize=14)
    fig.savefig(args.output/"runtime_timing.png",dpi=150)
    plt.close(fig)
    print(json.dumps({"passed":summary["software_contract_passed"],"cases":checks,"out":str(args.output)},indent=2))
    return 0 if summary["software_contract_passed"] else 1


if __name__=="__main__": raise SystemExit(main())
