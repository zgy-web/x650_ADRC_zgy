"""PC-side launch of an approved temporary USB DDS state session; logs stay here."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys
import zlib


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host",required=True)
    p.add_argument("--user",default="jetson")
    p.add_argument("--ssh",default="ssh")
    p.add_argument("--identity",required=True)
    p.add_argument("--known-hosts",type=Path,required=True)
    p.add_argument("--dialect-source",type=Path,required=True)
    p.add_argument("--expected-firmware-commit",required=True)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--approved-temporary-usb-switch",action="store_true",required=True)
    p.add_argument("--observe-existing-dds",action="store_true",
                   help="Only after a confirmed USB DDS switch: restart Agent/probe without sending any MAVLink")
    a=p.parse_args()
    source=a.dialect_source.read_bytes()
    request=dict(approved_temporary_usb_switch=True,observe_existing_dds=a.observe_existing_dds,
                 expected_firmware_commit=a.expected_firmware_commit,
                 dialect_zlib_base64=base64.b64encode(zlib.compress(source)).decode("ascii"),
                 dialect_sha256=hashlib.sha256(source).hexdigest())
    remote=Path(__file__).with_name("px4_usb_dds_session_remote.py").read_text(encoding="utf-8")
    args=[a.ssh,"-T","-o","BatchMode=yes","-o","StrictHostKeyChecking=yes","-o","IdentitiesOnly=yes",
          "-o","ConnectTimeout=8","-i",a.identity,"-o","UserKnownHostsFile="+str(a.known_hosts.resolve()),
          a.user+"@"+a.host,shlex.join(["env","PYTHONDONTWRITEBYTECODE=1","python3","-u","-c",remote])]
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open("xb") as log:
        child=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        child.stdin.write(json.dumps(request).encode());child.stdin.close()
        for line in child.stdout:
            log.write(line);log.flush()
            decoded=line.decode("utf-8",errors="replace")
            try:
                row=json.loads(decoded)
                if row.get("event")=="agent":
                    continue  # Full diagnostics retained in the PC log.
                if "schema" in row:
                    if row.get("final") or int(row.get("elapsed_s",0))%10==0:
                        print(json.dumps({k:row[k] for k in ("final","elapsed_s","core_topics_received",
                              "core_receipts_fresh","armed_seen")}),flush=True)
                    continue
            except ValueError:
                pass
            print(decoded,end="",flush=True)
        return child.wait()


if __name__=="__main__":sys.exit(main())
