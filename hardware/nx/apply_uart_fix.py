#!/usr/bin/env python3
"""Scoped R36.5 UARTA repair. --check is read-only; root changes require consent.

Code belongs in /home/jetson/flyarm/hw_uart_fix/v1/. No experiment logs are
written on NX. Compilation and the complete device-tree comparison use memfd.
The original primary entry remains selectable; rollback is manual, not automatic.
"""
import argparse
import difflib
import hashlib
import json
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile


DEPLOY = Path('/home/jetson/flyarm/hw_uart_fix/v1')
CONFIG = Path('/boot/extlinux/extlinux.conf')
BACKUP = CONFIG.with_name('extlinux.conf.flyarm-uart1-before')
OVERLAY = Path('/boot/dtb/flyarm-uart1-iommu-r365.dtbo')
BASE_DTB = Path('/boot/dtb/kernel_tegra234-p3768-0000+p3767-0000-nv.dtb')
BASE_DTB_SHA = '4c533869e70e6b0fe1426ad2f00e25b29450dab090997fa42385b8b16a55869f'
SOURCE_NAME = 'flyarm-uart1-iommu-r365.dts'
CONFIG_SHA = '5974f443a57f400acb31eaca5d5e74ec31ff7c97dc46baf79669df9a887025fc'
SOURCE_SHA = '90732d2390d60861d7dee9e131817f378d59a9cf49a34e47fe3c77b268b1b900'
OVERLAY_SHA = '6aadc1c08fe536cf20b90018867a95bf671f3736c41fbb2aacf64e7009709226'
LABEL = 'flyarm_uart1_iommu'
TREE = Path('/sys/firmware/devicetree/base')
UART = 'bus@0/serial@3100000'
DMA = 'bus@0/dma-controller@2600000'
SMMU = 'bus@0/iommu@12000000'


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def make_legacy_config(original):
    """Recognize the first, ineffective overlay-only installation for safe recovery."""
    require(sha(original) == CONFIG_SHA, 'Boot config differs from reviewed original')
    text = original.decode('utf-8')
    require(re.findall(r'^LABEL (\S+)\s*$', text, re.M) == ['primary'],
            'Expected exactly one original primary entry')
    lines = text.splitlines(keepends=True)
    entry = []
    for key in ('LINUX', 'INITRD', 'APPEND'):
        found = [line for line in lines if re.match(r'^\s+' + key + r'\s', line)]
        require(len(found) == 1, 'Ambiguous primary ' + key)
        entry.append(found[0])
    updated, count = re.subn(r'^DEFAULT primary$', 'DEFAULT ' + LABEL, text, flags=re.M)
    require(count == 1, 'Expected DEFAULT primary')
    updated += ('\n# Flyarm R36.5 UARTA IOMMU fix; primary above is the original fallback.\n'
                'LABEL ' + LABEL + '\n'
                '      MENU LABEL Flyarm UART1 IOMMU fix (original: primary)\n'
                + ''.join(entry) + '      OVERLAYS ' + str(OVERLAY) + '\n')
    return updated.encode('utf-8')


def make_config(original):
    """Keep primary intact; R36 requires explicit FDT to process OVERLAYS."""
    legacy = make_legacy_config(original)
    marker = ('LABEL ' + LABEL + '\n').encode()
    before, entry = legacy.split(marker)
    entry = entry.replace(b'      LINUX /boot/Image\n',
                          b'      LINUX /boot/Image\n      FDT ' + str(BASE_DTB).encode() + b'\n', 1)
    require(entry.count(b'      FDT ') == 1, 'Missing explicit FDT in new entry')
    return before + marker + entry


def plain_file(path):
    require(not path.is_symlink() and path.is_file(), 'Expected ordinary file: ' + str(path))
    return path.read_bytes()


def originals_and_state():
    current = plain_file(CONFIG)
    if sha(current) == CONFIG_SHA:
        original = current
        state = 'original'
    else:
        original = plain_file(BACKUP)
        require(sha(original) == CONFIG_SHA, 'Backup hash differs from reviewed original')
        if current == make_config(original):
            state = 'installed'
        elif current == make_legacy_config(original):
            state = 'legacy_overlay_only'
        else:
            raise RuntimeError('Boot config has unrelated changes; refusing overwrite')
    if BACKUP.exists() or BACKUP.is_symlink():
        require(plain_file(BACKUP) == original, 'Existing backup differs; refusing overwrite')
    return original, current, state


def run(command, data=None, pass_fds=()):
    result = subprocess.run(command, input=data, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, pass_fds=pass_fds, timeout=25)
    require(result.returncode == 0, command[0] + ' failed: ' + result.stderr.decode(errors='replace')[-1500:])
    return result.stdout


def canonical(blob):
    return run(['/usr/bin/dtc', '-q', '-I', 'dtb', '-O', 'dts', '-s', '-o', '-', '-'], blob)


def merge_diff(base, overlay, expected_added):
    fds = [os.memfd_create(name) for name in ('flyarm-base', 'flyarm-overlay', 'flyarm-merged')]
    try:
        os.write(fds[0], base)
        os.write(fds[1], overlay)
        run(['/usr/bin/fdtoverlay', '-i', '/proc/self/fd/' + str(fds[0]),
             '-o', '/proc/self/fd/' + str(fds[2]), '/proc/self/fd/' + str(fds[1])], pass_fds=tuple(fds))
        os.lseek(fds[2], 0, os.SEEK_SET)
        merged = os.read(fds[2], os.fstat(fds[2]).st_size)
    finally:
        for fd in fds:
            os.close(fd)
    diff = list(difflib.unified_diff(canonical(base).decode().splitlines(),
                                   canonical(merged).decode().splitlines(), lineterm=''))
    removed = [s[1:].strip() for s in diff if s.startswith('-') and not s.startswith('---')]
    added = [s[1:].strip() for s in diff if s.startswith('+') and not s.startswith('+++')]
    require(not removed and added == expected_added, 'Merged tree changes more than the reviewed UART mapping')
    return {'added_lines': added, 'removed_lines': removed, 'complete_tree_diff': diff}


def verify_live_tree(source):
    """Resolve symbols and prove the full merged tree has no other changes."""
    for name, target in (('uarta', UART), ('smmu_niso0', SMMU), ('gpcdma', DMA)):
        require((TREE / '__symbols__' / name).read_bytes().rstrip(b'\0').decode() == '/' + target,
                'Unexpected symbol: ' + name)
    require((TREE / UART / 'status').read_bytes() == b'okay\0', 'UARTA not enabled')
    require((TREE / UART / 'compatible').read_bytes() == b'nvidia,tegra194-hsuart\0', 'Unexpected UART driver')
    require((TREE / UART / 'dma-names').read_bytes() == b'rx\0tx\0', 'Unexpected UART DMA directions')
    dma_handle = struct.unpack('>I', (TREE / DMA / 'phandle').read_bytes())[0]
    require((TREE / UART / 'dmas').read_bytes() == struct.pack('>4I', dma_handle, 8, dma_handle, 8),
            'Unexpected UART DMA channels')
    smmu_handle = struct.unpack('>I', (TREE / SMMU / 'phandle').read_bytes())[0]
    mapping = struct.pack('>2I', smmu_handle, 4)
    require((TREE / DMA / 'iommus').read_bytes() == mapping, 'GPCDMA SID mapping differs')
    uart_mapping = TREE / UART / 'iommus'
    already_live = uart_mapping.exists()
    if already_live:
        require(uart_mapping.read_bytes() == mapping, 'UART already has a different mapping')
    overlay = run(['/usr/bin/dtc', '-q', '-@', '-I', 'dts', '-O', 'dtb', '-o', '-', '-'], source)
    require(sha(overlay) == OVERLAY_SHA, 'Compiled overlay hash differs from audited candidate')
    base = run(['/usr/bin/dtc', '-q', '-I', 'fs', '-O', 'dtb', '-o', '-', str(TREE)])
    expected = [] if already_live else [f'iommus = <0x{smmu_handle:02x} 0x04>;']
    diff = merge_diff(base, overlay, expected)
    return overlay, dict(diff, already_live=already_live, mapping_hex=mapping.hex())


def verify_base_dtb(overlay):
    base = plain_file(BASE_DTB)
    require(sha(base) == BASE_DTB_SHA, 'Installed base DTB differs from audited R36.5 board DTB')
    fields = {}
    for name in ('model', 'compatible'):
        expected = (TREE / name).read_bytes().rstrip(b'\0').decode().replace('\0', ' ')
        actual = run(['/usr/bin/fdtget', str(BASE_DTB), '/', name]).decode().strip()
        require(actual == expected, 'Base DTB board ' + name + ' differs from running board')
        fields[name] = actual
    fields.update(path=str(BASE_DTB), sha256=sha(base), bytes=len(base),
                  overlay_check=merge_diff(base, overlay, ['iommus = <0x04 0x04>;']))
    return fields


def preflight():
    original, current, state = originals_and_state()
    release = Path('/etc/nv_tegra_release').read_text()
    require(re.search(r'^# R36 \(release\), REVISION: 5\.0,', release), 'Expected Jetson Linux R36.5.0')
    source = plain_file(Path(__file__).resolve().parent / SOURCE_NAME)
    require(sha(source) == SOURCE_SHA, 'Overlay source differs from reviewed source')
    for path in (CONFIG.parent, OVERLAY.parent):
        require(path.resolve() == path and path.is_dir(), 'Unexpected boot directory: ' + str(path))
    for path in (Path('/boot/Image'), Path('/boot/initrd')):
        require(path.is_file(), 'Missing existing boot file: ' + str(path))
    users = subprocess.run(['/usr/bin/fuser', '/dev/ttyTHS1'], capture_output=True, timeout=5)
    require(users.returncode == 1 and not users.stdout.strip(), 'UART is in use or fuser check failed')
    overlay, tree = verify_live_tree(source)
    base_check = verify_base_dtb(overlay)
    if OVERLAY.exists() or OVERLAY.is_symlink():
        require(plain_file(OVERLAY) == overlay, 'Existing overlay differs; refusing overwrite')
    report = {'mode': 'check', 'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
              'state': state, 'config_sha256': sha(current), 'original_sha256': sha(original),
              'planned_sha256': sha(make_config(original)), 'overlay_sha256': sha(overlay),
              'overlay_bytes': len(overlay), 'tree_check': tree, 'base_dtb_check': base_check,
              'original_entry': 'primary',
              'new_default': LABEL, 'writes_performed': False, 'experiment_logs_written': False}
    return original, current, overlay, report


def atomic_write(path, data):
    """Same-directory temporary file, fsync, rename, directory fsync."""
    fd, name = tempfile.mkstemp(prefix='.' + path.name + '.flyarm-', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(data)
            stream.flush()
            os.fchmod(stream.fileno(), 0o644)
            os.fsync(stream.fileno())
        os.replace(name, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def require_root_deployment():
    require(os.geteuid() == 0, 'Run this explicit apply/rollback operation with sudo in the NX terminal')
    require(Path(__file__).resolve().parent == DEPLOY, 'Use the reviewed deployment under ' + str(DEPLOY))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check', action='store_true')
    mode.add_argument('--apply-and-reboot', action='store_true')
    mode.add_argument('--rollback-and-reboot', action='store_true')
    args = ap.parse_args()
    if args.rollback_and_reboot:
        require_root_deployment()
        original, current, state = originals_and_state()
        require(plain_file(CONFIG) == current, 'Boot config changed during check')
        if state != 'original':
            atomic_write(CONFIG, original)
        require(plain_file(CONFIG) == original, 'Rollback readback mismatch')
        report = {'mode': 'rollback', 'default': 'primary', 'config_sha256': sha(original)}
    else:
        if args.apply_and_reboot:
            require_root_deployment()
        original, current, overlay, report = preflight()
        if args.apply_and_reboot:
            require(plain_file(CONFIG) == current, 'Boot config changed during check')
            if not BACKUP.exists():
                atomic_write(BACKUP, original)
            require(plain_file(BACKUP) == original, 'Backup readback mismatch')
            if not OVERLAY.exists():
                atomic_write(OVERLAY, overlay)
            require(plain_file(OVERLAY) == overlay, 'Overlay readback mismatch')
            # Change the default only after backup and overlay have durable readback.
            planned = make_config(original)
            if current != planned:
                atomic_write(CONFIG, planned)
            require(plain_file(CONFIG) == planned, 'Boot config readback mismatch')
            report.update(mode='apply', writes_performed=True, backup=str(BACKUP))
    print(json.dumps(report, indent=2), flush=True)
    if not args.check:
        os.sync()
        print('Verified. Rebooting NX; keep ESC power disconnected.', flush=True)
        subprocess.run(['/usr/bin/systemctl', 'reboot'], check=True, timeout=15)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print('STOPPED: ' + str(exc), file=sys.stderr, flush=True)
        sys.exit(1)
