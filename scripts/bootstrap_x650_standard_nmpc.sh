#!/usr/bin/env bash
# Isolated, versioned numerical tooling. Does not alter PX4, ROS, or the legacy MPC.
set -euo pipefail
runtime=${1:-/root/autodl-tmp/x650-nmpc-standard-runtime}
mkdir -p "$runtime"
runtime=$(cd "$runtime" && pwd -P)
case "$runtime" in /root/autodl-tmp/*) ;; *) printf '%s\n' 'Runtime must be a data-disk subdirectory' >&2; exit 2;; esac
source_dir="$runtime/acados-v0.6.0"
venv_dir="$runtime/venv"
if [ ! -d "$source_dir/.git" ]; then
  git clone --depth 1 --branch v0.6.0 https://github.com/acados/acados.git "$source_dir"
fi
test "$(git -C "$source_dir" describe --tags --exact-match)" = v0.6.0
test "$(git -C "$source_dir" rev-parse HEAD)" = 503364817c872d474ab5bed219c26760ac267769
git -C "$source_dir" submodule update --init --depth 1 external/blasfeo external/hpipm
cmake -S "$source_dir" -B "$runtime/build-acados-v0.6.0" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$source_dir" \
  -DACADOS_WITH_OPENMP=OFF -DACADOS_EXAMPLES=OFF -DACADOS_UNIT_TESTS=OFF \
  -DBLASFEO_TARGET=X64_AUTOMATIC -DHPIPM_TARGET=GENERIC
cmake --build "$runtime/build-acados-v0.6.0" -j2
cmake --install "$runtime/build-acados-v0.6.0"
test "$(uname -s)" = Linux && test "$(uname -m)" = x86_64
renderer="$source_dir/bin/t_renderer"
renderer_sha=64a0a0f8d85be0b92234231fd1a9ff50c9d8f13d6208063707e56c9354a976d3
mkdir -p "$source_dir/bin"
if ! printf '%s  %s\n' "$renderer_sha" "$renderer" | sha256sum -c --status; then
  curl --fail --location --retry 3 --connect-timeout 15 --max-time 900 \
    https://github.com/acados/tera_renderer/releases/download/v0.2.1/t_renderer-v0.2.1-linux-amd64 \
    -o "$renderer.partial"
  printf '%s  %s\n' "$renderer_sha" "$renderer.partial" | sha256sum -c
  chmod 755 "$renderer.partial"
  mv "$renderer.partial" "$renderer"
fi
"$renderer" --version
printf '%s  %s\n' "$renderer_sha" "$renderer" > "$runtime/tera-renderer-sha256.txt"
if [ ! -x "$venv_dir/bin/python" ]; then
  /usr/bin/python3 -m venv "$venv_dir"
fi
"$venv_dir/bin/python" -m pip install --index-url https://pypi.org/simple --timeout 30 --retries 2 --disable-pip-version-check --no-cache-dir \
  'numpy==1.26.4' 'scipy==1.13.1' 'casadi==3.7.2'
"$venv_dir/bin/python" -m pip install --index-url https://pypi.org/simple --timeout 30 --retries 2 --disable-pip-version-check --no-cache-dir \
  -e "$source_dir/interfaces/acados_template"
"$venv_dir/bin/python" -m pip freeze > "$runtime/python-freeze.txt"
git -C "$source_dir" rev-parse HEAD > "$runtime/acados-commit.txt"
git -C "$source_dir" submodule status > "$runtime/acados-submodules.txt"
cat > "$runtime/environment.sh" <<EOF
export ACADOS_SOURCE_DIR='$source_dir'
export LD_LIBRARY_PATH='$source_dir/lib':\${LD_LIBRARY_PATH:-}
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export PYTHONNOUSERSITE=1
export PATH='$venv_dir/bin':\$PATH
EOF
source "$runtime/environment.sh"
"$venv_dir/bin/python" - <<'PY'
import casadi as ca
import acados_template
import numpy as np
import scipy
print({'casadi': ca.__version__, 'numpy': np.__version__, 'scipy': scipy.__version__,
       'acados_template': acados_template.__file__})
x = ca.SX.sym('x')
solver = ca.nlpsol('qualification_smoke', 'ipopt', {'x': x, 'f': (x - 2)**2},
                  {'ipopt.print_level': 0, 'print_time': False})
result = solver(x0=0)
assert abs(float(result['x']) - 2) < 1e-9
print('IPOPT_REFERENCE_SMOKE_PASS')
PY
printf '%s\n' 'STANDARD_NMPC_DEPENDENCIES_PASS'
