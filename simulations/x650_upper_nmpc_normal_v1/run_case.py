"""Dispatch the exact registered task to the shared upper-control lifecycle."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from simulations.x650_upper_nmpc_v1.run_case import scenario

if __name__ == "__main__":
    scenario()
