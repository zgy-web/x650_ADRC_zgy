#!/bin/bash
/bin/bash /root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/isolated_live_20261024/hold_v24_ready2.sh >/root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/isolated_live_20261024/hold_v24.log 2>&1
printf "%s\n" "$?" >/root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/isolated_live_20261024/hold_v24.exit
/bin/bash /root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/isolated_live_20261024/progress_v8.sh
