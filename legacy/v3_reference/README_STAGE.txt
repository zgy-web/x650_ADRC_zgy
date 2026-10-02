cloud_arm_workflow_v3_underfold

CURRENT STAGE:
  Integration check only.

REVIEWED MODEL:
  x500_underfold

RULES:
  - No ground_stow.
  - No root-small.
  - No arm motion.
  - No arming / takeoff until --check passes.
  - q1_sim = 0 and q2_sim = 0 mean the reviewed underfold pose.
  - reference_v2/ is reference material only; do not run those scripts here.
