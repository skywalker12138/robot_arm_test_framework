"""Configuration for the arm trajectory tests."""

JOINT_NAMES = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6',
]

# Each row is one legal target pose. Joint positions use radians.
TEST_CASES = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.2, -0.3, 0.4, 0.0, 0.2, 0.0],
    [0.4, -0.5, 0.3, 0.2, -0.2, 0.1],
    [-0.2, 0.3, -0.4, 0.0, 0.3, 0.0],
    [0.5, -0.2, 0.2, 0.3, 0.0, 0.2],
    [-0.4, 0.4, 0.1, -0.2, 0.2, -0.1],
    [0.1, -0.6, 0.5, 0.1, -0.3, 0.0],
    [0.3, 0.1, -0.3, 0.4, 0.1, 0.2],
    [-0.3, -0.2, 0.4, -0.3, 0.2, 0.1],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
]

ERROR_TOLERANCE = 0.01
TRAJECTORY_DURATION_SECONDS = 3
EXECUTION_TIMEOUT_SECONDS = 5.0
NEXT_TEST_DELAY_SECONDS = 0.5
FINAL_STATE_DELAY_SECONDS = 0.2
