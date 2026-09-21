# P2P Navigation Tests

This directory contains integration tests for point-to-point navigation with different robot models.

## Ackermann Navigation

The Ackermann test checks whether the navigation stack can guide a simulated vehicle through a sequence of goals within a total time limit.

- Waits for navigation readiness and the configured startup delay.
- Sends goals sequentially, advancing when move-base reports success.
- Records each goal's result, elapsed time, and measured pose for review.
- Fails if a goal is rejected, aborted, canceled, or the total time limit is exceeded.

Arrival tolerances are controlled by the local planner. Recorded pose errors are diagnostic and do not introduce separate test tolerances.

The goals, startup delay, total time limit, and result file are configured in [nav_ackermann_p2p_test.yaml](config/nav_ackermann_p2p_test.yaml).
