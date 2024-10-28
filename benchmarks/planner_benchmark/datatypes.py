from dataclasses import dataclass, field
from typing import Any

from nav2_msgs.action._compute_path_to_pose import ComputePathToPose_Result
from nav2_msgs.msg._costmap import Costmap


@dataclass
class PlannerResult:
    planner: str
    path: ComputePathToPose_Result | Any | None


@dataclass
class ResultRuns:
    valid: int = 0  # valid but might not have been run due to another invalid planner
    invalid: int = 0


@dataclass
class BenchmarkSummary:
    results: dict[str, ResultRuns] = field(default_factory=dict)


@dataclass
class BenchmarkMetrics:
    costmap: Costmap = field(default_factory=Costmap)
    planners: list[str] = field(default_factory=list[str])
    results: list[list[PlannerResult]] = field(
        default_factory=list[list[PlannerResult]]
    )
    summary: BenchmarkSummary = field(default_factory=BenchmarkSummary)
