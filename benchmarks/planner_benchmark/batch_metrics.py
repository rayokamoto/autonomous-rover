#!/usr/bin/env python3

# Same as metrics.py, but run multiple seeds
# Also processes the data

from random import seed
import glob
import os
import time

from nav2_simple_commander.robot_navigator import BasicNavigator
from tabulate import tabulate
import numpy as np
import rclpy

from datatypes import BenchmarkSummary, BenchmarkMetrics, PlannerResult, ResultRuns
import metrics
import process_data


def proc_data(metrics_list: list[BenchmarkMetrics]):
    if len(metrics_list) < 1:
        print("There is no data")
        exit(0)

    planner_count = len(metrics_list[0].planners)
    metrics_count = len(metrics_list)

    # For each planner:
    avg_path_lengths = [0] * planner_count
    avg_times = [0] * planner_count
    avg_average_path_costs = [0] * planner_count
    avg_max_path_costs = [0] * planner_count

    final_summary = BenchmarkSummary()
    for p in metrics_list[0].planners:
        final_summary.results[p] = ResultRuns()

    for m in metrics_list:

        costmap = m.costmap
        planners = m.planners
        results = m.results
        summary = m.summary

        paths = process_data.getPaths(results)
        path_lengths = []

        for path in paths:
            path_lengths.append(process_data.getPathLength(path))
        path_lengths = np.asarray(path_lengths)
        total_paths = len(paths)

        path_lengths.resize((int(total_paths / len(planners)), len(planners)))
        path_lengths = path_lengths.transpose()

        times = process_data.getTimes(results)
        times = np.asarray(times)
        times.resize((int(total_paths / len(planners)), len(planners)))
        times = np.transpose(times)

        # Costs
        average_path_costs = np.asarray(
            process_data.averagePathCost(paths, costmap, len(planners))
        )
        max_path_costs = np.asarray(
            process_data.maxPathCost(paths, costmap, len(planners))
        )

        for i in range(len(planners)):
            avg_path_lengths[i] += np.average(path_lengths[i])
            avg_times[i] += np.average(times[i])
            avg_average_path_costs[i] += np.average(average_path_costs[i])
            avg_max_path_costs[i] += np.average(max_path_costs[i])

        for p in planners:
            final_summary.results[p].valid += summary.results[p].valid
            final_summary.results[p].invalid += summary.results[p].invalid

    # Generate table
    planner_table = [
        [
            "Planner",
            "Average path length (m)",
            "Average Time (s)",
            "Average cost",
            "Max cost",
        ]
    ]

    for i in range(len(planners)):
        planner_table.append(
            [
                planners[i],
                avg_path_lengths[i] / metrics_count,
                avg_times[i] / metrics_count,
                avg_average_path_costs[i] / metrics_count,
                avg_max_path_costs[i] / metrics_count,
            ]
        )

    summary_table = [["Planner", "Valid paths generated", "Invalid paths generated"]]
    for p, rr in final_summary.results.items():
        summary_table.append(
            [
                p,
                rr.valid,
                rr.invalid,
            ]
        )

    print(tabulate(planner_table))
    print(tabulate(summary_table))


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set map to use, other options: 100by100_15, 100by100_10
    map_path = os.getcwd() + "/" + glob.glob("**/100by100_20.yaml", recursive=True)[0]
    navigator.changeMap(map_path)
    time.sleep(2)

    # Get the costmap for start/goal validation
    costmap_msg = navigator.getGlobalCostmap()
    costmap = np.asarray(costmap_msg.data)
    costmap.resize(costmap_msg.metadata.size_y, costmap_msg.metadata.size_x)

    # planners = ["Navfn", "ThetaStar", "SmacHybrid", "Smac2d", "SmacLattice"]
    # planners = ["SmacHybrid", "Smac2d", "SmacLattice"]
    planners = ["SmacHybrid"]
    max_cost = 210
    side_buffer = 100
    time_stamp = navigator.get_clock().now().to_msg()

    random_pairs = 100
    res = costmap_msg.metadata.resolution
    SEED_MAX = 10

    metrics_list: list[BenchmarkMetrics] = []

    for s in range(SEED_MAX):
        results: list[list[PlannerResult]] = []
        summary = BenchmarkSummary()
        for p in planners:
            summary.results[p] = ResultRuns()

        print("----------------------------------------------------------------------")
        print(f"Benchmarking with seed={s}")
        seed(s)
        i = 0

        while len(results) != random_pairs:
            print("Cycle: ", i, "out of: ", random_pairs)
            start = metrics.getRandomStart(
                costmap, max_cost, side_buffer, time_stamp, res
            )
            goal = metrics.getRandomGoal(
                costmap, start, max_cost, side_buffer, time_stamp, res
            )
            print("Start", start)
            print("Goal", goal)
            result = metrics.getPlannerResults(navigator, start, goal, planners)
            if len(result) == len(planners):
                results.append(result)
                i += 1
            else:
                valid_planners = [r.planner for r in result]
                for p in planners:
                    if p not in valid_planners:
                        summary.results[p].invalid += 1

                print(
                    f"One or more of the planners were invalid. Valid planners: {[p.planner for p in result]}"
                )
            for r in result:
                summary.results[r.planner].valid += 1

        metrics_pickle = BenchmarkMetrics(
            costmap=costmap_msg,
            planners=planners,
            results=results,
            summary=summary,
        )
        metrics_list.append(metrics_pickle)

    print("Process data...")
    proc_data(metrics_list)

    exit(0)


if __name__ == "__main__":
    main()
