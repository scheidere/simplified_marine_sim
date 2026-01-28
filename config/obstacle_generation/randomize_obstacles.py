#!/usr/bin/env python3

import argparse
import json
import math
import random
import sys
from typing import List, Tuple

Point = Tuple[int, int]
Polygon = List[Point]

TASK_CLEARANCE = 5  # to allow robots to reach tasks without running into obstacles (given robot 10 pixel size)

def distance(p1: Point, p2: Point) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def polygon_bbox(poly: Polygon) -> Tuple[int, int, int, int]:
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    return min(xs), min(ys), max(xs), max(ys)


def point_to_bbox_distance(
    point: Point,
    bbox: Tuple[int, int, int, int]
) -> float:
    px, py = point
    x1, y1, x2, y2 = bbox

    dx = max(x1 - px, 0, px - x2)
    dy = max(y1 - py, 0, py - y2)

    return math.hypot(dx, dy)


def bboxes_too_close(
    a: Polygon,
    b: Polygon,
    min_spacing: int
) -> bool:
    ax1, ay1, ax2, ay2 = polygon_bbox(a)
    bx1, by1, bx2, by2 = polygon_bbox(b)

    return not (
        ax2 + min_spacing < bx1 or
        ax1 - min_spacing > bx2 or
        ay2 + min_spacing < by1 or
        ay1 - min_spacing > by2
    )


def clamp_int(value: float) -> int:
    return int(round(value))


def generate_triangle(
    max_x: int,
    max_y: int,
    min_side: int,
    max_side: int
) -> Polygon:
    x = random.randint(0, max_x)
    y = random.randint(0, max_y)

    angle1 = random.uniform(0, 2 * math.pi)
    angle2 = angle1 + random.uniform(math.pi / 6, math.pi / 2)

    d1 = random.randint(min_side, max_side)
    d2 = random.randint(min_side, max_side)

    p1 = (x, y)
    p2 = (
        clamp_int(x + d1 * math.cos(angle1)),
        clamp_int(y + d1 * math.sin(angle1)),
    )
    p3 = (
        clamp_int(x + d2 * math.cos(angle2)),
        clamp_int(y + d2 * math.sin(angle2)),
    )

    return [p1, p2, p3]


def generate_parallelogram(
    max_x: int,
    max_y: int,
    min_side: int,
    max_side: int
) -> Polygon:
    x = random.randint(0, max_x)
    y = random.randint(0, max_y)

    angle = random.uniform(0, 2 * math.pi)
    d1 = random.randint(min_side, max_side)
    d2 = random.randint(min_side, max_side)

    v1 = (d1 * math.cos(angle), d1 * math.sin(angle))
    v2 = (
        d2 * math.cos(angle + math.pi / 2),
        d2 * math.sin(angle + math.pi / 2),
    )

    p1 = (x, y)
    p2 = (clamp_int(x + v1[0]), clamp_int(y + v1[1]))
    p3 = (
        clamp_int(p2[0] + v2[0]),
        clamp_int(p2[1] + v2[1]),
    )
    p4 = (clamp_int(x + v2[0]), clamp_int(y + v2[1]))

    return [p1, p2, p3, p4]


def polygon_within_bounds(poly: Polygon, max_x: int, max_y: int) -> bool:
    return all(0 <= x <= max_x and 0 <= y <= max_y for x, y in poly)


def sides_within_limits(
    poly: Polygon,
    min_side: int,
    max_side: int
) -> bool:
    n = len(poly)
    for i in range(n):
        d = distance(poly[i], poly[(i + 1) % n])
        if d < min_side or d > max_side:
            return False
    return True


def obstacle_respects_tasks(
    poly: Polygon,
    tasks: List[Point],
    clearance: int
) -> bool:
    bbox = polygon_bbox(poly)
    for task in tasks:
        if point_to_bbox_distance(task, bbox) < clearance:
            return False
    return True


def extract_task_locations(data: dict) -> List[Point]:
    try:
        return [
            (task["location"]["x"], task["location"]["y"])
            for task in data["tasks"]
            if "location" in task
        ]
    except KeyError:
        print("Missing required field: tasks[*].location", file=sys.stderr)
        sys.exit(1)

def extract_robot_locations(data: dict) -> List[Point]:
    try:
        return [
            (agent["start_x"], agent["start_y"])
            for agent in data["agents"]
        ]
    except KeyError:
        print("Missing required field: agents[*].start_x/start_y", file=sys.stderr)
        sys.exit(1)


def extract_task_locations(data: dict) -> List[Point]:
    try:
        return [
            (task["location"]["x"], task["location"]["y"])
            for task in data["tasks"]
            if "location" in task
        ]
    except KeyError:
        print("Missing required field: tasks[*].location", file=sys.stderr)
        sys.exit(1)

def main() -> None:
    parser = argparse.ArgumentParser(description="Generate random obstacles")
    parser.add_argument("input", help="Path to input.json")
    parser.add_argument(
        "--num-obstacles",
        type=int,
        required=True,
        help="Number of obstacles to generate",
    )
    parser.add_argument(
        "--min-spacing",
        type=int,
        default=0,
        help="Minimum spacing between obstacles (grid units)",
    )
    args = parser.parse_args()

    try:
        with open(args.input, "r") as f:
            data = json.load(f)
    except Exception as e:
        print(f"Failed to read input file: {e}", file=sys.stderr)
        sys.exit(1)

    world = data.get("world_attributes", {})
    size_x = world.get("size_x")
    size_y = world.get("size_y")

    if size_x is None or size_y is None:
        print("world_attributes.size_x and size_y are required", file=sys.stderr)
        sys.exit(1)

    tasks = extract_task_locations(data)
    robots = extract_robot_locations(data)

    protected_locations = tasks + robots

    grid_size = min(size_x, size_y)
    min_side = int(0.05 * grid_size)
    max_side = int(0.10 * grid_size)

    obstacles: List[Polygon] = []
    max_attempts_per_obstacle = 500

    for _ in range(args.num_obstacles):
        placed = False

        for _ in range(max_attempts_per_obstacle):
            poly = (
                generate_triangle(size_x, size_y, min_side, max_side)
                if random.choice([True, False])
                else generate_parallelogram(size_x, size_y, min_side, max_side)
            )

            if not polygon_within_bounds(poly, size_x, size_y):
                continue

            if not sides_within_limits(poly, min_side, max_side):
                continue

            # Check both tasks AND robots
            if not obstacle_respects_tasks(poly, protected_locations, TASK_CLEARANCE):
                continue

            if any(
                bboxes_too_close(poly, existing, args.min_spacing)
                for existing in obstacles
            ):
                continue

            obstacles.append(poly)
            placed = True
            break

        if not placed:
            print(
                "Warning: Could not place all obstacles with given constraints.",
                file=sys.stderr,
            )
            break

    world["obstacles"] = [
        {"points": [{"x": x, "y": y} for x, y in poly]}
        for poly in obstacles
    ]

    try:
        with open("output.json", "w") as f:
            json.dump(data, f, indent=4)
    except Exception as e:
        print(f"Failed to write output.json: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()

