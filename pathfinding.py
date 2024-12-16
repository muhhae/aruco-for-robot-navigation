from aruco_detector import Object
from heapq import heappop, heappush
from typing import Dict, List, Optional


def GetObject(maps: list[Object], id: int):
    for e in maps:
        if e.id == id:
            return e
    return None


def FindPath(
    maps: list[list[Object]], start_id: int, goal_id: int
) -> Optional[list[int]]:
    open_list = []
    heappush(open_list, (0, start_id))  # (f_cost, node_id)

    g_cost = {start_id: 0}
    came_from = {}

    while open_list:
        current_f, current_id = heappop(open_list)
        current_node = GetObject(maps, current_id)

        # Goal check
        if current_id == goal_id:
            # Reconstruct path
            path = []
            while current_id in came_from:
                path.append(current_id)
                current_id = came_from[current_id]
            path.append(start_id)
            return path[::-1]

        for direction, neighbor_id in current_node.neighbour.items():
            # neighbor_node = GetObject(neighbor_id)
            if neighbor_id is None:
                continue

            tentative_g_cost = g_cost[current_id] + 1  # Assuming edge cost = 1

            if neighbor_id not in g_cost or tentative_g_cost < g_cost[neighbor_id]:
                g_cost[neighbor_id] = tentative_g_cost
                f_cost = tentative_g_cost
                heappush(open_list, (f_cost, neighbor_id))
                came_from[neighbor_id] = current_id

    return None  # No path found


if __name__ == "__main__":
    from util import JSONToMarkers

    maps = [
        ["A5", "A2", None],
        ["A3", "A1", "A4"],
        [None, "A0", None],
    ]

    maps = JSONToMarkers(maps)
    print("routes:", FindPath(maps, 0, 5))
