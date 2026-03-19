import heapq

# Road distances (in km) between major Indian cities
# Data sourced from approximate real road distances
india_road_graph = {
    "Delhi": {
        "Jaipur": 282, "Agra": 233, "Chandigarh": 274,
        "Lucknow": 555, "Amritsar": 449
    },
    "Mumbai": {
        "Pune": 149, "Surat": 284, "Nashik": 167,
        "Ahmedabad": 524, "Goa": 597
    },
    "Bangalore": {
        "Chennai": 347, "Hyderabad": 570, "Mysore": 150,
        "Coimbatore": 365, "Mangalore": 352
    },
    "Chennai": {
        "Bangalore": 347, "Hyderabad": 626, "Coimbatore": 496,
        "Pondicherry": 162, "Madurai": 461
    },
    "Hyderabad": {
        "Bangalore": 570, "Chennai": 626, "Pune": 559,
        "Nagpur": 503, "Vijayawada": 274
    },
    "Kolkata": {
        "Bhubaneswar": 441, "Patna": 591, "Guwahati": 1031,
        "Ranchi": 401
    },
    "Pune": {
        "Mumbai": 149, "Hyderabad": 559, "Nashik": 212,
        "Solapur": 252
    },
    "Ahmedabad": {
        "Mumbai": 524, "Surat": 265, "Jaipur": 656,
        "Udaipur": 262, "Rajkot": 216
    },
    "Jaipur": {
        "Delhi": 282, "Ahmedabad": 656, "Agra": 240,
        "Jodhpur": 343, "Udaipur": 421
    },
    "Lucknow": {
        "Delhi": 555, "Agra": 363, "Kanpur": 84,
        "Patna": 529, "Varanasi": 286
    },
    "Chandigarh": {
        "Delhi": 274, "Amritsar": 229, "Shimla": 115,
        "Ludhiana": 95
    },
    "Agra": {
        "Delhi": 233, "Jaipur": 240, "Lucknow": 363,
        "Gwalior": 119
    },
    "Amritsar": {
        "Delhi": 449, "Chandigarh": 229, "Ludhiana": 141
    },
    "Surat": {
        "Mumbai": 284, "Ahmedabad": 265, "Vadodara": 131
    },
    "Nashik": {
        "Mumbai": 167, "Pune": 212, "Aurangabad": 102
    },
    "Nagpur": {
        "Hyderabad": 503, "Raipur": 295, "Aurangabad": 535,
        "Bhopal": 357
    },
    "Bhopal": {
        "Nagpur": 357, "Indore": 194, "Gwalior": 423,
        "Jabalpur": 320
    },
    "Indore": {
        "Bhopal": 194, "Ahmedabad": 395, "Ujjain": 53
    },
    "Patna": {
        "Lucknow": 529, "Kolkata": 591, "Ranchi": 330,
        "Varanasi": 247, "Gaya": 103
    },
    "Varanasi": {
        "Lucknow": 286, "Patna": 247, "Allahabad": 122
    },
    "Coimbatore": {
        "Bangalore": 365, "Chennai": 496, "Madurai": 213
    },
    "Mysore": {
        "Bangalore": 150, "Mangalore": 251
    },
    "Vijayawada": {
        "Hyderabad": 274, "Chennai": 440
    },
    "Bhubaneswar": {
        "Kolkata": 441, "Visakhapatnam": 443
    },
    "Visakhapatnam": {
        "Bhubaneswar": 443, "Vijayawada": 347
    },
    "Goa": {
        "Mumbai": 597, "Mangalore": 383, "Bangalore": 560
    },
    "Mangalore": {
        "Bangalore": 352, "Goa": 383, "Mysore": 251
    },
    "Jodhpur": {
        "Jaipur": 343, "Udaipur": 255, "Ahmedabad": 463
    },
    "Udaipur": {
        "Jaipur": 421, "Ahmedabad": 262, "Jodhpur": 255
    },
    "Guwahati": {
        "Kolkata": 1031, "Shillong": 103
    },
    "Shimla": {
        "Chandigarh": 115, "Manali": 270
    },
    "Gwalior": {
        "Agra": 119, "Bhopal": 423, "Jhansi": 101
    },
    "Aurangabad": {
        "Nashik": 102, "Nagpur": 535, "Solapur": 270
    },
    "Jabalpur": {
        "Bhopal": 320, "Nagpur": 301, "Raipur": 291
    },
    "Raipur": {
        "Nagpur": 295, "Jabalpur": 291, "Ranchi": 427
    },
    "Ranchi": {
        "Kolkata": 401, "Patna": 330, "Raipur": 427
    },
    "Allahabad": {
        "Varanasi": 122, "Lucknow": 200, "Kanpur": 201
    },
    "Kanpur": {
        "Lucknow": 84, "Allahabad": 201, "Agra": 284
    },
    "Ludhiana": {
        "Chandigarh": 95, "Amritsar": 141, "Delhi": 320
    },
    "Vadodara": {
        "Surat": 131, "Ahmedabad": 113
    },
    "Rajkot": {
        "Ahmedabad": 216
    },
    "Madurai": {
        "Coimbatore": 213, "Chennai": 461
    },
    "Pondicherry": {
        "Chennai": 162
    },
    "Gaya": {
        "Patna": 103
    },
    "Shillong": {
        "Guwahati": 103
    },
    "Manali": {
        "Shimla": 270
    },
    "Solapur": {
        "Pune": 252, "Aurangabad": 270
    },
    "Ujjain": {
        "Indore": 53
    },
    "Jhansi": {
        "Gwalior": 101
    }
}

# Make the graph bidirectional
def make_bidirectional(graph):
    bidirectional = {}
    for city in graph:
        bidirectional[city] = {}
    for city, neighbors in graph.items():
        for neighbor, dist in neighbors.items():
            bidirectional[city][neighbor] = dist
            if neighbor not in bidirectional:
                bidirectional[neighbor] = {}
            bidirectional[neighbor][city] = dist
    return bidirectional

india_road_graph = make_bidirectional(india_road_graph)


def dijkstra(graph, source):
    """
    Dijkstra's algorithm (Uniform Cost Search) to find
    shortest paths from source to all reachable nodes.

    Returns:
        dist      : dict of shortest distances from source
        prev      : dict to reconstruct the shortest path
    """
    dist = {city: float('inf') for city in graph}
    prev = {city: None for city in graph}
    dist[source] = 0

    # Min-heap: (cost, node)
    priority_queue = [(0, source)]

    while priority_queue:
        current_cost, current_city = heapq.heappop(priority_queue)

        # Skip stale entries in the heap
        if current_cost > dist[current_city]:
            continue

        for neighbor, road_dist in graph[current_city].items():
            new_cost = current_cost + road_dist
            if new_cost < dist[neighbor]:
                dist[neighbor] = new_cost
                prev[neighbor] = current_city
                heapq.heappush(priority_queue, (new_cost, neighbor))

    return dist, prev


def reconstruct_path(prev, source, target):
    """Reconstruct the shortest path from source to target using prev map."""
    path = []
    node = target
    while node is not None:
        path.append(node)
        node = prev[node]
    path.reverse()
    if path[0] == source:
        return path
    return []  # No path found


def print_all_shortest_paths(source):
    """Print shortest distances from source to all Indian cities."""
    if source not in india_road_graph:
        print(f"City '{source}' not found in the graph.")
        return

    dist, prev = dijkstra(india_road_graph, source)

    print(f"\nShortest road distances from {source}:")
    print("=" * 55)
    print(f"{'Destination':<25} {'Distance (km)':<15} {'Path'}")
    print("-" * 55)

    sorted_cities = sorted(
        [(d, city) for city, d in dist.items() if city != source and d < float('inf')]
    )

    for distance, city in sorted_cities:
        path = reconstruct_path(prev, source, city)
        path_str = " -> ".join(path)
        print(f"{city:<25} {distance:<15} {path_str}")

    unreachable = [city for city, d in dist.items() if d == float('inf') and city != source]
    if unreachable:
        print(f"\nUnreachable from {source}: {', '.join(unreachable)}")


def shortest_path_between(source, target):
    """Find and display shortest path between two specific cities."""
    if source not in india_road_graph:
        print(f"City '{source}' not in graph.")
        return
    if target not in india_road_graph:
        print(f"City '{target}' not in graph.")
        return

    dist, prev = dijkstra(india_road_graph, source)
    path = reconstruct_path(prev, source, target)

    if not path:
        print(f"No road path found between {source} and {target}.")
        return

    print(f"\nShortest road path: {source} → {target}")
    print(f"Distance : {dist[target]} km")
    print(f"Route    : {' -> '.join(path)}")
    print(f"Stops    : {len(path) - 1} intermediate stops")


# ─────────────────────────────────────────────────────────────
# Demo
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 60)
    print("  DIJKSTRA'S ALGORITHM — INDIAN CITY ROAD NETWORK")
    print("=" * 60)

    print("\n[Demo 1] All shortest paths from Delhi")
    print_all_shortest_paths("Delhi")

    print("\n\n[Demo 2] Point-to-point queries")
    pairs = [
        ("Mumbai",    "Kolkata"),
        ("Bangalore", "Delhi"),
        ("Amritsar",  "Chennai"),
        ("Guwahati",  "Goa"),
    ]
    for src, tgt in pairs:
        shortest_path_between(src, tgt)

    print("\n\n[Demo 3] All shortest paths from Bangalore")
    print_all_shortest_paths("Bangalore")
