import heapq

# i got the road distances from google maps and a few travel websites
# not 100% accurate but close enough for this assignment
# added most major cities, around 50 of them

city_roads = {
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
    "Rajkot": {"Ahmedabad": 216},
    "Madurai": {"Coimbatore": 213, "Chennai": 461},
    "Pondicherry": {"Chennai": 162},
    "Gaya": {"Patna": 103},
    "Shillong": {"Guwahati": 103},
    "Manali": {"Shimla": 270},
    "Solapur": {"Pune": 252, "Aurangabad": 270},
    "Ujjain": {"Indore": 53},
    "Jhansi": {"Gwalior": 101}
}


# roads go both ways obviously, so making it bidirectional
def make_bidirectional(graph):
    result = {city: {} for city in graph}
    for city, neighbors in graph.items():
        for neighbor, dist in neighbors.items():
            result[city][neighbor] = dist
            if neighbor not in result:
                result[neighbor] = {}
            result[neighbor][city] = dist
    return result

city_roads = make_bidirectional(city_roads)


def dijkstra(graph, source):
    # initialise all distances to infinity first
    dist = {city: float('inf') for city in graph}
    prev = {city: None for city in graph}
    dist[source] = 0

    pq = [(0, source)]  # (cost, city)

    while pq:
        curr_cost, curr_city = heapq.heappop(pq)

        # if we already found a shorter way to this city, skip
        if curr_cost > dist[curr_city]:
            continue

        for neighbor, road_km in graph[curr_city].items():
            new_dist = curr_cost + road_km
            if new_dist < dist[neighbor]:
                dist[neighbor] = new_dist
                prev[neighbor] = curr_city
                heapq.heappush(pq, (new_dist, neighbor))

    return dist, prev


def get_path(prev, source, target):
    # trace back from target to source using the prev pointers
    path = []
    node = target
    while node is not None:
        path.append(node)
        node = prev[node]
    path.reverse()
    if path and path[0] == source:
        return path
    return []


def show_all_from(source):
    if source not in city_roads:
        print(f"'{source}' not in the graph, check spelling")
        return

    dist, prev = dijkstra(city_roads, source)

    print(f"\nAll shortest road distances starting from {source}:")
    print("=" * 55)
    print(f"{'City':<25} {'km':<12} Route")
    print("-" * 55)

    # sort by distance so nearest cities show first
    reachable = sorted(
        [(d, c) for c, d in dist.items() if c != source and d < float('inf')]
    )

    for d, city in reachable:
        route = get_path(prev, source, city)
        print(f"{city:<25} {d:<12} {' -> '.join(route)}")

    # check if any cities are cut off
    stuck = [c for c, d in dist.items() if d == float('inf') and c != source]
    if stuck:
        print(f"\nCan't reach these from {source}: {', '.join(stuck)}")


def find_route(src, dst):
    if src not in city_roads or dst not in city_roads:
        print("one of those cities isn't in the graph")
        return

    dist, prev = dijkstra(city_roads, src)
    route = get_path(prev, src, dst)

    if not route:
        print(f"No path exists between {src} and {dst}")
        return

    print(f"\n{src} to {dst}")
    print(f"  distance : {dist[dst]} km")
    print(f"  route    : {' -> '.join(route)}")
    print(f"  stops    : {len(route) - 2} cities in between")


if __name__ == "__main__":
    print("Dijkstra's Algorithm on Indian Road Network")
    print("=" * 50)

    print("\n--- From Delhi ---")
    show_all_from("Delhi")

    print("\n\n--- Some specific routes ---")
    find_route("Mumbai", "Kolkata")
    find_route("Bangalore", "Delhi")
    find_route("Amritsar", "Chennai")
    find_route("Guwahati", "Goa")

    print("\n\n--- From Bangalore ---")
    show_all_from("Bangalore")
