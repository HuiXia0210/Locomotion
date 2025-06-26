class TrainGraph:
    """
    Class description: 
        Represents the combined road-train network for interception pathfinding.
        Supports shortest path computation with time remainders for looping train movement.

    Author: Ooi Hui Xia 
    """

    def __init__(self, stations, roads):
        """
        Function description: 
            Initializes the TrainGraph by storing station data and building an adjacency list of roads.

        Approach: 
            Iterates through roads to determine the number of vertices, initializes empty lists, and populates each location with outgoing Road objects.

        Input: 
            stations: list of Station objects (location, travel_time)
            roads: list of Road objects (u, v, cost, time)

        Output: None 

        Time complexity: O(r), where r is number of roads  
        Time complexity analysis: One pass to find max vertex ID, one pass to populate adjacency list → O(r)

        Space complexity: O(l + r), where l is the number of unique locations  
        Space complexity analysis: O(l) for vertex lists + O(r) for road storage
        """
        self.stations = stations
        self.num_vertices = 0
        for road in roads:
            self.num_vertices = max(self.num_vertices, road.u + 1, road.v + 1)
        self.vertices = [[] for _ in range(self.num_vertices)]
        for road in roads:
            self.vertices[road.u].append(road)

    


    def dijkstra(self, start, friend_time, total_loop_time):
        """
        Function description: 
            Runs Dijkstra’s algorithm to find the cheapest and shortest paths from a start location, considering time offsets on a circular train loop.

        Approach: 
            Tracks each location's cost/time at every possible remainder modulo the train loop time. Uses a custom MinHeap to process Vertex states based on (location, remainder). Relaxes neighbors using fuel cost as primary priority and time as tie-breaker.

        Input: 
            start: starting location 
            friend_time: friend's starting offset on the train loop 
            total_loop_time: total time to complete the train loop 

        Output: Tuple of (cost, time, prev, vertices)

        Time complexity: O(R * log(L * T)), where:
            - R is the number of roads (edges)
            - L is the number of locations (vertices)
            - T is the total loop time (modulo base)
        
        Time complexity analysis: 
            - Each (location, remainder) pair is treated as a unique state, resulting in up to L * T states.
            - Each edge may lead to a transition between states and can be relaxed once per state.
            - Each heap operation takes O(log(L * T)), and O(R) edges are relaxed.
            - Therefore, total time is O(R * log(L * T)).

        Space complexity: O(L * T) 
        
        Space complexity analysis: All 2D arrays store L * T values, and heap holds up to L * T entries
        """
        INF = float('inf')
        n = self.num_vertices
        max_r = total_loop_time
        cost = [[INF] * max_r for _ in range(n)]
        time = [[INF] * max_r for _ in range(n)]
        prev = [[(-1, -1)] * max_r for _ in range(n)]
        vertices = [[None] * max_r for _ in range(n)]

        heap = MinHeap(n * max_r + 10)

        # Reset all vertices to prepare for Dijkstra's algorithm 
        start_vertex = Vertex(start, 0)
        vertices[start][0] = start_vertex
        cost[start][0] = 0 # Starting cost is 0
        time[start][0] = 0 # Starting time is 0
        heap.add((start_vertex, (0, 0)))

        while len(heap) > 0:
            u, (d_cost, d_time) = heap.get_min()
            u_id = u.location
            r = u.remainder

            if (d_cost, d_time) > (cost[u_id][r], time[u_id][r]):
                continue

            for edge in self.vertices[u_id]:
                v = edge.v
                new_cost = cost[u_id][r] + edge.cost
                new_time = time[u_id][r] + edge.time
                new_r = (r + edge.time) % total_loop_time

                if new_cost < cost[v][new_r] or (new_cost == cost[v][new_r] and new_time < time[v][new_r]):
                    cost[v][new_r] = new_cost
                    time[v][new_r] = new_time
                    prev[v][new_r] = (u_id, r)

                    if vertices[v][new_r] is None:
                        vertices[v][new_r] = Vertex(v, new_r)
                        heap.add((vertices[v][new_r], (new_cost, new_time)))
                    else:
                        heap.update(vertices[v][new_r], (new_cost, new_time))

        return cost, time, prev, vertices
    



class Road:
    """
    Class description: Represents a road between two locations in the network, including the cost and time to travel.

    Attributes:
        u: Starting location of the road 
        v: Ending location of the road 
        cost: Cost to travel the road 
        time: Time to travel the road 
    
    Time Complexity: Best and Worst Case O(1),
                          assigning values to variables is constant.
        
    Aux Space Complexity: O(1), assigning values  to variables, therefore constant.
    """
    def __init__(self, u, v, cost, time):
        self.u = u
        self.v = v
        self.cost = cost
        self.time = time


class Station:
    """
        Class description: 
            This class represents a train station node on a circular train loop, 
            storing its location and the travel time required to reach the next station on the loop.

        Purpose: 
            Used to represent one segment of the train path when simulating the movement of 
            the passenger on the circular train system. This information is vital for determining 
            when and where interception is possible.

        Input: 
            location:  The station's location (ℓT_S)
            travel_time:  Time it takes to travel from this station to the next

        Output: A Station object with its internal attributes assigned

        Time complexity: O(1), assigning values to variables is constant.

        Aux Space complexity: O(1)

        Aux Space complexity analysis: 
            - Only two attributes (location and travel_time) are stored, so the space used is constant.
    """
    def __init__(self, location, travel_time):
        self.location = location
        self.travel_time = travel_time


class Vertex:
    """
        Class description: 
            This class represents a node in the graph used for pathfinding, 
            storing both the location and the remainder (the modulo time with respect to the train loop).

        Purpose: 
            Used in Dijkstra’s algorithm to track not just the physical location in the graph, 
            but also the time remainder to correctly synchronize with a moving train passenger.

        Input:
            location: The location ID in the graph
            remainder: The time remainder modulo the total train loop time

        Output: A Vertex object with its attributes initialized

        Time complexity: O(1)

        Time complexity analysis:
            - Only constant-time assignments are made to object attributes.

        Aux Space complexity: O(1)

        Aux Space complexity analysis:
            - The object stores three simple integer attributes (location, remainder, position), 
              so the space required is constant.
    """
    def __init__(self, location, remainder):
        self.location = location
        self.remainder = remainder
        self.position = -1

class MinHeap:
    """
    A MinHeap implementation that stores tuples of (item, (cost, time)), 
    allowing prioritization based on lexicographical order of (cost, time).
    
    Attributes:
        length (int): Number of elements currently in the heap.
        array (List[Tuple]): Internal array representation of the heap.
    """
    def __init__(self, size):
        # keep track of the number of items in the MinHeap 
        self.length = 0
        self.array = [None] * (size + 1)

    def __len__(self):
        return self.length # Get the number of elements currently in the heap 

    def is_full(self):
        return self.length + 1 == len(self.array) # Check whether the heap has reached its capacity

    def rise(self, k):
        while k > 1 and self.array[k][1] < self.array[k // 2][1]:  # compare (cost, time)
            self.swap(k, k // 2)
            k = k // 2

    def add(self, element):
        if self.is_full():
            return False
        vertex = element[0]
        self.length += 1
        self.array[self.length] = element
        vertex.position = self.length
        self.rise(self.length)
        return True

    def smallest_child(self, k):
        """
        Returns the index of the smallest child of element at k.
        
        :Input:
            k: index of element[k]
        
        :Output, return or postcondition:
            Returns the index of the smallest child of element[k].
        
        :Time Complexity: Best and Worst Case O(1),
                            compares the left child and right child, return the smaller one
                            constant. 
        
        :Aux Space Complexity: O(1), constant operation and only return the smaller child.
        """
        if 2 * k == self.length or self.array[2 * k][1] < self.array[2 * k + 1][1]:
            return 2 * k
        else:
            return 2 * k + 1

    def sink(self, k):
        """
        Makes the element at index k sink to the correct position
        
        Input:
            k: index of element[k]
        
        Output, return or postcondition:
            Element[k] at its correct position now. 
        
        Time Complexity: Best Case O(1),
                            the element is smaller than its children,
                            therefore is already at its correct position
                          Worst Case O(logV), where logv is the depth of MinHeap,
                            the element sinks all the way to the bottom.
        
        Aux Space Complexity: O(1), performs swap therefore does not require extra space. 
        """
        while 2 * k <= self.length:
            child = self.smallest_child(k)
            if self.array[k][1] <= self.array[child][1]:  # compare (cost, time)
                break
            self.swap(child, k)
            k = child

    def swap(self, i, j):
        # need to swap position when swap array
        self.array[i], self.array[j] = self.array[j], self.array[i]
        self.array[i][0].position = i
        self.array[j][0].position = j

    def get_min(self):
        """ 
        Returns the minimum element in the MinHeap and heapifies after it is removed from the heap. 
        
        Output, return or postcondition:
            Returns the minimum element in the MinHeap, the heap is heapified.
        
        Time Complexity: Best Case O(1) where the new element is smaller than or equal to its child.
                          Worst Case O(logN), where logN is the depth of the MinHeap
                          the new element is sinked all the way to the bottom of the heap.
        
        Aux Space Complexity: O(1), swaps the element therefore does not require extra space.
        """
        min_item = self.array[1]
        self.array[1] = self.array[self.length]
        self.length -= 1
        if self.length > 0:
            self.sink(1)
        return min_item

    def update(self, vertex, cost_time):  # cost_time is a tuple (cost, time)
        """
            Function description: Updates an existing vertex's cost and time, and reorders 
            the heap if necessary to maintain heap property.

            Input:
                vertex: a Vertex object already in the heap
                cost_time: a tuple (cost, time) representing the new values

            Output: None

            Time complexity: Best Case O(1) where the element is already at its correct position.
                            Worst Case O(logV), where logV is the depth of the MinHeap
                            the element is rised all the way to the top. 

            Space complexity: O(1), , swaps the element therefore does not require any extra space.
        """
        k = vertex.position
        self.array[k] = (vertex, cost_time)
        self.rise(k)




def intercept(roads, stations, start, friendStart):
    """
    Function description:
        Finds the least-cost path to intercept a moving friend traveling in a loop of train stations.
        The function returns the total driving cost, time taken, and path if interception is possible.

    Approach description:
        A modified Dijkstra's algorithm is used on a state-space graph of (location, time mod loop_time).
        The friend's arrival times are precomputed, and interception is checked when reaching a station
        at the same time (modulo loop) as the friend.

    Input:
        roads: list of tuples (u, v, cost, time) representing the road network.
        stations: list of tuples (location, travel_time) forming the train loop.
        start: starting location of the driver.
        friendStart: station where the friend begins their train loop.

    Output:
        A tuple (total_cost, total_time, path) if interception is possible, else None.

    Time complexity: O(|R| log |L|), L = total locations in the city , R = roads 

    Time complexity analysis:
        - Each road is relaxed at most once per state.
        - Each heap operation takes O(log |L|), giving overall O(|R| log |L|).

    Auxiliary space complexity: O(|L| + |R|), L = total locations in the city , R = roads 

    Space complexity analysis:
        - Space for graph = O(|R|), and state tracking arrays = O(|L|) (loop time is bounded).
        """
    roads = [Road(*road) for road in roads]
    stations = [Station(location, travel_time) for location, travel_time in stations]

    graph = TrainGraph(stations, roads)

    # Total train loop time
    total_loop_time = sum(station.travel_time for station in stations)

    # Friend's arrival times at stations
    friend_time = [0] * len(stations)
    index = 0
    while stations[index].location != friendStart:
        index += 1
    offset = 0
    for i in range(len(stations)):
        idx = (index + i) % len(stations)
        friend_time[idx] = offset
        offset += stations[idx].travel_time

    max_r = total_loop_time
    n = graph.num_vertices
    INF = float('inf')

    cost = [[INF] * max_r for _ in range(n)]
    time = [[INF] * max_r for _ in range(n)]
    prev = [[(-1, -1)] * max_r for _ in range(n)]
    vertices = [[None] * max_r for _ in range(n)]

    heap = MinHeap(n * max_r + 10)

    start_vertex = Vertex(start, 0)
    vertices[start][0] = start_vertex
    cost[start][0] = 0
    time[start][0] = 0
    heap.add((start_vertex, (0, 0)))

    while len(heap) > 0:
        u, (d_cost, d_time) = heap.get_min()
        u_id = u.location
        r = u.remainder

        if (d_cost, d_time) > (cost[u_id][r], time[u_id][r]):
            continue


        # Check for interception
        for i in range(len(stations)):
            if stations[i].location == u_id:
                arrival = friend_time[i]
                if time[u_id][r] >= arrival and (time[u_id][r] - arrival) % total_loop_time == 0:
                    route = []
                    cur_u, cur_r = u_id, r
                    while cur_u != -1:
                        route.append(cur_u)
                        cur_u, cur_r = prev[cur_u][cur_r]
                    route.reverse()
                    return cost[u_id][r], time[u_id][r], route

        # Relax edges
        for edge in graph.vertices[u_id]:
            v = edge.v
            new_cost = cost[u_id][r] + edge.cost
            new_time = time[u_id][r] + edge.time
            new_r = (r + edge.time) % total_loop_time

            if new_cost < cost[v][new_r] or (new_cost == cost[v][new_r] and new_time < time[v][new_r]):
                cost[v][new_r] = new_cost
                time[v][new_r] = new_time
                prev[v][new_r] = (u_id, r)

                if vertices[v][new_r] is None:
                    vertices[v][new_r] = Vertex(v, new_r)
                    heap.add((vertices[v][new_r], (new_cost, new_time)))  
                else:
                    heap.update(vertices[v][new_r], (new_cost, new_time))  

    return None


