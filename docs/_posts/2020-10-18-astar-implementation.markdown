---
title: Icaria A* Pathfinding Implementation
---

### What Makes This A* Special
Almost any game is going to have a unique solution to pathfinding--while A\* is a very common approach, almost any
real game which needs high performance path finding will need to customize it in one way or another. Here's a some
of the tweaks we are using:

- Orientation matters for our pathfinding so we have four separate pathfinding nodes for each coordinate on the grid corresponding to each heading: North, South, East or West.
- When we path we cache the actual distance to the destination which helps optimize repathing if the the path is blocked while we are in transit.
- The movement rules are variable so we abstract out the node transitions.
- The code is designed to minimize allocations and thereby C# garbage collection time.

Is it better than another implementation? Probably not, but there might be some ideas here that will help with your game.

### Overview of the Problem

All movement in Icaria lives on a two dimensional grid. There are a few key requirements of our path finding algorithm:

- Not all entities move by the same rules. For instance some can move diagonally and others can't.
- Different types of movement can take different amounts of time. For instance, backing up is significantly slower than moving forward.
- In addition, different kinds of tiles have different speed modifiers. Going up hills is slower than the flat and, while deep water is forbidden, shallow water is slow.
- Some entities are bigger than one tile and not all are square. We need to keep track of orientation as a 1x2 entity touches different tiles when its orientation changes.

We use `struct MapPoint` as a simple structure to reference a location on the map.
In addition, we generate paths for entities which occupy more than one grid square so we need to track the orientation as well.
We define a `struct Placement` to hold both a `MapPoint`
and a heading where the heading is defined like so:

    public enum Heading : byte
    {
        Any = 0,
        East = 1,
        North = 2,
        West = 3,
        South = 4
    }

where east in facing in the +X direction and North is facing in the +Y direction. Any is a way to tell the the path finding
code that the orientation at the destination doesn't matter.

### Data Types

We store two data structures, a three dimensional array of `Node`:

    public struct Node
    {
        public int GCost;
        public int HCost;
        public int ZCost;
        public int FCost
        {
            get { return ZCost < int.MaxValue ? GCost + ZCost : GCost + HCost; }
        }

        public override string ToString()
        {
            return string.Format("G:{0} H:{0} Z:{0}", GCost, HCost, ZCost);
        }
    }

where, in traditional form `GCost` is the total cost of getting to the node, `HCost` is the heuristic
cost of getting to the destination and `FCost` is the estimated value of the node used to decide which node to explore next.
Choosing a heuristic for A\* is difficult given that speed of travel varies fairly widely.
We currently use Euclidean distance with a speed modifier which is tuned per rule set.
The one value here that is unusual if you compare standard implementations, say [Amit's A* Pages](https://www.redblobgames.com/pathfinding/a-star/introduction.html"),
is the `ZCost`. The `ZCost` is only filled in when back tracing the path and holds the real cost of getting to the destination rather than the heuristic.
The reason for keeping the ZCost is that its a value which be cached and reused for other attempts to path through the same area.
In particular it speeds up pathing around obstacles which crop during travel.

In addition we keep a binary heap priority queue of open nodes:

    public struct OpenNode
    {
        public OpenNode(Node node, Placement placement)
        {
            Node = node;
            Placement = placement;
        }
        public Node Node { get; }
        public Placement Placement { get; }
    }
   
### Walking the Graph

Open nodes are defined on Amit's page, but the gist of the algorithm is that nodes are stored in three sets:
Unexplored, Open, and Closed. Each iteration picks the lowest FCost open node, and promotes all its Unexplored neighbors
to Open by adding them to the priority queue. Then the currently explored node is closed.
Our code for this operation looks like this:

    bool Explore()
    {
        const int SearchMax = 10000;
        for (int count = 0; count < SearchMax; ++count)
        {
            AStar.OpenNode node;
            if (!AStar.TryPopOpenNode(out node))
                return false;
            int moveCount;
            Entity.Movement.GetCandidateMoves(
                _candidateMoves, out moveCount, node.Placement);
            for (int i = 0; i < moveCount; ++i)
            {
                int time = MovementTime(
                    node.Placement,
                    _candidateMoves[i].Placement,
                    _candidateMoves[i].BaseTime);
                if (Movement.CanMove(time))
                {
                           AStar.PushOpenNode(_candidateMoves[i].Placement,
                        node.Node.GCost + time);
                    if (_candidateMoves[i].Placement.Match(Destination))
                    {
                        Destination = _candidateMoves[i].Placement;
                        return true;
                    }
                }
            }
        }
        return false;
    }


Each entity has its own movement rules which are managed by the Movement class.
- `GetCandidateMoves()` returns an array of all possible moves the entity could make ignoring obstacles,
- `MovementTime()` calculates the actual cost of the movement, and
- `CanMove()` looks at the result of `MovementTime()` to determine if the move is actually legal (`MovementTime()` returns a variety of error codes for illegal moves).

### Back Tracing the Path

Generating the actual path involves working backward from the destination to the starting point, each time picking the backward movement
which has the minimum `GCost`; i.e. the one closest to the starting point. `GetCandidateMoves()` has an extra bool parameter which generates
reverse moves which is how we back trace. During the back trace operation we fill in every `ZCost` we can to aid if we need to repath later.

    bool BackTrace()
    {
        List&ltplacement&gt path = new List&ltplacement&gt();
        Placement placement = Destination;
        int ZCost = 0;
        path.Add(placement);
        int GCost = AStar.NodeForPlacement(placement).GCost;
        for (int i = 0; i < 1000; ++i)
        {
            int moveCount;
            Entity.Movement.GetCandidateMoves(_candidateMoves,
                out moveCount, placement, true);
            int moveTime = int.MaxValue;
            for (int j = 0; j < moveCount; ++j)
            {
                int candidateMoveTime = MovementTime(
                    _candidateMoves[j].Placement, placement,
                    _candidateMoves[j].BaseTime);
                if (Movement.CanMove(candidateMoveTime))
                {
                    AStar.Node node = AStar.NodeForPlacement(
                        _candidateMoves[j].Placement);
                    if (ZCost + candidateMoveTime < node.ZCost)
                    {
                        AStar.SetZCost(_candidateMoves[j].Placement,
                            ZCost + candidateMoveTime);
                    }
                    if (node.GCost < GCost)
                    {
                        GCost = node.GCost;
                        placement = _candidateMoves[j].Placement;
                        moveTime = candidateMoveTime;
                    }
                }
            }
            if (moveTime == int.MaxValue)
            {
                return false;
            }
            path.Add(placement);
            if (placement == Placement)
            {
                Path = path;
                return true;
            }
            ZCost += moveTime;
        }
        return false;
    }

### Binary Heap Priority Queue

We keep track of the open nodes using a binary heap priority queue; it a standard algorithm in any textbook
but this implementation is ours.

`PushOpenNode()` just adds a node to the binary heap:

    public void PushOpenNode(Placement placement, int GCost)
    {
        int i = placement.Location.x - Bounds.sw.x;
        int j = placement.Location.y - Bounds.sw.y;
        int k = IndexForHeading(placement.Heading);
        if (i < 0 || i >= Bounds.size.w || j < 0 || j >= Bounds.size.h)
        {
            return;
        }
        InitNode(i, j, ref _nodeData[i, j, k]);
        if (GCost < _nodeData[i, j, k].GCost)
        {
            _nodeData[i, j, k].GCost = GCost;
            _openNodes.Add(new OpenNode(_nodeData[i, j, k], placement));
            // Binary Heap insert.
            int nodeIndex = _openNodes.Count;
            while (nodeIndex > 1)
            {
                int parentIndex = nodeIndex / 2;
                if (_openNodes[nodeIndex-1].Node.FCost > _openNodes[parentIndex-1].Node.FCost)
                {
                    break;
                }
                SwapOpenNodes(nodeIndex - 1, parentIndex - 1);
                nodeIndex = parentIndex;
            }
        }
    }

And `TryPopOpenNode()` returns the node with the minimum `FCost`:

    public bool TryPopOpenNode(out OpenNode openNode)
    {
        if (_openNodes.Count == 0)
        {
            openNode = new OpenNode(new Node(), new Placement());
            return false;
        }
        openNode = _openNodes[0];
        if (_openNodes.Count > 1)
        {
            // Binary Heap delete_min.
            _openNodes[0] = _openNodes[_openNodes.Count - 1];
            _openNodes.RemoveAt(_openNodes.Count - 1);
            int nodeIndex = 2;
            int count = _openNodes.Count;
            while (nodeIndex <= count)
            {
                if (nodeIndex < count && _openNodes[nodeIndex].Node.FCost < _openNodes[nodeIndex-1].Node.FCost)
                {
                    ++nodeIndex;
                }
                int parentIndex = nodeIndex / 2;
                if (_openNodes[parentIndex-1].Node.FCost < _openNodes[nodeIndex-1].Node.FCost)
                {
                    break;
                }
                SwapOpenNodes(nodeIndex - 1, parentIndex - 1);
                nodeIndex *= 2;
            }
        }
        else
        {
            _openNodes.Clear();
        }
        return true;
    }

All in all, its pretty much a bog-standard implementation of A\* with the added spin of stashing the actual for getting
to the destination for future use. One key aspect of this particular design is the use of C# `array` and `struct`
rather than `List` and `class` to avoid heap allocations and the corresponding garbage collection lag spikes. End-to-end, finding a
path involves fewer than 10 separate new heap objects.
