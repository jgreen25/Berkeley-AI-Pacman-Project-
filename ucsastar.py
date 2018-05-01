#ucs
    startState = problem.getStartState()
    visited = []
    fringe = util.Queue()
    fringe.push((startState, ()))

    while not fringe.isEmpty():
        currNode = fringe.pop()
        currState = currNode[0]
        currPlan = currNode[1]

        if problem.isGoalState(currState):
            return list(currPlan)

        if not currState in visited:
            visited.append(currState)
            paths = problem.getSuccessors(currState)
            costs = []

            for path in paths:
                newPlan = list(currPlan)
                newPlan.append(path[1])
                costs.append(problem.getCostOfActions(newPlan))

            costs.sort()
            
            for cost in costs:
                for path in paths:
                    newPlan = list(currPlan)
                    newPlan.append(path[1])
                    if problem.getCostOfActions(newPlan) == cost:
                        nextNode = (path[0], tuple(newPlan))

                        if not path[0] in visited:
                            fringe.push(nextNode)

    util.raiseNotDefined()

    #astar
    startState = problem.getStartState()
    visited = []
    fringe = util.Queue()
    fringe.push((startState, ()))

    while not fringe.isEmpty():
        currNode = fringe.pop()
        currState = currNode[0]
        currPlan = currNode[1]

        if problem.isGoalState(currState):
            return list(currPlan)
        
        if not currState in visited:
            visited.append(currState)
            paths = problem.getSuccessors(currState)
            costs = []

            for path in paths:
                newPlan = list(currPlan)
                newPlan.append(path[1])
                first = problem.getCostOfActions(newPlan)
                second = heuristic(path[0], problem)
                cost = first + second
                costs.append(cost)

            costs.sort()
            
            for cost in costs:
                for path in paths:
                    newPlan = list(currPlan)
                    newPlan.append(path[1])
                    if problem.getCostOfActions(newPlan) + heuristic(path[0], problem) == cost:
                        nextNode = (path[0], tuple(newPlan))
                        if not path[0] in visited:
                            fringe.push(nextNode)

    util.raiseNotDefined()

    #cornersheuristic
    position = state[0]
    corner_state = list(state[1])

    if problem.isGoalState(state):
        return 0

    unvisited_corners = []
    for i in range(len(corner_state)):
        if corner_state[i] == 0:
            unvisited_corners.append(corner_state[i])

    current_pos = position
    cost = 0
    while len(unvisited_corners) != 0:
        idx, dist = findClosestDist(current_pos, unvisited_corners)
        cost += dist
        current_pos = unvisited_corners[idx]
        unvisited_corners.remove(unvisited_corners[idx])

    return cost