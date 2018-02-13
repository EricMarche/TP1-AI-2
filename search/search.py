# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).

"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    stack = util.Stack()
    stack.push(problem.getStartState())
    visitedNodes = []
    path = {}
    #On insert l'etat depart afin de savoir que notre chemin est complet
    path[problem.getStartState()] = [(problem.getStartState(), )]
    solution = []
    while stack.isEmpty() != True:
        node = stack.pop()
        if node not in visitedNodes:
            visitedNodes.append(node)
            if problem.isGoalState(node):
                #On part de la fin et on remonte au debut
                state = node
                while True:
                    if state == problem.getStartState():
                        break
                    solution.append(path[state][1])
                    state = path[state][0]
                solution.reverse()
                return solution
            for edges in problem.getSuccessors(node):
                if edges[0] not in visitedNodes:
                    path[edges[0]] = (node, edges[1])
                    stack.push(edges[0])

    #Si il y a un but atteignable nous ne devrions pas arrive ici
    return []

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    queue = util.Queue()
    queue.push(problem.getStartState())
    visitedNodes = []
    path = {}
    path[problem.getStartState()] = [(problem.getStartState(), )]
    solution = []
    while queue.isEmpty() != True:
        node = queue.pop()
        if node not in visitedNodes:
            visitedNodes.append(node)
            if problem.isGoalState(node):
                #On part de la fin et on remonte au debut
                state = node
                while True:
                    if state == problem.getStartState():
                        break
                    solution.append(path[state][1])
                    state = path[state][0]
                solution.reverse()
                return solution
            for edges in problem.getSuccessors(node):
                if edges[0] not in visitedNodes:
                    queue.push(edges[0])
                if edges[0] not in path:
                    path[edges[0]] = (node, edges[1])


    #Si il y a un but atteignable nous ne devrions pas arrive ici
    return []

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    #Ajout d'import pour avoir une valeur proche infini
    import sys
    maxInt = sys.maxint
    nodes = util.PriorityQueue()
    distance = {}
    previousNode = {}
    nodes.push(problem.getStartState(), 0)
    distance[problem.getStartState()] = 0
    solution = []
    while nodes.isEmpty() != True:
        node = nodes.pop()

        if problem.isGoalState(node):
            while True:
                if node == problem.getStartState():
                    break
                solution.append(previousNode[node][1])
                node = previousNode[node][0]
            solution.reverse()
            return solution

        for nextNode in problem.getSuccessors(node):
            if nextNode[0] not in distance.keys():
                distance[nextNode[0]] = maxInt
            newCost = distance[node] + nextNode[2]
            if newCost < distance[nextNode[0]]:
                distance[nextNode[0]] = newCost
                previousNode[nextNode[0]] = (node, nextNode[1])
                nodes.update(nextNode[0], newCost)

    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    import sys
    maxInt = sys.maxint
    nodes = util.PriorityQueue()
    distance = {}
    previousNode = {}
    startState = problem.getStartState()
    nodes.push(startState, heuristic(startState, problem))
    distance[startState] = 0
    solution = []
    while nodes.isEmpty() != True:
        node = nodes.pop()
        if problem.isGoalState(node):
            while True:
                if node == problem.getStartState():
                    break
                solution.append(previousNode[node][1])
                node = previousNode[node][0]
            solution.reverse()
            return solution

        for nextNode in problem.getSuccessors(node):
            if nextNode[0] not in distance.keys():
                distance[nextNode[0]] = maxInt
            newCost = distance[node] + nextNode[2]
            if newCost < distance[nextNode[0]]:
                distance[nextNode[0]] = newCost
                previousNode[nextNode[0]] = (node, nextNode[1])
                nodes.update(nextNode[0], newCost + heuristic(nextNode[0], problem))
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
