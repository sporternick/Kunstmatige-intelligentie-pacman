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
    #Add starting state to stack
    frontier = util.Stack()
    startState = problem.getStartState()

    tup = (startState,[])
    frontier.push(tup)

    #Frontier
    visitedNodes = []

    #While moves are avaible
    while(not frontier.isEmpty()):
        #Get next move in frontier
        node = frontier.pop()
        state = node[0]

        #Check if node was already visited
        evaluateNode = True
        for vis in visitedNodes:
            if(vis == state):
                evaluateNode = False
                break
        if not evaluateNode:
            continue

        #Add to visisted nodes
        visitedNodes.append(state)

        #Return path if goal found
        if(problem.isGoalState(state)):
            print(node[1])
            return node[1]
        #Add succesors to list
        else:
            successors = problem.getSuccessors(state)
            for s in successors:
                succ = s[0]
                #Add new move to path
                path = node[1] + [s[1]]
                #Push succesor onto stack with updated path
                frontier.push( (succ,path) )



def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    #Add starting state to stack
    frontier = util.Queue()
    startState = problem.getStartState()

    tup = (startState,[])
    frontier.push(tup)

    #Frontier
    visitedNodes = []

    #While moves are avaible
    while(not frontier.isEmpty()):
        #Get next move in frontier
        node = frontier.pop()
        state = node[0]

        #Check if node was already visited
        evaluateNode = True
        for vis in visitedNodes:
            if(vis == state):
                evaluateNode = False
                break
        if not evaluateNode:
            continue

        #Add to visisted nodes
        visitedNodes.append(state)

        #Return path if goal found
        if(problem.isGoalState(state)):
            print(node[1])
            return node[1]
        #Add succesors to list
        else:
            successors = problem.getSuccessors(state)
            for s in successors:
                succ = s[0]
                #Add new move to path
                path = node[1] + [s[1]]
                #Push succesor onto stack with updated path
                frontier.push( (succ,path) )


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    #Add starting state to stack
    frontier = util.PriorityQueue()
    startState = problem.getStartState()

    #Frontier tuple: (x,y,path,cost)
    frontier.push(startState+([],0,),0)

    #Frontier
    visitedNodes = []

    #While moves are avaible
    while(not frontier.isEmpty()):
        #Get next move in frontier
        state = frontier.pop()

        #Check if node was already visisted
        evaluateNode = True
        for vis in visitedNodes:    #Perhaps recalculate nodes if current cost is smaller than visistedNode's cost
            if(vis == (state[0],state[1])):
                evaluateNode = False
                break
        if not evaluateNode:
            continue

        #Add to visisted nodes
        visitedNodes.append((state[0],state[1]))

        #Return path if goal found
        if(problem.isGoalState((state[0],state[1]))):
            return state[2]
        #Add succesors to list
        else:
            successors = problem.getSuccessors((state[0],state[1]))
            for s in successors:
                succ = s[0]
                #Add new move to path
                path = state[2]+[s[1]]
                #Increase cost
                cost = state[3] + s[2]
                #Push successor onto stack with updated path
                frontier.push(succ+(path,cost, ),s[2])

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    #Add starting state to stack
    frontier = util.PriorityQueue()
    startState = problem.getStartState()

    tup = (startState,[])
    frontier.push(tup,0)

    #Frontier
    visitedNodes = []

    #While moves are avaible
    while(not frontier.isEmpty()):
        #Get next move in frontier
        node = frontier.pop()
        state = node[0]

        #Get path length
        pathlength = len(node[1])

        #Check if node was already visited and if path can be improved
        evaluateNode = True
        for vis in visitedNodes:
            if(vis[0] == state ):
                if(vis[1] > pathlength):
                    #remove this node from visited
                    visitedNodes.remove(vis)
                    break
                else:
                    evaluateNode = False
                    break
        if not evaluateNode:
            continue

        #Add state and pathlength to visited nodes
        visitedNodes.append( (state,pathlength) )

        #Return path if goal found
        if(problem.isGoalState(state)):
            return node[1]
        #Add succesors to list
        else:
            successors = problem.getSuccessors(state)
            successorLength = pathlength+1
            for s in successors:
                succ = s[0]
                #Add new move to path
                path = node[1]+[s[1]]
                #Calculate heuristic value
                heur = successorLength + heuristic(state,problem)
                #Push succesor onto stack with updated path
                frontier.push( (succ,path) ,heur)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
