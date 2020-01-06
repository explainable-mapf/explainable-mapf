import networkx as nx
import itertools
import heapq
from timeit import default_timer as timer

class AstarNode:
    #type of data:
    #data: dictionary with three elements: 'current', 'history', and 'segments'
    #data['current']: array of current nodes, indexed on agent.
    #data['history']: array of history nodes, indexed on agent, each history is a set.
    #data['segments']: number of segments so far

    GOAL_STR="G"
    hashCount=0
    hashTime=0

    def __init__(self, parent, data, graph, target):
        self.parent = parent
        self.graph=graph
        self.data=data
        self.target= target
        self.hashVal=0
        self.runtime=0
        self.computeHash()

    def getNeighborsNonGoal(self, ag):
        if self.data['current'][ag]== self.target[ag] or self.data['current'][ag]== AstarNode.GOAL_STR:
            return [AstarNode.GOAL_STR] #reached goal
        return self.graph.neighbors(self.data['current'][ag])

    def getChildren(self):
        num_agents = len(self.data['current'])
        ags = list(range(num_agents))
        children=[]
        neighbors=[self.getNeighborsNonGoal(ag) for ag in ags]

        for tup in list(itertools.product(*neighbors)):
            newchild_current = list(tup)

            if not self.isLegalChild(newchild_current):
                continue

            newchild_segments=self.data['segments']
            newchild_history={}
            for ag in ags:
                if tup[ag]!=AstarNode.GOAL_STR:
                    newchild_history[tup[ag]] = ag

            if self.isNewSegmentChild(newchild_current):
                newchild_segments += 1
            else:
                newchild_history = self.data['history'].copy()
                for ag in ags:
                    if tup[ag] != AstarNode.GOAL_STR:
                        newchild_history[tup[ag]] = ag

            children.append(AstarNode(self,{'current':newchild_current, 'history':newchild_history, 'segments':newchild_segments},self.graph,self.target))
        return children

    def isLegalChild(self, newCurrent):
        num_agents = len(self.data['current'])
        for ag1 in range(num_agents):
            if newCurrent[ag1] != AstarNode.GOAL_STR:
                for ag2 in range(ag1+1,num_agents):
                    if newCurrent[ag2] != AstarNode.GOAL_STR:
                        if newCurrent[ag1] == newCurrent[ag2]: # Collision occurred
                            return False
                        if newCurrent[ag1] == self.data['current'][ag2] and newCurrent[ag2] == self.data['current'][ag1]: # Swap occurred
                            return False
        return True

    def isNewSegmentChild(self, newCurrent):
        num_agents = len(self.data['current'])
        for ag1 in range(num_agents):
            if newCurrent[ag1] != AstarNode.GOAL_STR:
                if self.data['history'].get(newCurrent[ag1], ag1) != ag1:
                    return True

        return False

    def __eq__(self, other):
        if self.data['segments'] != other.data['segments']:
            return False
        for i in range(len(self.data['current'])):
            if self.data['current'][i]!=other.data['current'][i]:
                return False
        if self.data['history']!=other.data['history']:
            return False
        return True

    def computeHash(self):
        self.hashVal=hash(str(self.data['current'])+str(self.data['segments'])+str(frozenset(self.data['history'].items())))

    def __hash__(self):
        return self.hashVal

    def __lt__(self, other):
        return self.data['segments'] <= other.data['segments']

    def __str__(self):
        return str(self.data)


class AstarSolver:
    def __init__(self,graph,initNodes,targetNodes):
        self.graph=graph
        self.initNodes=initNodes
        self.targetNodes=targetNodes
        self.numAgents=len(initNodes)
        self.plan=None
        self.allShortestPaths = None

    def computeHeuristic(self):
        self.allShortestPaths=dict(nx.all_pairs_shortest_path_length(self.graph))

    def heuristicVal(self,node):
        val = 0
        for ag in range(self.numAgents):
            if node.data['current'][ag]!= AstarNode.GOAL_STR:
                val += self.allShortestPaths[node.data['current'][ag]][self.targetNodes[ag]]
        return val

    def isGoal(self,node):
        for ag in range(self.numAgents):
            if node.data['current'][ag] != AstarNode.GOAL_STR and node.data['current'][ag] != self.targetNodes[ag]:
                return False
        return True

    def astar(self, numSegments,timeout=300):
        stdata={}
        stdata['current'] = self.initNodes
        stdata['history'] = {} #[set() for ag in range(self.numAgents)]
        for ag in range(self.numAgents):
            stdata['history'][self.initNodes[ag]]=ag
        stdata['segments'] = 0
        startNode=AstarNode(None,stdata,self.graph,self.targetNodes)
        #print(startNode)

        gscore={startNode:0}
        fscore={startNode:self.heuristicVal(startNode)}
        openSet=[]
        closedSet=set()
        openSetHash=set()
        heapq.heappush(openSet,(fscore[startNode],startNode))
        openSetHash.add(startNode)

        diagCounter=0
        start = timer()

        while openSet:
            #diagCounter+=1
            curNode = heapq.heappop(openSet)[1]
            openSetHash.remove(curNode)

            # if diagCounter == 1000:
            #     end=timer()
            #     if int(end)-int(start)>timeout:
            #         self.runtime=end-start
            #         return False
            #     diagCounter=0
            #     print("Open Set Size: "+ str(len(openSet)))
            #     print("Closed Set Size: " + str(len(closedSet)))
            #     print("Current Node: "+ str(curNode))


            if self.isGoal(curNode):
                #print("Reached the goal!"+str(curNode))
                self.computePlan(curNode)
                end=timer()
                self.runtime=end-start
                return True

            closedSet.add(curNode)

            children = curNode.getChildren()

            for child in children:
                if child.data['segments']>numSegments:
                    continue
                if child in closedSet:
                    continue

                tentative_g=gscore[curNode]+1

                bcheck=(child not in openSetHash)


                if bcheck:
                    child.parent = curNode
                    gscore[child] = tentative_g
                    fscore[child] = tentative_g + self.heuristicVal(child)
                    heapq.heappush(openSet, (fscore[child], child))
                    openSetHash.add(child)


        end=timer()
        self.runtime=end-start
        return False

    def computePlan(self, curNode):
        plan=[]
        while curNode is not None:
            plan.append(curNode.data['current'])
            curNode=curNode.parent
        self.plan=plan[::-1]

    def getPlan(self,ag):
        plan=[tup[ag] for tup in self.plan]
        end = len(plan)
        if AstarNode.GOAL_STR in plan:
            end= plan.index(AstarNode.GOAL_STR)
        return plan[:end]







