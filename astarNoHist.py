import networkx as nx
import itertools
import heapq
from timeit import default_timer as timer

class AstarNodeNoHist:
    #type of data:
    #data: dictionary with three elements: 'current', 'history', and 'segments'
    #data: array of current nodes, indexed on agent.

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
        if self.data[ag]== self.target[ag] or self.data[ag]== AstarNodeNoHist.GOAL_STR:
            return [AstarNodeNoHist.GOAL_STR] #reached goal
        retVal = list(self.graph.neighbors(self.data[ag]))
        retVal.append(self.data[ag])
        return retVal

    def getChildren(self):
        num_agents = len(self.data)
        ags = list(range(num_agents))
        children=[]
        neighbors=[self.getNeighborsNonGoal(ag) for ag in ags]

        for tup in list(itertools.product(*neighbors)):
            newchild_current = list(tup)

            if not self.isLegalChild(newchild_current):
                continue

            children.append(AstarNodeNoHist(self,newchild_current,self.graph,self.target))
        return children

    def isLegalChild(self, newCurrent):
        num_agents = len(self.data)
        for ag1 in range(num_agents):
            if newCurrent[ag1] != AstarNodeNoHist.GOAL_STR:
                for ag2 in range(ag1+1,num_agents):
                    if newCurrent[ag2] != AstarNodeNoHist.GOAL_STR:
                        if newCurrent[ag1] == newCurrent[ag2]: # Collision occurred
                            return False
                        if newCurrent[ag1] == self.data[ag2] and newCurrent[ag2] == self.data[ag1]: # Swap occurred
                            return False
        return True

    def __eq__(self, other):
        #if len(self.data) !=len(other.data):
        #    return False
        for i in range(len(self.data)):
            if self.data[i]!=other.data[i]:
                return False
        return True

    def computeHash(self):
        self.hashVal=hash(str(self.data))

    def __hash__(self):
        return self.hashVal

    def __lt__(self, other):
        return str(self.data) <= str(other.data)

    def __str__(self):
        return str(self.data)


class AstarSolverNoHist:
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
            if node.data[ag]!= AstarNodeNoHist.GOAL_STR:
                val += self.allShortestPaths[node.data[ag]][self.targetNodes[ag]]
        return val

    def isGoal(self,node):
        for ag in range(self.numAgents):
            if node.data[ag] != AstarNodeNoHist.GOAL_STR and node.data[ag] != self.targetNodes[ag]:
                return False
        return True

    def astar(self, numSegments,timeout=300):
        stdata={}
        stdata = self.initNodes
        startNode=AstarNodeNoHist(None,stdata,self.graph,self.targetNodes)
        print(startNode)

        # t_num_elem=0
        # t_time=0
        # t_num_checks=0

        gscore={startNode:0}
        fscore={startNode:self.heuristicVal(startNode)}
        openSet=[]
        closedSet=set()
        openSetHash=set()
        heapq.heappush(openSet,(fscore[startNode],startNode))
        openSetHash.add(startNode)
        #print(openSet)

        diagCounter=0
        start = timer()

        while openSet:
            diagCounter+=1
            curNode = heapq.heappop(openSet)[1]
            openSetHash.remove(curNode)

            if diagCounter == 1000:
                end=timer()
                if int(end)-int(start)>timeout:
                    self.runtime=end-start
                    return False
                #sOpenset=set([tup[1] for tup in openSet])
                #diff1=sOpenset.difference(openSetHash)
                #print("diff1 size: "+str(len(diff1)))
                #for el in diff1:
                #    print(el)
                #return False
                diagCounter=0
                print("Open Set Size: "+ str(len(openSet)))
                print("Closed Set Size: " + str(len(closedSet)))
                print("Current Node: "+ str(curNode))
                # print("ListCheck total time: "+str(t_time))
                # print("ListCheck Total Elem: " + str(t_num_elem)+" num checks: "+str(t_num_checks))

            if self.isGoal(curNode):
                print("Reached the goal!"+str(curNode))
                self.computePlan(curNode)
                end=timer()
                self.runtime=end-start
                return True

            closedSet.add(curNode)
            #print("Open Set:"+str(openSet))
            #print("Closed Set:"+str(closedSet))

            children = curNode.getChildren()

            for child in children:
                if child in closedSet:
                    continue

                tentative_g=gscore[curNode]+1
                #start=timer()
                #bcheck=(child not in [tup[1] for tup in openSet])
                bcheck=(child not in openSetHash)
                #end = timer()
                #t_num_checks+=1
                #t_num_elem+=len(openSet)
                #t_time+=(end-start)

                if bcheck:
                    child.parent = curNode
                    gscore[child] = tentative_g
                    fscore[child] = tentative_g + self.heuristicVal(child)
                    heapq.heappush(openSet, (fscore[child], child))
                    openSetHash.add(child)

                # elif tentative_g< gscore.get(child,0):
                #     childInd=openSet.index((fscore[child],child))
                #
                #     child.parent = curNode
                #     gscore[child]=tentative_g
                #     fscore[child]=tentative_g+self.heuristicVal(child)




                    #heapq.heappush(openSet,(fscore[child],child))
                    #openSetHash.add(child)

        end=timer()
        self.runtime=end-start
        return False

    def computePlan(self, curNode):
        plan=[]
        while curNode is not None:
            plan.append(curNode.data)
            curNode=curNode.parent
        self.plan=plan[::-1]

    def getPlan(self,ag):
        plan=[tup[ag] for tup in self.plan]
        end = len(plan)
        if AstarNodeNoHist.GOAL_STR in plan:
            end= plan.index(AstarNodeNoHist.GOAL_STR)
        return plan[:end]







