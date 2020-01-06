import networkx as nx

import astar
import astarNoHist

class MultiAgentGraph:
    def __init__(self,graph,num_agents):
        self.graph=graph
        self.num_agents=num_agents
        self.agents={}
        self.decomposition=[]
        self.planTime=None
        self.foundPlan=False

    @staticmethod
    def readGraphFile(filename):
        f=open(filename,"r")
        if f.mode != "r":
            print("cannot read graph file")
            exit(0)

        contents = f.read()

        h_idx = contents.find("height")
        w_idx = contents.find("width")
        m_idx = contents.find("map")
        height = int(contents[h_idx + len("height "):w_idx])
        width = int(contents[w_idx + len("width "):m_idx])
        map_str = contents[m_idx + len("map") + 1:]
        map_lines = map_str.split('\n')

        graph = nx.DiGraph(nx.grid_2d_graph(width, height))  # type: digraph

        for j in range(height):
            for i in range(width):
                if map_lines[j][i] == "@":
                    graph.remove_node((i, j))

        return graph


    def updateSource(self,agent,node):
        if not agent in self.agents: self.agents[agent]={}
        self.agents[agent]['source']=node

    def updateTarget(self,agent,node):
        if not agent in self.agents: self.agents[agent] = {}
        self.agents[agent]['target']=node

    def getSource(self,ag):
        if ag in self.agents and 'source' in self.agents[ag]: return self.agents[ag]['source']
        return None

    def getTarget(self,ag):
        if ag in self.agents and 'target' in self.agents[ag]: return self.agents[ag]['target']
        return None

    def getNodeString(self,node):
        outstr=""
        for ag in self.agents:
            if self.getSource(ag) == node: outstr += "S" + str(ag)
            if self.getTarget(ag) == node: outstr += "T" + str(ag)
        return outstr

    # def planPath(self,ag):
    #     src=self.getSource(ag)
    #     tgt=self.getTarget(ag)
    #     if src is not None and tgt is not None:
    #         try:
    #             path = nx.shortest_path(self.graph, src, tgt)
    #             self.agents[ag]['plan'] = path
    #             return path
    #         except nx.NetworkXNoPath:
    #             print("No Path for agent "+str(ag))
    #             return None

    def getPlan(self,ag):
        if ag in self.agents and 'plan' in self.agents[ag]:
            return self.agents[ag]['plan']
        return None

    def planAstar(self, num_seg, timeout=300):
        solver = astar.AstarSolver(self.graph, [self.getSource(ag) for ag in range(self.num_agents)],
                                 [self.getTarget(ag) for ag in range(self.num_agents)])
        solver.computeHeuristic()
        if solver.astar(num_seg,timeout):
            self.foundPlan = True
            for ag in self.agents:
                plan = solver.getPlan(ag)
                #print(plan)
                self.agents[ag]['plan'] = plan
        else:
            print("No plan")
            self.planTime = solver.runtime
            return False
        self.planTime=solver.runtime
        return True

    def planAstarNoHist(self):
        solver = astarNoHist.AstarSolverNoHist(self.graph, [self.getSource(ag) for ag in range(self.num_agents)],
                                 [self.getTarget(ag) for ag in range(self.num_agents)])
        solver.computeHeuristic()
        if solver.astar(240):
            self.foundPlan = True
            for ag in self.agents:
                plan = solver.getPlan(ag)
                #print(plan)
                self.agents[ag]['plan'] = plan
        else:
            print("No plan")
            self.planTime = solver.runtime
            return False
        self.planTime=solver.runtime
        return True

    @staticmethod
    def isDisjointBySize(sets):
        expectedSize = 0
        for s in sets: expectedSize += len(s)
        unionSet = set.union(*sets)
        if len(unionSet) == expectedSize: return True
        return False

    def computeMinimalDisjointDecomposition(self):
        if self.checkCollision():
            return None
        decomposition = [0]
        T = 0  # max length of a path
        paths=[]
        for ag in self.agents:
            plan=self.getPlan(ag)
            if plan is not None:
                paths.append(plan)
                l = len(plan)
                if l > T: T = l
        i = 0
        lasti = 0
        while i <= T:
            partialPaths = [set(path[lasti:i]) for path in paths]
            if not MultiAgentGraph.isDisjointBySize(partialPaths):
                decomposition.append(i-1)
                lasti = i-1
            else:
                i += 1
        decomposition.append(T+1)
        self.decomposition=decomposition
        return decomposition

    def checkCollision(self):
        paths=[]
        T=0
        for ag in self.agents:
            plan = self.getPlan(ag)
            if plan is not None:
                paths.append(plan)
                l = len(plan)
                if l > T: T = l
        for i in range(T):
            s=[set(path[i:i+1]) for path in paths]
            if not MultiAgentGraph.isDisjointBySize(s):
                return True
        return False


    def getDecompositionNodes(self,decindex):
        if decindex>=len(self.decomposition):
            print("decomposition index out of bounds")
            return
        retnodes={}
        for ag in self.agents:
            plan = self.getPlan(ag)
            if plan is not None:
                retnodes[ag]=plan[self.decomposition[decindex]:self.decomposition[decindex + 1]]
        return retnodes

    def __str__(self):
        out="Num Agents: "+str(self.num_agents)+"\r\n"
        out+="Graph: "+ str(len(self.graph.edges))+" Edges, and "+ str(len(self.graph.nodes))+" Nodes.\r\n"
        for ag in self.agents:
            out += str(ag)+": "+str(self.agents[ag])+"\r\n"
        return out

    def resultOutput(self):
        sout=""
        if not self.foundPlan:
            return "NO PLAN\r\n TIMEOUT:\t "+str(self.planTime)
        for ag in self.agents:
            sout+= "AGENT "+str(ag)+ " PLAN:\t"+str(self.getPlan(ag))+"\r\n"
        sout+="DECOMPOSITION:\t"+str(self.decomposition)+"\r\n"
        sout+="DECOMP PARTS:\t" + str(len(self.decomposition)-1) + "\r\n"
        sout+="RUNTIME:\t"+str(self.planTime)+"\r\n"
        return sout
