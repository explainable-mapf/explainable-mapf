import MultiAgentGraph
import argparse


class BenchTester:
    def __init__(self):
        self.mag = None  # Multi Agent Graph object, to be initialized later
        self.graph = None
        self.resetParams(1,2)
        self.graph_file=None
        self.bench_file=None


    def resetParams(self, decomp_parts,num_agents):
        self.num_agents = num_agents
        self.decomp_parts = decomp_parts  # minimum 1

    def minimalDecomposition(self):
        if not self.mag.foundPlan:
            return -1
        decomp = self.mag.computeMinimalDisjointDecomposition()
        print("Decomposition: " + str(decomp))
        if decomp is None:
            self.log("Collision Detected, no decomposition.")
            return -1
        return len(decomp)

    def showDecompPart(self, val):
        if not self.mag.foundPlan:
            return
        part = self.mag.getDecompositionNodes(int(val) - 1)
        print(part)

    def planAll(self,timeout):
        #toggle between history-dependent A* and standard A*
        return self.mag.planAstar(self.decomp_parts-1,timeout)
        #return self.mag.planAstarNoHist()

    def setSource(self,agent,node):
        if self.mag is not None:
            self.mag.updateSource(agent, node)
            self.log(
                "Set source for agent " + str(agent) + " as " + str(node))

    def setTarget(self,agent,node):
        if self.mag is not None:
            self.mag.updateTarget(agent, node)
            self.log(
                "Set target for agent " + str(agent) + " as " + str(node))
            self.log(str(self.mag))

    def setupGraph(self):
        self.mag = MultiAgentGraph.MultiAgentGraph(self.graph, self.num_agents)

    def readBenchmarkData(self):
        f = open(self.bench_file, "r")

        if f.mode != "r":
            self.log("cannot read scenario file")
            exit(0)

        lines = f.read().splitlines()
        for i in range(self.num_agents):
            dat = lines[i+1].split()
            node_source=(int(dat[4]),int(dat[5]))
            self.setSource(i,node_source)
            node_target=(int(dat[6]),int(dat[7]))
            self.setTarget(i, node_target)


    def readGraph(self,graphFile,benchFile):
        self.graph_file=graphFile
        self.bench_file=benchFile

        self.graph = MultiAgentGraph.MultiAgentGraph.readGraphFile(self.graph_file)
        self.setupGraph()
        self.readBenchmarkData()

    def writeResult(self):
        outstr=""
        outstr+= "GRAPH FILE:\t"+self.graph_file+"\r\n"
        outstr+= "BENCH FILE:\t" + self.bench_file + "\r\n"
        outstr += "NUM_AGENTS:\t" + str(self.num_agents) + "\r\n"
        if self.decomp_parts is not None:
            outstr += "MAX_DECOMP_PARTS:\t" + str(self.decomp_parts) + "\r\n"
        outstr += self.mag.resultOutput()
        return outstr


    def log(self, text):
        print("LOG: "+text)




if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("mapfile", help=".map file representing the graph")
    parser.add_argument("scenfile", help=".scen file representing the scenario")
    parser.add_argument("numagent", help="number of agents (minimum 2)", type=int)
    parser.add_argument("numseg", help="number of segments (minimum 1)", type=int)
    parser.add_argument("timeout", help="time limit (in seconds)", type=int)

    args = parser.parse_args()

    #print(args)
    
    gv = BenchTester()

    gv.resetParams(args.numseg, args.numagent)
    gv.readGraph(args.mapfile, args.scenfile)

    res = gv.planAll(args.timeout)
    last_dec = gv.minimalDecomposition()
    print(gv.writeResult())




