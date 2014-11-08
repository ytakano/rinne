import random


class node:
    def __init__(self, id):
        self.link = []
        self.id = id

class cnn:
    def _initialize(self):
        random.seed()

        self.nodes = {}
        self.nodes[0] = node(0)
        self.nodes[1] = node(1)
        self.nodes[2] = node(2)

        self.nodes[0].link = [self.nodes[2]]
        self.nodes[1].link = [self.nodes[2]]

        self.nodes[2].link = [self.nodes[0]]
        self.nodes[2].link = [self.nodes[1]]

        self.neighbor = [(0, 1)]

        self.n = 2


    def generate(self, num, p):
        self._initialize()

        i = 0
        while True:
            rnd = random.random()
            if rnd < p:
                self._addLink()
            else:
                self._addNode()
                i += 1
                if i >= num:
                    return


    def _addNode(self):
        self.n += 1

        self.nodes[self.n] = node(self.n)

        rnd = int(random.uniform(0, self.n - 1))

        self.neighbor += [(link.id, self.n) for link in self.nodes[rnd].link]

        self.nodes[self.n].link += [self.nodes[rnd]]
        self.nodes[rnd].link    += [self.nodes[self.n]]


    def _addLink(self):
        if len(self.neighbor) == 0:
            return

        rnd = int(random.uniform(0, len(self.neighbor)))
        id1, id2 = self.neighbor[rnd]
        del self.neighbor[rnd]
        
        self.nodes[id1].link += [self.nodes[id2]]
        self.nodes[id2].link += [self.nodes[id1]]

    def printLink(self):
        print "digraph cnn {"

        for n in range(self.n):
            if len(self.nodes[n].link) == 0:
                continue

            for link in self.nodes[n].link:
                if n < link.id:
                    print '%d -> %d;' % (n, link.id)

        print "}"

if __name__ == "__main__":
    graph = cnn()
    graph.generate(500, 0.666)

    graph.printLink()
