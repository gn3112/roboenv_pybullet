class Test(object):
    def __init__(self):
        self.a = []

    def train(self):
        self.a.append(1) 

    @train
    def add(self):
        pass



t = Test()
for _ in range(5):
    t.add()
for _ in range(5):
    t.add()