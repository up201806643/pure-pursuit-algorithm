
class tool(object):

    def __init__(self):

        self.sum = 0
        self.size = 3
        self.data = [0, 0, 0]
        self.dataI = 0
        

    def push(self,val):
        
        self.sum -= self.data[self.dataI] 
        self.sum += val

        self.data[self.dataI] = val
        self.dataI = (self.dataI +1 ) % self.size
        
    def get(self):
        return self.sum / self.size


#class 