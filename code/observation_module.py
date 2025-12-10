

class ObsMod:
    def __init__ (self, name, size):
        self.name = name
        self.size = size 

    def __len__(self):
        return self.size 

    def __str__(self):
        return f"ObsMod:{self.name}"

    def cf_init(self, scf):
        pass 

        

    
    # ### in crazyflie
    # init in crazyflie 
    # extract from cf.log 
    
    # ### in training 
    # _observationspace size contribution
    # _compute observation 

    # ### common
    # shape 
    # __len__()
    # __str__
