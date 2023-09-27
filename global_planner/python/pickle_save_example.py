import pickle as pkl
import numpy as np

class Cat():

    g = 9.8
    t = 0
    jump_theta = np.deg2rad(45)
    def __init__(self, name:str, age:int, breed:str, weight_kg:float, 
                 x_pos:float, y_pos:float, speed:float, 
                 jump_vel:float, color:str):
        self.name = name
        self.age = age
        self.breed = breed
        self.weight_kg = weight_kg
        self.x_pos = x_pos
        self.y_pos= y_pos
        self.speed = speed
        self.jump_vel = jump_vel
        self.color = color
        self.x_position_history = []
        self.y_position_history = []
        self.time_history= []
        

    def run(self):
        print('How long does the cat run for: ')
        self.t = input()
        self.x_pos = self.x_pos + self.speed * int(self.t)
        print(self.name + " runs " + str(self.x_pos) + " meters")
    
    def jump(self) -> list:
        dt = 0.01
        while self.y_pos >= 0:
            self.x_pos = self.x_pos + np.cos(self.jump_theta)*(self.jump_vel*self.t)
            self.y_pos = self.y_pos + np.sin(self.jump_theta)*(self.jump_vel*self.t) - (.5*self.g * pow(self.t,2))
            self.x_position_history.append(self.x_pos)
            self.y_position_history.append(self.y_pos)
            print("x y position is ", self.x_pos, self.y_pos)
            self.time_history.append(self.t)
            self.t = self.t + .01
    
        return self.time_history, self.x_position_history, self.y_position_history
    
    def meow(self):
        print(self.name + ' says "Meow"')  


baxter = Cat('Baxter', 5, 'Mini Panther', 4, 0, 0, 2, 2, 'black')
samwise = Cat('Samwise', 3, 'Mini Tiger', 4, 0, 0, 2, 2, 'Orange')

# Pickle the object and save it to a file
with open('baxter.pickle', 'wb') as file:
    pkl.dump(baxter, file)
    pkl.dump(samwise, file)


# Now, let's unpickle the object from the file
# with open('data.pickle', 'rb') as file:
#     loaded_data = pickle.load(file)
