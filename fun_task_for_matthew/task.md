# So you like Cats huh?

For funsies 
- Your task is to create a class Cat in Python
- Since you have a bunch of cat's you're creating multiple instances of cats
  - You should be ableto define name,speed, and breed
  - It should have methods:
    - To meow and says it name
    - It should be able to run using the simple kinematic equations 
    - It should be able to jump 

Here's some pseudcode to help you out

```python

Class Cat():
    def __init__(self, name:str, speed:str, breed:str, start_x:float,   start_y:float, weight_kg:float):
        #define the attributes
        self.name = name # name of your cat 
        self.speed = #how fast can it go meters ()
        self.breed = #breed of cat
        self.current_position = [start_x, start_y] #some x y position

    def run(self, dt_val:float):
        #set the current position based on its max velocity
        pass

    def jump(self, dt_val:float) -> float:
        #return a list of the trajectory x,y


I want you to simulate a cat race amongst your three cats
I want you to simulate your cat jumping 
I want you to plot it with  its name
```


Watch the first three videos and you should be good to go:
https://www.youtube.com/watch?v=ZDa-Z5JzLYM&list=PL-osiE80TeTsqhIuOqKhwlXsIBIdSeYtc&ab_channel=CoreySchafer

