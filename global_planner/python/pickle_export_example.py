import pickle as pkl
import numpy as np
from pickle_save_example import Cat

# Now, let's unpickle the object from the file
with open('baxter.pickle', 'rb') as file:
    loaded_data = pkl.load(file)

print("baxter is here", loaded_data)