import numpy as np

# say you have three radars with these probabilities
# prob_array = [0.4, 0.5, 0.3]
prob_array = [0.4]
mult_prob = np.prod(prob_array)
# print(mult_prob)
mult_prob = 1 - mult_prob
# print(mult_prob)



radar_probs = []
radar_cost = 0  # cost of all radars
for prob in prob_array:
    #radar_probs.append(1 - prob)
    radar_probs.append(prob)

if len(radar_probs) > 1:
    mult_prob = 1 - np.prod(1 - np.array(radar_probs))
else:
    mult_prob = radar_probs[0]

print(mult_prob)