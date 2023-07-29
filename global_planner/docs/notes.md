# Notes
- I need to provide a shift based on where I start
- If I start at 0,0 I won't be able to move left or right 
- I need to specify map bounds 
  - So say -100m to 100m for x,y,z 
  - That means I need to allocate 200 grid indices for x,y,z
  - I'm not building an occupancy grid so won't need to worry about that