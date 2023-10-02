import torch

x = torch.Tensor([1/2, -1, 1/2])
m = torch.stack([3*torch.eye(3), -2*torch.eye(3)], dim=0)
print(m)
value = torch.nn.functional.bilinear(x, x, m)


# # Define the weight matrix A
# A = torch.tensor([[3.0, 0.0, 0.0],
#                   [0.0, 3.0, 0.0],
#                   [0.0, 0.0, 3.0],
#                   [-2.0, 0.0, 0.0],
#                   [0.0, -2.0, 0.0],
#                   [0.0, 0.0, -2.0]])

# # Define the input vector x
# x = torch.tensor([1/2, -1, 1/2])

# m = torch.stack([3*torch.eye(3), -2*torch.eye(3)], dim=0)

# # Apply the bilinear transformation using torch.nn.functional.bilinear
# output = torch.nn.functional.bilinear(x, x, m)

# print(output.item())  # Convert to a Python float and print the result