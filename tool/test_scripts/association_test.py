import numpy as np
from scipy.optimize import linear_sum_assignment

def calculate_cost_matrix(objects, targets):
    num_objects = len(objects)
    num_targets = len(targets)
    
    # Initialize cost matrix with high values for dummy assignments
    cost_matrix = np.full((num_objects, num_targets), np.inf)
    
    # Calculate the cost matrix (Euclidean distance)
    for i in range(num_objects):
        for j in range(num_targets):
            cost_matrix[i, j] = np.linalg.norm(np.array(objects[i]) - np.array(targets[j]))
    
    return cost_matrix

def optimal_assignment(objects, targets):
    cost_matrix = calculate_cost_matrix(objects, targets)
    print(cost_matrix)
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    
    optimal_pairs = [(row_ind[i], col_ind[i]) for i in range(len(row_ind)) if row_ind[i] < len(objects) and col_ind[i] < len(targets)]
    print(row_ind)
    print(type(col_ind))
    return optimal_pairs

# Example usage
objects = [(1, 2), (3, 4), (5, 6)]
targets = [(2, 3), (4, 5)]

optimal_pairs = optimal_assignment(objects, targets)
print(optimal_pairs[0][0])
for a, b in optimal_pairs:
    print(a)
    print(b)

lll = [1,2,10,3,4,5,6]
lll.pop(2)
print(lll)

print("Optimal assignment (object index, target index):", optimal_pairs)
