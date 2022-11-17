import numpy as np

actions = [0, 1, 2, 3, 4] # [-10,-5,0,5,10]
Nodes = 3
edges = Nodes- 1
odd_n = [1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31,33,35,37]
def find_odd(n):
    return (n*2 + 1)

total_actions = ((len(actions)-1) * edges) + 1

print("Nodes: ", Nodes, "Total Actions: ", total_actions)
n_actions = [0] * edges
lst = []
#print(n_actions)
if Nodes == 2:
    print("actions: ", actions)
    #lst.append(actions)
for i in range(edges-1, -1, -1):
    #lst.clear()
    #print("0's in step ", i , ": ", find_odd(i))
    lst.append([0] * find_odd(i))
    #print("1's in step ", i , ": ", 1)
    lst.append([1])
    #print("2's in step ", i , ": ", total_actions - find_odd(i)*2 -2)
    lst.append([2] * (total_actions - find_odd(i)*2 -2))
    #print("3's in step ", i , ": ", 1)
    lst.append([3])
    #print("4's in step ", i , ": ", find_odd(i))
    lst.append([4] * find_odd(i))
    #print(lst)

final_actions = np.zeros((edges,total_actions), dtype=int)
x,y = 0,0    
for i in range(5*edges):
    if i%5 ==0 and i!=0:
        #print("New column")
        x+=1
        y= 0
    for j in lst[i]:
        #print(j)
        final_actions[x][y] = j
        y+=1
print(final_actions)
final_actions_1 = np.roll(final_actions, 1)
print(final_actions_1)
flip_e = int((total_actions+1)/2)
final_actions_1[:,[2, int((total_actions+1)/2)]] = final_actions_1[:,[int((total_actions+1)/2), 2]]
print(final_actions_1)

# print("reset environment")
# for i in range(final_actions.shape[1]):
#     print("set previous environment")
#     for j in range(final_actions.shape[0]):
#         print(final_actions[j][i])
arr = []
for i in final_actions_1:
    #i[edges:6] = 0
    print(i[0:3])
    arr.append(i[0:3].tolist())
final_actions_1 = arr
print(final_actions_1)
print(np.array(final_actions_1))

def expand_tree(Nodes, actions):
    edges = Nodes- 1
    total_actions = ((actions-1) * edges) + 1
    lst = []
    for i in range(edges-1, -1, -1):
        #lst.clear()
        #print("0's in step ", i , ": ", find_odd(i))
        lst.append([0] * find_odd(i))
        #print("1's in step ", i , ": ", 1)
        lst.append([1])
        #print("2's in step ", i , ": ", total_actions - find_odd(i)*2 -2)
        lst.append([2] * (total_actions - find_odd(i)*2 -2))
        #print("3's in step ", i , ": ", 1)
        lst.append([3])
        #print("4's in step ", i , ": ", find_odd(i))
        lst.append([4] * find_odd(i))
        #print(lst)

    final_actions = np.zeros((edges,total_actions), dtype=int)
    x,y = 0,0    
    for i in range(5*edges):
        if i%5 ==0 and i!=0:
            #print("New column")
            x+=1
            y= 0
        for j in lst[i]:
            #print(j)
            final_actions[x][y] = j
            y+=1

    return final_actions