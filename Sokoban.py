#imports
import copy
import time
from nanoid import generate

#user inputs
file = input("Enter the path to your input file(.txt format) here")
with open(file.strip()) as f:
    contents = f.read()
Lines = contents.strip().split("\n")
print("1. Breadth First Serach")
print("2. Depth First Serach")
print("3. Greedy Breadth First Search using Manhattan Distance")
print("4. A* using Manhattan Distance")
print("5. Greedy Breadth First Search using non-trivial heuristic")
print("6. A* using non-trivial heuristic")
algo = input("Please enter a number from the above given options")
algo = int(algo)
if algo > 6 or algo < 1:
    print("Please select a valid option")

#global declaration of variables
state = Lines
initial_state = []
stack = []
trace = []
stages = []
StorageX = []
StorageY = []
count = 0
level = 0

start_time = time.time()

#Representing the given input as a 2 dimensional list
for positions in state:
  initial_state.append(list(positions))
  
#Storing the positions of the storage(s)
for states in initial_state:  
  if 'S' in states:
    StorageX.append(initial_state.index(states))
    StorageY.append(states.index('S'))

#Heuristic computation using manhattan distance
def cal_heuristic(current_state):
  sum = 0
  for position in current_state:
    dist = []
    if 'B' in position:
      for i in range(0, len(StorageX)):
        dist.append(abs(current_state.index(position) - StorageX[i]) + abs(position.index('B')-StorageY[i]))
      sum = sum + min(dist) 
  return sum

#Computing the heuristic for the initial state only for GBS and A*
if algo == 1 or algo == 2:
    initial_heuristic = 0
else:
    initial_heuristic = cal_heuristic(initial_state)

#Creating a fringe for search algorithms and maintaing a copy of all the states traversed
stack.append({
    'parent': 0,
    'move': 'none',
    'child': 1,
    'state': initial_state,
    'pd': 0,
    'heuristic': initial_heuristic
})
trace = copy.deepcopy(stack)

# Function to append children of the current node into the fringe and to the states traversed list
def push_state(dist,temp_state, cnt, move, parent_state):
  #storing the states traversed into a seperate list
  trace_states = []
  for val in trace:
    trace_states.append(val['state'])
  
  #if the current state is not already traversed
  if temp_state not in trace_states:
      #Check for goal state (position of all blocks in current state is equal to the positions of the storages)
      for ele in range(0, len(StorageX)):
        if temp_state[StorageX[ele]][StorageY[ele]] == 'B':
          cnt = cnt + 1

      #Calculate heuristic for each child and store it as a property of the child for GBS and A* using manhattan distance
      #Other algorithms assign heuristic property to 0

      #For non trivial heuristic, the position of the obstacle are consider to idenfy the deadlock states.
      #For the given problem the deadlock state is considered to be the state where the block is surrounded by atleast 2 obstacles.
      #In such cases it becomes impossible to move the block to any other position by the robot.
      #Thus, the heuristic for such blocks is considered to be the maximum to ensure that the algorithm never searches that state.
      if algo == 1 or algo == 2:
        heuristic = 0
      elif algo == 5 or algo == 6:
        x=[]
        y=[]
        for position in temp_state:
            if 'B' in position:
                x.append(temp_state.index(position))
                y.append(position.index('B'))
        
        for i in range(0, len(x)):
            dead_cnt= 0
            if x[i] == StorageX[i] and y[i] == StorageY[i]:
                break
            else:
                if temp_state[x[i]+1][y[i]] == 'O':
                    dead_cnt = dead_cnt + 1
                if temp_state[x[i]-1][y[i]] == 'O':
                    dead_cnt = dead_cnt + 1
                if temp_state[x[i]][y[i]-1] == 'O':
                    dead_cnt = dead_cnt + 1
                if temp_state[x[i]][y[i]+1] == 'O':
                    dead_cnt = dead_cnt + 1
                if dead_cnt == 2:
                    break

        if dead_cnt == 2:    
            heuristic = 2500
        else:
            heuristic = cal_heuristic(temp_state)
      else:
        heuristic = cal_heuristic(temp_state)
      
      #Generate a unique identifier for each child and store properties of the current state to the fringe and the states traversed list
      child = generate()
      stack.append({
        'parent' : parent_state,
        'move': move,
        'child': child,
        'state': temp_state,
        'pd': dist,
        'heuristic': dist + heuristic
      })
      trace.append({
        'parent' : parent_state,
        'move': move,
        'child': child,
        'state': temp_state,
        'pd': dist,
        'heuristic': dist + heuristic
      })

      #If goal state, end game
      if cnt == len(StorageX):
        return "won"

#Function to calculate distance from the root node to the current board state
def cal_parentDist(parent_state, board_state):
  x1 = []
  x2 = []
  y1 = []
  y2 = []
  
  #get the parent of the current board state
  filter_tree = filter(lambda x: x['child'] == parent_state , trace)
  parent_node = copy.deepcopy(list(filter_tree)[0])

  #get positions of block for the current board state
  for board in board_state:
    if 'B' in board:
      x1.append(board_state.index(board))
      y1.append(board.index('B'))
  
  #get positions of block for parent node of the current block state
  for state in parent_node['state']:
    if 'B' in state:
      x2.append(parent_node['state'].index(state))
      y2.append(state.index('B'))

  #If change in block position add 1 to the distance of parent node from root node. 
  #Else, pass the distance of the parent node from root node
  if x1 == x2:
    dist = parent_node['pd']
  else:
    dist = parent_node['pd'] + 1
  return dist

#Function to generate children from the current board state 
def gen_states(parent_state, board_state, X, Y):
  #Calculate the distance from root node to the board state for A*
  if algo == 4 or algo == 6:
    prev_dist = cal_parentDist(parent_state,board_state)
  else:
    prev_dist = 0

  #Check all possible moves from the current board state to generate children
  #If a block is moved, add a one to the previous distance for A*
  if ' ' == board_state[X+1][Y] or 'S' == board_state[X+1][Y]:
    count = 0
    temp_state = []
    temp_state = copy.deepcopy(board_state)
    temp_state[X][Y] = ' '
    temp_state[X+1][Y] = 'R'
    value = push_state(prev_dist,temp_state, count, 'D', parent_state)
    if value == "won":
      return "won"
    
    
  if ' ' == board_state[X-1][Y] or 'S' == board_state[X-1][Y]:
    count = 0
    temp_state = []
    temp_state = copy.deepcopy(board_state)
    temp_state[X][Y] = ' '
    temp_state[X-1][Y] = 'R'
    value = push_state(prev_dist,temp_state, count, 'U', parent_state)
    if value == "won":
      return "won"

  if ' ' == board_state[X][Y+1] or 'S' == board_state[X][Y+1]:
    count = 0
    temp_state = []
    temp_state = copy.deepcopy(board_state)
    temp_state[X][Y] = ' '
    temp_state[X][Y+1] = 'R'
    value = push_state(prev_dist,temp_state, count, 'R', parent_state)
    if value == "won":
      return "won"
  
  if ' ' == board_state[X][Y-1] or 'S' == board_state[X][Y-1]:
    count = 0
    temp_state = []
    temp_state = copy.deepcopy(board_state)
    temp_state[X][Y] = ' '
    temp_state[X][Y-1] = 'R'
    value = push_state(prev_dist,temp_state, count, 'L', parent_state)
    if value == "won":
      return "won"

  if 'B' == board_state[X+1][Y]:
    count = 0
    if board_state[X+2][Y] != 'B' and board_state[X+2][Y] != 'O':
      temp_state = []
      temp_state = copy.deepcopy(board_state)
      temp_state[X][Y] = ' '
      temp_state[X+1][Y] = 'R'
      temp_state[X+2][Y] = 'B'
      if algo == 4 or algo == 6:
          prev_dist = prev_dist + 1
      value = push_state(prev_dist,temp_state, count, 'D', parent_state)
      if value == "won":
        return "won"

  if 'B' == board_state[X-1][Y]:
    count = 0
    if board_state[X-2][Y] != 'B' and board_state[X-2][Y] != 'O':
      temp_state = []
      temp_state = copy.deepcopy(board_state)
      temp_state[X][Y] = ' '
      temp_state[X-1][Y] = 'R'
      temp_state[X-2][Y] = 'B'
      if algo == 4 or algo == 6:
          prev_dist = prev_dist + 1
      value = push_state(prev_dist,temp_state, count, 'D', parent_state)
      if value == "won":
        return "won"

  if 'B' == board_state[X][Y+1]:
    count = 0
    if board_state[X][Y+2] != 'B' and board_state[X][Y+2] != 'O':
      temp_state = []
      temp_state = copy.deepcopy(board_state)
      temp_state[X][Y] = ' '
      temp_state[X][Y+1] = 'R'
      temp_state[X][Y+2] = 'B'
      if algo == 4 or algo == 6:
          prev_dist = prev_dist + 1
      value = push_state(prev_dist,temp_state, count, 'D', parent_state)
      if value == "won":
        return "won"

  if 'B' == board_state[X][Y-1]:
    count = 0
    if board_state[X][Y-2] != 'B' and board_state[X][Y-2] != 'O':
      temp_state = []
      temp_state = copy.deepcopy(board_state)
      temp_state[X][Y] = ' '
      temp_state[X][Y-1] = 'R'
      temp_state[X][Y-2] = 'B' 
      if algo == 4 or algo == 6:
          prev_dist = prev_dist + 1
      value = push_state(prev_dist,temp_state, count, 'D', parent_state)
      if value == "won":
        return "won"

#Function to get the position of the robot in the current board state                                         
def get_ordinates(parent_state, board_state):
  for states in board_state:  
    if 'R' in states:
      X = board_state.index(states) 
      Y = states.index('R')
  
  output = gen_states(parent_state,board_state,X,Y)
  if output == "won":
    return "won"

#Loop through every state until the goal state is reached
while count < len(StorageX):
  level = level + 1
  #For DFS assign the current state to the last element of the fringe in every iteration
  if algo == 2:
    current_state = copy.deepcopy(stack[len(stack)-1]['state'])
    current_parent = stack[len(stack)-1]['child']
    stack.remove(stack[len(stack)-1])
  else:
    #For algorithm other than BFS and DFS sort the fringe for every iteration based on the heuristic values and
    #Assign the current state to the first element of the fringe for every iteration
    if algo != 1:
        stack = sorted(stack, key=lambda i : i['heuristic'])
    current_state = copy.deepcopy(stack[0]['state'])
    current_parent = stack[0]['child']
    stack.remove(stack[0])
  result = get_ordinates(current_parent,current_state)
  if result == "won":
    break

#Function to print the steps taken by the search algorithm to play the Sokoban game
def gen_stages(parent):
  #back traverse the tree from goal state to root node
  #append the move for each iteration
  #Once the root node is reached, reverse the list of moves and print all the moves
  #Print the tottal number of steps taken to complete the game
  if parent != 1:
    filtered_output = filter(lambda x: x['child'] == parent , trace)
    current_move = copy.deepcopy(list(filtered_output)[0])
    stages.append(current_move['move'])
    gen_stages(current_move['parent'])
  else:
    stages.reverse()
    print("********Final Ouput********")
    print(stages)
    print("Number of steps taken to complete the game is: " + str(len(stages)))
  
stages.append(trace[len(trace)-1]['move'])
gen_stages(trace[len(trace)-1]['parent'])
print("Number of iterations taken by the search algorithm is: " + str(level))
print("Time taken ----- "+str(time.time()-start_time))