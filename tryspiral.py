MAX = 100

def hello():
    print('hi')

def printSpiral(mat, a, b):
    r = len(mat)
    c = len(mat[0])

    low_row = 0 if (0 > a) else a
    low_column = 0 if (0 > b) else b - 1
    high_row = r-1 if ((a + 1) >= r) else a + 1
    high_column = c-1 if ((b + 1) >= c) else b + 1
  
    while ((low_row > 0 - r and low_column > 0 - c)):
  
        i = low_column + 1
        while (i <= high_column and 
                i < c and low_row >= 0):
            print( mat[low_row][i])
            # hello()
            i += 1
        low_row -= 1
  
        i = low_row + 2
        while (i <= high_row and
                i < r and high_column < c):
            # hello()
            print(mat[i][high_column])
            i += 1
        high_column += 1
  
        i = high_column - 2
        while (i >= low_column and 
                i >= 0 and high_row < r):
            # hello()
            print(mat[high_row][i])
            i -= 1
        high_row += 1
  
        i = high_row - 2
        while (i > low_row and 
                i >= 0 and low_column >= 0):
            # hello()
            print(mat[i][low_column])
            i -= 1
        low_column -= 1
      
    print()
  
# Driver code
if __name__ == "__main__":
      
    mat = [[ 1, 2, 3 ], 
            [ 4, 5, 6 ], 
            [ 7, 8, 9 ]]
    r = 3
    c = 3
    printSpiral(mat, 1,1)
    
# import numpy as np 
# mat = np.array([[ 1, 2, 3 ], 
#             [ 4, 5, 6 ], 
#             [ 7, 8, 9 ]])

# def spiral(arr, x,y):
#     X = len(arr)
#     Y= len(arr[0])
#     # x = y = 0
#     dx = 0
#     dy = -1
#     for i in range(max(X, Y)**2):
#         # if (-X/2 < x <= X/2) and (-Y/2 < y <= Y/2):
#         if 0 <= x < X and 0 <= y < Y:
#             print (x, y)
#             # DO STUFF...
#         if x == y or (x < 0 and x == -y) or (x > 0 and x == 1-y):
#             dx, dy = -dy, dx
#         x, y = x+dx, y+dy
        
        
# spiral(mat,1,1)