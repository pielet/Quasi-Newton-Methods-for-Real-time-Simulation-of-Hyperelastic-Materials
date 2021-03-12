n_move = 30
total_frame = 120
theta = 90
n_node = 6
length = 2

with open('twiat.txt', 'w') as f:
    for i in range(n_move):
        f.write('0 0 1 {} {}')