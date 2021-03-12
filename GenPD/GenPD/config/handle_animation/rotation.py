from math import sin, cos, pi

file_name = "rotation.txt"
dt = 0.001
radius = 3.5
n_move = 30
total_frame = 450
delta_frame = total_frame / n_move
delta_theta = pi / 4 / n_move
displacement = 2 * sin(delta_theta / 2) * radius / delta_frame

with open(file_name, 'w') as f:
    theta = 0
    frame = 0
    for i in range(n_move):
        theta += delta_theta
        frame += delta_frame
        f.write("0 {} {} {} {:d}\n".format(-cos(theta), -sin(theta), displacement, int(frame)))