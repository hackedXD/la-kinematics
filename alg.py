from sympy import nonlinsolve, simplify, symbols, sin, cos, Matrix, pi


# DH Parameters (from previous step):
# Joint 1: alpha0=0,   a0=0,   d1=0.0545,  theta1=t1
# Joint 2: alpha1=90°, a1=0,   d2=0.123,   theta2=t2
# Joint 3: alpha2=0,   a2=0,   d3=0.317,   theta3=t3
# Joint 4: alpha3=0,   a3=0,   d4=0.202,   theta4=t4
# Joint 5 (EE): alpha4=0, a4=0, d5=0.1605, theta5=0 (fixed)

# Define symbolic variables for joint angles
t1, t2, t3, t4 = symbols('t_1 t_2 t_3 t_4', real=True)
l1, l2, l3, l4 = symbols('l1 l2 l3 l4', real=True)
dh_parameters = [
    # [0, t1, 0, l1],
    # [0, t2, 0, l2]

    [0.0545, 0, 0, 0],
    [0.123, t1, 90, 0],
    [0, t2 + pi / 2, 0, 0.317],
    [0, t3, 0, 0.202],
    [0, t4, 0, 0.1605]
]

for i in range(len(dh_parameters)):
    dh_parameters[i][2] *= pi / 180

T = Matrix.eye(4)
for [d, theta, alpha, r] in dh_parameters:
    # Create a symbolic transformation matrix
    # based on the DH parameters
    T_new = Matrix([
        [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), r * cos(theta)],
        [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), r * sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

    # print(T_new)
    
    T = T * T_new

    # print new basis vectors
    # x, y, z, t = T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3]
    # print("-----")
    # print(f"vector(({t[0]}, {t[1]}, {t[2]}), ({t[0] + x[0]}, {t[1] + x[1]}, {t[2] + x[2]}))")
    # print(f"vector(({t[0]}, {t[1]}, {t[2]}), ({t[0] + y[0]}, {t[1] + y[1]}, {t[2] + y[2]}))")
    # print(f"vector(({t[0]}, {t[1]}, {t[2]}), ({t[0] + z[0]}, {t[1] + z[1]}, {t[2] + z[2]}))")
    # print("-----")
    

    joint_pos = simplify(T * Matrix([0, 0, 0, 1]))
    joint_string = f"({joint_pos[0]}, {joint_pos[1]}, {joint_pos[2]})"
    # joint_string = joint_string.replace("*", "\\cdot")
    print(joint_string)
print("---------")
final_pos = simplify(T * Matrix([0, 0, 0, 1]))
x_expr, y_expr, z_expr = simplify(final_pos[0]), simplify(final_pos[1]), simplify(final_pos[2])
x, y, z = symbols('x y z', real=True)
solutions = nonlinsolve([x - x_expr, y - y_expr, z - z_expr], [t1, t2, t3, t4])

print("Solutions:")
for sol in solutions:
    print(sol)