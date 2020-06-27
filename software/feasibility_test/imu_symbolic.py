import sympy as sp

def initial_values():
    m0, m1, m2 = sp.symbols("m0 m1 m2")
    a0, a1, a2 = sp.symbols("a0 a1 a2")

    m = sp.Matrix([[m0], [m1], [m2]])
    a = sp.Matrix([[a0], [a1], [a2]])

    c = a.cross(m.cross(a))

    print("a x (m x a)")
    print(sp.latex(c))

    # simulated ym
    #mb = sp.Matrix([[a1*(-a0*m1 + 1*m0) - a2*(a0*m2 - a2*m0)],
    #                [-a0*(-a0*m1 + a1*m0) + ],
    #                []])
    

    c_m0 = c.diff(m0)
    c_m1 = c.diff(m1)
    c_m2 = c.diff(m2)

    c_m = c_m0
    c_m = c_m.row_join(c_m1)
    c_m = c_m.row_join(c_m2)

    print("c_m")
    print(sp.latex(c_m))


def compute_f():
    q0, q1, q2, q3 = sp.symbols("q_{0} q_{v1} q_{v2} q_{v3}")
    w0, w1, w2 = sp.symbols("\omega_0 \omega_1 \omega_2")
    e0, e1, e2 = sp.symbols("e_0 e_1 e_2")

    T = sp.symbols("T")
    
    A = sp.Matrix([[q0, -q1, -q2, -q3 ],
                   [q1,  q0, -q3,  q2 ],
                   [q2,  q3,  q0, -q1 ],
                   [q3, -q2,  q1,  q0 ]])

    w = sp.Matrix([[1], 
                   [T/2*w0], 
                   [T/2*w1], 
                   [T/2*w2]])

    f = A*w

    f_q0 = f.diff(q0)
    f_q1 = f.diff(q1)
    f_q2 = f.diff(q2)
    f_q3 = f.diff(q3)

    f_q = f_q0
    f_q = f_q.row_join(f_q1)
    f_q = f_q.row_join(f_q2)
    f_q = f_q.row_join(f_q3)

    f = A*w

    print("f")
    print(sp.latex(f))

    print("f_q")
    print(sp.latex(f_q))

    w = sp.Matrix([[1], 
                   [T/2*(w0 - e0)], 
                   [T/2*(w1 - e1)], 
                   [T/2*(w2 - e2)]])
                
    f = A*w
    f_e0 = f.diff(e0)
    f_e1 = f.diff(e1)
    f_e2 = f.diff(e2)

    f_e = f_e0
    f_e = f_e.row_join(f_e1)
    f_e = f_e.row_join(f_e2)
    
    f_e.subs([(f_e0, 0), (f_e1, 0), (f_e2, 0)])
    print("f_e")
    print(sp.latex(f_e))


def compute_R_q():
    q0, q1, q2, q3 = sp.symbols("q_0 q_1 q_2 q_3")

    R = sp.Matrix([[2*q0*q0+2*q1*q1-1, 2*q1*q2-2*q0*q3,   2*q1*q3 + 2*q0*q2],
                   [2*q1*q2+2*q0*q3,   2*q0*q0+2*q2*q2-1, 2*q2*q3 - 2*q0*q1],
                   [2*q1*q3-2*q0*q2,   2*q2*q3 + 2*q0*q1, 2*q0*q0 + 2*q3*q3-1]])

    gn = sp.Matrix([[0],[0], [1]])
    mn = sp.Matrix([[1],[0], [0]])

    Rg = R*gn    

    Rg_q0 = Rg.diff(q0)
    Rg_q1 = Rg.diff(q1)
    Rg_q2 = Rg.diff(q2)
    Rg_q3 = Rg.diff(q3)
    
    Rg_q = Rg_q0
    Rg_q = Rg_q.row_join(Rg_q1)
    Rg_q = Rg_q.row_join(Rg_q2)
    Rg_q = Rg_q.row_join(Rg_q3)

    Rm = R*mn

    Rm_q0 = Rm.diff(q0)
    Rm_q1 = Rm.diff(q1)
    Rm_q2 = Rm.diff(q2)
    Rm_q3 = Rm.diff(q3)

    Rm_q = Rm_q0
    Rm_q = Rm_q.row_join(Rm_q1)
    Rm_q = Rm_q.row_join(Rm_q2)
    Rm_q = Rm_q.row_join(Rm_q3)

    print("R")
    print(sp.latex(R))

    print(sp.latex(Rg_q))

    print(sp.latex(Rm_q))


if __name__ == "__main__":

    initial_values()

    compute_f()

    compute_R_q()



