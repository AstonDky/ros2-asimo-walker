
print("import utils module sucessfully.")
import numpy as np
import math

def sgn(x):
     if x<0:
          return -1
     elif x==0:
          return 0
     return 1

def transl(x,y,z):
    mat = np.eye(4)
    mat[0,3] = x
    mat[1,3] = y
    mat[2,3] = z
    return mat

def trotz(rot):
     mat = np.eye(4)
     mat[0,0] = math.cos(rot)
     mat[0,1] = - math.sin(rot)
     mat[1,0] = math.sin(rot)
     mat[1,1] = math.cos(rot)
     return mat

def trotx(rot):
     mat = np.eye(4)
     mat[1,1] = math.cos(rot)
     mat[1,2] = -math.sin(rot)
     mat[2,1] = math.sin(rot)
     mat[2,2] = math.cos(rot)
     return mat

def t2r(T):
     mat = T[0:3,0:3]
     return mat

def Jacob(q):
     '''
     especially for UR5
     '''
     d = np.array([0.0895, 0 ,0 ,0.1091 ,0.0946 ,0.0823])
     a = np.array([0, -0.4250, -0.3922, 0, 0, 0])
     alpha = np.array([1.5708, 0, 0, 1.5708, -1.5708, 0])
     offset = np.array([0, 0, 0, 0, 0, 0])
     thd = q + offset
     
     #T0=trotz(0)@transl(0,0,0)@trotx(0)@transl(0,0,0)
     T0 = np.matmul(trotz(0), transl(0,0,0))
     T0 = np.matmul(T0, trotx(0))
     T0 = np.matmul(T0, transl(0,0,0))
     # T1=trotz(thd[0])@transl(0,0,d[0])@trotx(alpha[0])@transl(a[0],0,0)
     T1 = np.matmul(trotz(thd[0]), transl(0,0,d[0]))
     T1 = np.matmul(T1, trotx(alpha[0]))
     T1 = np.matmul(T1, transl(a[0],0,0))
     # T2=trotz(thd[1])@transl(0,0,d[1])@trotx(alpha[1])@transl(a[1],0,0)
     T2 = np.matmul(trotz(thd[1]), transl(0,0,d[1]))
     T2 = np.matmul(T2, trotx(alpha[1]))
     T2 = np.matmul(T2, transl(a[1],0,0))
     # T3=trotz(thd[2])@transl(0,0,d[2])@trotx(alpha[2])@transl(a[2],0,0)
     T3 = np.matmul(trotz(thd[2]), transl(0,0,d[2]))
     T3 = np.matmul(T3, trotx(alpha[2]))
     T3 = np.matmul(T3, transl(a[2],0,0))
     # T4=trotz(thd[3])@transl(0,0,d[3])@trotx(alpha[3])@transl(a[3],0,0)
     T4 = np.matmul(trotz(thd[3]), transl(0,0,d[3]))
     T4 = np.matmul(T4, trotx(alpha[3]))
     T4 = np.matmul(T4, transl(a[3],0,0))
     # T5=trotz(thd[4])@transl(0,0,d[4])@trotx(alpha[4])@transl(a[4],0,0)
     T5 = np.matmul(trotz(thd[4]), transl(0,0,d[4]))
     T5 = np.matmul(T5, trotx(alpha[4]))
     T5 = np.matmul(T5, transl(a[4],0,0))
     # T6=trotz(thd[5])@transl(0,0,d[5])@trotx(alpha[5])@transl(a[5],0,0)
     T6 = np.matmul(trotz(thd[5]), transl(0,0,d[5]))
     T6 = np.matmul(T6, trotx(alpha[5]))
     T6 = np.matmul(T6, transl(a[5],0,0))

     T00 = T0
     T01 = T1
     T02 = np.matmul(T01, T2)
     T03 = np.matmul(T02, T3)
     T04 = np.matmul(T03, T4)
     T05 = np.matmul(T04, T5)
     T06 = np.matmul(T05, T6)

     # T16 = T2@T3@T4@T5@T6
     T16 = np.matmul(T2, T3)
     T16 = np.matmul(T16, T4)
     T16 = np.matmul(T16, T5)
     T16 = np.matmul(T16, T6)
     # T26 = T3@T4@T5@T6
     T26 = np.matmul(T3, T4)
     T26 = np.matmul(T26, T5)
     T26 = np.matmul(T26, T6)
     # T36 = T4@T5@T6
     T36 = np.matmul(T4, T5)
     T36 = np.matmul(T36, T5)
     # T46 = T5@T6
     T46 = np.matmul(T5, T6)
     T56 = T6

     R00 = t2r(T00)
     R01 = t2r(T01)
     R02 = t2r(T02)
     R03 = t2r(T03)
     R04 = t2r(T04)
     R05 = t2r(T05)
     R06 = t2r(T06)

     Z0 = R00[: , 2:3]
     Z1 = R01[: , 2:3]
     Z2 = R02[: , 2:3]
     Z3 = R03[: , 2:3]
     Z4 = R04[: , 2:3]
     Z5 = R05[: , 2:3]
     Z6 = R06[: , 2:3]

     P06 = T06[0:3, 3:4]
     P16 = T16[0:3, 3:4]
     P26 = T26[0:3, 3:4]
     P36 = T36[0:3, 3:4]
     P46 = T46[0:3, 3:4]
     P56 = T56[0:3, 3:4]
     # P66 = np.zeros((3,1))
     J1 = np.append(np.cross(Z0.T, np.matmul(R00, P06).T).T, Z0,axis = 0)
     J2 = np.append(np.cross(Z1.T, np.matmul(R01, P16).T).T, Z1,axis = 0)
     J3 = np.append(np.cross(Z2.T, np.matmul(R02, P26).T).T, Z2,axis = 0)
     J4 = np.append(np.cross(Z3.T, np.matmul(R03, P36).T).T, Z3,axis = 0)
     J5 = np.append(np.cross(Z4.T, np.matmul(R04, P46).T).T, Z4,axis = 0)
     J6 = np.append(np.cross(Z5.T, np.matmul(R05, P56).T).T, Z5,axis = 0)
     
     J = np.concatenate((J1, J2, J3, J4, J5, J6), axis = 1)

     return J

if __name__ == "__main__":
     print(Jacob(np.array([1,1,1,1,1,1]))) #Debug
