
import numpy as np
import math

# Least Square Method 
# inliers들로 구성된 기준식 하나 구하기 (=a, b, c 구하기)
# inliersList는 RANSAC이 return한 maxInliers 
# inliersList = [angle, distance]


J=np.empty((0,3), float) # Jacobian matrix (mx3)
F=np.empty((0,1), float) # F matrix (mx1)
P=np.empty((3,1), float) # P matrix (3x1)
param=[1, 2, 3]
inliersList=[[10,10], [11,10], [11,11], [12,12]]
X=np.array(param).reshape((3,1)) # X=[a, b, c] # X의 초기값을 param 값으로 설정

while 1:

        # 현재의 X 추정값 (a,b,c 추정값)에 대해 J, F를 계산함
    for i in inliersList:
        x=param[0]*math.cos(i[0]) # angle of i번째 inliersList to x value
        y=i[1] # y value of i번째 inliersList
        a=X[0,0]
        b=X[1,0]
        c=X[2,0]

        Ja=(a**2 - b*x)/(a**2*math.sqrt(1 - x**2/a**2))
        Jb=-math.acos(x/a)         
        J = np.append(J, np.array([[Ja, Jb, 1]]), axis=0)

        fi=a*math.sin(math.acos(x/a))-b*math.acos(x/a)+c-y
        F = np.append(F, np.array([[fi]]), axis=0)

        # P 계산, X 업데이트

    P=np.linalg.inv(J.T@J)@J.T@F # @ : 행렬곱
    bX=X # 업데이트 전 X

    X=X-P # X 업데이트

        # 역행렬은 정방 행렬 (nxn)에서만 정의됨. J는 정방 행렬 될 수 없음. 따라서 역행렬이 존재하지 않음 
    if all(abs(X-bX)<0.5): 
        break




