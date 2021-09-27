import numpy as np
import math

# Least Square Method 
# inliers들로 구성된 기준식 하나 구하기 (=a, b, c 구하기)
# inliersList는 RANSAC이 return한 maxInliers 
# 이때 maxInliers의 x값이 angle이 아니라 a*cos(angle)값인 찐 x가 넘어와야 하는데...
# inliersList = [[x1, y1], [x2, y2], ...]

def LSM(inliersList, param): 
    J=np.empty((0,3), float) # Jacobian matrix (mx3)
    F=np.empty((0,1), float) # F matrix (mx1)
    P=np.empty((3,1), float) # P matrix (3x1)
    X=np.array(param).reshape((3,1)) # X=[a, b, c] # X의 초기값을 param 값으로 설정

    while 1:

        # 현재의 X 추정값 (a,b,c 추정값)에 대해 J, F를 계산함
        for i in inliersList:
            x=param[0]*math.cos(i[0]) # x value of i번째 inliersList
            y=i[1] # y value of i번째 inliersList
        
            Ja=(X[0]**2 - X[1]*x)/(X[0]**2*sqrt(1 - x**2/X[0]**2))
            Jb=-math.acos(x/X[0])         
            J = np.append(J, np.array([[Ja, Jb, 1]]), axis=0)

            fi=X[0]*math.sin(math.acos(x/X[0]))-X[1]*math.acos(x/X[0])+X[2]-y
            F = np.append(F, np.array([[fi]]), axis=0)
        
        # P 계산, X 업데이트
        P=np.linalg.inv(J.T@J)@J.T@F # @ : 행렬곱
        bX=X # 업데이트 전 X 
        X=X-P # X 업데이트

        # 역행렬은 정방 행렬 (nxn)에서만 정의됨. J는 정방 행렬 될 수 없음. 따라서 역행렬이 존재하지 않음 
        if X==bX-np.linalg.inv(J.T@J)@J.T@F: 
            break

    return X.tolist() # np.array를 list로 바꿔 return
