
import numpy as np

# Least Square Method 
# inliers들로 구성된 기준식 하나 구하기 (=a, b, c 구하기)
# inliersList는 RANSAC이 return한 maxInliers = [x, distance (y)]

def LSM(inliersList, param): 
    J=np.empty((0,3), float) # Jacobian matrix (mx3)
    F=np.empty((0,1), float) # F matrix (mx1)
    P=np.empty((3,1), float) # P matrix (3x1)
    X=param.reshape((3,1)) # X=[a, b, c] # X의 초기값을 param 값으로 설정 # RANSAC에서 param을 np.array 형태로 받아옴

    while True:

        # 현재의 X 추정값 (a,b,c 추정값)에 대해 J, F를 계산함
        for i in inliersList:
            x=i[0] # x value로 변환 of i번째 inliersList
            y=i[1] # y value of i번째 inliersList
            a=X[0,0]
            b=X[1,0]
            c=X[2,0]

            Ja=(a**2 - b*x)/(a**2*np.sqrt(1 - x**2/a**2)) # f를 a로 편미분
            Jb=-np.acos(x/a)         
            J = np.append(J, np.array([[Ja, Jb, 1]]), axis=0)

            fi=a*np.sin(np.acos(x/a))-b*np.acos(x/a)+c-y
            F = np.append(F, np.array([[fi]]), axis=0)
        
        # P 계산, X 업데이트
        P=np.linalg.inv(J.T@J)@J.T@F # @ : 행렬곱
        bX=X # 업데이트 전 X 
        X=X-P # X 업데이트

        # 역행렬은 정방 행렬 (nxn)에서만 정의됨. J는 정방 행렬 될 수 없음. 따라서 역행렬이 존재하지 않음
        # 종료 조건은 x 값의 변화가 거의 없을 때
        if all(abs(X-bX)<0.5):  
            break

    return X # np.array X를 return


