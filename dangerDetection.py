import numpy as np
import Main

class dangerDetection:
    MIN_ANGLE = 2*np.pi/9
    MAX_ANGLE = 7*np.pi/9

    # y expression
    def getBase(radius:float):
        main = Main.getInstance()
        return radius, (1 if main.goLeft else -1) * 9*main.velocity*main.onewayTime/(5*np.pi), (-7 if main.goLeft else 2)*main.velocity*main.onewayTime/5

    def RANSAC(XPOS, H): #XH 평면을 바라보고
        #XPOS : 라이다에서 측정 포인트까지의 x축 방향 distance
        #YPOS :
        #H = heightList : 지면에서 측정 포인트까지의 높이 #1차원 리스트
        
        maxInliers = []
        finOutliers = [] # final outliers list #[i1, i2, …, in] 인덱스 번호
        
        # algo rotation num is already set: 14
        for i in range(0, 14):
            while True: 
                i1, i2 = np.random.randint(0, len(XPOS), size=2)
                if (i1 != i2):
                    p1 = np.array([XPOS[i1], H[i1]])
                    p2 = np.array([XPOS[i2], H[i2]])
                    break
            # a 계산 시 분모가 0이 되는 걸 방지

            #두 점을 지나는 직선 (z=)f(x)=ax+b 구하기 
            a = (p2[1]-p1[1])/(p2[0]-p1[0])
            b = p1[1]-a*p1[0]

            inliers = []
            outliers = []
    
            for i in range(len(XPOS)) :
                x = XPOS[i] # x value of ith point
                z = H[i] # height of ith point

                # 임계값 넘어가면 outlier 
                # 임계값 내에 있으면 inlier

                z_th = 0.3 # threshold value [m]

                pz = a*x+b # p1, p2로 만든 식에 만족하는 z값

                if abs(z - pz) > z_th:
                    outliers.append([i])
                else:
                    inliers.append([i])

            if len(inliers) > len(maxInliers):
                maxInliers = inliers
                finOutliers = outliers
                param = [a, b]

        return maxInliers, finOutliers, param
    
    # Least Square Method 
    # inliers들로 구성된 기준식 하나 구하기 (=a, b 구하기)
    # inliersList는 RANSAC이 return한 maxInliers = [i]
    def LSM(inliersList, XPOS, H):
        A=np.empty((0,2), float) # A = [x, 1] (mx2)
        B=np.empty((0,1), float) # B = [z] (mx1)

        ### 다시 고쳐야함

        # A, B 행렬 만들기
        for i in inliersList:
            A = np.append(A, np.array([XPOS[i], 1]), axis=0)
            B = np.append(B, np.array([H[i]]))

        X=np.linalg.inv(A.T@A)@A.T@B # X 업데이트

        return X # np.array X = [a, b]를 return

