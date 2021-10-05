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
   
    # 좌우 경사 판단 Method # roll 각도 구하기 # [x, z]
    def lrSlope(maxInliers, XPOS, H): # inliers = inliers에 해당하는 인덱스 리스트

        p1=[XPOS[inliers[0]], H[inliers[0]]] # 첫번째 inlier의 [x,z] 값 
        p2=[XPOS[inliers[-1]], H[inliers[-1]]] # 마지막 inlier의 [x,z] 값 
    
        rol_ang = np.arctan((p2[1]-p1[1])/(p2[0]-p1[0])) # [rad] # 항상 p2[0] != p1[0] 
        return rol_ang
    
    # 상하 경사 판단 Method # pitch 각도 구하기  # [y, z] 
    def udSlope(maxInliers, YPOS, H):

        p1=[YPOS[inliers[0]], H[inliers[0]]]
        p2=[YPOS[inliers[-1]], H[inliers[-1]]]

        pit_ang = np.arctan((p2[1]-p1[1])/(p2[0]-p1[0]))
        return pit_ang
    
    # 예상되는 roll, pitch를 계산해서 상하, 좌우 picto 번호와 led 번호를 반환
    def estiSlope(pit_ang, rol_ang):
        carRol, carPit = getRollPitch() # car roll, pitch 받아오기
        
        if pit_ang*carPit > 0:
            estPit=pit_ang-carPit # 차의 현재 각도와 라이다의 예상 각도의 부호가 같을 때 (내리막 > 내리막, 오르막 > 오르막)
        else: estPit=pit_ang+carPit # 각도가 0이거나 두 각도의 부호가 다를 때 (내리막 > 오르막, 오르막 > 내리막)

        if rol_ang*carRol > 0:
            estRol=rol_ang-carRol 
        else: estRol=rol_ang+carRol

        pitTh = 2*np.pi/9 # pitch 위험 기준 각도 40[deg] = 2pi/9[rad]
        rolTh = np.pi/6 # roll 위험 기준 각도 30[deg] = pi/6[rad]

        if estPit > pitTh:
            pictoPit = "1000"
            if POS == 2: ledPit="001"
            else: ledPit="110"

        if estRol > rolTh:
            pictoRol = "0100"
            if estRol > 0: ledRol="010"
            elif estRol < 0: ledRol="100"
            else: ledRol="000"
        return pictoPit, pictoRol, ledPit, ledRol
        
    

