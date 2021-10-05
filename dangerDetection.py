import numpy as np
import Main

class dangerDetection:
    MIN_ANGLE = 2*np.pi/9
    MAX_ANGLE = 7*np.pi/9
    
    minpwid = 2 # outliers 인덱스가 최소 몇개 이상 연속돼야하는지 기준값
    zth = 2 # 장애물끼리의 높이 차 허용 기준값

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

    def Obstacle(finOutliers, XPOS, H, minpwid, zth):       
        # minpwid개 이상 연속으로 모인 점들을 리스트에 저장
        packet = []
        tmp = []

        v = finOutliers.pop(0)
        tmp.append(v)

        while(len(finOutliers)>0):
            vv = finOutliers.pop(0)
            if v+1 == vv:
                tmp.append(vv)
                v = vv
            else:
                if len(tmp) > minpwid:
                    packet.append(tmp)
                tmp = []
	            tmp.append(vv)
	            v = vv
        
        if len(tmp) > minpwid:
            packet.append(tmp)
            
        finobs=[]
        ledpos=[]
        pictopos=[]

        # 1개의 장애물을 구성하는 인덱스들끼리 z값 비교하여 큰 차이 없으면 취함
        for i in range(len(packet)):
            tmpack = np.array(packet[i])
            tmph=[]
            tmpx=[]
            for j in tmpack:
                tmph.append(H[j])
            if all((np.array(tmph)-tmph[0])<zth): 
                finobs.append(packet[i])

                # 오목인지 볼록인지 판단
                if all(np.array(tmph)>0): pictopos.append("0001")
                elif all(np.array(tmph)<0): pictopos.append("0010")
                else: pitcopos.append("0000")

                # 장애물 좌/우/전방 위치 판단
                if POS == 2:
                    ledpos.append("001")
                else:
                    for k in tmpack:
                        tmpx.append(XPOS[k])
                    if all(np.array(tmpx)>0): ledpos.append("010")
                    elif all(np.array(tmpx)<0): ledpos.append("100")
                    else: ledpos.append("110")
        
	obspicto=[0,0,0,0]
	obsled=[0,0,0]

	for i in range(len(pictopos)):
   		for j in range(4):
        		tmppicto = pictopos[i]
        		obspicto[j] += int(tmppicto[j])
        		if obspicto[j]!=0: obspicto[j]=1

	for i in range(len(ledpos)):
    		for j in range(3):
        		tmpled = ledpos[i]
        		obsled[j] += int(tmpled[j])
        		if obsled[i]!=0: obsled[i]=1
				
	return obspicto, obsled

    #roll, pitch, obstacle 각각의 picto, led 결합하여 아두이노로 넘겨줄 최종 picto, led 구하기
    def finalPictoLed(pictoPit, pictoRol, obspicto, ledPit, ledRol, obsled):
	finpicto=[0,0,0,0]
	finled=[0,0,0]
	repicto="" #아두이노로 return할 picto 위치 ex. "0001"
	reled="" #아두이노로 return할 led 위치 ex. "100"
	
	for i in range(4):
		finpicto[i] = int(pictoPit[i])+int(pictoRol[i])+obspicto[i]
    		if finpicto[i]!=0: finpicto[i]=1
    		repicto+=str(finpicto[i])
	#print(finpicto)
	#print(repicto)

	for i in range(3):
    		finled[i] = int(ledPit[i])+int(ledRol[i])+obsled[i]
    		if finled[i]!=0: finled[i]=1
    		reled+=str(finled[i])
	#print(finled)
	#print(reled)
	return repicto, reled
	



