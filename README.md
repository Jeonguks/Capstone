
- 2024 Sep ~ 2024 Dec  **캡스톤 디자인 “스마트팩토리를 위한 팔로우미 로봇 기반 물류 솔류션”**
    
    <aside>
    🧑🏻‍💻
    
    - 진행기간
        
        2024 September ~ 2024 December
        
    - 주요 사용기술 및 장비
        
        Python, ROS1 Noetic , C++(Arduino)
        
        Intel Realsense D435i , OPEN CR , Hokuyo Lidar 
        
    - 주요 역할
        
        Depth Camera로 부터 획득한 정보 ROS Topic으로 처리 및 관련 통합제어
        
        주요 구동부 설계 및 ROS패키지 커스터마이징 작업
        
        Motor의 Encoder 센서 처리를 통한 RPM값 처리
        
        Differential Drive 구조 구동 최적화 
        
        하드웨어 구조설계 및 회로설계
        
    - 주요 성과
        
        캡스톤디자인 경진대회 금상 수상
        
    </aside>
    
    ![스크린샷 2025-05-30 오후 1.37.43.png](README/%E1%84%89%E1%85%B3%E1%84%8F%E1%85%B3%E1%84%85%E1%85%B5%E1%86%AB%E1%84%89%E1%85%A3%E1%86%BA_2025-05-30_%E1%84%8B%E1%85%A9%E1%84%92%E1%85%AE_1.37.43.png)
    
    **[ 경진대회 포스터]**
    
    ![스크린샷 2025-05-30 오후 2.52.10.png](README/%E1%84%89%E1%85%B3%E1%84%8F%E1%85%B3%E1%84%85%E1%85%B5%E1%86%AB%E1%84%89%E1%85%A3%E1%86%BA_2025-05-30_%E1%84%8B%E1%85%A9%E1%84%92%E1%85%AE_2.52.10.png)
    
    **[ 동작 프레임워크 ]**
    
    ![스크린샷 2025-05-30 오후 1.38.20.png](README/%E1%84%89%E1%85%B3%E1%84%8F%E1%85%B3%E1%84%85%E1%85%B5%E1%86%AB%E1%84%89%E1%85%A3%E1%86%BA_2025-05-30_%E1%84%8B%E1%85%A9%E1%84%92%E1%85%AE_1.38.20.png)
    
    **[하드웨어 회로설계]**
    
    ![스크린샷 2025-05-30 오후 2.56.58.png](README/%E1%84%89%E1%85%B3%E1%84%8F%E1%85%B3%E1%84%85%E1%85%B5%E1%86%AB%E1%84%89%E1%85%A3%E1%86%BA_2025-05-30_%E1%84%8B%E1%85%A9%E1%84%92%E1%85%AE_2.56.58.png)
    
    ![스크린샷 2025-05-30 오후 2.57.31.png](README/%E1%84%89%E1%85%B3%E1%84%8F%E1%85%B3%E1%84%85%E1%85%B5%E1%86%AB%E1%84%89%E1%85%A3%E1%86%BA_2025-05-30_%E1%84%8B%E1%85%A9%E1%84%92%E1%85%AE_2.57.31.png)
    
    **[ 객체인식 결과에 따른 ROS Topic 발행 Pseudo-Code ]**
    
    ```
    **Function** Setup
    	Set pinMode for encoder
    	Attach interrupts to encoders
    **End** **Function**
    
    **Function** ReadEncoder1
    	**if** digitalRead(ENCODER1_A) = digitalRead(ENCODER1_B) then
    		encoder1.pos++
    	**else**
    		encoder1.pos--
    **End** **Function**
    ```
    
    [Motor의 Encoder를 이용한 속도값 획득 방법 Pseudo-Code ]
    
    ```
    **Function** AdjustMotorSpeed
    		diff <- encoder1.pos-encoder2.pos
    	**If** abs(diff) > ENCODER_ERROR_THRESHOLD then
    		motor1.pwm <- max(0, motor1.pwm - PWM_ADJUSTMENT)
    		motor2.pwm <- min(255,motor2.pwm - PWM_ADJUSTMENT)
    	**else if** diff< 0 then
    		motor1.pwm <- max(0,motor2.pwm - PWM_ADJUSTMENT)
    		motor2.pwm <- min(255,motor1.pwm + PWM_ADJUSTMENT)
    	**end if
    End Function**
    ```
    
    [DC Motor의 공정상 오차로 인한 속도 보정 Pseudo-Code]
    
    ### 주요 작업 설명
    
    - Differential Drive 구조에서 , DC Motor의 공정상 오차로 인하여 동일한 전압을 인가하여도
        
        양쪽 Motor간의 속도가 다른 문제가 발생
        
    - 특정 속도값 조건에서 차량의 rpm을 측정하여 실제 바퀴의 rpm 을 좌우 동일하게 맞춰지도록 수정
    
    ```
    **Function** Control_callback(msg)
    	command <- msg.data
    	**if** command = "REVERSE" then
    		Set leftMotorDir to reverse
    		Set rightMotorDir to reverse
    	**else** **if** command = "FORWARD" then
    		Set leftMotorDir to forward
    		Set rightMotorDir to forward 
    	**else** **if** command = "RIGHT" then
    		Set leftMotorDir to forward
    		Set rightMotorDir to reverse
    	**else** **if** command = "LEFT" then
    		Set leftMotorDir to reverse
    		Set rightMotorDir to forward
    	**else
    		Pass
    	End if
    End Function**
    ```
    
    ![스크린샷 2025-05-30 오후 3.26.26.png](README/%E1%84%89%E1%85%B3%E1%84%8F%E1%85%B3%E1%84%85%E1%85%B5%E1%86%AB%E1%84%89%E1%85%A3%E1%86%BA_2025-05-30_%E1%84%8B%E1%85%A9%E1%84%92%E1%85%AE_3.26.26.png)
    
    **[제작된 로봇의 구조]**
    
    - Diffential Drive 구조가 비교적 로봇의 속도를 제어하기에 간단하므로 이를 이용하도록 하였음.
    - 기존의 4개의 바퀴를 가지는 로봇을 설계하였으나, 모터를 2개만 구매 가능한 제약적인 조건으로 인하여
        
        Differential Drive 구조를 이용하기 위하여 각각의 모터를 사선으로 배치하였음.
        
    
    ![스크린샷 2025-05-30 오후 3.28.28.png](README/%E1%84%89%E1%85%B3%E1%84%8F%E1%85%B3%E1%84%85%E1%85%B5%E1%86%AB%E1%84%89%E1%85%A3%E1%86%BA_2025-05-30_%E1%84%8B%E1%85%A9%E1%84%92%E1%85%AE_3.28.28.png)
    
    - 위의 그림과 같이 모터가 서로 반대방향으로 작동할 경우 로봇은 반시계 방향으로 회전,
        
        그 반대의 경우 로봇이 시계방향으로 회전하는점을 이용하여 방향을 제어 하였음.
        
    - 객체 탐지및 추종이 주 임무이므로, 카메라가 항상 객체의 가운데를 바라보도록 설계
    - 객체가 로봇의 오른쪽으로 이동할경우 로봇을 오른쪽으로 회전,
        
        객체가 로봇의 왼쪽으로 이동할 경우 로봇을 왼쪽으로 회전 하도록 설계 
        
    
    ![스크린샷 2025-05-30 오후 3.34.33.png](README/%E1%84%89%E1%85%B3%E1%84%8F%E1%85%B3%E1%84%85%E1%85%B5%E1%86%AB%E1%84%89%E1%85%A3%E1%86%BA_2025-05-30_%E1%84%8B%E1%85%A9%E1%84%92%E1%85%AE_3.34.33.png)
    
    - 위와 같은 과정을 통해, 카메라가 한번 탐지 및 추종을 시작한 객체를 계속 따라다니도록 설계
    - 따라서 객체를 놓치지 않고 안정적으로 추종이 가능하도록 하였음.
    
    ![스크린샷 2025-05-30 오후 3.45.26.png](README/%E1%84%89%E1%85%B3%E1%84%8F%E1%85%B3%E1%84%85%E1%85%B5%E1%86%AB%E1%84%89%E1%85%A3%E1%86%BA_2025-05-30_%E1%84%8B%E1%85%A9%E1%84%92%E1%85%AE_3.45.26.png)
    
    ![스크린샷 2025-05-30 오후 3.39.37.png](README/%E1%84%89%E1%85%B3%E1%84%8F%E1%85%B3%E1%84%85%E1%85%B5%E1%86%AB%E1%84%89%E1%85%A3%E1%86%BA_2025-05-30_%E1%84%8B%E1%85%A9%E1%84%92%E1%85%AE_3.39.37.png)
    
    [ 로봇의 중심을 기준으로 객체가 벗어나는 방향에 따라 로봇을 해당 방향으로 회전시키도록 
    
    Python → ROS → MCU 제어 구성 ]
    
    ![스크린샷 2025-05-30 오후 3.41.39.png](README/%E1%84%89%E1%85%B3%E1%84%8F%E1%85%B3%E1%84%85%E1%85%B5%E1%86%AB%E1%84%89%E1%85%A3%E1%86%BA_2025-05-30_%E1%84%8B%E1%85%A9%E1%84%92%E1%85%AE_3.41.39.png)
    
    객체간의 일정 거리를 유지하기 위해, Depth 정보를 이용
    
    거리값이 특정값 이하가 되면, 로봇에게 정지 명령을 전송하여, 모터 구동을 정지시킴
    
    ## 회고
    
    ### ***구동부 제어***
    
    - **한계점 분석** : 같은 RPM을 유도하도록 전압값을 다르게 하드코딩 한것이기 때문에,  바닥의 재질, 바퀴의 상태 등에 따라 직진시 다시 오차가 발생하는 문제가 있음. 또한, 입력전압 및 속도간 관계가 선형이 아니므로, 목표속도가 달라질경우 오차가 다시 발생하는 문제가 있음.
    - **해결방향 분석**: PID제어기와 같은 피드백 제어를 이용하여, 목표 속도값과 주행 속도값의 error를 줄이는 방향으로 개선 가능.
    
    ### ***로봇의 제어***
    
    - **한계점 분석** : soft start, soft stop의 기능이 없어, 급출발 급정거의 문제 발생
        
        이러한 문제점으로 인하여, 획득한 영상정보가 흔들리는 문재 및 차량의 안정성이 하락
        
        또한, 객체가 일정한 속도를 유지하며 걷는것을 가정하였으므로
        
        객체의 속도 변화에 따른 객체-로봇간 일정거리 유지 기능 부재
        
    - **해결방향 분석** : 속도 증감값을 단계적으로 증감시켜 가속도를 낮추는 방안이나, 이동평균필터 등을 이용하여 급격한 변화를 방지하도록 하여야함.  객체의 속도 변화에 따른 객체-로봇간 일정거리 유지 기능에 대해서는 실시간 정보를 송수신하는 ROS System의 장점을 이용하여, 거리가 좁아지면 속도를 낮추고, 거리가 늘어나면 속도를 올리는 방향으로 개선 가능.
    
