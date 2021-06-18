# 차선 변경&슬라이딩 윈도우

- Line Change
- Lane Recognition Polygon
- OpenCV
- Python


## 1. Line Change
xycar X 시리즈를 사용해 왼쪽 차선에서 출발한 차가 n초 후 차선을 변경하는 ROS패키지 제작.

- [Line Change](https://github.com/hyejeong99/ROS/tree/master/X1_line_change) - Line Change

## 코드 설명 

![20210115_102322](https://user-images.githubusercontent.com/59854960/113097461-e2bcaa00-9231-11eb-84c0-168473d6157d.jpg)

직선 차선을 기준으로 차가 차선 변경을 잘 하는지 살펴본다. 직선으로 가지 않고 휘어져서 주행한다면, vesc.yaml 파일에서 steering_angle_to_serve_offset 값을 조정한다. 
(기준 값은 5004이다.)

- 차가 왼쪽으로 치우쳐서 주행하면
:5004보다 값을 크게 설정
- 차가 오른쪽으로 치우쳐서 주행하면
:5004보다 값을 작게 설정

## 2. Lane Recognition Polygon
- [Line Detection](https://github.com/hyejeong99/ROS/tree/master/X1_line_detection) - Lane Recognition Polygon Code

## 코드 설명

![image](https://user-images.githubusercontent.com/59854960/113098056-ebfa4680-9232-11eb-9ee7-b44a597f54ef.png)

위와 같이 차선에 다각형을 그려줘 차선 인식을 하는 것이다. fillPoly 함수를 사용해 왼쪽 선과 오른쪽 선을 포함하는 다각형을 그린다.

![image (1)](https://user-images.githubusercontent.com/59854960/113099061-5495f300-9234-11eb-82fc-2f8067db24d3.png)

#### lane view
1. 캠 영상을 불러온다.
2. 영상을 블러처리 한다.
3. BRG 영상을 HLS으로 바꿔준다.
4. 영상을 gray scale 이미지로 바꾼다.

#### viewer view
슬라이딩 윈도우를 사용해 차선을 그려준다.
슬라이딩 윈도우란 이미지에서 "슬라이드"되는 고정된 폭과 넓이를 가지는 직사각형 영역이다.

1. 왼쪽, 오른쪽 차선을 그려준다.
2. 왼쪽, 오른쪽 슬라이딩 윈도우를 그려준다.

## 3. 데이터 받아서 dqn 처리하기
#### ultra_sonic 데이터를 이용해 dqn 처리하기
#### 사용 파일 : c1-dqn&window

c1에서 xycar_sensor라는 토픽명으로 메세지를 보내준다. 
윈도우에서 이를 sub해서 메세지별로 나눠준다. 
나눠준 메세지 토픽 중, ultra_sonic을 dqn에서 활용할 것이다.

angle 값을 -20, 0, 20 세 값으로만 처리하도록 해줬다.

## 4. 데이터 받아서 yolo 처리하기
#### image_raw데이터를 이용해 dqn 처리하기
#### 사용 파일 : c1-yolo&window

①xycar 캠 영상을 가져오는 "/usb_cam/image_raw"를 sub한다.

②xycar c1에서 "xycar_sensor_info"라는 토픽명으로 pub해준다. 여기에는 받아온 캠영상이 담겨져 있다.

③window에서는 xycar_sensor_info라는 토픽명으로 발행된 토픽을 sub한다.

④만약 camera_b가 True라면(camera 영상이 담겨져 왔다면), "/usb_cam/image_raw"라는 토픽명으로 pub해준다.

⑤yolo에서 이 캠 영상을 가지고 물체를 분석해준다.

## 5. 파일 병합
키보드 입력 1->차선에 점 찍는 파이썬 파일 실행

키보드 입력 2->점 찍은 이미지 확인하는 파있너 파일 실행

## 6. Control Cam Exposure
캠 영상을 불러와서 exposure 값을 조절하는 코드다.
exposure 값을 조정한 후 launch 파일에 해당 exposure 값을 저장할 수 있다.

- [Cam Exposure](https://github.com/hyejeong99/cotrol_exposure) - Control Cam Exposure

### exposure 조절하기

시각적으로 cam exposure 값 조정하기

- Use OpenCV
- Python

### 코드 설명
캠 영상을 불러와서 exposure 값을 조절하는 코드다. 
이 코드는, 시각적으로 영상의 exposure를 확인할 수 있다. 또한, 영상에 그려져있는 다각형으로 exposure 값을 조정할 수도 있다.

●exposure 텍스트
:현재 exposure 값이 얼마인지 나타내는 부분이다.

●up&down 사각형
:클릭해 exposure 값을 높이거나/낮출 수 있는 부분이다.

●exit 사각형
:클릭해 창을 닫을 수 있는 부분이다.

●save 원
:클릭해 launch 파일 선택 후, 해당 lanch 파일의 exposure값을 조정한 값으로 저장해주는 부분이다.

![최종본](https://user-images.githubusercontent.com/59854960/113079141-75981d00-920f-11eb-9615-3cae8089f2a5.png)\

