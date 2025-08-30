# ros-recyclebot
시각장애인을 위한 분리수거 자동화 쓰레기통
- 음성 명령에 따라 플라스틱 또는 캔 칸으로 뚜껑 회전
- 분리수거장 이동을 위한 라인트래킹 주행
---

실행 방법

**[terminal 1]** 

roslaunch line_tracking line_tracking.launch

**[terminal 2]** 

sudo pigpiod

rosrun trash_sorter servo_controller_pigpio.py

**[terminal 3]** 

rosrun trash_sorter speech_kws_google.py _use_audio_topic:=true _audio_topic:=/audio/raw _topic_rate:=44100
