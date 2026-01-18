# Autonomous-Emergency-Braking 긴급제동 시스템 모듈
## 🚗 Autonomous Emergency Braking (AEB) System
초음파센서 기반 거리측정을 적용한 자동 긴급 제동 시스템

## 1. 시스템 개요
본 시스템은 주행 중 전방 장애물을 감지하여 사고를 방지하는 자동 긴급 제동 시스템(AEB)을 구현한 것입니다. 

## 2. 주요 기능
<img width="1271" height="112" alt="image" src="https://github.com/user-attachments/assets/7cebf097-766c-4991-9886-6e8a5562a7ff" />

- 디스크 제동: 서보모터의 braking 상태와 unbranking 상태에 따른 제어를 통해 디스크를 완전제동시킵니다.
- 실시간 장애물 감지: 초음파 센서를 활용하여 $1\mu s$ 정밀도로 전방 거리를 측정하고 25cm이내 도달 시 즉각 제동합니다.
- 제동 상태 표시: LED와 LCD를 통해 제동 상태를 표시하여 사용자 및 주변환경에 긴급 제동 상태임을 알립니다.

## 3. 시스템 아키텍처

<img width="745" height="186" alt="image" src="https://github.com/user-attachments/assets/4c0647f0-1c5e-4310-b37e-e258fb227aa9" />


## 4. 개발 환경
언어: C / C++

도구: Keil uvision, STM32CubeMX

사용 기술: PWM, TIM
