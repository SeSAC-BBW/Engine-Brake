# 🏎️ Engine Start-up and Brake Control System

본 저장소는 **SeSAC Mobility Embedded Training Program**의 **Brake-by-Wire(BBW)** 프로젝트 중 메인 제어부 로직을 담당

## 💻 개발 환경 (Development Environment)

| 항목 | 상세 내용 |
| :--- | :--- |
| **OS** | Windows 10 |
| **IDE** | Keil MDK-ARM |
| **Tool** | STM32CubeMX |
| **Language** | C (HAL Library) |
| **Library** | `lcd.h` |
| **Hardware** | STM32L073RZ (Nucleo-64), LCD 1602 Keypad Shield, Potentiometer |


## 📌 주요 기능 (Core Features)

* **안전 시동 로직:** 브레이크 페달이 일정 임계치(`BRAKE_THRESHOLD`: 1500) 이상 눌린 상태에서만 엔진 시동(ON)이 가능하도록 설계되어 급발진 및 안전사고를 방지

* **브레이크 감도 매핑:** ADC로 읽어온 브레이크 값을 0~4095 범위로 매핑하여 제동 민감도를 조절하고, 현재 제동 강도를 LCD에 실시간으로 표시

* **상태 출력 및 디스플레이:** LCD 1602 Keypad Shield를 통해 현재 엔진의 시동 상태(`Engine: ON/OFF`)와 브레이크 수치를 시각화하여 사용자에게 전달

* **가속 제어부(Arduino) 데이터 전송:** UART4 통신을 통해 0.1초(100ms) 간격으로 엔진 상태 및 비상 제동 신호(Brake 수치 4000 이상 시)를 아두이노 보드로 전송


## 🛠 하드웨어 구성 (Hardware Pin Map)

| 구분 | 기능 | 핀 번호 | I/O | 상세 설명 |
| :---: | :--- | :---: | :---: | :--- |
| **Sensor** | 브레이크 입력 | **PA1** | Input (ADC) | 브레이크 페달 아날로그 입력 |
| **Sensor** | 버튼 입력 | **PA0** | Input (ADC) | ADC 기반 버튼 상태 수신 |
| **Comm.** | UART TX | **PC10** | Output (UART4) | 아두이노(D7)로 제어 데이터 전송 |
| **LCD** | Register Select | **PA9** | Output | LCD RS 제어 |
| **LCD** | Enable | **PC7** | Output | LCD EN 제어 |
| **LCD** | Data 4-7 | **PB5,4,10 / PA8** | Output | LCD 데이터 전송 |

## 📂 폴더 구조 (Folder Structure)

* `Core/Src/main.c`: 메인 엔진 제어 및 통신 로직 구현
* `Core/Src/lcd.c`: LCD 4-bit 통신 드라이버
* `Core/Inc/`: 프로젝트 헤더 파일 (`main.h`, `lcd.h`)
