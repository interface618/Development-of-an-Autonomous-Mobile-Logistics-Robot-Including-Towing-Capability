<p align='center'>
<img src="https://capsule-render.vercel.app/api?type=cylinder&height=200&color=2c2c2a&text=Autonomous-Mobile-Logistics-Robot&section=header&textBg=false&fontColor=FFFFFF&fontAlign=50&fontAlignY=40&fontSize=40&desc=Including-Towing-Capability&stroke=3-Axis-Cartesian-Robot%20&descAlignY=65&descSize=20" />

<div align="center">
<h2>[2025] 견인기능을 포함한 물류로봇 </h2>
본 시스템은 고장 로봇 발생 시 별도의 인력 개입 없이 작업 흐름을 유지할 수 있도록 하여 다운타임을 감소시키고, 운영 비용 절감과 처리 효율 향상을 가능하게 한다. 더 나아가 로봇 간 협업 기반 자동 복구 체계를 구축함으로써 물류 시스템의 복원 탄력성과 고가용성 확보에 기여하며, 스마트 물류센터 및 무인 물류 환경으로의 확장 가능성을 제공한다.
</div>

## 목차
  - [개요](#개요) 
  - [작품 설명](#작품-설명)
  - [작품 작동 방식](#작품-작동-방식)
  - [역할 및 내용](#역할-및-내용)
  - [트러블 슈팅](#트러블-슈팅)
  - [코멘트](#코멘트)

## 개요
- 프로젝트 이름: 견인기능을 포함한 물류로봇
- 프로젝트 지속기간: 2025.01-2025.12
- 개발 엔진 및 언어: C & C++ & Python 
- 멤버: Interface (이윤승 ,박승규 ,최성원 ,김혁진)

## 작품 설명
본 작품은 자율이동로봇(AMR) 환경에서 고장 로봇을 정상 로봇이 자율적으로 인식하고 도킹·견인하여 물류 작업 흐름을 유지하는 자동 복구형 협업 로봇 시스템이다. 다중 센서 융합 기반 정렬 및 결합 메커니즘과 견인 모드 전환에 따른 제어 파라미터 자동 조정을 통해 하중 변화에 안정적으로 대응할 수 있도록 설계되었다. 이를 통해 물류센터 내 작업 중단을 최소화하고 운영 안정성과 고가용성을 확보한다.

## 작품 작동 방식
<img width="640" height="480" alt="image" src="https://github.com/user-attachments/assets/c358708f-4f65-433b-838a-32cf11c9d10c" />

https://youtu.be/UkVe2rLXZ-U<br>

또는 demonstration video참조<br>

## 역할 및 내용

<table align="center">
  <thead>
    <tr>
      <th align="center">이름</th>
      <th align="center">역할</th>
      <th align="center">내용</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td nowrap align="center">이윤승</td>
      <td align="center">프로젝트 총괄<br>자율주행 알고리즘 개발<br>ROS2를 이용한 매핑 및 군집제어</td>
      <td align="center"></td>
    </tr>
    <tr>
      <td nowrap align="center">박승규</td>
      <td align="center">모터 제어</td>
      <td align="center">STM32 기반 엔코더 속도 측정 및 모터 PWM 제어 루프 구현<br>목표 선속도/각속도 추종을 위한 증분형 PID 제어기 적용<br>견인/비견인 모드 전환 시 PID 게인 자동 스위칭 구조 설계<br>엔코더 값을 이용해 Dead-reckoning 오도메트리로 (x, y, yaw) 위치를 추정<br>IMU + 2D LiDAR EKF 융합으로 장기 드리프트 보정</td>
    </tr>
    <tr>
      <td nowrap align="center">최성원</td>
      <td align="center">영상인식 장치 제작<br>자율주행 알고리즘 개발<br>로봇정렬</td>
      <td align="center"></td>
    </tr>
    <tr>
      <td nowrap align="center">김혁진</td>
      <td align="center">하드웨어, 회로도 제작</td>
      <td align="center">3D프린터를 이용한 외관제작<br>소자간의 밀집도가 높은 회로도 제작<br>회로도, 베터리, 젯슨오린나노 등 모듈간의 밀집도높은 내부 설계</td>
    </tr>
  </tbody>
</table>

## 트러블 슈팅
- **로봇의 크기와 무게 문제**
  - 문제: 로봇의 크기와 무게가 예상보다 커 줄일 필요가 생겼습니다.
  - 원인: 필요이상의 많은 소자를 사용하고 소자간의 밀집도가 떨어져 프로젝트의 크기가 커졌습니다.
  - 해결: 중요도가 떨어지는 소자를 줄이고 소자간의 밀집도를 중심으로 재설계를 했습니다.
  - 결과: 초기 모델보다 크기와 무게를 줄였습니다.

- **견인 모드 하중 변화로 인한 주행 오차 증가**
  - **문제:** 견인 상태에서 속도·자세(heading) 오차가 커지며 경로 추종 안정성이 저하됨
  - **원인:** 견인 하중 증가로 시스템 동특성(관성/저항)이 변해 기존 PID 이득으로는 추종 성능이 부족함
  - **해결:** **견인 모드 전용 PID 제어 이득을 재조정(튜닝)**하고, 견인/비견인 모드에 따라 이득을 분리 적용
  - **결과:** 견인 주행 시 속도·자세 오차가 감소하고 경로 추종이 안정화됨

- **Dead-reckoning 기반 오도메트리 누적 드리프트**
  - **문제:** 장시간 주행 시 (x, y, yaw) 추정값이 누적 오차로 인해 실제 위치와 점점 벌어짐
  - **원인:** 엔코더 기반 Dead-reckoning은 슬립, 노면 변화, 엔코더/시간 동기 오차가 누적되어 장기 드리프트가 발생함
  - **해결:** **IMU + 2D LiDAR를 EKF로 융합**하여 오도메트리 예측값을 센서 측정으로 주기적으로 보정
  - **결과:** 장기 드리프트가 억제되어 위치추정 신뢰도가 향상되고 도킹/견인 시 정렬 안정성이 개선됨
## 코멘트

- PID 게인을 실험적으로 조정해 성능을 맞췄지만, 시간·환경 제약으로 충분한 반복 실험 데이터를 쌓지 못한 점이 아쉬웠습니다. 다음에는 더 다양한 조건에서 데이터를 축적해 근거 기반으로 최적 게인을 도출하고 싶습니다.
- 너무 밀집도만 집중해서 놓친게 많아 아쉽다.
