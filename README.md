# Joint Matrix System

다양한 관절 유형에 대한 변환 행렬을 계산하는 C++ 기반 라이브러리입니다.  
다물체 동역학, 인체 모델링, 로봇 시뮬레이션 등에 활용 가능합니다.

## 지원 관절 유형
- 자유 관절 (Quaternion, Euler, Bryant)
- 회전 관절 (Revolute)
- 병진 관절 (Translational)
- 구형 관절 (Spherical)
- 유니버설 관절 (Universal)
- 복합 관절 (Cylinder, Planar, Revo+Trans 등)

## 사용 예시
```cpp
double Q[4] = {1.0, 0.0, 0.0, 0.0};
double T[3] = {0.0, 0.0, 0.0};
ON_Xform xform = ON_JointMatrix::CalcJointFree(Q, T);
