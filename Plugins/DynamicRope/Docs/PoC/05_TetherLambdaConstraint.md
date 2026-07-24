# 05. Wrapped 테더의 λ 제약 재편 (설계)

작성: 2026-07-20. 상태: **A~F 전체 완료** — A(순수 수학) CL 536, B+C(질량 정정·Constraint 배선) CL 545,
D(재현 리그) CL 546, 랙돌 물리 제약 전환(§3.4-1 정정) CL 556, Pull 장전 토글+climb-in 공유 CL 564,
E(Constraint 기본 승격) CL 568, **F(레거시 제거) 이 문서와 동반 CL**: ERopeTetherMode/레거시 노브 9종/
두 정책 함수/슬랙 브레이크·장부/RopeTraction 사장 함수 4종+테스트 6종/Rope.Test.TetherMode 삭제 —
λ 제약(+랙돌 물리 제약)이 유일한 테더 경로다. PIE 실측: 벽 탈출/프랍 드래그/랙돌(BareWrap·Pierce)/
장전 토글 통과. **후속 정정 §9(2026-07-22)**: 프랍 무중력/휙·wielder 들썩임 — 회수 상한 분리·직교
감쇠 dt 보정·팽팽 게이트 해제 유예·스켈레탈 특례 축소.
미결 결정(§7): TensionRelease의 λ 단위 이행, 코너 마찰(capstan).
관련: `01_PostWrapModel.md`(성립 이후 = 데이터 + 제약), 현행 구현 `RopeComponent.cpp`
(UpdateTether / ApplyMassShareTether / ApplyBinaryPullableTether), `Logic/RopeTractionSolver.h`.

---

## 1. 배경 — 왜 재편인가

현행 테더는 "양끝을 **각자 독립적으로**, 속도 레벨에서, 비례(P) 제어로 기하 목표에 서보"하는
구조다. 로프의 실제 물리는 "양끝 사이 **단일 비신축 제약**"인데, 그걸 끝별 서보 두 개로 흉내
내면서 다음이 구조적으로 발생한다:

- **운동량 비보존**: 각 끝이 독립 서보라 에너지가 주입되기도(근접 랙돌 폭주 — 방향 churn ×
  매 프레임 속도 add), 비일관하게 소실되기도(랙돌 끌기 탄성 — 명령은 하는데 마찰/관절이 소화,
  경계 미관철) 한다.
- **정적 앵커 = 능동 윈치**: 초과분을 닫는 유일한 수단이 고정 리엘 속도(TetherReelSpeed)라,
  벽에 감고 벗어나려는 wielder가 "정지"가 아니라 "벽으로 400cm/s 견인"을 당한다.
  "윈치 없는 하드 스톱" 설정이 존재하지 않는다.
- **보정 장치의 누적**: 위 구멍들을 각각 막느라 방향 EMA, 가속 상한, 속력 상한, 주입 장부
  (TowedVelDebt), 팽팽 게이트 3종(chord 비율·최소 장력·처짐), 앵커 속도 피드포워드가 쌓였다.
  각 장치는 국소적으로 옳지만 조합이 다음 버그를 만든다(2026-07 수정 이력 전체가 이 패턴).

데모 3종(입체기동 / 도르래 / 드래곤 라이딩)은 전부 "팽팽한 줄이 두 몸을 결합"하는 시나리오다.
특히 **도르래는 끝별 서보로는 표현 자체가 안 되고**(양끝에 같은 장력이 걸려야 함), 단일 λ
제약에서는 공짜로 나온다.

## 2. 목표 / 비목표

**목표**
- 테더를 프레임당 **스칼라 장력 임펄스 λ 하나**를 풀어 양끝에 ±로 인가하는 단방향(λ ≥ 0)
  제약으로 재편한다.
- 질량 분배(MassShare)·이진 양보(BinaryPullable)를 단일 모델로 통합 — 분배는 역질량에서
  자연 유도되고, 이진 거동은 그 극한이다.
- 프레임률 독립(모든 게인은 시상수 또는 dt 정규화), dt 나눗셈으로 만들어지던 임의 게인 제거.
- 정적 앵커 = 수동 제약(바깥 상쇄 + 경계 유지), 되감기(rest 길이 축소)만이 능동 견인.
- 기존 확장 계약 유지: `ResolveTetherEndpoint` 래더, `ApplyTractionToReceiver` 단일 관문,
  `FRopeTractionRequest`.

**비목표**
- XPBD 솔버/GPU 경로 변경 없음(자유 구간 솔브는 그대로 — 이 설계는 **끝점 인가**만 다룬다).
- 능동 Pull(장력 상한 속도 드라이브)은 유지 — λ 위에 얹는 사용자 힘.
- 코너 마찰(capstan) 없음 — v1은 이상 도르래(코너 무마찰). 필요해지면 λ 감쇠 계수로 후속.
- 네트워킹 없음(현행과 동일).

## 3. 물리 모델

### 3.1 제약 정의

매 Wrapped 프레임(GT, ③ 인가 단계):

```
C = L_path − L_rest          (cm, C > 0 = 초과 = 위반)
L_path = ComputePull의 코너-다리 chord 합에서 "앵커→손" 경로 부분
         (현행 TautChordLen — 다리별 rest 클램프 포함)
L_rest = AnchorNode × SegmentLength + TetherSlack
```

현행 overshoot(`UpdateTether`의 fractional aim 기반 첫 다리 초과분)와 달리 **전 체인** 기하를
쓴다. 처짐/구김 = chord 합 < rest = C < 0 = λ = 0.

> **정정(2026-07-20 랙돌 PIE)**: 초안은 "C ≤ 0이 슬랙 게이트의 전부"라고 주장했으나 틀렸다.
> C의 소스(비클램프 chord 합)는 **부분 스트레치에 오염**된다 — 랙돌 본이 요동치면 앵커 인접
> 다리만 strain limit(1.5×)까지 늘어나, 나머지가 늘어져 있어도 합이 rest를 넘어 슬랙 로프에서
> C > 0이 된다. 그 가짜 C에 λ가 상한까지 발화 → 쌍 임펄스 견인 → 요동 가속 → 더 큰 스트레치의
> 정귀환(실측: T=상한 클램프 빨강 + 랙돌·wielder 동반 요동). 따라서 발화 조건은
> **C > 0 ∧ 전 체인 팽팽(bChainTaut — 클램프 chord 비율·최소 전달 장력·처짐 3중 게이트)**이다.
> 레거시가 3차 보강(CL 466→470)으로 얻은 게이트가 Constraint에서도 정본으로 남는다.

### 3.2 끝점 관측

| 기호 | 의미 | 소스(현행 재사용) |
|---|---|---|
| `d_t` | 대상 끝 안쪽 방향(앵커→첫 다리) | `PullDrive.SmoothedPullDir` |
| `d_w` | wielder 끝 안쪽 방향(손→첫 다리) | `ComputeSmoothedWielderDir` |
| `v_t` | 대상 끝 속도 | 본/프리미티브 `GetPhysicsLinearVelocity`, CMC `Velocity`, 앵커 0 |
| `v_w` | wielder 끝 속도 | 동일 래더 |
| `w_t, w_w` | 유효 역질량 | `ResolveTetherEndpoint`(§3.4 정정 반영) |

벌어지는 속도(separation rate):

```
s = −(v_t·d_t + v_w·d_w) − dL_rest/dt        (+ = 벌어지는 중)
```

`dL_rest/dt`는 되감기(ReelRate에 의한 SegmentLength 축소)의 이번 프레임 변화율 — 감는 중이면
rest가 줄어 s가 커지고, λ가 그만큼 당긴다. **앵커 속도 피드포워드는 불필요해진다**: 움직이는
앵커(드래곤)의 속도는 `v_t`로 s에 이미 실측된다(순항 추종이 제약의 정의).

### 3.3 λ 솔브 (프레임당 해석적 1회 — 반복 불필요)

비신축(`α = 0`) 제약:

```
β      = 1 − exp(−dt / TetherSettleTime)     // 위치 오차 회수 게인(프레임률 독립)
s*     = −β · C / dt                          // 목표: 이번 프레임 C의 β 비율 회수
Δs     = s* − s                               // 필요한 접근 속도 변화(음수여야 당김)
λ      = max( −Δs / (w_t + w_w), 0 )
```

비신축 로프는 임의의 kinematic 입력에도 exact length를 지키므로 유한 force cap과 양립할 수 없다.
따라서 `MaxTetherTension`은 이 경로에서 반력을 자르는 값이 아니라 overload 기준선이다.

탄성(`α > 0`) 재료는 후단 상태에서 Kelvin-Voigt 힘을 평가하도록 동시에 푼다:

```
k      = 1 / α
m_eff  = 1 / (w_t + w_w)
c_crit = 2 · sqrt(k · m_eff)
s+     = s − (w_t + w_w) · λ
C+     = max(C, 0) + dt · s+
λ      = dt · (k · C+ + c_crit · s+)
T      = clamp(λ / dt, 0, MaxTetherTension)
λ      = T · dt
```

폐형식은 `r=2√(α/(w_t+w_w))`일 때
`T=[max(C,0)+(r+dt)s] / [α+(w_t+w_w)dt(r+dt)]`다. `α`를 persistent λ 없는 1회
속도 솔브의 단순 `α/dt²`로 쓰거나 현재 힘 `T·dt`를 explicit 적용하면 저FPS에서 감쇠가 상대속도를
뒤집는다. 후단 implicit 식은 모든 dt에서 안정적이고, 일정 하중 F에는 프레임률과 무관하게
`C→αF`, `T→F`로 수렴한다.

인가(임펄스 쌍 — 크기 같고 각 끝의 다리 방향):

```
Δv_t = d_t · (λ · w_t)      → 대상 끝
Δv_w = d_w · (λ · w_w)      → wielder 끝
```

성질:
- **λ ≥ 0(단방향)**: 접근 중이거나 슬랙이면 인가 없음. 로프는 밀지 못한다.
- **에너지 주입 없음**: 임펄스 쌍은 상대 접근만 만든다. 벌어지는 속도를 상쇄하고 β·C/dt만큼만
  회수 — 경계를 지나쳐 쏘는 잔류 속도가 구조적으로 없다(현행 요요 사이클 소멸).
- **분배 자동**: 각 끝의 ΔV = λ·w. 무거운 쪽이 덜 움직인다(MassShare), 한쪽이 앵커(w=0)면
  다른 쪽이 전부(BinaryPullable의 양보), 벽(양끝만... 대상=벽)이면 wielder ΔV = λ·w_w가 정확히
  "바깥 속도 상쇄 + 경계 유지"까지만 나온다(s*가 β·C/dt로 유한하므로 윈치화 불가능).
- **가속 상한이 물리적**: 끝별 ΔV ≤ λ_max·w = (MaxTetherTension/질량)·dt. 무거울수록 천천히 —
  현행 TetherMaxAcceleration(질량 무관 일괄 20g)을 대체한다.
- **장력 관측치**: `T_tether = λ / dt` (kg·cm/s²). 디버거/BP/절단 판정의 새 정본 후보.
  주의: 기존 `SegmentTension`은 "질량 1 노드 기준 상대 힘"으로 단위계가 다르다 —
  TensionReleaseForce는 당분간 SegmentTension 기준을 유지하고(동작 불변), `GetTetherTension()`
  (λ/dt)을 신설해 병행 관측 후 이행을 결정한다.

### 3.4 수신자별 인가와 유효질량 (래더 유지 + 3건 정정)

기존 `ResolveTetherEndpoint` 래더와 `ApplyToTetherEndpoint` 디스패치, `ApplyTractionToReceiver`
관문을 그대로 쓴다. 정정/규약:

1. **풀 랙돌/물리 대상**: 비신축 모드는 Chaos 거리 제약이 서브스텝에서 중력·관절·접촉과 함께
   반력을 푼다. 탄성 모드는 공통 analytic λ를 쓰되 실제 wrap attachment의 점 속도
   `v_point=vCOM+ω×r`, 점 역질량 `w=1/m+(r×d)ᵀI⁻¹(r×d)`를 관측/솔브하고,
   같은 점에 `AddImpulseAtLocation(λ·d, point, bone)`을 인가한다. COM 속도·전신 질량으로
   계산한 ΔV를 단일 본에 `bVelChange=true`로 직접 쓰는 질량/Jacobian 불일치는 금지한다.
2. **부분 랙돌(시뮬 본이 키네마틱 체인에 묶임)**: rung 1에서 **Character로 폴스루**한다
   (유효질량 = CMC 질량 × 브레이스, 인가 = CMC). 감긴 본에는 시각 반응용 소량 임펄스만 옵션.
   → `ResolveTetherEndpoint` rung 1 주석의 "알려진 한계"(2026-07-15 보류)를 이 설계로 해소.
3. **CMC**: `Movement->Velocity += Δv_w` 직접(이번 프레임 반영 계약 유지). 접지 브레이스
   (GroundBraceFactor)는 유효질량으로 계속 표현. 접지 마찰이 다음 틱에 ΔV를 깎는 것은
   "버티는 발"의 정당한 물리로 취급한다(λ가 다음 프레임 C로 다시 관측 — 정상 수렴 루프).
4. **컴포넌트 시뮬 바디**: 풀 랙돌과 같은 backend 정책을 쓴다. 비신축은 hard Chaos limit,
   탄성은 공통 analytic λ의 attachment-point impulse다.
5. **앵커**: w=0, 인가 없음(현행 위치 오프셋 폴백 삭제 — 앵커는 정의상 안 움직인다).
6. **자기 랩(owner == 대상)**: 양끝이 같은 몸 — 현행대로 대상 몫 전량 특례 유지.
7. **2차 방어 유지**: 인가 결과 속력을 `ClampInjectedVelocity`(TetherMaxSpeed)로 클램프 —
   **모든 경로에**(현행은 랙돌 add 경로에만 없어 폭주 구멍이었다). `TetherPerpDamping`은
   물리 바디 한정으로 유지(방향 급전환 잔여 관성).

### 3.5 무엇이 필요 없어지는가

| 제거 대상 | 근거 |
|---|---|
| ~~팽팽 게이트 3종의 테더 게이트 역할~~ | **유지로 정정**(§3.1 정정 참조) — C는 부분 스트레치에 오염돼 단독 게이트가 못 된다. Constraint 발화 = C > 0 ∧ bChainTaut |
| 앵커 속도 피드포워드 (SmoothedAnchorVelocity의 견인 사용) | s가 실측 상대 속도 — 순항 앵커 자동 추종. EMA 자체는 디버거 표시용으로 유지 |
| 슬랙 브레이크 + TowedVelDebt 장부 | λ는 벌어질 때만·회수량만 인가 — 과잉 주입이 없어 회수할 장부가 없다 |
| TetherReelSpeed의 상시 리엘 | 능동 견인은 되감기(rest 축소)가 유일한 경로 — s의 dL_rest/dt 항으로 자연 유입 |
| MassShare/BinaryPullable 이원화 | 분배가 λ·w에서 유도. `UpdateTargetPullable`은 능동 Pull climb-in 방향 판정용으로만 유지(§3.4의 질량 정정 적용) |

## 4. 노브 마이그레이션

### 4.1 신설 (3개)

| 노브 | 단위/기본 | 의미 |
|---|---|---|
| `TetherSettleTime` | s, 기본 0.08 | 초과분 회수 시상수(β = 1−exp(−dt/τ)). 작을수록 단단 |
| `MaxTetherTension` | kg·cm/s², 기본 0 = 무제한 | λ 상한 = 테더가 낼 수 있는 최대 장력. "몇 kg부터 못 버티나"의 정본 |
| `TetherCompliance` | 기본 0 | 의도적 탄성(연출용). 0 = 비신축 |

### 4.2 유지 (의미 불변)

`TetherSlack`, `GroundBraceFactor`, `TetherMaxSpeed`(2차 안전망), `TetherPerpDamping`(물리 바디),
`DistanceReleaseSlack`, `PullBendThresholdDeg`, `PullDirSmoothTime`, `PullAimSmoothTime`,
능동 Pull 전부(`PullForce`/`ActivePullMaxLinearSpeed`/`ActivePullMaxAngularSpeed`/taut 게이트),
`TensionReleaseForce`/`Time`(SegmentTension 기준 유지).

### 4.3 대체/삭제

| 기존 | 처리 |
|---|---|
| `TetherMode` (MassShare/BinaryPullable) | 이행기에 `Constraint` 값 추가 → 검증 후 enum 자체 deprecate |
| `TetherResponse` | → `TetherSettleTime` (0.2/frame@60fps ≈ τ 0.075s로 환산) |
| `TetherReelSpeed` | 삭제 — 리엘은 `SetReelRate`(rest 축소)로 일원화. 프리셋 이행 필요 |
| `TetherSettleDist` | 삭제(β·C/dt가 taper를 대체) |
| `TetherMaxAcceleration` | → `MaxTetherTension` (질량 의존 가속 상한으로 물리화) |
| `bAutoTetherShare` / `TetherMassBias` / `TetherTargetShare` | 삭제 — 분배는 λ·w 자동. 강제 분배가 실수요면 후속으로 끝별 w 스케일 1개 검토 |
| `TetherCharacterSmoothTime` | 삭제(주석 자체가 "0이 정답"이라 실질 무사용) |
| `TetherSlackBrakeTime` | 삭제(장부 폐기) |
| Taut 3종 + 히스테리시스 | **이름/값 유지**, 소비처가 능동 Pull 게이트로 축소(주석 갱신) |

### 4.4 파생 상태/API 재정의

- `LastTargetShare` = `w_t / (w_t + w_w)` (λ 분배 비율 — `IsWielderTetherActive`/디버거 호환).
- `GetTetherOvershoot()` = `max(0, C)` (의미 동일, 산출원만 전 체인 C로).
- `IsChainTaut()` = `C > 0` 래치(Wielder GroundExit 게이트 호환 — 의미 강화되지만 방향 동일).
- `FRopeTractionRequest`: Source=Tether의 `Amount` = **이번 프레임 축 ΔV(cm/s)** 로 명문화
  (현행은 cm 스텝 — 서브클래스 오버라이드 팀 공지 필요).
- 신설 `GetTetherTension()` = λ/dt.
- 프리셋(`URopePreset`)/DemoPresets: TetherMode·Response·ReelSpeed 스탬프 → 신 노브로 이행.

## 5. 데모 3종 매핑

| 데모 | λ 모델에서의 표현 |
|---|---|
| **입체기동** (키 입력 → 대상 쪽 고속 이동) | 되감기(ReelRate↑) → dL_rest/dt가 s에 유입 → λ가 wielder를 당김(w_w 몫). 목표 속도감은 ReelRate가, 힘 한계는 MaxTetherTension이 담당. 기존 GroundExit/스윙 에어컨트롤 게이트 호환(§4.4) |
| **도르래 느낌** (감긴 대상 ↔ wielder) | 현재 단일 material-length 제약의 같은 λ가 양쪽 수신자에 질량비로 전달되어 비신축 장력/하중 교환은 표현한다. 다만 wrap anchor는 material point에 고정되므로, 줄이 가이드를 마찰 없이 미끄러지며 두 leg 길이를 재분배하는 **완전한 2-sided pulley topology는 별도 후속**이다. |
| **드래곤** (로프 박고 이동) | v_t = 드래곤 본 실측 속도 → 순항 추종이 제약 정의에 내장(피드포워드 삭제). Pierce 앵커도 동일 경로 |

## 6. 구현 계획 (CL 단위)

각 CL: cpp 수정 시 `-DisableAdaptiveUnity` 빌드 + 자동화 스위트 + 해당 PIE 씬. P4 설명 영어.

1. **CL A — 순수 수학**: `RopeTraction::SolveTetherLambda(C, s, w_t, w_w, dt, β, α, λ_max)`
   + 유닛 테스트(수렴/단방향/상한/분배/무동작 경계). 기존 코드 무변경.
2. **CL B — 유효질량 정정**(독립 가치): 풀 랙돌 전신 질량, 부분 랙돌 → Character 폴스루.
   현행 두 모드의 분배/pullable도 즉시 개선(= 증상 2의 절반). 테스트 + PIE(랙돌 끌기).
3. **CL C — Constraint 모드 병행 배선**: `ERopeTetherMode::Constraint` 신설, `UpdateTether`에서
   분기. 관측(C/s/d) 산출 + 인가(기존 디스패치/관문 재사용) + 디버거에 λ/T_tether/C 표시.
   구 모드 무변경 — PIE A/B 가능 상태.
4. **CL D — PIE 검증 3씬**: ①근접 랙돌 wrap(폭주), ②랙돌 BinaryPullable 끌기(탄성),
   ③벽 테더 탈출(윈치) + 데모 3종 스모크. 고정 재현 레벨로 커밋(회귀 자산).
5. **CL E — 노브 이행**: 신 노브 3종 추가, 구 노브 deprecate(meta), 프리셋/데모 BP 갱신,
   Constraint를 기본 모드로. 팀 공지(TractionRequest.Amount 의미, TetherReelSpeed 폐지).
6. **CL F — 정리**: 슬랙 브레이크/장부/피드포워드/게이트 강등/구 모드 함수 제거, 문서 갱신
   (본 문서 상태 → 확정, CLAUDE.md 아키텍처 절).

## 7. 리스크 / 열린 결정

- **CMC 틱 순서**: 로프는 TG_PostPhysics 후행 보정 — λ의 ΔV를 CMC가 다음 틱에 소비한다.
  β가 프레임 지연을 흡수하지만, 고속(입체기동)에서 1프레임 랙이 보이면 τ를 낮춰 대응.
- **랙돌 전체 평행이동의 연출**: "감긴 본부터 끌려오는" 국소감이 사라진다. 필요하면 λ의
  일부(비율 노브)를 감긴 본에 추가 인가하는 후속 — v1은 단순화 우선.
- **GPU 미러 지연**: C/d의 소스(Sim.Positions)가 1~2프레임 지연 미러 — 현행 EMA(방향/조준)
  유지로 흡수. λ 자체는 스칼라라 노이즈에 상대적으로 둔감.
- **다중 로프 한 대상**: 로프별 λ 순차 인가(sequential impulse와 동형) — v1 허용, 경합 시
  순서 의존이 보이면 서브시스템 레벨 2-pass 검토.
- **강제 분배 실수요**(`TetherTargetShare` 사용처): 프리셋/데모에서 실제 사용 중인지 이행 전
  확인 — 사용 중이면 "끝별 w 스케일" 노브 1개로 승계.
- **결정 필요**: ①TensionRelease를 λ 기준으로 옮길지(단위계 변경 — 팀 튜닝값 재조정 필요),
  ②BinaryPullable enum을 언제 제거할지(외부 BP 참조), ③코너 마찰(capstan) 도입 여부.

## 8. PIE 검증 씬/절차 (CL D)

검증 당시 고정 .umap 대신 **코드 스폰 리그**(`Debug/RopeTetherTestScenes.cpp`, 개발 빌드 전용)를 썼다 —
텍스트라 리뷰/머지가 되고, 플레이어 기준 상대 배치라 각자의 `Lvl_*Test` 맵에서 그대로 돌았다.
**리그는 검증 완료 후 삭제됨**(아래 커맨드는 더 이상 존재하지 않는다) — 표는 무엇을 어떻게 확인했는지의
기록으로만 남긴다.

| 커맨드(삭제됨) | 씬 | 재현 절차 | 기대(레거시 → Constraint) |
|---|---|---|---|
| `Rope.Test.TetherScene wall` | 전방 4m 기둥(지름 60cm) | 감기 → 뒤로 걷기/점프 탈출 | 벽 쪽 400cm/s 윈치 → **로프 끝 정지(끌림 없음)** |
| `Rope.Test.TetherScene drag [kg=100]` | 전방 6m 물리 큐브 | 감기 → 걷기/되감기로 끌기 | 탄성 룩/서보 진동 → **질량비 분배·경계 유지**(500kg이면 내가 양보) |
| `Rope.Test.TetherScene ragdoll` | 배치된 랙돌 캐릭터를 전방 2.5m 소환 | 근접 wrap(자동 랙돌) → 유지/되감기 | 요요/관절 슬램 폭주 → **λ 단방향 = 폭주 없음, 전신 질량 끌림** |

A/B에 쓰던 모드 전환 커맨드(`Rope.Test.TetherMode`)는 F단계의 모드 제거와 함께 이미 삭제됐다.
관찰 지표는 그대로 유효하다: 게임플레이 디버거 pull 라인의 `constraint T=현재/상한`(상한 근접 노랑/
클램프 빨강), `chain`/`tether` 값, 그리고 `Rope.Ragdoll`(수동 랙돌 토글) 병용.

## 9. 후속 정정 (2026-07-22) — 프랍 무중력/휙·wielder 들썩임

A~F 완료 후 PIE에서 남은 3증상(물리 프랍이 무중력으로 따라옴 / 슬랙 전환과 함께 날아감 /
wielder 들썩임)의 정정. 원인과 수정:

1. **회수 상한 분리(`TetherMaxBiasSpeed`, 기본 150)**: λ의 위치 회수(bias) 항은 벌어짐 상쇄와 달리
   운동량으로 남고(단방향 제약 = 슬랙 전환 후 무제동 코스팅), 상한이 `TetherMaxSpeed`(1500) 재사용이라
   가벼운 대상이 한두 프레임에 15m/s로 가속된 뒤 그대로 날아갔다("휙"). §3.5의 "MaxBiasSpeed로
   유계라 장부 불필요" 논거는 상한 자체가 과대해서 성립하지 않았다 — 유계 ≠ 작음. 회수는 안착용
   150cm/s로 충분하고, `TetherMaxSpeed`는 인가 결과 속력의 2차 클램프 역할만 남는다.
2. **직교 감쇠 dt 보정(`TetherPerpDamping`)**: 프레임당 고정 비율이라 고프레임률일수록 감쇠가
   세지는 프레임률 의존 물리였다 — 설정값을 60fps 기준 비율로 정의해 1−(1−d)^(dt·60)로 적용한다.
   전 축 감쇠는 유지: 중력 낙하도 함께 깎여 평형 낙하가 g·프레임dt/비율(기본값·60fps에 ≈55cm/s)에
   갇히므로, 끌리는 대상이 너무 떠 보이면("무중력" 룩) 이 값을 낮춰 대응한다. (수직 성분 제외안과
   CMC 접지 수평 투영안은 구현 후 검토에서 되돌림 — Z 성분을 인위로 지우지 않는다.)
3. **팽팽 게이트 해제 유예(`TautReleaseGraceTime`, 기본 0.1s)**: 3중 게이트 중 최소 전달 장력은
   임계 0(기본)에서 진입/유지 임계가 동일해 히스테리시스가 소멸하고, GPU 로프의 SegmentTension은
   1~2프레임 지연 미러다 — 경계에서 taut↔slack이 프레임 단위로 퍼덕이며 "전량 속도 삭감 ↔ 자유"가
   교대(들썩임). 진입 즉시·해제만 시간 유예로 채터링을 끊는다(가짜 C 노출도 이 시간으로 유계).
4. ~~**스켈레탈 특례 축소(§3.4-1 정정의 정정)**: 끌 수 있는 자유 랙돌은 CL 545의 전신 균일 속도 가산
   경로로 λ 분배를 받게~~ → **8번(물리 제약의 시뮬 바디 전면 확장)으로 대체**: 제약이 대상을 실제로
   끌므로 "끌림 판정과 λ 분배(w=0)의 모순"이 근본 소멸 — 전신 가산 경로는 도입 당일 폐기.
6. **λ 관측 클로버 수정**: `UpdatePhysicalTether`가 실은 제약 실측 장력을 λ 발화 프레임의 대입이
   덮어쓰던 것을 Max 병합으로 — 랙돌 제약 장력이 디버거에서 사라지던 관측 버그.
7. **Anchor-kind 끝 속도 실측(1번의 동반 수정 — 데모 towing 회귀)**: 정적/키네마틱/애니메이션 구동
   대상은 `ResolveTetherEndpoint`가 Anchor로 분류하고 끝 속도를 0으로 잡는다 — §3.2의 "움직이는 앵커
   순항은 실측 vT에 실린다"는 물리 바디/CMC 끝에만 참이었고, 키네마틱 앵커의 towing은 사실 bias 회수
   항(구 상한 1500)이 가리고 있던 부수 효과였다. 1번이 상한을 150으로 내리자 앵커 이탈 속도 > 150에서
   C만 순증(줄 늘어남·견인 무력)하는 회귀가 드러남. 수정: 앵커 점(WorldPoint) 프레임 차분 + EMA
   (`SmoothedAnchorPointVelocity`, rest 변화율과 같은 앵커 노드 불변 가드)를 Anchor-kind 대상의 끝
   속도로 s에 실어 towing을 **벌어짐 상쇄 항**(bias 상한 무관, 2차 클램프 TetherMaxSpeed만 적용)으로
   복원 — 레거시 앵커 피드포워드(CL 428)의 λ 모델판. 정지 앵커는 ≈0이라 벽 하드 스톱 불변.
8. **물리 제약 테더의 시뮬 바디 전면 확장(§3.4-1의 일반화 — 공중 하중 3증상)**: PIE에서 매달린 프랍이
   ①부유(내려가야 할 상황에 둥둥) ②사방으로 날아가려는 잔류 힘 ③wielder 이동 간섭을 보임. 근본 =
   GT 프레임 속도 임펄스는 공중 하중의 동역학을 표현할 수 없다(랙돌 사가와 동일 결론): 중력·스윙이
   물리 서브스텝에서 진행되는 동안 GT는 한 박자 늦게 사후 상쇄만 하므로 부유(중력 상쇄 톱니)·진자
   펌핑(방향 EMA 래그의 접선 오차 주입)·직교 감쇠 의존(펌핑 억제 ↔ 부유 심화의 트레이드오프)이
   구조적이다. 수정: **시뮬 바디 대상 전부(스켈레탈 본 + 컴포넌트 바디)를 UpdatePhysicalTether로** —
   매달림 = Chaos가 서브스텝에서 중력과 함께 푸는 진짜 진자, 슬랙 = 자유 낙하, 지면 끌기 = 마찰과 함께
   솔브. λ는 wielder 절반만(대상 w=0, 랙돌과 같은 분업), TetherPerpDamping은 wielder 시뮬 루트 전용으로
   강등. 동반: 공중 스윙 중 wielder 방향 EMA 생략(순간 기하) — 궤도 회전을 EMA(ω·τ ≈ 십수도 래그)가
   못 따라와 접선 오차가 매 발화 프레임 스윙을 제동/가속, AirControl 부스트를 무력화하던 "특정 순간의
   방해력" 제거(접지는 코너 노이즈 때문에 EMA 유지, 상태는 계속 시드해 착지 전환 연속).
9. **컴포넌트 바디 제약 = 투영 off + 소프트 리밋(8번의 후속 — 스태틱 메시 당김 고주파 진동)**: 8번의
   확장 직후 PIE에서 프랍 당김 폭주가 10배 악화 — 단일 강체에 하드 리밋 + 엔진 기본 위치 투영
   (projection)을 걸면, 코너 추종 프록시가 매 프레임 이동하며 리밋을 어길 때마다 위반량이 위치 스냅으로
   닫히고, 강체는 관절 완충이 없어 스냅→물리 반동→재위반이 프레임 주기로 반복된다(랙돌이 조용했던
   이유 = 관절 사슬이 흡수). 수정: 컴포넌트 바디 한정 bEnableProjection=false + 소프트 리밋
   (`PhysicalTetherStiffness` 1000 / `PhysicalTetherDamping` 100, restitution 0 — **PIE 튜닝 대상 노브**,
   강성 0 = 하드 복귀). 스켈레탈은 PIE 검증된 하드 리밋 + 기본 투영 유지. 동반: 바디-로컬 앵커 드리프트
   가드 — 제약 프레임(Frame2)은 생성 시 고정이라 같은 (대상,본) 안에서 wrap 앵커가 재배치되면 상시
   위반=진동이 되므로, 5cm 초과 드리프트 시 해체 후 즉시 재생성.
10. **탄성/장력 단일 material solve(2026-07-23, 9번 대체)**: `TetherCompliance=0`은 Pawn의
    PrePhysics hard projection과 물리 대상의 hard Chaos limit가 material length를 보존하고, 그 제약이
    거부한 속도/Chaos force가 곧 authoritative tension이다. `TetherCompliance>0`은 별도 Chaos spring을
    만들지 않고 모든 endpoint가 §3.3의 implicit Kelvin-Voigt λ를 공유한다. 물리 endpoint는 §3.4의
    attachment-point Jacobian/impulse를 사용한다. 따라서 `PhysicalTetherStiffness/Damping`은 기존
    asset 역직렬화용 deprecated 값일 뿐이며, 9번의 soft-limit 튜닝 경로는 더 이상 실행되지 않는다.
    비신축 SimBody의 위치 위반은 Wielder/target generalized-mass 몫으로 나누고 Chaos proxy를 보정 후
    **실제 hand point**에 둔다. 손을 경계까지 전량 투영한 뒤 target도 움직이는 이중 보정은 실제 span을
    material length보다 짧게 만들어 다음 프레임 장력이 끊기는 원인이므로 금지한다.
