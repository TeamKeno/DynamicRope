# 05. Wrapped 테더의 λ 제약 재편 (설계)

작성: 2026-07-20. 상태: **설계 초안 — 리뷰 대기**.
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
쓴다. 이러면 "sub-leg overshoot가 슬랙 로프에서 발화"하던 문제(팽팽 게이트 3종을 낳은 원인)가
정의상 사라진다: 처짐/구김 = chord 합 < rest = C < 0 = λ = 0. **C ≤ 0이면 이번 프레임 무동작**
— 이것이 슬랙 게이트의 전부다.

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

```
β      = 1 − exp(−dt / TetherSettleTime)     // 위치 오차 회수 게인(프레임률 독립)
s*     = −β · C / dt                          // 목표: 이번 프레임 C의 β 비율 회수
Δs     = s* − s                               // 필요한 접근 속도 변화(음수여야 당김)
λ      = clamp( −Δs / (w_t + w_w + α/dt²),  0,  λ_max )
λ_max  = MaxTetherTension · dt                // 장력 상한(임펄스로 환산)
α      = TetherCompliance                     // 0 = 비신축(기본), > 0 = 의도적 탄성
```

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

1. **풀 랙돌**: 유효질량 = **전신 바디 질량 합**(현행: 감긴 본 바디 질량 — 팔 3kg으로 오판해
   분배/pullable을 오염시키던 것). 인가 = `SetAllPhysicsLinearVelocity(bAddToCurrent=true)`
   전체 평행이동(BinaryPullable이 이미 쓰는 방식 — 상대 속도 보존, 관절 다이내믹 유지).
   **MassShare의 단일 본 슬램(ServoVelocity)은 폐기**한다(관절 에너지 펌핑 = 폭주 입력).
2. **부분 랙돌(시뮬 본이 키네마틱 체인에 묶임)**: rung 1에서 **Character로 폴스루**한다
   (유효질량 = CMC 질량 × 브레이스, 인가 = CMC). 감긴 본에는 시각 반응용 소량 임펄스만 옵션.
   → `ResolveTetherEndpoint` rung 1 주석의 "알려진 한계"(2026-07-15 보류)를 이 설계로 해소.
3. **CMC**: `Movement->Velocity += Δv_w` 직접(이번 프레임 반영 계약 유지). 접지 브레이스
   (GroundBraceFactor)는 유효질량으로 계속 표현. 접지 마찰이 다음 틱에 ΔV를 깎는 것은
   "버티는 발"의 정당한 물리로 취급한다(λ가 다음 프레임 C로 다시 관측 — 정상 수렴 루프).
4. **컴포넌트 시뮬 바디**: `AddImpulse(bVelChange)`. 위치 스윕 이동은 v1에서 제거(β·C/dt가
   위치 오차를 닫는다). PIE에서 관통/드리프트가 보이면 위치 패스를 후속 추가.
5. **앵커**: w=0, 인가 없음(현행 위치 오프셋 폴백 삭제 — 앵커는 정의상 안 움직인다).
6. **자기 랩(owner == 대상)**: 양끝이 같은 몸 — 현행대로 대상 몫 전량 특례 유지.
7. **2차 방어 유지**: 인가 결과 속력을 `ClampInjectedVelocity`(TetherMaxSpeed)로 클램프 —
   **모든 경로에**(현행은 랙돌 add 경로에만 없어 폭주 구멍이었다). `TetherPerpDamping`은
   물리 바디 한정으로 유지(방향 급전환 잔여 관성).

### 3.5 무엇이 필요 없어지는가

| 제거 대상 | 근거 |
|---|---|
| 팽팽 게이트 3종의 **테더 게이트 역할** (TautSlackRatio/TautMinTension/TautMaxSag) | C가 전 체인 chord 합 기반 — 슬랙이면 C<0=무동작. 게이트는 **능동 Pull 전용**으로 강등(IsPullTaut API 유지) |
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
| **도르래** (턱에 걸린 줄, 낙하 wielder ↔ 대상) | 코너 다리 방향 d_t/d_w가 각자 계산되고 같은 λ가 양끝에 걸림 = 이상 도르래 그 자체. wielder 낙하가 C를 늘리고 λ가 질량비대로 대상을 끌어올린다. **별도 구현 없음** — 현행 구조에선 표현 불가였던 것 |
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

고정 .umap 대신 **코드 스폰 리그**(`Debug/RopeTetherTestScenes.cpp`, 개발 빌드 전용)로 재현한다 —
텍스트라 리뷰/머지가 되고, 플레이어 기준 상대 배치라 각자의 `Lvl_*Test` 맵에서 그대로 돈다.

| 커맨드 | 씬 | 재현 절차 | 기대(레거시 → Constraint) |
|---|---|---|---|
| `Rope.Test.TetherScene wall` | 전방 4m 기둥(지름 60cm) | 감기 → 뒤로 걷기/점프 탈출 | 벽 쪽 400cm/s 윈치 → **로프 끝 정지(끌림 없음)** |
| `Rope.Test.TetherScene drag [kg=100]` | 전방 6m 물리 큐브 | 감기 → 걷기/되감기로 끌기 | 탄성 룩/서보 진동 → **질량비 분배·경계 유지**(500kg이면 내가 양보) |
| `Rope.Test.TetherScene ragdoll` | 배치된 랙돌 캐릭터를 전방 2.5m 소환 | 근접 wrap(자동 랙돌) → 유지/되감기 | 요요/관절 슬램 폭주 → **λ 단방향 = 폭주 없음, 전신 질량 끌림** |

모드 전환: `Rope.Test.TetherMode <mass|binary|constraint>` (월드 내 전 로프, 런타임 한정) — 같은 씬을
번갈아 A/B. 관찰: 게임플레이 디버거 pull 라인의 `constraint T=현재/상한`(상한 근접 노랑/클램프 빨강),
`chain`/`tether` 값, 그리고 `Rope.Ragdoll`(수동 랙돌 토글) 병용. 데모 3종(입체기동/도르래/드래곤)
스모크는 기존 데모 맵에서 `Rope.Test.TetherMode constraint`로 전환해 확인한다.
