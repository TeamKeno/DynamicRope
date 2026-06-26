# GPU 컴퓨트 솔버 / 렌더 — 작업 흐름 정리 (Tier 1 ~ M5)

DynamicRope 시뮬레이션을 CPU 단일 → 중앙 구동 → 병렬 → **GPU 상주**로 옮긴 작업의 전체 흐름.
"흐름 놓친 부분" 복기를 위한 지도. 코드보다 *왜/무엇을/어디서*에 초점.

---

## 0. 로드맵 한눈에 (Perforce CL)

| 단계 | 내용 | CL |
|---|---|---|
| Tier 1 | `RopeSimSubsystem` — 컴포넌트 self-tick 폐지, 중앙 구동 | 43 |
| Tier 1+ | provider collider gather 프레임당 1회 디둡(`BuiltFrame`) | 44 |
| Tier 2 | 프레임 3분할 `Prepare(GT)/Solve(ParallelFor)/Finalize(GT)` | 46 |
| 준비 | 로그 카테고리 + UE_LOG 정비 | 48 |
| **M1** | GPU 컴퓨트 솔버(물리 전용: integrate+distance+bending), 셰이더 배선 | 49 |
| **M2** | GPU 캡슐 충돌(push-out+friction+swept) | 50 |
| **M3** | GPU per-bone SDF 충돌(StructuredBuffer + trilinear) | 51 |
| **M4** | async 리드백 — **폐기**(진동). 단독 서브밋 없음, M5a로 진화 | — |
| **M5a** | 센터라인 GPU **상주**(매 프레임 in-place 전진) — 진동 해결 | 55 |
| **M5b** | GPU 직접 튜브 렌더(resident PosBuf 직독) — 위치 무지연 | 56 |
| 정리 | collider 수집 `RopeSimSubsystem` 중앙화 | 57 |

> 사이사이 다른 작업자 CL(45 throw, 52 whip, 53/54 SDF 에디터 등)이 끼어 있어 머지/resolve가 잦았음.

---

## 1. 가장 중요한 그림: **독립된 두 토글**

GPU화는 *시뮬레이션*과 *렌더*가 **별개 스위치**다. 직교한다.

| CVar | 0 (기본) | 1 |
|---|---|---|
| `r.DynamicRope.GPUSolver` | CPU `ParallelFor` 솔브 | GPU 상주 컴퓨트 솔브 |
| `r.DynamicRope.GPUTube` | CPU `BuildTube`(렌더 스레드) | GPU 컴퓨트로 튜브 정점 생성 |

- 둘 다 0 = **원본 CPU 경로와 바이트 동일**(추가분은 `SimGeneration++` 같은 무해 코드뿐).
- **위치 무지연 GPU = 둘 다 1**. (Tube만 1이면 CPU 미러를 읽어 여전히 지연; Solver만 1이면 시뮬은 GPU지만 렌더는 CPU 미러)
- CPU 솔버는 **ground-truth**로 영구 유지(패리티 테스트 기준).

---

## 2. 시뮬레이션 경로 (`GPUSolver`)

### phase machine (불변)
```
Free → Flight → Contacting → Wrapped → Releasing → Free
└─ 물리(solver) ─┘ └──── 로직(wrap controller, CPU/UObject) ────┘
```
GPU 상주는 **물리 phase(Free/Flight)만**, 그것도 **whip 아닐 때만**. 나머지(whip/Contacting/Wrapped/Releasing)는 CPU.

### `URopeSimSubsystem::Tick` 프레임 구조
```
Phase 0   GPU GetLatest   : 지난 프레임 RT 리드백 결과를 GpuLatest 캐시로 회수
Phase 1a  Collider 수집   : provider 레지스트리에서 1회 빌드 → 로프별 FrameColliders (CL57)
Phase 1b  Prepare(GT)     : init/pin 전진/whip/로직 phase
Phase 2   Solve           : GPU면 resident step(whip 로프는 CPU 폴백) / 아니면 ParallelFor CPU
Phase 3   Finalize(GT)    : Flight 접촉 감지·캡처 + 렌더 dirty
```

### 핵심 원리 (M5a)
로프 솔브는 **순차적**(N+1은 N의 *풀린* 결과 필요). 그래서 위치 버퍼(`PosBuf`)를 **GPU에 상주**시켜
매 프레임 **in-place로 전진**한다 → 순차 의존성이 GPU 안에서 충족, CPU 왕복 불필요.
- 재시드 트리거: `URopeComponent::SimGeneration`(init/throw/logic phase/whip에서 증가). 정상 Free/Flight(비-whip)엔 불변 → 상주 유지.
- CPU `Sim`은 렌더/충돌용 **지연 미러**(RT 리드백을 GT로 회수, ~1~2프레임). node 0은 현재 핀으로 덮어써 잡은 끝이 손을 정확히 추종.
- 리드백 Lock/consume/재무장은 **전부 렌더 스레드**에서(매 프레임 step 커맨드가 직전 리드백 처리) → GT 스톨 없음.

---

## 3. 렌더 경로 (`GPUTube`, M5b)

튜브 컴퓨트(`Shaders/Private/RopeBuildTube.usf`)가 센터라인 → parallel-transport frame(ring 간 순차라 thread 0가
groupshared에 직렬 전파) → 정점 위치를 **UAV vertex buffer**(`FLocalVertexFactory` position stream)에 기록.

입력 센터라인 소스 2가지:
- **resident**(엔트리 `RopeBuildTubeResidentCS`): 솔버 `PosBuf`(월드, `StructuredBuffer<float4>`) 직독 + `WorldToLocal` → **위치 무지연**. **`bGpuSteppedThisFrame`일 때만**.
- **폴백 B1**(엔트리 `RopeBuildTubeCS`): CPU 센터라인 미러(`Data.Points`, local). whip/throw/CPU-솔버/스폰 직후.

> `bGpuSteppedThisFrame`(서브시스템이 매 프레임 설정 → `FRopeDynamicData`로 전달)이 **throw 렌더 버그의 핵심 게이트**:
> whip 중엔 PosBuf가 stale이라 그 프레임은 반드시 CPU 미러로 그려야 한다.

tangent/UV/color는 아직 **CPU**(미세한 노멀 지연만 남음) → B2-full에서 GPU로.

---

## 4. M4 → M5 피벗 (가장 비싼 교훈)

**M4(async 리드백)는 폐기됐다.** `FRHIGPUBufferReadback::Lock`이 **렌더 스레드 전용**이라 디스패치→ready→락→회수에
~3프레임 라운드트립. 순차 솔버가 그 동안 진도를 못 나가 **실효 ~20fps 슬로모 + 핀 스냅 → 손에 든 로프 이동 시 큰 진동**.

→ 결론: **순차 솔버 + 프레임 간 CPU 왕복 = 부적합.** 해법은 상태를 GPU에 두고(M5a) 거기서 매 프레임 전진.
async가 맞는 경우는 "처리량(많은 로프) + 지연 허용"이지, 손에 붙은 단일 로프의 매 프레임 추종이 아니다.

---

## 5. 어디에 뭐가 있나 (파일 지도)

- `Source/DynamicRopeShaders/`(PostConfigInit 모듈, .usf 가상경로 매핑 소유)
  - `RopeGPUSolver.h/.cpp` — 상주 솔버: `Step`(매 프레임 in-place) / `GetLatest`(미러 회수) / `GetResidentPositionSRV_RenderThread`(M5b 렌더용) / `ReleaseRope`. RT 상태는 pimpl.
  - `RopeTubeBuilder.h/.cpp` — 튜브 컴퓨트 디스패치 헬퍼(opaque RHI view, 타입 결합 없음).
- `Shaders/Private/RopeXPBD.usf` — 솔버 커널(로프=스레드그룹, red-black/stride-3 컬러링, capsule/SDF 충돌).
- `Shaders/Private/RopeBuildTube.usf` — 튜브 생성 커널(엔트리 2: B1/resident).
- `Source/DynamicRope/`
  - `Subsystem/RopeSimSubsystem` — 중앙 구동 + GPU 솔버 소유 + **collider 레지스트리/중앙 수집(CL57)**.
  - `RopeComponent` — Facade, phase machine, `SimGeneration`/`bGpuSteppedThisFrame`.
  - `Render/RopeSceneProxy` — 튜브 렌더(CPU `BuildTube` / GPU `BuildTubeGPU`), UAV position buffer 바인딩.
  - `Collision/` — `IRopeCollider`(FROZEN contract), provider(캡슐/SDF), GPU 추출자(`GetGPUCapsule`/`GetGPUSDF`).

---

## 6. 남은 작업

- **B2-full**: tangent도 GPU 생성(packed normal 컴퓨트) → 완전 무지연 + CPU `BuildTube`/렌더 리드백 완전 제거.
- **M5c**: DecideWrap용 contact만 작은 async 리드백 + wrap 핸드오프 시 GPU→CPU 위치 정밀 동기.
- (선택) 대량 로프 throughput: per-rope dispatch → 슬랩 할당 단일 dispatch.

---

## 7. 함정 메모

- **Perforce 비유니코드 서버** → CL 설명은 ASCII 영어(한글은 P4V에서 mojibake). 코드 주석 한글은 파일을 `utf8+C`로.
- **CRLF 강제**(.cpp/.h). md는 LF.
- `FRHIGPUBufferReadback::Lock`/RDG `CreateUAV` 등 **렌더 스레드 전용** API 주의.
- **UAV 버퍼는 `Dynamic` 불가**(lock 시 orphan → 영속 SRV 무효). 영속+컴퓨트write는 non-dynamic.
- 셰이더 가상경로: 파일 `Shaders/Private/X.usf` → `/Plugin/DynamicRope/Private/X.usf`.
