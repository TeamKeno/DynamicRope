# Convex Edge Via-Point (박스 볼록 엣지 감김)

## 문제

정적 박스(`RopeStaticBodyProvider`)를 로프가 걸칠 때, 볼록 90° 모서리에서 노드 하나가 **면에 붙은 채 90°로 각지고** 그 다음에 캐릭터로 이어진다. 깔끔한 135°(엣지에서 한 번 꺾임)가 안 나온다.

원인(마찰 아님 — `Friction=0`에서도 재현):

- 박스 점질의는 노드를 **가장 가까운 표면점**으로 밀어낸다. 코너 근처지만 아직 면 범위 안(엣지를 안 지난) 노드는 **면으로 투영**된다(정상 nearest-surface 동작).
- 노드가 **대각(엣지) 법선**을 받으려면 코너를 대각선으로 지나쳐 있어야 하는데, 유한 노드 간격 + 장력 기하상 **엣지에 정확히 놓이는 노드가 없다**.
- 기존 세그먼트-chord 충돌은 세그먼트가 박스를 **관통할 때만** 밀어낸다. 볼록 엣지에서 자유단 세그먼트는 코너를 비켜 지나가 관통이 없어 보정이 안 걸린다.
- 즉 로프를 **엣지로 당겨 감는 힘**이 없어 면에 눌어붙는다.

## 모델

팽팽한 로프는 자유 공간 최단 경로(geodesic)를 따르고, 볼록 박스 위에서는 **볼록 엣지를 스치며(via-point)** 그 사이는 직선이다. 이 "엣지에 감김"을 XPBD 노드에 위치 제약으로 추가한다: **로프의 bend 정점이 볼록 엣지를 감싸는 상황이면 그 노드를 엣지에 닿게 당긴다.**

## 알고리즘 (`RopeSolveNodeBoxEdges` / CPU `SolveBoxEdgeViaPoints`)

노드 `i`(이웃 `a=P[i-1]`, `p=P[i]`, `b=P[i+1]`), 박스(Center/Rot/HalfExtents `H`), `R=CollisionRadius`:

1. **로컬 변환**: `Lp = Rot^-1(p-Center)`, `La`, `Lb`. 노드가 박스 안이면 스킵(면 충돌 담당).
2. **최근접 볼록 엣지**(해석적, 12개 루프 불필요): `n[axis]=|Lp[axis]|/H[axis]`. 자유축 `f = argmin(n)`, 나머지 두 축 `g,h`를 `sign(Lp)*H`로 고정. 엣지점 `Ec_local`(자유축=`clamp(Lp[f])`, 고정축=경계), 외향 대각 법선 `m_local = normalize(e_g*sign(Lp[g]) + e_h*sign(Lp[h]))`.
3. **wrap 판정**(taut-string): chord `La→Lb`의 `Ec`에 대한 최근접점 `Cc`, `s = dot(Cc-Ec_local, m_local)`. `s < 0`(chord가 엣지 내부측 통과 = 직선이면 박스 관통) → **활성**. `s ≥ 0`(로프가 엣지 안 감음) → 비활성. 양 이웃 chord 기반이라 노드 노이즈에 강함.
4. **근접 게이트**: `edgeLineDist=|Lp-Ec_local|`가 `(R, R+EdgeAttractBand)` 이내일 때만(닿아있으면 안 당기고, 멀면 안 끌어옴).
5. **보정**(엣지에 감기게): `delta = EdgeSnapStrength*(edgeLineDist-R)*normalize(Ec_world-p)`, `|delta|`는 `0.5*SegmentLength`로 클램프(안정). `gPos[i]+=delta; gPrev[i]+=delta`(속도 중립 — 매 substep 보정이 속도 주입해 튀는 것 방지).

결과: bend 정점이 엣지에 앉아 로프가 엣지에서 한 번 깔끔히 꺾인다(면 붙는 90° → 엣지 135°).

## 통합

- **GPU**(`RopeXPBD.usf`): 노드 충돌 스테이지 배리어 **뒤**, 세그먼트-chord와 같은 **2색(i%2) 스테이지**로 실행(노드 `i`가 이웃 `i±1`을 읽고 자기 `i`를 써 인접 노드와 race → 2색+배리어로 방지). `i∈[1,N-2]`, `P.NumBoxes>0`, `P.bBoxEdgeViaPoints`.
- **CPU**(`FRopeXPBDSolver`): iteration 루프에서 `SolveContacts` 뒤에 `SolveBoxEdgeViaPoints`(순차라 race 없음). 박스 지오메트리는 `IRopeCollider::GetGPUBox`로 취득.
- **config**(`FRopeSolverConfig`): `bBoxEdgeViaPoints`(기본 on), `EdgeAttractBand`(cm), `EdgeSnapStrength`(0..1). GPU 파라미터 struct의 `Pad5/6/7` 슬롯 재사용(정렬 불변).
- **테스트**: `GPUBoxEdgeViaPoint` 신규 — 박스 엣지에 로프 걸쳐 수렴 후 경계 노드가 엣지선에서 ≈`R`에 앉는지 단언. 기존 `GPUBoxCornerParity`는 CPU/GPU 미러가 같아 유지.

## 범위 / 후속

- **박스(OBB) 먼저**(사용자 케이스). 해석적 최근접 엣지가 깔끔.
- **컨벡스는 후속**: 임의 평면교차라 엣지 열거가 복잡(평면 쌍 교선). 박스 검증 후 일반화.
- GDF/캡슐 비대상(둥근/암시적 표면엔 날카로운 엣지 없음).

## 리스크 / 완화

- **과당김**: wrap 조건(chord 막힘) + 근접 밴드 + strength로 스코프.
- **떨림**: chord 기반 판정 + 속도 중립 + delta 클램프.
- **파리티**: CPU/GPU 동일 기하 → `GPUBoxCornerParity` 유지 목표(수렴 tolerance 내).
