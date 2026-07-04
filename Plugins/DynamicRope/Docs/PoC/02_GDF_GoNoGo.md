# PoC 설계 노트 ② — GDF(Global Distance Field) 도입 선행 파악 결과

> 상태: 조사 완료 (2일 타임박스 스파이크) · 대상: DynamicRope / DynamicRopeShaders
> 목적: 엔진 Global Distance Field를 GPU 로프 솔버의 3급 collider(월드 정적 지오메트리 밀어내기)로
> 붙일 수 있는지 Go/No-Go를 위해, 검증용 "일회용 스파이크"로 두 가지를 확인했다.
> 참고: Notion "🌐 5. GDF 도입 — 조사 결과와 Go/No-Go 판단 자료".
> 스파이크 코드(뷰 확장 + 더미 컴퓨트)는 검증 후 제거했고, 이 문서가 그 결과와 재현법을 남긴다.

---

## 0. 한 줄 결론

**엔진 GDF는 public API로 뷰 확장에서 샘플 가능함이 확인됐다(PoC 통과). 다만 결정적 제약이 하나 드러났다:
GDF는 "소비자가 필요하다고 플래그할 때만" 빌드되며, 우리처럼 읽기만 하는 코드는 빌드를 유발하지 않는다.**
→ 정식 통합 시 **로프 솔버가 GDF 소비자로 등록**되도록 만드는 작업이 추가로 필요하다(리스크 항목).

---

## 1. 무엇을 검증했나 (2 PoC)

CVar 게이트(`r.DynamicRope.GDFSpike*`)로 켜지는 `FSceneViewExtensionBase` 하나를 만들어,
`PrePostProcessPass_RenderThread`(opaque 이후·GDF 유효·RDG 열림)에서 두 실험을 수행했다.

### PoC(1) — GDF 파라미터 획득 + 더미 컴퓨트 샘플
- `UE::FXRenderingUtils::GetGlobalDistanceFieldParameterData(TConstStridedView<FSceneView>)`로 파라미터를 얻고,
  `FGlobalDistanceFieldParameters2`를 바인딩한 컴퓨트에서 `GetDistanceToNearestSurfaceGlobal(TranslatedWorld)`를
  샘플해 리드백 → 로그로 거리값을 찍었다.
- **결과: 통과.** 셋업(§3)만 갖추면 프로브별로 물리적으로 타당한 부호 있는 거리값이 나온다.

### PoC(2) — resident 버퍼가 씬 렌더러 그래프에서도 유지되는가
- M5a 상주 패턴(`ConvertToExternalBuffer` → 다음 프레임 `RegisterExternalBuffer`)을 씬 렌더러가 넘겨준
  `FRDGBuilder`에서 구동해, in-place 카운터가 프레임마다 단조 증가하는지 확인했다.
- **결과: 통과.** pooled 버퍼는 그래프 비종속이라 씬 렌더러의 `FRDGBuilder`에서 register/dispatch가 정상 동작했다.
  → 솔버 dispatch를 뷰 확장으로 옮겨도 기존 상주 버퍼 설계는 깨지지 않는다.

---

## 2. 핵심 발견 — GDF는 "소비자"가 있어야 빌드된다

처음엔 모든 샘플이 `distance=0.00`으로만 나왔다. `GetDistanceToNearestSurfaceGlobal`가 **정확히 0.00**을
돌려주는 경로는 하나뿐이다: 내부 루프 `for (i < NumGlobalSDFClipmaps)`가 **한 번도 안 도는 경우**,
즉 **클립맵 0개**. 진단 로그를 추가해 확인하니 `NumClipmaps=0, PageAtlas=null` — GDF 자체가 씬에서 빌드되지 않았다.

엔진의 빌드 게이트는 `ShouldPrepareGlobalDistanceField()`
(`Engine/Source/Runtime/Renderer/Private/DistanceFieldAmbientOcclusion.cpp`):

- **전제(둘 다 필수)**: `r.GenerateMeshDistanceFields=True` + `r.AOGlobalDistanceField != 0`(기본 1).
- **그리고 아래 소비자 조건 중 최소 하나**가 참이어야 실제로 빌드된다:
  1. **Niagara**가 GDF 요구 (`FXSystem->UsesGlobalDistanceField()`)
  2. 뷰의 **머티리얼**이 GDF 샘플 (DistanceToNearestSurface 노드 → 관련성 플래그, **1프레임 지연**)
  3. **Distance Field AO** 활성
  4. **Lumen** — 단, **소프트웨어 트레이싱 모드일 때만** (`Lumen::UseGlobalSDFObjectGrid`)
  5. MegaLights
  6. **Visualize → Global Distance Field** 쇼플래그

### 함정 두 가지 (우리가 실제로 밟은 것)
- **Lumen을 켰다고 클래식 GDF가 빌드되지 않는다.** Lumen이 하드웨어 레이트레이싱이면
  `UseGlobalSDFObjectGrid`가 false → 페이지 아틀라스가 안 채워진다. (프로젝트가 Lumen인데 `PageAtlas=null`이던 원인.)
- **`r.AOGlobalDistanceField 1`은 전제일 뿐 강제 스위치가 아니다.** 켜도 소비자 조건이 없으면 빌드 안 됨.

---

## 3. GDF를 실제로 빌드/검증하는 법

테스트로 GDF를 강제하는 가장 확실한 방법:

- **에디터 뷰포트: Lit 드롭다운 → Visualize → Global Distance Field** (또는 `ShowFlag.VisualizeGlobalDistanceField 1`).
  빌드를 강제하면서 **필드 존재를 눈으로 확인**까지 된다. (게이트 조건 6)
- **오버레이 없이**: `r.AOGlobalDistanceField.DetailedNecessityCheck 0` (+ `r.AOGlobalDistanceField 1`).
  데스크톱 디퍼드에서 매 프레임 GDF를 상시 빌드한다(전역 성능 비용 감수).
- 또는 씬에 GDF 샘플 머티리얼 배치(1프레임 지연), 적절한 Niagara 시스템, DFAO 활성 중 택1.

전제(mesh DF 생성)는 프로젝트 설정에서 켠 뒤 **에디터 재시작 + 셰이더 재컴파일**이 필요하다.

---

## 4. 엔진 API 레퍼런스 (재현용)

정식 통합/재스파이크 시 그대로 재사용:

- 파라미터 획득: `UE::FXRenderingUtils::GetGlobalDistanceFieldParameterData(Views)` → `const FGlobalDistanceFieldParameterData*`
  (null 가능 — 반드시 체크). `Renderer/Public/FXRenderingUtils.h`.
- 셰이더 파라미터: CS `FParameters`에 `SHADER_PARAMETER_STRUCT_INCLUDE(FGlobalDistanceFieldParameters2, ...)`
  (prefix 없이 loose 전역에 바인딩 — Niagara와 동일). `Renderer/Public/GlobalDistanceFieldParameters.h`.
- 파라미터 채우기: `SetupGlobalDistanceFieldParameters_Minimal(Data)`는 **CoverageAtlasTexture와 샘플러 3개를 빠뜨린다.**
  호출 후 CoverageAtlas 텍스처(+ null이면 `GBlackVolumeTexture`)와 샘플러 3개(PageAtlas/Coverage=Trilinear/Wrap,
  Mip=Trilinear/Clamp)를 직접 세팅해야 한다. (미세팅 시 검은 텍스처 샘플 → 무의미한 값.)
  참고 원본: `NiagaraDistanceFieldHelper::SetGlobalDistanceFieldParameters`.
- 셰이더 함수: `/Engine/Private/DistanceField/GlobalDistanceFieldShared.ush`
  — `GetDistanceToNearestSurfaceGlobal`, `GetDistanceFieldGradientGlobal`.
  좌표는 **TranslatedWorld(=World + View.PreViewTranslation)**. 비-머티리얼 글로벌 CS는
  `DISTANCE_FIELD_IN_VIEW_UB` off이므로 loose `Global*` 파라미터가 위 struct에서 바인딩된다.
- 뷰 확장 훅: `PrePostProcessPass_RenderThread`(opaque 이후·GDF 유효·RDG 열림). `PreRenderView_RenderThread`는 너무 이르다.
- 모듈 의존성: `DynamicRopeShaders.Build.cs`에 `Renderer`(FXRenderingUtils/GDF 파라미터) + `Engine`(SceneViewExtension) 추가 필요.
- `distance=0.00`의 의미: 부호 있는 표면 거리(음수=내부, ≈0=표면, 양수=자유공간, 먼 곳은 influence range로 포화).
  **전부 0.00이면 "클립맵 0개 = GDF 미빌드"** 진단.

---

## 5. Go/No-Go 판단과 리스크

- **기술 타당성**: PoC(1)(GDF 샘플)·PoC(2)(상주 버퍼) 모두 통과 → "GDF 밀어내기(충돌 전용) 1차"는 기술적으로 가능.
- **새로 확인된 통합 리스크 — GDF 빌드 강제/소비자 등록**:
  읽기만으로는 GDF가 안 만들어진다. 정식 통합은 다음 중 하나로 GDF 빌드를 보장해야 한다:
  - (a) 쇼플래그/`DetailedNecessityCheck 0`로 상시 강제 — **전역 성능 비용**, 프로젝트 전체에 영향.
  - (b) 씬에 GDF 샘플 머티리얼 상주 — 콘텐츠 규약 강제, 취약.
  - (c) **Niagara식 FXSystem 소비자 등록 복제** — 가장 정합적이나, **임의 뷰 확장용 public API가 없어** 엔진 내부 메커니즘을 모사해야 함(작업량/유지보수 리스크).
- **기존 문서의 리스크는 유효**: 해상도(clipmap voxel이 로프 CollisionRadius보다 큼 → 얇은 지오메트리 터널링),
  SurfaceVelocity 없음(정적 스냅샷), wrap 불가(본 귀속 없음) → **GDF는 per-bone SDF의 대체가 아니라
  벽/바닥 등 정적 월드 광역 충돌 보완재**.

### 권고
GDF는 "정적 월드 밀어내기 보완재"로 한정하고 도입하되, **소비자 등록 방식(위 c 우선, 불가 시 a)**을 먼저
설계 항목으로 못 박아야 한다. 이 항목이 해결되지 않으면 씬 상황에 따라 GDF 충돌이 무력화될 수 있으므로,
마일스톤 포함 시 "GDF 빌드 보장" 작업을 반드시 범위에 넣을 것.
