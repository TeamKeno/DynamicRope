# 03. GDF 월드 충돌 스파이크 — 실측 결과 (2026-07-03)

`02_GDF_GoNoGo.md`의 Go/NoGo 판단 뒤, "일단 넣어보고 느낌부터 본다"로 GDF 월드 충돌을
스파이크 구현해 PIE에서 실측했다. **느낌은 확인(정적 벽/바닥 밀어내기 동작 OK)**. 스파이크 코드는
프로덕션에 남기지 않고 revert했고, 이 문서가 그 기록이자 재구현 레시피다.

> 결론 요약: 충돌 커널은 예상대로 간단(~40줄)했고 느낌도 났다. 하지만 **정식 도입을 막는 두
> 구조적 문제(뷰 확장 이전, GDF 빌드 보장)는 스파이크가 우회했을 뿐 풀지 않았다.** GDF는 여전히
> per-bone SDF 대체가 아니라 정적 월드 광역 보완재. 실제 채택은 별도 마일스톤에서 두 blocker를
> 먼저 설계 항목으로 못 박고 진행할 것.

## 무엇을 만들었나 (스파이크 구조)

기존 GPU 솔버의 독립 dispatch(`FRopeGPUSolver::Step`, 서브시스템 Tick 트리거)는 **건드리지 않고**,
별도 뷰 확장이 씬 렌더러 그래프 안에서 push-out 패스를 얹는 방식으로 우회했다.

- `FRopeGDFViewExtension : FSceneViewExtensionBase` — `PrePostProcessPass_RenderThread`에서:
  1. `UE::FXRenderingUtils::GetGlobalDistanceFieldParameterData(Views)`로 이 뷰의 GDF 파라미터 획득
     (null이거나 `NumGlobalSDFClipmaps<=0`이면 미빌드 → 조용히 skip).
  2. `FRopeGPUSolver::ForEachSteppedRope_RenderThread`로 **이번 프레임 실제 적분한** 상주 로프별
     (Pos/Prev/InvMass RDG 버퍼 + 노드수 + CollisionRadius)를 받아, 로프당 push-out 컴퓨트 dispatch.
- `RopeGDFCollision.usf` (`RopeGDFCollisionCS`, numthreads 256 = 로프 1개/스레드그룹):
  노드 월드 위치를 TranslatedWorld(`World + PreViewTranslation`)로 옮겨 `GetDistanceToNearestSurfaceGlobal`
  질의, `Dist < CollisionRadius`면 `GetDistanceFieldGradientGlobal` 방향으로 `(Radius-Dist)`만큼 밀어냄.
  핀/latch 노드(`InvMass<=0`)는 제외, 접선 성분 절반 감쇠(간이 마찰). 기존 SDF push-out을 월드공간
  미러한 것 — 커널 로직 자체는 ~40줄.
- 게이트: `r.DynamicRope.GDFSpike`(0/1, 기본 0) + `r.DynamicRope.GDFSpikeRadiusScale`(밴드 배율, 터널링 회피).
- 뷰 확장은 `URopeSimSubsystem::Initialize`에서 생성(렌더 가능 시), `Deinitialize`에서 flush 후 reset.

### 검증된 엔진 API (UE 5.7, 재구현 시 그대로)

- `Renderer/Public/FXRenderingUtils.h` — `UE::FXRenderingUtils::GetGlobalDistanceFieldParameterData(TConstStridedView<FSceneView>)`.
  단일 뷰는 `MakeStridedView(sizeof(FSceneView), &InView, 1)`로 만든다.
- `Renderer/Public/GlobalDistanceFieldParameters.h` — `FGlobalDistanceFieldParameters2`(셰이더 파라미터
  구조체) + `SetupGlobalDistanceFieldParameters_Minimal(Data)`(inline, 링크 가능).
  **주의: 전체 `SetupGlobalDistanceFieldParameters`는 `RENDERER_API`가 아니라 링크 불가** → `_Minimal`을
  쓰고 CoverageAtlas 텍스처 + 샘플러 3개를 수동 보강해야 한다(`GlobalDistanceField.cpp` L316-346과 동일):
  - `GlobalDistanceFieldCoverageAtlasTexture = Data.CoverageAtlasTexture ?: GBlackVolumeTexture`
  - PageAtlas/Coverage 샘플러 = `Trilinear/Wrap`, Mip 샘플러 = `Trilinear/Clamp`.
  - `GetDistanceToNearestSurfaceGlobal`/`Gradient`는 expand-surface 스칼라를 안 쓰므로 `_Minimal`로 충분.
- 셰이더: `#include "DistanceField/GlobalDistanceFieldShared.ush"` + C++ FParameters에
  `SHADER_PARAMETER_STRUCT_INCLUDE(FGlobalDistanceFieldParameters2, GDF)`. 둘을 함께 써도 충돌 없음 —
  엔진 자체 precomputation 셰이더(`DistanceFieldScreenGridLighting`, `VolumetricFog`,
  `DistanceFieldVisualization`)가 **정확히 이 패턴**을 쓴다(비-머티리얼 CS는 `DISTANCE_FIELD_IN_VIEW_UB`
  off → .ush의 `#else` loose 전역이 파라미터 구조체로 바인딩). 함수는 `PLATFORM_SUPPORTS_DISTANCE_FIELDS`
  가드 안에만 있으므로 커널 본문을 `#if PLATFORM_SUPPORTS_DISTANCE_FIELDS`로 감쌀 것.
- 모듈 의존: `DynamicRopeShaders.Build.cs`에 `Renderer` + `Engine` 추가.

## GDF 강제 빌드 레시피 (옵션 a — 실측에 사용)

GDF는 소비자(Lumen/Niagara 등)가 플래그하지 않으면 빌드되지 않는다. 스파이크는 전역 강제로 우회:

1. **프로젝트 설정 → Rendering → "Generate Mesh Distance Fields" = ON → 에디터 재시작** (근본 게이트).
2. `r.AOGlobalDistanceField 1`
3. `r.AOGlobalDistanceField.DetailedNecessityCheck 0` — 소비자 없어도 GDF 강제 빌드(**성능 비용**).
4. `r.DynamicRope.GDFSpike 1` (+ 필요 시 `r.DynamicRope.GDFSpikeRadiusScale 2`).

## 실측 소견

- 정적 스태틱메시(벽/바닥, distance field 생성됨) 근처에서 로프가 표면 밖으로 밀려남 — 느낌 OK.
- 밀어냄이 **다음 프레임 렌더에 반영**되는 1프레임 지연(파이프라인상 튜브 빌드가 이번 프레임 PosBuf를
  먼저 읽음). 밀어낸 위치는 상주 상태에 남아 다음 솔브에 피드백되므로 붕괴는 없으나, 빠른 접촉에선
  살짝 파고들었다가 튀어나오는 느낌. 지연 제거는 솔버 dispatch를 뷰 확장으로 이전해야 함(아래 blocker 1).
- GDF 해상도 한계로 얇은 벽은 RadiusScale를 키우지 않으면 통과 가능(터널링).

## 정식 도입 시 남는 문제 (스파이크가 우회한 것 = 진짜 작업)

1. **솔버 dispatch를 뷰 확장으로 이전(중간 규모)**: 스파이크는 push-out을 *별도 패스*로 씬 그래프에
   얹어 1프레임 지연을 감수했다. 지연 없이 하려면 메인 솔브 자체가 GDF 파라미터를 봐야 하고, 그 파라미터는
   씬 렌더러 그래프 안에서만 유효하므로 `FRopeGPUSolver::Step`을 서브시스템 Tick 트리거 → 뷰 확장
   `PrePostProcessPass` 트리거로 옮기는 재구조가 필요하다(상주 버퍼는 PoC(2)가 그래프 간 유효 확인).
2. **GDF 빌드 보장(어렵고 미해결, 사실상 blocker)**: 옵션 a 전역 강제는 성능 비용이라 프로덕션 기본값
   부적합. public 소비자 등록 API가 없어 (b)씬 머티리얼 규약=취약, (c)Niagara식 FXSystem 소비자 모사=
   엔진 내부 복제 — 전부 나쁨. **마일스톤화 시 이 항목을 최우선 설계 과제로 못 박을 것.** 안 풀리면
   씬 상황에 따라 충돌이 조용히 무력화된다.

## 본질적 한계 (불변, `02_GDF_GoNoGo.md`와 동일)

정적 월드 전용 · 해상도(clipmap voxel > CollisionRadius면 얇은 벽 터널링) · SurfaceVelocity 없음 ·
본 귀속 없음(→ wrap 불가). **per-bone SDF의 대체가 아니라 벽/바닥 광역 밀어내기 보완재**로만 유효.

## 권고

느낌은 확인됐으나 두 blocker(특히 GDF 빌드 보장)가 프로덕션 채택을 막는다. 정적 환경 충돌 수요가
실제로 생기면 별도 마일스톤으로, "GDF 빌드 보장" 설계를 먼저 확정한 뒤 dispatch 이전까지 묶어 진행한다.
스파이크 커널/뷰 확장/API 세부는 이 문서로 즉시 재구현 가능.
