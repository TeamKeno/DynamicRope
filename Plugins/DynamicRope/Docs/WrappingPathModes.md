# Wrapping Path Mode 설명

Dynamic Rope의 `WrappingPathMode`는 로프가 wrapping 상태로 들어간 뒤, 최초 latch node 뒤쪽의 tail node들이 어느 표면 경로를 목표로 이동할지 정하는 프로젝트 전역 설정이다.

설정 위치:

`Project Settings > Plugins > Dynamic Rope > Wrapping > Wrapping Path Mode`

현재 기본값은 `Surface Vector Field`다.

## 한 줄 요약

| Mode | 요약 | 추천 용도 |
|---|---|---|
| `Surface Walk` | 최초 latch tangent를 따라 SDF 표면 위를 한 걸음씩 걷는다. | 가장 보수적인 폴백, 축 정보가 부정확한 대상 |
| `Analytic Helix` | bone-parent 축 기준으로 수학적 나선을 만든 뒤 SDF 표면에 투영한다. | 일정한 간격의 깔끔한 나선 실루엣 |
| `Surface Vector Field` | 매 step마다 축 기준 원주 방향을 다시 계산하고 SDF tangent plane에 투영한다. | 의도적으로 원주를 돌면서 표면 굴곡도 따라가는 기본 추천 |

## Surface Walk

`Surface Walk`는 최초 latch node가 가진 tangent 방향을 출발 방향으로 삼고, tail 방향 node마다 일정 거리씩 전진한 뒤 SDF 표면으로 다시 projection한다.

장점:

- 구현이 가장 단순하고 안정적이다.
- bone 축이나 parent bone 정보가 어색해도 비교적 예측 가능하게 동작한다.
- 다른 모드가 실패하면 기본 폴백으로 쓰기 좋다.

단점:

- "의도적으로 원주를 돈다"는 힘이 약하다.
- 최초 tangent가 애매하면 감기는 방향도 애매해질 수 있다.
- 팔, 다리처럼 원통형 대상에서는 감긴다는 느낌이 부족할 수 있다.

## Analytic Helix

`Analytic Helix`는 latch bone과 parent bone으로 감김 축을 만들고, 그 축을 기준으로 일정한 반지름과 pitch를 가진 나선 위치를 계산한다. 계산된 위치는 마지막에 SDF 표면으로 projection한다.

장점:

- 나선 간격과 실루엣이 가장 일정하다.
- 튜닝 결과가 예측 가능하다.
- 원통형 팔, 다리처럼 축이 명확한 대상에서 보기 좋다.

단점:

- 먼저 수학적 나선을 만든 뒤 표면에 붙이므로, 울퉁불퉁한 SDF 굴곡을 따라가는 감각은 상대적으로 약하다.
- 실제 표면의 급격한 변화보다 축 기준 모양이 우선된다.
- 비원통형 대상에서는 경로가 다소 인위적으로 보일 수 있다.

## Surface Vector Field

`Surface Vector Field`는 현재 기본 추천 모드다. 매 step마다 현재 표면 위치에서 bone-parent 축 기준 radial 방향을 구하고, 그 radial에서 원주 방향 벡터를 만든다. 이후 이 방향에 pitch를 섞고, 현재 SDF normal의 tangent plane에 투영한 다음 다시 SDF 표면으로 projection한다.

장점:

- "의도적으로 원주를 돈다"는 방향성이 가장 강하다.
- 동시에 SDF 표면 normal을 계속 반영해서 표면 굴곡을 따라간다.
- 게임적으로 보기 좋은 감김을 만들기 쉽다.

단점:

- 축 정보가 이상하면 원주 방향도 같이 이상해질 수 있다.
- 표면 projection 품질에 영향을 더 많이 받는다.
- `Surface Walk`보다 계산량이 조금 더 있다.

## 선택 기준

처음 튜닝할 때는 `Surface Vector Field`로 시작한다. 감김이 너무 작위적이거나 축이 이상한 대상에서 흔들리면 `Surface Walk`로 비교한다. 일정한 나선 실루엣이 더 중요하면 `Analytic Helix`를 쓴다.

CPU 테스트에서는 세 모드를 모두 비교해도 된다. 최종 GPU 이관을 고려하면, 세 방식 모두 tail node별 target 생성으로 분리할 수 있다. 다만 `Surface Walk`와 `Surface Vector Field`는 step 적분 결과에 의존하므로, GPU에서는 prefix/path precompute 또는 소수 step 고정 루프 형태로 옮기는 쪽이 자연스럽다.

## 관련 코드

- `ERopeWrappingPathMode`: `Source/DynamicRope/Public/Core/RopeTypes.h`
- 프로젝트 설정: `Source/DynamicRope/Public/Settings/DynamicRopeSettings.h`
- 모드 분기: `URopeComponent::ComputeWrapSurfaceTarget`
- 1번 방식: `URopeComponent::ComputeAnalyticHelixWrapTarget`
- 2번 방식: `URopeComponent::ComputeSurfaceVectorFieldWrapTarget`
