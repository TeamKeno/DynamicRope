// Copyright Epic Games, Inc. All Rights Reserved.
//
// 'stat DynamicRopeGPU' 그룹 선언 — DynamicRopeShaders 모듈 소유. GPU RT 타이밍/VRAM/전송 대역폭 stat이 여기
// 모인다. 런타임 GT 프레임 대시보드('stat DynamicRope', 런타임 모듈 RopeStats.h)와는 **다른 그룹**이다 —
// 원래 한 그룹이었는데 53행까지 불어나 stat HUD 한 화면을 넘겼다(HUD는 cycle→memory→counter 순으로 그리고
// 스크롤이 없어서 맨 아래 counter 섹션이 잘려나갔다. stats.MaxPerGroup 기본 25 상한도 근접). 그래서 GT 프레임
// 비용은 저쪽, GPU/RT는 이쪽으로 갈랐다 — 새 stat을 추가할 때 이 경계를 지켜라. 둘을 같이 보려면 두 그룹을
// 각각 켜면 된다('stat DynamicRope' + 'stat DynamicRopeGPU').
//
// 단, 이 그룹의 'GPU *' CYCLE stat은 전부 RT CPU 시간이다 — GPU가 실제로 커널을 돌린 시간은 엔진 GPU 그룹
// 소관이라 'stat gpu'에서 DynamicRope Solve / Detect / Tube로 본다(선언은 각 cpp의 DECLARE_GPU_STAT_NAMED).
//
// 그룹 struct(DECLARE_STATS_GROUP)는 TU당 정확히 한 번만 정의돼야 한다. 유니티 빌드가 여러 cpp를 한 TU로
// 합치므로 각 cpp에 인라인 선언하면 struct 재정의(C2011)가 난다 — 그래서 헤더+include guard로 TU당 1회만
// 나오게 하고, 이 그룹으로 stat을 선언하는 shaders cpp(RopeGPUSolver.cpp / RopeTubeBuilder.cpp)가 모두 이
// 헤더를 include한다. 개별 stat(DECLARE_*_STAT)은 static이라 각 cpp에 그대로 둔다(TU 로컬).

#pragma once

#include "Stats/Stats.h"

DECLARE_STATS_GROUP(TEXT("DynamicRopeGPU"), STATGROUP_DynamicRopeGPU, STATCAT_Advanced);
