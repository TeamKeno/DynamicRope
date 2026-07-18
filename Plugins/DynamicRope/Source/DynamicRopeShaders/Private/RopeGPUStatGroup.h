// Copyright Epic Games, Inc. All Rights Reserved.
//
// 'stat DynamicRope' 그룹 선언 — DynamicRopeShaders 모듈 전용. 이 모듈은 런타임 DynamicRope 모듈보다 하위라
// 그쪽 RopeStats.h(같은 그룹의 런타임 소유자)를 include할 수 없어, GPU RT 타이밍/메모리/대역폭 stat을 같은
// "DynamicRope" 그룹명으로 여기서 자체 선언한다(그룹 identity = 이름+카테고리 — STATCAT_Advanced로 일치시켜
// program-wide ODR 안전, 런타임 카운터와 한 HUD에 합쳐진다).
//
// 그룹 struct(DECLARE_STATS_GROUP)는 TU당 정확히 한 번만 정의돼야 한다. 유니티 빌드가 여러 cpp를 한 TU로
// 합치므로 각 cpp에 인라인 선언하면 struct 재정의(C2011)가 난다 — 그래서 헤더+include guard로 TU당 1회만
// 나오게 하고, 이 그룹으로 stat을 선언하는 shaders cpp(RopeGPUSolver.cpp / RopeTubeBuilder.cpp)가 모두 이
// 헤더를 include한다. 개별 stat(DECLARE_*_STAT)은 static이라 각 cpp에 그대로 둔다(TU 로컬).

#pragma once

#include "Stats/Stats.h"

DECLARE_STATS_GROUP(TEXT("DynamicRope"), STATGROUP_DynamicRope, STATCAT_Advanced);
