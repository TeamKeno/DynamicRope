// Copyright Epic Games, Inc. All Rights Reserved.
//
// DynamicRope(Runtime) 로그 카테고리. 아키텍처의 경계(파사드/솔버/로직/충돌·렌더)에 맞춰 나눈다.
// 어떤 .cpp든 이 헤더만 include하면 UE_LOG(LogRope..., ...)를 쓸 수 있다.
// 정의(DEFINE)는 DynamicRope.cpp 한 곳에 있다.
//
// 사용 기준:
//   LogDynamicRope     — 컴포넌트/서브시스템/모듈 수명주기, phase 전이 등 상위 흐름.
//   LogRopeSolver      — XPBD 솔버(물리). substep/constraint/수렴 관련 hot-path 진단.
//   LogRopeWrap        — wrap 컨트롤러(로직). DecideWrap/BeginWrap/Hold/Pull/Release.
//   LogRopeCollision   — collider/provider/SDF 런타임 질의(브로드·내로우 페이즈).

#pragma once

#include "Logging/LogMacros.h"

/** 상위 흐름: 컴포넌트(파사드)·서브시스템·모듈 수명주기·phase 전이. */
DECLARE_LOG_CATEGORY_EXTERN(LogDynamicRope, Log, All);

/** 물리: XPBD 솔버. substep/distance·bending·collision constraint 진단. */
DECLARE_LOG_CATEGORY_EXTERN(LogRopeSolver, Log, All);

/** 로직: wrap 컨트롤러(DecideWrap/BeginWrap/Hold/Pull/Release). */
DECLARE_LOG_CATEGORY_EXTERN(LogRopeWrap, Log, All);

/** 충돌: collider/provider/SDF 런타임 질의(broad/narrow phase). */
DECLARE_LOG_CATEGORY_EXTERN(LogRopeCollision, Log, All);
