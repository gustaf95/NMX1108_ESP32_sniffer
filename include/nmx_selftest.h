// 디코더 자체 검증. NMX-1106 문서의 예제 프레임을 그대로 넣어
// 체크섬 판정과 디코딩 문자열이 기대값과 일치하는지 확인한다.
//
// 장비 없이 보드만 연결한 상태에서 시리얼 모니터에 'selftest' 를 입력하면 실행된다.

#pragma once

namespace nmx {

// 실패한 케이스 수를 반환한다. 결과는 Serial로 출력된다.
int runSelfTest();

}  // namespace nmx
