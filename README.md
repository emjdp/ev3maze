# EV3 Maze Simulator

EV3 미로 라인트레이서 알고리즘을 브라우저에서 확인하는 React 시뮬레이터입니다.

- EXPLORE: 좌선우선, 111 peek, 한 분기 우선 규칙으로 모든 노드를 지나 7에 도착
- RETURN: MOVELOG를 역순 + 좌우 반전으로 재생해 0으로 복귀
- 배포: Vercel 기본 빌드, GitHub Pages용 `/ev3maze/` base 빌드 지원

## Run

```bash
npm install
npm run dev
```

## Build

```bash
npm run build
npm run build:pages
```

## Files

- `EV3Sim.jsx`: 원본 단일 파일 구현 참고본
- `미로_알고리즘_구현명세.md`: 알고리즘 기준 명세
- `src/`: 배포 가능한 Vite + React 앱
- `maze.png`: 원본 미로 참고 이미지
