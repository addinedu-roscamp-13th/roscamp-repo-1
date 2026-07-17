# roscamp-repo-1

Automato(방울토마토 재배 로봇) 프로젝트. ROS2(Jazzy) 기반 멀티 로봇 시스템
(DdaGo 순찰, Ddagi 수확팔, ACS, HQ/DCS, AI Service 등).

## Git 브랜치 컨벤션

상세: https://robot8.atlassian.net/wiki/spaces/Robot8/pages/33488899/Git

- 구조: `main` ── `dev` ── `feature/`. `main`은 배포 가능한 안정 버전만,
  `dev`는 통합 브랜치, `feature/`(`fix/`, `refactor/`, `docs/`, `hotfix/`)에서
  작업 후 `dev`로 병합.
- 브랜치명: `<타입>/<Jira키>-<kebab-설명>` (예: `feature/RP-78-corridor-reservation`).
  Jira 키를 넣으면 GitHub-Jira 연동으로 이슈에 자동 링크됨.
- PR 병합은 **Squash and merge**로 통일 (Jira 이슈 하나 = dev 커밋 하나).
- 새 작업 시작 시 항상 최신 `dev`에서 새 브랜치를 딴다
  (`git checkout dev && git pull && git checkout -b feature/...`).
  squash 병합된 브랜치는 `git branch -d`가 거부될 수 있음 → `-D`로 삭제.
- **PR 전에 반드시 로컬에서 `colcon build` 성공 + 최소 실행 확인.** 리뷰가
  없는 프로젝트라 이게 dev가 깨지는 걸 막는 유일한 안전선.
