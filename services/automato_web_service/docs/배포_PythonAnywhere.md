# Automato 실시간 콘솔 — 영구 배포 (PythonAnywhere)

항상 켜져 있고(잠들지 않음) 주소가 안 바뀌는 무료 영구 호스팅.
업로드할 파일: **`~/Desktop/automato-live.zip`** (이미 만들어져 있음)
최종 주소: **`https://<사용자이름>.pythonanywhere.com`**

---

## 1. 가입 (무료 Beginner)
<https://www.pythonanywhere.com/registration/register/beginner/>
- **Username** 이 곧 주소가 됩니다 → 예: `automato` → `automato.pythonanywhere.com`
- 카드 필요 없음.

## 2. Flask 설치
대시보드 → **Consoles** → **Bash** 콘솔 시작 → 입력:
```bash
pip3.10 install --user flask
```

## 3. 코드 업로드
상단 **Files** 탭 → 홈 디렉터리에서 **Upload a file** →
`~/Desktop/automato-live.zip` 선택해 업로드.

## 4. 압축 풀기
다시 **Bash** 콘솔에서:
```bash
unzip automato-live.zip -d automato-live
ls automato-live        # app.py, static/ 가 보이면 OK
```

## 5. 웹앱 생성
상단 **Web** 탭 → **Add a new web app** →
- 도메인 그대로 **Next**
- **Manual configuration** 선택 (※ "Flask" 아님)
- **Python 3.10** 선택 → 완료

## 6. 경로 · WSGI 설정 (가장 중요)
**Web** 탭에서:
- **Source code** 칸 → `/home/<사용자이름>/automato-live`
- **WSGI configuration file** 링크 클릭 → 내용을 **전부 지우고** 아래로 교체
  (`<사용자이름>` 두 곳을 본인 것으로):
```python
import sys
path = '/home/<사용자이름>/automato-live'
if path not in sys.path:
    sys.path.insert(0, path)
from app import app as application
```
- **Save**

## 7. 실행
**Web** 탭 맨 위 초록색 **Reload** 버튼 클릭 →
**`https://<사용자이름>.pythonanywhere.com`** 접속 → 끝!

이 주소를 팀에 공유하면, 누가 버튼을 눌러 상태를 바꾸면 ~1초 내 전원 화면이 같이 바뀝니다.

---

## 문제 해결
- **"Something went wrong" / 500 에러**: Web 탭의 **Error log** 링크 확인.
  - `No module named 'flask'` → 2번을 `pip3.10 install --user flask` 로 다시.
  - `No module named 'app'` → 6번 Source code 경로 / WSGI path 오타 확인.
- **화면은 뜨는데 동기화 안 됨**: 두 기기에서 같은 주소를 열고, 한쪽에서
  주간/야간·구역 승인·추종 토글을 눌러 확인. (우하단 `👥 N명 접속` 배지로 연결 표시)
- 상태는 `automato-live/state.json` 에 저장되어 재시작 후에도 유지됩니다.
  처음으로 되돌리려면 관리자 대시보드의 **↺ 데모 초기화** 버튼.
