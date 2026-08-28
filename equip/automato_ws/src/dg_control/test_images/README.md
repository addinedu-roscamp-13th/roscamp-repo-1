# dg_control 수동 테스트 이미지 폴더

이 폴더에 jpg/jpeg/png 이미지를 넣고 아래 명령을 실행하면
`dg_control` → `dg_ai_service` 로 시나리오1 E2 `analyze_frame` 요청을
실제로 보내볼 수 있습니다.

```bash
python3 -m dg_control.send_test_frame
```

- rotten/disease 가 감지된 이미지는 같은 폴더에 `<파일명>_labeled.jpg` 로
  바운딩 박스가 그려진 결과가 저장됩니다.
- 이미지 파일 자체는 git에 올리지 않습니다(`.gitignore` 참고). 이 폴더와
  README 만 추적됩니다.
