# 시나리오2 E3 : 검출된 토마토 1개 (DetectTomatoes 응답의 tomatoes[] 원소)
#
# tomato_id 는 '사진 1장' 안에서만 유효하다. AI 는 프레임마다 번호를 새로 매기므로
# 라운드가 바뀌면 같은 토마토라도 다른 id 를 받는다 → 라운드 간 식별에 쓰면 안 된다.
# 대신 Ddagi 는 물리 좌표(x,y,z)로 '3cm 이내면 같은 토마토'를 판정해 제외 목록을 관리한다.
int32   tomato_id
string  grade               # NORMAL(수확품 바구니) / DISCARD(폐기품 바구니). 안익은 열매는 애초에 반환 안 됨
float64 x
float64 y
float64 z                   # camera_link 좌표 (TF 변환은 Ddagi 가 수행)
