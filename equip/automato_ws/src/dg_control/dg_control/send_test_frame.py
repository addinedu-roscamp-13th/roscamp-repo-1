#!/usr/bin/env python3
"""시나리오1 E2 analyze_frame 수동 테스트 스크립트.

test_images/ 폴더의 이미지를 하나씩 DG AI Service 로 보내 분석 결과를
확인한다. rotten/disease 가 감지된 이미지는 레이블링된 결과 이미지를
파일로 저장한다.

사전 준비: DG AI Service 가 떠 있어야 한다.
  export DG_AI_MODEL_PATH=~/Projects/Eval_Yolo/tomato_4cls_model.pt
  ros2 run dg_ai_service analysis_server        # 또는
  python3 -m dg_ai_service.analysis_server

실행:
  python3 -m dg_control.send_test_frame
  python3 -m dg_control.send_test_frame --dir some/other/folder --host 192.168.6.1
"""
import argparse
import os

from dg_control.ai_client import analyze_frame, decode_labeled_image

IMAGE_EXTS = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_IMAGE_DIR = os.path.normpath(os.path.join(HERE, '..', 'test_images'))


def collect_images(folder):
    if not os.path.isdir(folder):
        return []
    return sorted(
        os.path.join(folder, name)
        for name in os.listdir(folder)
        if os.path.splitext(name)[1].lower() in IMAGE_EXTS
        and not os.path.splitext(name)[0].endswith('_labeled')
    )


def parse_args():
    parser = argparse.ArgumentParser(description='dg_control -> dg_ai_service analyze_frame 수동 테스트')
    parser.add_argument('--dir', default=DEFAULT_IMAGE_DIR,
                         help=f'테스트 이미지 폴더 (기본: {DEFAULT_IMAGE_DIR})')
    parser.add_argument('--out', default=None,
                         help='레이블링 이미지 저장 폴더 (기본: --dir 과 동일)')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=9100)
    parser.add_argument('--task-id', type=int, default=1)
    return parser.parse_args()


def main():
    args = parse_args()
    in_dir = os.path.abspath(args.dir)
    out_dir = os.path.abspath(args.out) if args.out else in_dir

    images = collect_images(in_dir)
    if not images:
        print(f'❌ 테스트 이미지가 없습니다: {in_dir}\n   (jpg/jpeg/png 파일을 이 폴더에 넣어주세요)')
        return

    print(f'📷 이미지 {len(images)}장 전송 시작 -> {args.host}:{args.port}\n')
    for waypoint_id, path in enumerate(images, start=1):
        name = os.path.basename(path)
        with open(path, 'rb') as f:
            image_bytes = f.read()

        try:
            result = analyze_frame(
                image_bytes, task_id=args.task_id, waypoint_id=waypoint_id,
                host=args.host, port=args.port,
            )
        except (OSError, RuntimeError) as exc:
            print(f'  ✗ {name}: 요청 실패 - {exc}')
            continue

        percents = {k: v for k, v in result.items() if k.endswith('_percent')}
        print(f'  • {name} -> {percents}')

        labeled = decode_labeled_image(result)
        if labeled is not None:
            os.makedirs(out_dir, exist_ok=True)
            out_path = os.path.join(out_dir, f'{os.path.splitext(name)[0]}_labeled.jpg')
            with open(out_path, 'wb') as f:
                f.write(labeled)
            print(f'    -> rotten/disease 감지: 레이블링 이미지 저장 {out_path}')

    print('\n✅ 완료')


if __name__ == '__main__':
    main()
