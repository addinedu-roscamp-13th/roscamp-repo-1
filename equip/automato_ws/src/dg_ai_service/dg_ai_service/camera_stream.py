import pyrealsense2 as rs

class CameraStream:
    def __init__(self, width=640, height=480, fps=30):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        profile = self.pipeline.start(config)
        # 컬러·depth 센서는 물리적으로 떨어져 있어 픽셀이 기본으로는 안 맞는다.
        # depth를 컬러 픽셀 좌표계로 정렬해야 YOLO bbox 중심 픽셀로 depth를
        # 그대로 조회할 수 있다.
        self._align = rs.align(rs.stream.color)
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self._intrinsics = color_profile.get_intrinsics()  # align 이후 depth도 이 좌표계를 따름

    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned = self._align.process(frames)
        return aligned.get_color_frame(), aligned.get_depth_frame()

    def get_intrinsics(self):
        """(fx, fy, ppx, ppy) — deproject_pixel()에 그대로 넘길 수 있는 형태."""
        intr = self._intrinsics
        return intr.fx, intr.fy, intr.ppx, intr.ppy

    def stop(self):
        self.pipeline.stop()


if __name__ == '__main__':
    # 단독 실행 시 연결 테스트용
    cam = CameraStream()
    color, depth = cam.get_frames()
    print("컬러:", color, "/ depth:", depth)
    cam.stop()