import pyrealsense2 as rs

class CameraStream:
    def __init__(self, width=640, height=480, fps=30):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.pipeline.start(config)

    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        return frames.get_color_frame(), frames.get_depth_frame()

    def stop(self):
        self.pipeline.stop()


if __name__ == '__main__':
    # 단독 실행 시 연결 테스트용
    cam = CameraStream()
    color, depth = cam.get_frames()
    print("컬러:", color, "/ depth:", depth)
    cam.stop()