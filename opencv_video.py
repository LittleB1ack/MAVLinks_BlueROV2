
import sys
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class UDPVideoReceiver:
    def __init__(self, port=5000):
        Gst.init(None)
        self.port = port
        self.pipeline = None
        self.create_pipeline()

    def create_pipeline(self):
        # GStreamer管道定义
        pipeline_str = (
            f"udpsrc port={self.port} caps=\"application/x-rtp,media=video,encoding-name=H264\" ! "
            "rtph264depay ! "
            "avdec_h264 ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink name=sink emit-signals=True"
        )

        self.pipeline = Gst.parse_launch(pipeline_str)
        appsink = self.pipeline.get_by_name("sink")
        appsink.connect("new-sample", self.on_new_sample)

    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample:
            # 使用OpenCV处理帧
            buffer = sample.get_buffer()
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if not success:
                return Gst.FlowReturn.ERROR
            
            # 转换为OpenCV格式
            caps = sample.get_caps()
            width = caps.get_structure(0).get_value("width")
            height = caps.get_structure(0).get_value("height")
            
            import numpy as np
            import cv2
            
            frame = np.ndarray(
                shape=(height, width, 3),
                dtype=np.uint8,
                buffer=map_info.data
            )
            
            # 显示帧
            cv2.imshow("Video Stream", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop()
            
            buffer.unmap(map_info)
            return Gst.FlowReturn.OK
        return Gst.FlowReturn.ERROR

    def start(self):
        # 启动管道
        self.pipeline.set_state(Gst.State.PLAYING)
        print(f"开始接收UDP端口 {self.port} 的视频流...")
        
        # 运行GLib主循环
        self.loop = GLib.MainLoop()
        try:
            self.loop.run()
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        # 停止管道
        self.pipeline.set_state(Gst.State.NULL)
        self.loop.quit()
        print("视频接收已停止")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
    else:
        port = 5000
    
    receiver = UDPVideoReceiver(port)
    receiver.start()



