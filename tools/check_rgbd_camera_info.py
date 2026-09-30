#!/usr/bin/env python3
"""Read-only RGB-D metadata audit; never starts a camera or hardware controller.

Checks paired Image/CameraInfo dimensions, headers and calibration validity.
It does not measure optical calibration accuracy or prove distortion correction.
"""
import argparse
import json
import math
import time


def main():
    import rclpy
    from sensor_msgs.msg import Image, CameraInfo
    from rclpy.qos import qos_profile_sensor_data
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=15)
    parser.add_argument('--frames', type=int, default=5)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0 or args.frames <= 0:
        parser.error('timeout and frames must be positive')
    rclpy.init()
    node = rclpy.create_node('rgbd_camera_info_audit')
    cache = {key: {} for key in ('color_image','depth_image','color_info','depth_info')}
    records = []
    seen = set()
    subs = []
    def receive(key, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        cache[key][stamp] = msg
        if len(cache[key]) > 50:
            del cache[key][next(iter(cache[key]))]
    for stream in ('color', 'depth'):
        for suffix, typ, topic in [('image', Image, 'image_raw'), ('info', CameraInfo, 'camera_info')]:
            key = stream+'_'+suffix
            subs.append(node.create_subscription(typ, '/camera/'+stream+'/'+topic,
                         lambda msg, key=key: receive(key,msg), qos_profile_sensor_data))
    try:
        deadline = time.monotonic()+args.timeout
        while time.monotonic() < deadline and not all(
            sum(r["stream"] == s for r in records) >= args.frames for s in ("color", "depth")
        ):
            rclpy.spin_once(node, timeout_sec=.1)
            for stream in ('color','depth'):
                for stamp in cache[stream+'_image'].keys() & cache[stream+'_info'].keys():
                    if (stream,stamp) in seen: continue
                    seen.add((stream,stamp))
                    im=cache[stream+'_image'][stamp]; info=cache[stream+'_info'][stamp]
                    errors=[]
                    if (im.width,im.height)!=(info.width,info.height):errors.append('dimension mismatch')
                    if im.header.frame_id != info.header.frame_id:errors.append('frame mismatch')
                    if not all(math.isfinite(v) for v in [*info.k,*info.d,*info.p]):errors.append('nonfinite calibration')
                    if info.k[0]<=0 or info.k[4]<=0:errors.append('invalid focal length')
                    counts={'plumb_bob':5,'rational_polynomial':8,'equidistant':4}
                    if info.distortion_model not in counts or len(info.d)!=counts.get(info.distortion_model):errors.append('unsupported distortion model/length')
                    if len(im.data)!=im.step*im.height:errors.append('image byte count mismatch')
                    records.append(dict(stream=stream,stamp=stamp,width=im.width,height=im.height,
                                        frame_id=im.header.frame_id,encoding=im.encoding,k=list(info.k),
                                        distortion_model=info.distortion_model,d=list(info.d),errors=errors))
        enough=all(sum(r['stream']==s for r in records)>=args.frames for s in ('color','depth'))
        ok=enough and all(not r['errors'] for r in records)
        print(json.dumps(dict(metadata_consistent=ok,required_frames_per_stream=args.frames,
                             optical_accuracy_verified=False,records=records),indent=2))
        return 0 if ok else 1
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    raise SystemExit(main())
