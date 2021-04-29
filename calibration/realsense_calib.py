import pyrealsense2 as rs2
pipe = rs2.pipeline()
cfg = rs2.config()
cfg.enable_stream(rs2.stream.depth, 256, 144, rs2.format.z16, 90)
dev = pipe.start(cfg).get_device()
cal = rs2.auto_calibrated_device(dev)


def cb(progress):
	print('.'	)

timeout_ms=15000
json_content=''
# json={
# "average_step_count": 20,
# "step_count": 20,
# "accuracy": 2,
# "scan_parameter": 0,
# "data_sampling": 0
# }
res, health = cal.run_on_chip_calibration(json_content=json_content,timeout_ms=15000)

print(res)
print(health)
print(json_content)
# res = cal.run_tare_calibration(ground_thruth, timeout_ms, json, cb)
