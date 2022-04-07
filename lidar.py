import math
import pyglet
import pyglet.gl as gl
import numpy as np
import pyrealsense2 as rs

import lidar_util
import ui_util
import graphics_util


class AppState:
    def __init__(self, *args, **kwargs):

        # user settable variables
        self.sesor_height = 0.5 # sensor height off the floor, meters
        self.distance = 3.0 # center of rotation from camera, meters
        self.spin_speed = 0.25 # view rotation speed, degrees-per-frame
        self.ui_window_width = 1000 # window width when not in fullscreen, pixels
        self.ui_window_height = 600 # widow height when not in fullscreen, pixels
        self.ui_fullscreen = True # enable auto-fullscreen on start
        self.ui_fontsize = 18 # font size of onscreen text
        self.ui_pointsize = 1 # size of points in pointcloud

        # sensor config
        self.sensor_alternate_ir = 1 # apply gamma curve to IR video stream, 0,1
        self.sensor_laser_power = 100 # laser power in percent, 0-100
        self.sensor_confidence_threshold = 2 # threshold to consider depth data good, 0-3
        self.sensor_min_distance = 0 # minimum depth sensing distance, mm, def=490mm
        self.sensor_noise_filtering = 4 # apply noise filtering to depth data, 1-6 def=4
        self.sensor_invalidation_bypass = 0 # 1=show invalid lidar points, 0,1

        # system variables
        self.imu_sample_window = 5 # number of IMU samples to average for noise reduction
        self.capture_fps = 30 # Lidar data capture speed
        self.gl_config = gl.Config(double_buffer=True, samples=8) # open GL config

        # modified during runtime
        self.camera_pitch = 0
        self.camera_yaw = 0
        self.sensor_pitch = 0
        self.sensor_roll = 0
        self.translation = np.array([0, 0.5, 0], np.float32)
        self.imu_samples = np.array([[0.0,-9.8, 0.0]]*self.imu_sample_window, np.float32)
        self.imu_average = (0.0,-9.8, 0.0)

    def update_imu(self, new_vector):
        # rolling average for imu samples
        self.imu_samples = np.roll(self.imu_samples, 1, 0)
        self.imu_samples[0] = new_vector
        self.imu_average = np.average(self.imu_samples, 0)

    def update_scene(self):
        x = self.imu_average[0]
        y = self.imu_average[1]
        z = self.imu_average[2]
        self.sensor_pitch = math.atan2(y, z) * 180/math.pi + 90
        self.sensor_roll = math.atan2(-x, math.sqrt(y*y + z*z)) * 180/math.pi
        self.camera_yaw -= self.spin_speed



def main():
    state = AppState()
    window = ui_util.init_window(state)
    pipeline = rs.pipeline()

    # sensor settings
    depth_sensor = lidar_util.get_depth_sensor(pipeline)
    depth_sensor.set_option(rs.option.alternate_ir, state.sensor_alternate_ir)
    depth_sensor.set_option(rs.option.laser_power, state.sensor_laser_power)
    depth_sensor.set_option(rs.option.confidence_threshold, state.sensor_confidence_threshold)
    depth_sensor.set_option(rs.option.min_distance, state.sensor_min_distance)
    depth_sensor.set_option(rs.option.noise_filtering, state.sensor_noise_filtering)
    depth_sensor.set_option(rs.option.invalidation_bypass, state.sensor_invalidation_bypass)

    config = lidar_util.get_config(pipeline)
    config.enable_stream(rs.stream.depth, rs.format.z16, state.capture_fps)
    config.enable_stream(rs.stream.color, rs.format.rgb8, state.capture_fps)
    config.enable_stream(rs.stream.infrared, rs.format.y8, state.capture_fps)
    config.enable_stream(rs.stream.confidence, rs.format.raw8, state.capture_fps)
    config.enable_stream(rs.stream.accel)

    pipeline.start(config)

    depth_intrinsics = lidar_util.get_stream_intrinsics(pipeline, rs.stream.depth)
    depth_w, depth_h = depth_intrinsics.width, depth_intrinsics.height

    vertex_list = pyglet.graphics.vertex_list(depth_w * depth_h, 'v3f/stream', 't2f/stream')
    
    colorizer = rs.colorizer()
    colorizer.set_option(rs.option.visual_preset, 0) # 0=Dynamic, 1=Fixed, 2=Near, 3=Far
    
    pc = rs.pointcloud()
    filters = [rs.disparity_transform(),
               rs.spatial_filter(),
               rs.temporal_filter(),
               #rs.hole_filling_filter(), # can cause visual artifacts on sparse point clouds.
               rs.disparity_transform(False)]

    color_image, color_fmt = lidar_util.image_from_stream(pipeline, rs.stream.color)
    infrared_image, infrared_fmt = lidar_util.image_from_stream(pipeline, rs.stream.infrared)
    depth_image, depth_fmt = lidar_util.image_from_stream(pipeline, rs.stream.depth)
    confidence_image, confidence_fmt = lidar_util.image_from_stream(pipeline, rs.stream.confidence)


    @window.event
    def on_draw():
        window.clear()
        width, height = window.get_size()

        video_width = int(width/4)
        video_height = int(video_width * (depth_h/depth_w))
        height -= video_height

        # Raw sensor data
        gl.glViewport(0, 0, width, video_height)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        gl.glOrtho(0, width, video_height, 0, -1, 1)
        gl.glMatrixMode(gl.GL_MODELVIEW)

        graphics_util.draw_image(color_image, 0, 0, video_width, video_height)
        graphics_util.draw_image(infrared_image, video_width, 0, video_width, video_height)
        graphics_util.draw_image(depth_image, video_width*2, 0, video_width, video_height)
        graphics_util.draw_image(confidence_image, video_width*3, 0, video_width, video_height)

        # 3D turntable
        gl.glViewport(0, video_height, width, height)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        gl.gluPerspective(60, width / float(height), 0.01, 20)

        # render camera xform 
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()
        gl.gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0)
        gl.glTranslatef(0, 0, state.distance)
        gl.glRotated(state.camera_pitch, 1, 0, 0)
        gl.glRotated(state.camera_yaw, 0, 1, 0)
        #graphics_util.axes(0.1, 4) # camera center axes, useful for debugging
        gl.glTranslatef(*state.translation)
        graphics_util.grid(5, 50)
        gl.glTranslatef(0, 0, -state.distance)

        # pointcloud xform
        gl.glPushMatrix()
        gl.glTranslatef(0, -state.sesor_height, 0)
        gl.glRotated(state.sensor_pitch, 1, 0, 0)
        gl.glRotated(state.sensor_roll, 0, 0, 1)

        # points
        gl.glPointSize(state.ui_pointsize)
        gl.glColor3f(1, 1, 1)
        gl.glTexParameteri(
        gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_NEAREST)
        gl.glEnable(gl.GL_POINT_SPRITE)
        gl.glDisable(gl.GL_MULTISAMPLE)
        vertex_list.draw(gl.GL_POINTS)
        gl.glEnable(gl.GL_MULTISAMPLE)
        gl.glDisable(gl.GL_LIGHTING)
    
        # frustum
        gl.glColor3f(0.25, 0.25, 0.25)
        graphics_util.frustum(depth_intrinsics)
        #graphics_util.axes(0.1)    # camera axes, useful for debugging
        
        gl.glPopMatrix()
    
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        gl.glOrtho(0, width, 0, height, -1, 1)
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()
        gl.glMatrixMode(gl.GL_TEXTURE)
        gl.glLoadIdentity()
        gl.glDisable(gl.GL_DEPTH_TEST)

        graphics_util.draw_text(state.ui_fontsize, 5, height, video_width)

        # update camera and sensor positions
        state.update_scene()


    def get_sensor_data(dt):
        nonlocal depth_w, depth_h
        window.set_caption("LiDAR (%dx%d) %dFPS (%.2fms)" %
                           (depth_w, depth_h, 0 if dt == 0 else 1.0 / dt, dt * 1000))

        success, frames = pipeline.try_wait_for_frames(timeout_ms=0)
        if not success:
            return

        # IMU update pitch and roll
        accel_frame = frames.first_or_default(rs.stream.accel)
        accel = accel_frame.as_motion_frame().get_motion_data()
        accel_data = np.asarray([accel.x, accel.y, accel.z])
        state.update_imu(accel_data)

        # image buffers
        color_frame = frames.first(rs.stream.color).as_video_frame()
        color_array = np.asanyarray(color_frame.get_data())
        color_image.set_data(color_fmt, color_array.strides[0], color_array.ctypes.data)

        infrared_frame = frames.first(rs.stream.infrared).as_video_frame()
        infrared_array = np.asanyarray(infrared_frame.get_data())
        infrared_image.set_data(infrared_fmt, infrared_array.strides[0], infrared_array.ctypes.data)

        depth_frame = frames.get_depth_frame().as_video_frame() 
        for f in filters:
            depth_frame = f.process(depth_frame)

        colorized_depth = colorizer.colorize(depth_frame)
        depth_array = np.asanyarray(colorized_depth.get_data())
        depth_image.set_data(depth_fmt, depth_array.strides[0], depth_array.ctypes.data)

        confidence_frame = frames.first(rs.stream.confidence).as_video_frame()
        confidence_array = np.asanyarray(confidence_frame.get_data())
        confidence_image.set_data(confidence_fmt, confidence_array.strides[0], confidence_array.ctypes.data)

        # point cloud  
        points = pc.calculate(depth_frame)
    
        verts = np.asarray(points.get_vertices(2)).reshape(depth_h, depth_w, 3)
        texcoords = np.asarray(points.get_texture_coordinates(2))
    
        if len(vertex_list.vertices) != verts.size:
            vertex_list.resize(verts.size // 3)
            # need to reassign after resizing
            vertex_list.vertices = verts.ravel()
            vertex_list.tex_coords = texcoords.ravel()
    
        graphics_util.copy(vertex_list.vertices, verts)
        graphics_util.copy(vertex_list.tex_coords, texcoords)
        
    pyglet.clock.schedule(get_sensor_data)

    try:
        pyglet.app.run()
    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()
