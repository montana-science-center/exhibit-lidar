import pyrealsense2 as rs
import pyglet.gl as gl
import pyglet
import numpy as np

def convert_fmt(fmt):
    """rs.format to pyglet format string"""
    return {
        rs.format.rgb8: 'RGB',
        rs.format.bgr8: 'BGR',
        rs.format.rgba8: 'RGBA',
        rs.format.bgra8: 'BGRA',
        rs.format.y8: 'L',
        rs.format.raw8: 'L',
        rs.format.z16: 'RGB', #special for depth image color mapping 
    }[fmt]


def get_config(pipeline):
    config = rs.config()
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break

    if not found_rgb:
        raise IOError("The app requires Depth camera with Color sensor")

    return config

def get_depth_sensor(pipeline):
    config = rs.config()
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device().first_depth_sensor()
    return device


def get_stream_profile(pipeline, stream):
    profile = pipeline.get_active_profile()
    return rs.video_stream_profile(profile.get_stream(stream))


def get_stream_intrinsics(pipeline, stream):
    return get_stream_profile(pipeline, stream).get_intrinsics()


def get_converted_steam_fmt(pipeline, stream):
    native_fmt = get_stream_profile(pipeline, stream).format()
    return convert_fmt(native_fmt)

def image_from_stream(pipeline, stream):
    intrinsics = get_stream_intrinsics(pipeline, stream)
    w, h = intrinsics.width, intrinsics.height
    empty_image = (gl.GLubyte * (w * h * 3))()
    fmt = get_converted_steam_fmt(pipeline, stream)
    image = pyglet.image.ImageData(w, h, fmt, empty_image)
    return image, fmt

