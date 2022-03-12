import pyglet
import pyglet.gl as gl
import pyrealsense2 as rs
import numpy as np


def axes(size=1, width=1):
    """draw 3d axes"""
    gl.glLineWidth(width)
    pyglet.graphics.draw(6, gl.GL_LINES,
                         ('v3f', (0, 0, 0, size, 0, 0,
                                  0, 0, 0, 0, size, 0,
                                  0, 0, 0, 0, 0, size)),
                         ('c3f', (1, 0, 0, 1, 0, 0,
                                  0, 1, 0, 0, 1, 0,
                                  0, 0, 1, 0, 0, 1,
                                  ))
                         )


def frustum(intrinsics):
    """draw camera's frustum"""
    w, h = intrinsics.width, intrinsics.height
    batch = pyglet.graphics.Batch()

    for d in range(1, 6, 2):
        def get_point(x, y):
            p = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], d)
            batch.add(2, gl.GL_LINES, None, ('v3f', [0, 0, 0] + p))
            return p

        top_left = get_point(0, 0)
        top_right = get_point(w, 0)
        bottom_right = get_point(w, h)
        bottom_left = get_point(0, h)

        batch.add(2, gl.GL_LINES, None, ('v3f', top_left + top_right))
        batch.add(2, gl.GL_LINES, None, ('v3f', top_right + bottom_right))
        batch.add(2, gl.GL_LINES, None, ('v3f', bottom_right + bottom_left))
        batch.add(2, gl.GL_LINES, None, ('v3f', bottom_left + top_left))

    batch.draw()


def grid(interval_size=1, n=10, width=1, major_int = 10):
    """draw a grid on xz plane"""
    gl.glLineWidth(width)
    s = interval_size / float(n)
    s2 = 0.5 * interval_size
    batch = pyglet.graphics.Batch()
    minor_color = ('c3f', (0.1, 0.1, 0.1, 0.1, 0.1, 0.1))
    major_color = ('c3f', (0.5, 0.5, 0.5, 0.5, 0.5, 0.5))

    # lines along x
    for i in range(0, n + 1):
        x = -s2 + i * s
        if i % major_int == 0:
            color = major_color
        else:
            color = minor_color
        batch.add(2, gl.GL_LINES, None, ('v3f', (x, 0, -s2, x, 0, s2)), color)

    # lines along z
    for i in range(0, n + 1):
        z = -s2 + i * s
        if i % major_int == 0:
            color = major_color
        else:
            color = minor_color
        batch.add(2, gl.GL_LINES, None, ('v3f', (-s2, 0, z, s2, 0, z)), color)

    batch.draw()


def copy(dst, src):
    np.array(dst, copy=False)[:] = src.ravel()


def draw_image(image, x=0, y=0, w=100, h=100):
    gl.glColor3f(1, 1, 1)
    texture = image.get_texture()   
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_NEAREST)                                                                                                                               
    texture.width = w                                                                                                                                                                  
    texture.height = h                                                                                                                                                                                                                                                                                                                       
    texture.blit(x, y)


def draw_text(fontsize, margin, height, video_width):
    batch = pyglet.graphics.Batch()

    font = 'Verdana'
    color = (128, 128, 128, 255)

    #pointcloud
    pyglet.text.Label('LiDAR POINTCLOUD',
                      font_name=font,
                      font_size=fontsize,
                      color=color,
                      x=margin, y=height-margin,
                      anchor_x='left', anchor_y='top',
                      batch = batch)
    # Color
    pyglet.text.Label('COLOR',
                      font_name=font,
                      font_size=fontsize,
                      color=color,
                      x=margin, y=0,
                      anchor_x='left', anchor_y='bottom',
                      batch = batch)
    # IR
    pyglet.text.Label('INFRARED',
                      font_name=font,
                      font_size=fontsize,
                      color=color,
                      x=margin + video_width, y=0,
                      anchor_x='left', anchor_y='bottom',
                      batch = batch)
    # Depth
    pyglet.text.Label('DEPTH',
                      font_name=font,
                      font_size=fontsize,
                      color=color,
                      x=margin + (2*video_width), y=0,
                      anchor_x='left', anchor_y='bottom',
                      batch = batch)
    # Confidence
    pyglet.text.Label('CONFIDENCE',
                      font_name=font,
                      font_size=fontsize,
                      color=color,
                      x=margin + (3*video_width), y=0,
                      anchor_x='left', anchor_y='bottom',
                      batch = batch)

    batch.draw()