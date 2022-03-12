import pyglet


def init_window(app_state):
    window = pyglet.window.Window(width=app_state.ui_window_width,
                                  height=app_state.ui_window_height,
                                  config=app_state.gl_config,
                                  resizable=True, 
                                  vsync=True)
    window.set_fullscreen(app_state.ui_fullscreen)

    return window


