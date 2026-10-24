"""Explicit camera selection, imported only by the owned camera worker."""


def create_camera(cfg):
    backend=cfg.get('backend','fx2')
    if backend=='fx2':
        from .usb_camera import Camera
    elif backend=='v4l2':
        from .v4l2_camera import Camera
    else:raise ValueError('Select camera backend fx2 or v4l2 explicitly')
    return Camera(cfg)
