"""Sensor-only distance culling; observation range is explicit, physics unchanged."""

import numpy as np


def sensor_frame(renderer, model, data, camera, max_range=12.0, shadows=True):
    # shortcut: distant detail meshes are clipped at 12 m; increase with measured onboard sensor needs.
    camera_id = model.camera(camera).id
    original = model.geom_group.copy()
    distance = np.linalg.norm(data.geom_xpos - data.cam_xpos[camera_id], axis=1)
    hidden = (original == 2) & (distance > max_range)
    try:
        model.geom_group[hidden] = 4
        renderer.update_scene(data, camera=camera)
        import mujoco

        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = bool(shadows)
        return renderer.render().copy()
    finally:
        model.geom_group[:] = original
