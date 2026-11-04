"""Contact solver-derived vertical support loads (normal force magnitudes)."""
import numpy as np
import mujoco


def foot_ground_normal_loads(model, data, floor_geom_id, left_foot_geom_id, right_foot_geom_id):
    """Return (left, right) positive normal loads in N for floor/foot contacts.

    Uses actual MuJoCo contact solver forces rather than tiny site-based touch
    sensors, which can miss foot-edge contacts. Assumes a flat horizontal floor.
    """
    loads = {int(left_foot_geom_id): 0.0, int(right_foot_geom_id): 0.0}
    floor_id = int(floor_geom_id)
    for i in range(data.ncon):
        contact = data.contact[i]
        a, b = int(contact.geom1), int(contact.geom2)
        foot = b if a == floor_id else (a if b == floor_id else None)
        if foot not in loads:
            continue
        wrench = np.zeros(6, dtype=float)
        mujoco.mj_contactForce(model, data, i, wrench)
        normal = float(wrench[0])  # contact-frame normal component
        if np.isfinite(normal) and normal > 0.0:
            loads[foot] += normal
    return loads[int(left_foot_geom_id)], loads[int(right_foot_geom_id)]


def polygon_metrics_from_points(points, com_xy):
    """Signed static CoM support margin; undefined for degenerate contact hulls.

    Points and CoM XY are in the world frame. A zero-area contact line
    is NOT a valid 2D support polygon, even with multiple loaded contacts.
    This is a quasi-static diagnostic, not a dynamic stability proof.
    """
    import math
    pts = sorted(set((round(float(p[0]), 7), round(float(p[1]), 7)) for p in points))
    count = len(pts)
    if count < 3:
        return dict(valid=False, area_m2=0.0, margin_m=None)

    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]
    if len(hull) < 3:
        return dict(valid=False, area_m2=0.0, margin_m=None)

    area = abs(sum(a[0]*b[1]-b[0]*a[1]
                   for a,b in zip(hull, hull[1:]+hull[:1]))) / 2.0
    if area <= 1.0e-7:  # reject nearly collinear configurations
        return dict(valid=False, area_m2=area, margin_m=None)

    cx, cy = float(com_xy[0]), float(com_xy[1])
    margins = []
    for a,b in zip(hull, hull[1:]+hull[:1]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        edge_length = math.hypot(dx, dy)
        if edge_length > 1e-9:
            margins.append((dx*(cy-a[1])-dy*(cx-a[0]))/edge_length)
    return dict(valid=True, area_m2=area, margin_m=min(margins))


def support_polygon_from_contacts(model, data, floor_geom_id,
                                  left_foot_geom_id, right_foot_geom_id):
    """Use loaded contact-point XY coordinates from the MuJoCo solver."""
    floor = int(floor_geom_id)
    feet = {int(left_foot_geom_id), int(right_foot_geom_id)}
    pts = []
    for i in range(data.ncon):
        c = data.contact[i]
        a, b = int(c.geom1), int(c.geom2)
        if floor not in (a, b):
            continue
        other = b if a == floor else a
        if other not in feet or c.efc_address < 0:
            continue
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, force)
        if np.isfinite(force[0]) and force[0] > 0.1:
            pts.append((float(c.pos[0]), float(c.pos[1])))

    mass = float(model.body_mass.sum())
    com_xy = (model.body_mass[:,None] * data.xipos).sum(axis=0)[:2] / mass
    metrics = polygon_metrics_from_points(pts, com_xy)
    metrics['loaded_contact_points'] = len(pts)
    return metrics
