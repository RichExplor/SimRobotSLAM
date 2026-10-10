"""Validation for Web missions and conservative footprint checks on Nav2 costmaps."""

import json
import math
import uuid
import hashlib


def occupancy_fingerprint(message):
    info = message.info
    origin = [info.origin.position.x, info.origin.position.y, yaw_of(info.origin.orientation)]
    fields = [int(info.width), int(info.height), round(float(info.resolution), 9),
              [round(float(value), 9) for value in origin]]
    header = json.dumps(fields, separators=(",", ":")).encode("ascii")
    return hashlib.sha256(header + bytes(value & 255 for value in message.data)).hexdigest()


def number(value, label, minimum, maximum):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} 必须是有限数值")
    result = float(value)
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ValueError(f"{label} 超出范围")
    return result


def parse_mission(text):
    if not isinstance(text, str) or len(text) > 256000:
        raise ValueError("巡检任务配置大小无效")
    document = json.loads(text)
    if not isinstance(document, dict):
        raise ValueError("巡检任务必须是对象")
    for key in ("task_id", "map_id", "version_id"):
        document[key] = str(uuid.UUID(document[key]))
    if not isinstance(document.get("fingerprint"), str) or len(document["fingerprint"]) != 64:
        raise ValueError("地图摘要无效")
    stations = document.get("stations")
    if not isinstance(stations, list) or not 2 <= len(stations) <= 24:
        raise ValueError("巡检任务需要 2～24 个点位")
    seen = set()
    for index, station in enumerate(stations):
        if not isinstance(station, dict):
            raise ValueError("巡检点格式无效")
        node_id = station.get("node_id")
        if isinstance(node_id, bool) or not isinstance(node_id, int) or not 0 <= node_id <= 65535 or node_id in seen:
            raise ValueError("巡检节点 ID 无效或重复")
        seen.add(node_id)
        if not isinstance(station.get("name"), str) or not 1 <= len(station["name"]) <= 40:
            raise ValueError("巡检点名称无效")
        station["index"] = index
        for key in ("worldX", "worldY"):
            station[key] = number(station.get(key), key, -1e6, 1e6)
        station["heading"] = number(station.get("heading", 0), "到点朝向", -360, 360)
        actions = station.get("actions", [])
        if not isinstance(actions, list) or len(actions) > 16:
            raise ValueError("点位动作列表无效")
        for action in actions:
            if not isinstance(action, dict):
                raise ValueError("点位动作无效")
            if action.get("type") == "wait":
                action["duration_sec"] = number(action.get("duration_sec"), "等待时长", 0, 3600)
            elif action.get("type") == "spin":
                action["angle_deg"] = number(action.get("angle_deg"), "旋转角度", -3600, 3600)
                action["timeout_sec"] = number(action.get("timeout_sec", 30), "旋转超时", 1, 3600)
            else:
                raise ValueError("任务只支持等待和旋转")
        station["actions"] = actions
    return document


def yaw_of(quaternion):
    return math.atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1 - 2 * (quaternion.y**2 + quaternion.z**2))


def cells_on_line(first, second):
    x, y = first
    ex, ey = second
    column, row, ec, er = math.floor(x), math.floor(y), math.floor(ex), math.floor(ey)
    dx, dy = ex - x, ey - y
    sx = 1 if dx > 0 else -1 if dx < 0 else 0
    sy = 1 if dy > 0 else -1 if dy < 0 else 0
    tx = ((column + 1 - x) / dx if sx > 0 else (x - column) / -dx) if sx else math.inf
    ty = ((row + 1 - y) / dy if sy > 0 else (y - row) / -dy) if sy else math.inf
    dtx = 1 / abs(dx) if sx else math.inf
    dty = 1 / abs(dy) if sy else math.inf
    for _ in range(10000):
        yield column, row
        if column == ec and row == er:
            return
        if abs(tx - ty) < 1e-12 and tx <= 1 and ty <= 1:
            yield column + sx, row
            yield column, row + sy
            column += sx
            row += sy
            tx += dtx
            ty += dty
        elif tx < ty:
            column += sx
            tx += dtx
        else:
            row += sy
            ty += dty
    raise ValueError("轮廓栅格检查超出预算")


def validate_footprints(paths, stations, costmap, footprint):
    """Check filled robot polygons and all station rotations; refuse unknown cells."""
    info = costmap.info
    if (costmap.header.frame_id != "map" or not math.isfinite(info.resolution)
            or info.resolution <= 0 or info.width == 0 or info.height == 0
            or len(costmap.data) != info.width * info.height):
        raise ValueError("全局代价地图无效或坐标系不匹配")
    if not 3 <= len(footprint) <= 64 or any(not math.isfinite(v) for point in footprint for v in point):
        raise ValueError("机器人轮廓未接入")
    oyaw = yaw_of(info.origin.orientation)
    co, so = math.cos(oyaw), math.sin(oyaw)
    budget = 0

    def grid_point(x, y):
        dx, dy = x - info.origin.position.x, y - info.origin.position.y
        return ((co * dx + so * dy) / info.resolution,
                (-so * dx + co * dy) / info.resolution)

    def check_cell(column, row):
        nonlocal budget
        budget += 1
        if budget > 2000000:
            raise ValueError("路线轮廓校验超过预算，请缩短路线")
        if not 0 <= column < info.width or not 0 <= row < info.height:
            raise ValueError("机器人轮廓超出全局代价地图")
        value = costmap.data[row * info.width + column]
        if value < 0 or value >= 99:
            raise ValueError("机器人轮廓与障碍、膨胀禁行区或未知区域相交")

    def check_pose(x, y, yaw):
        cy, sy = math.cos(yaw), math.sin(yaw)
        polygon = [grid_point(x + cy * px - sy * py, y + sy * px + cy * py) for px, py in footprint]
        pairs = list(zip(polygon, polygon[1:] + polygon[:1]))
        for first, second in pairs:
            for column, row in cells_on_line(first, second):
                check_cell(column, row)
        min_x, max_x = math.floor(min(p[0] for p in polygon)), math.floor(max(p[0] for p in polygon))
        min_y, max_y = math.floor(min(p[1] for p in polygon)), math.floor(max(p[1] for p in polygon))
        if (max_x - min_x + 1) * (max_y - min_y + 1) > 10000:
            raise ValueError("机器人轮廓过大，无法校验")
        for row in range(min_y, max_y + 1):
            for column in range(min_x, max_x + 1):
                cx, cy_ = column + .5, row + .5
                inside = False
                for (ax, ay), (bx, by) in pairs:
                    if (ay > cy_) != (by > cy_) and cx < (bx - ax) * (cy_ - ay) / (by - ay) + ax:
                        inside = not inside
                if inside:
                    check_cell(column, row)

    radius = max(math.hypot(x, y) for x, y in footprint)
    angle_step = min(.2, info.resolution / max(2 * radius, .01))
    for path in paths:
        if path.header.frame_id != "map" or len(path.poses) < 2:
            raise ValueError("路线坐标系或路径点数量无效")
        previous = None
        for stamped in path.poses:
            pose = stamped.pose
            x, y, yaw = pose.position.x, pose.position.y, yaw_of(pose.orientation)
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                raise ValueError("路线含非有限坐标")
            if previous is not None:
                px, py, pyaw = previous
                delta = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw))
                count = max(1, math.ceil(math.hypot(x - px, y - py) / (info.resolution / 2)),
                            math.ceil(abs(delta) / angle_step))
                if count > 20000:
                    raise ValueError("路段过长，无法校验")
                for index in range(count):
                    fraction = index / count
                    check_pose(px + (x - px) * fraction, py + (y - py) * fraction,
                               pyaw + delta * fraction)
            check_pose(x, y, yaw)
            previous = x, y, yaw
    # Inspection stations may spin or reorient before the next leg.
    count = math.ceil(2 * math.pi / angle_step)
    if count > 20000:
        raise ValueError("点位旋转轮廓校验超出预算")
    for station in stations:
        for index in range(count):
            check_pose(station["worldX"], station["worldY"], index * 2 * math.pi / count)
