#!/usr/bin/env python3
"""Pilot-facing video overlay for FAST-LIVO2 assist suggestions.

This node is intentionally a HUD, not a flight controller. It consumes the
assist planner's key-value status and draws the recommendation onto a camera
image so rosbag replay can stand in for the pilot's first-person view.
"""

import math
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


def parse_kv(text):
    out = {}
    if not text:
        return out
    for item in text.split(";"):
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        out[key.strip()] = value.strip()
    return out


def to_float(value, default=float("nan")):
    try:
        if value is None or value == "" or value == "NA":
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class AssistVideoOverlay:
    def __init__(self):
        self.bridge = CvBridge()
        self.status = {}
        self.health = {}
        self.goal = None
        self.odom = None
        self.last_status_wall = 0.0
        self.last_health_wall = 0.0
        self.last_goal_wall = 0.0
        self.last_odom_wall = 0.0
        self.filtered_bearing_deg = None
        self.filtered_distance_m = None
        self.filtered_dz_m = None
        self.last_direction_label = "HOLD HEADING"
        self.last_label_change_wall = 0.0

        self.image_topic = rospy.get_param("~image_topic", "/left_camera/image")
        self.compressed_image_topic = rospy.get_param(
            "~compressed_image_topic", "/left_camera/image/compressed"
        )
        self.use_compressed_input = rospy.get_param("~use_compressed_input", False)
        self.status_topic = rospy.get_param("~status_topic", "/fast_livo2_assist/status")
        self.health_topic = rospy.get_param("~health_topic", "/fast_livo2_health/advice")
        self.goal_topic = rospy.get_param("~goal_topic", "/fast_livo2_assist/recommended_goal")
        self.odom_topic = rospy.get_param("~odom_topic", "/aft_mapped_to_init")
        self.overlay_topic = rospy.get_param(
            "~overlay_topic", "/fast_livo2_assist/overlay_image"
        )
        self.publish_compressed = rospy.get_param("~publish_compressed", True)
        self.status_timeout_s = float(rospy.get_param("~status_timeout_s", 1.5))
        self.goal_timeout_s = float(rospy.get_param("~goal_timeout_s", 1.5))
        self.odom_timeout_s = float(rospy.get_param("~odom_timeout_s", 1.5))
        self.horizontal_fov_deg = float(rospy.get_param("~horizontal_fov_deg", 90.0))
        self.invert_bearing = rospy.get_param("~invert_bearing", False)
        self.draw_debug_numbers = rospy.get_param("~draw_debug_numbers", True)
        self.bearing_smoothing_alpha = float(rospy.get_param("~bearing_smoothing_alpha", 0.18))
        self.distance_smoothing_alpha = float(rospy.get_param("~distance_smoothing_alpha", 0.25))
        self.z_smoothing_alpha = float(rospy.get_param("~z_smoothing_alpha", 0.25))
        self.bearing_deadband_deg = float(rospy.get_param("~bearing_deadband_deg", 12.0))
        self.slight_turn_threshold_deg = float(rospy.get_param("~slight_turn_threshold_deg", 18.0))
        self.strong_turn_threshold_deg = float(rospy.get_param("~strong_turn_threshold_deg", 35.0))
        self.min_label_hold_s = float(rospy.get_param("~min_label_hold_s", 0.8))
        self.z_deadband_m = float(rospy.get_param("~z_deadband_m", 0.25))
        self.nav_ball_radius_px = int(rospy.get_param("~nav_ball_radius_px", 96))
        self.nav_ball_z_scale_m = float(rospy.get_param("~nav_ball_z_scale_m", 1.5))
        self.standoff_min_m = float(rospy.get_param("~standoff_min_m", 1.5))
        self.standoff_ideal_m = float(rospy.get_param("~standoff_ideal_m", 3.0))
        self.standoff_max_m = float(rospy.get_param("~standoff_max_m", 6.0))

        self.pub_image = rospy.Publisher(self.overlay_topic, Image, queue_size=1)
        self.pub_compressed = None
        if self.publish_compressed:
            self.pub_compressed = rospy.Publisher(
                self.overlay_topic + "/compressed", CompressedImage, queue_size=1
            )

        rospy.Subscriber(self.status_topic, String, self.status_cb, queue_size=10)
        rospy.Subscriber(self.health_topic, String, self.health_cb, queue_size=10)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_cb, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)
        if self.use_compressed_input:
            rospy.Subscriber(
                self.compressed_image_topic,
                CompressedImage,
                self.compressed_image_cb,
                queue_size=1,
                buff_size=2 ** 24,
            )
            rospy.loginfo("[assist_overlay] using compressed image input: %s",
                          self.compressed_image_topic)
        else:
            rospy.Subscriber(
                self.image_topic,
                Image,
                self.image_cb,
                queue_size=1,
                buff_size=2 ** 24,
            )
            rospy.loginfo("[assist_overlay] using raw image input: %s", self.image_topic)

        rospy.logwarn(
            "[assist_overlay] pilot HUD only; publishes images, never flight-control commands"
        )

    def status_cb(self, msg):
        self.status = parse_kv(msg.data)
        self.last_status_wall = time.time()
        self.update_smoothed_status()

    def health_cb(self, msg):
        self.health = parse_kv(msg.data)
        self.last_health_wall = time.time()

    def goal_cb(self, msg):
        self.goal = msg
        self.last_goal_wall = time.time()
        self.update_smoothed_z()

    def odom_cb(self, msg):
        self.odom = msg
        self.last_odom_wall = time.time()
        self.update_smoothed_z()

    def update_smoothed_status(self):
        raw_bearing = to_float(self.status.get("target_bearing_deg"), None)
        if raw_bearing is not None and math.isfinite(raw_bearing):
            if self.invert_bearing:
                raw_bearing = -raw_bearing
            raw_bearing = clamp(raw_bearing, -85.0, 85.0)
            self.filtered_bearing_deg = self.smooth_scalar(
                self.filtered_bearing_deg, raw_bearing, self.bearing_smoothing_alpha
            )

        raw_distance = to_float(self.status.get("target_distance_m"), None)
        if raw_distance is not None and math.isfinite(raw_distance):
            self.filtered_distance_m = self.smooth_scalar(
                self.filtered_distance_m, raw_distance, self.distance_smoothing_alpha
            )

    def update_smoothed_z(self):
        if self.goal is None or self.odom is None:
            return
        raw_dz = self.goal.pose.position.z - self.odom.pose.pose.position.z
        if math.isfinite(raw_dz):
            self.filtered_dz_m = self.smooth_scalar(
                self.filtered_dz_m, raw_dz, self.z_smoothing_alpha
            )

    def smooth_scalar(self, previous, current, alpha):
        alpha = clamp(alpha, 0.0, 1.0)
        if previous is None or not math.isfinite(previous):
            return current
        return alpha * current + (1.0 - alpha) * previous

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "[assist_overlay] cv_bridge failed: %s", exc)
            return
        overlay = self.draw_overlay(frame)
        self.publish(overlay, msg.header)

    def compressed_image_cb(self, msg):
        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn_throttle(2.0, "[assist_overlay] failed to decode compressed image")
            return
        overlay = self.draw_overlay(frame)
        self.publish(overlay, msg.header)

    def publish(self, frame, header):
        try:
            out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            out.header = header
            self.pub_image.publish(out)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "[assist_overlay] publish image failed: %s", exc)

        if self.pub_compressed is not None:
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            if ok:
                msg = CompressedImage()
                msg.header = header
                msg.format = "jpeg"
                msg.data = encoded.tobytes()
                self.pub_compressed.publish(msg)

    def draw_overlay(self, frame):
        img = frame.copy()
        h, w = img.shape[:2]
        now = time.time()
        status_age = now - self.last_status_wall if self.last_status_wall else float("inf")
        status_fresh = status_age <= self.status_timeout_s

        goal_age = (now - self.last_goal_wall) if self.last_goal_wall else float("inf")
        odom_age = (now - self.last_odom_wall) if self.last_odom_wall else float("inf")
        goal_fresh = goal_age <= self.goal_timeout_s
        odom_fresh = odom_age <= self.odom_timeout_s

        # Clear smoothed dz when goal or odom data is stale so the flight
        # director does not keep showing a stale climb/descend cue.
        if not goal_fresh or not odom_fresh:
            self.filtered_dz_m = None

        mode = self.status.get("mode", "DISCONNECTED" if not status_fresh else "UNKNOWN")
        pilot_mode = self.status.get("pilot_mode", "FREE_ASSIST")
        pilot_intent = self.status.get("pilot_intent", "FREE")
        health = self.status.get("health_state", self.health.get("state", "UNKNOWN"))
        confidence = self.status.get("confidence_level", self.health.get("confidence_level", "NA"))
        reason = self.status.get("suppressed_reason", "")
        prompt = self.prompt_for(mode, health, reason, status_fresh)
        color = self.color_for(mode, health, status_fresh)

        self.draw_top_bar(img, mode, pilot_mode, pilot_intent, health, confidence, color, status_fresh)
        self.draw_center_reticle(img)

        # Show flight director only when status, goal, and odom are all fresh.
        # If any is stale the height cue would be outdated, so fall back to HOLD.
        if status_fresh and goal_fresh and odom_fresh and mode not in ("SUPPRESSED", "DISCONNECTED"):
            self.draw_flight_director(img, color)
            self.draw_standoff_bar(img, color)
        else:
            self.draw_hold_symbol(img, color)

        self.draw_bottom_prompt(img, prompt, color)
        if self.draw_debug_numbers:
            self.draw_numbers(img, color, status_age)
        return img

    def draw_top_bar(self, img, mode, pilot_mode, pilot_intent, health, confidence, color, status_fresh):
        h, w = img.shape[:2]
        cv2.rectangle(img, (0, 0), (w, 58), (18, 20, 24), -1)
        cv2.rectangle(img, (0, 58), (w, 61), color, -1)
        title = "FAST-LIVO2 ASSIST HUD"
        cv2.putText(img, title, (24, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.85,
                    (245, 245, 245), 2, cv2.LINE_AA)

        status_text = "ASSIST STALE" if not status_fresh else "{} / {}".format(health, mode)
        (tw, _), _ = cv2.getTextSize(status_text, cv2.FONT_HERSHEY_SIMPLEX, 0.72, 2)
        cv2.putText(img, status_text, (w - tw - 24, 36), cv2.FONT_HERSHEY_SIMPLEX,
                    0.72, color, 2, cv2.LINE_AA)

        intent_text = "INTENT {} / {}   CONF {}".format(pilot_mode, pilot_intent, confidence)
        cv2.putText(img, intent_text, (24, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (230, 230, 230), 1, cv2.LINE_AA)

    def draw_center_reticle(self, img):
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.line(img, (cx - 18, cy), (cx - 6, cy), (210, 210, 210), 1, cv2.LINE_AA)
        cv2.line(img, (cx + 6, cy), (cx + 18, cy), (210, 210, 210), 1, cv2.LINE_AA)
        cv2.line(img, (cx, cy - 18), (cx, cy - 6), (210, 210, 210), 1, cv2.LINE_AA)
        cv2.line(img, (cx, cy + 6), (cx, cy + 18), (210, 210, 210), 1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), 4, (210, 210, 210), 1, cv2.LINE_AA)

    def draw_guidance_arrow(self, img, color):
        h, w = img.shape[:2]
        bearing = self.filtered_bearing_deg
        if bearing is None or not math.isfinite(bearing):
            bearing = 0.0

        fov = math.radians(max(10.0, min(170.0, self.horizontal_fov_deg)))
        b = math.radians(clamp(bearing, -85.0, 85.0))
        x_norm = -math.tan(b) / max(0.1, math.tan(fov * 0.5))
        x_norm = clamp(x_norm, -0.88, 0.88)

        start = (w // 2, int(h * 0.78))
        end = (int(w * (0.5 + 0.38 * x_norm)), int(h * 0.46))
        if self.status.get("mode") == "RETREAT_OR_HOLD":
            end = (int(w * (0.5 - 0.25 * x_norm)), int(h * 0.86))

        shadow = (0, 0, 0)
        cv2.arrowedLine(img, start, end, shadow, 10, cv2.LINE_AA, tipLength=0.18)
        cv2.arrowedLine(img, start, end, color, 6, cv2.LINE_AA, tipLength=0.18)

        label = self.direction_label(bearing)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.85, 2)
        tx = int(clamp(end[0] - tw / 2, 12, w - tw - 12))
        ty = int(clamp(end[1] - 22, 92, h - 80))
        self.draw_label(img, label, (tx, ty), color, 0.85)

    def draw_goal_box(self, img, color):
        h, w = img.shape[:2]
        distance = self.filtered_distance_m
        text = "TARGET"
        if distance is not None and math.isfinite(distance):
            text += " {:.1f} m".format(distance)
        x, y = int(w * 0.66), int(h * 0.72)
        self.draw_label(img, text, (x, y), color, 0.65)

    def draw_flight_director(self, img, color):
        h, w = img.shape[:2]
        radius = int(clamp(self.nav_ball_radius_px, 64, min(w, h) * 0.22))
        cx = w - radius - 34
        cy = h - radius - 144
        if cx - radius < 12:
            cx = w - radius - 12
        if cy - radius < 92:
            cy = 92 + radius

        bearing = self.filtered_bearing_deg
        if bearing is None or not math.isfinite(bearing):
            bearing = 0.0
        dz = self.filtered_dz_m
        if dz is None or not math.isfinite(dz):
            dz = 0.0

        # Instrument cue, not AR projection. The pilot keeps the green target
        # dot near the center instead of chasing a long image-space arrow.
        x_norm = clamp(bearing / 85.0, -1.0, 1.0)
        y_norm = clamp(-dz / max(0.1, self.nav_ball_z_scale_m), -1.0, 1.0)
        dot_x = int(cx + x_norm * radius * 0.72)
        dot_y = int(cy + y_norm * radius * 0.72)

        overlay = img.copy()
        cv2.circle(overlay, (cx, cy), radius + 12, (0, 0, 0), -1, cv2.LINE_AA)
        cv2.addWeighted(overlay, 0.68, img, 0.32, 0, img)

        cv2.circle(img, (cx, cy), radius, (105, 115, 125), 2, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), int(radius * 0.58), (70, 78, 86), 1, cv2.LINE_AA)
        cv2.line(img, (cx - radius + 12, cy), (cx + radius - 12, cy),
                 (95, 105, 115), 1, cv2.LINE_AA)
        cv2.line(img, (cx, cy - radius + 12), (cx, cy + radius - 12),
                 (95, 105, 115), 1, cv2.LINE_AA)

        cv2.circle(img, (cx, cy), int(radius * 0.16), (225, 225, 225), 1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), 3, (245, 245, 245), -1, cv2.LINE_AA)

        intent_bearing = to_float(self.status.get("intent_bearing_deg"), None)
        pilot_intent = self.status.get("pilot_intent", "FREE")
        if intent_bearing is not None and math.isfinite(intent_bearing) and pilot_intent not in ("FREE", "HOLD_STABLE"):
            intent_norm = clamp(intent_bearing / 85.0, -1.0, 1.0)
            intent_x = int(cx + intent_norm * radius * 0.72)
            intent_color = (180, 220, 255)
            cv2.line(img, (intent_x, cy - radius - 7), (intent_x, cy - radius + 13),
                     intent_color, 2, cv2.LINE_AA)
            cv2.circle(img, (intent_x, cy - radius + 3), 4, intent_color, -1, cv2.LINE_AA)
            cv2.putText(img, "INTENT", (intent_x - 28, cy - radius - 13),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, intent_color, 1, cv2.LINE_AA)

        cv2.line(img, (cx, cy), (dot_x, dot_y), color, 2, cv2.LINE_AA)
        cv2.circle(img, (dot_x, dot_y), 11, (0, 0, 0), -1, cv2.LINE_AA)
        cv2.circle(img, (dot_x, dot_y), 9, color, -1, cv2.LINE_AA)
        cv2.circle(img, (dot_x, dot_y), 13, color, 2, cv2.LINE_AA)

        cv2.putText(img, "L", (cx - radius + 12, cy - 8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(img, "R", (cx + radius - 25, cy - 8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(img, "UP", (cx - 14, cy - radius + 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(img, "DN", (cx - 14, cy + radius - 12), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (220, 220, 220), 1, cv2.LINE_AA)

        title = "FLIGHT DIRECTOR"
        (tw, _), _ = cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
        cv2.putText(img, title, (cx - tw // 2, cy - radius - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (245, 245, 245), 1, cv2.LINE_AA)

        direction = self.direction_label(bearing)
        cv2.putText(img, direction, (max(12, cx - radius), cy + radius + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
        cv2.putText(img, self.z_label(dz), (max(12, cx - radius), cy + radius + 56),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (245, 245, 245), 1, cv2.LINE_AA)

    def draw_standoff_bar(self, img, color):
        h, w = img.shape[:2]
        distance = self.filtered_distance_m
        if distance is None or not math.isfinite(distance):
            text = "STEP CHECK"
            bar_value = 0.0
        else:
            # "TARGET STEP" clarifies this is the distance to the recommended
            # next waypoint, not the UAV-to-surface standoff distance.
            text = "TARGET STEP {:.1f} m".format(distance)
            bar_value = (distance - self.standoff_min_m) / max(0.1, self.standoff_max_m - self.standoff_min_m)
            bar_value = clamp(bar_value, 0.0, 1.0)

        x0, y0 = 26, h - 168
        width, height = 330, 20
        cv2.rectangle(img, (x0 - 12, y0 - 34), (x0 + width + 12, y0 + height + 36),
                      (0, 0, 0), -1)
        cv2.putText(img, text, (x0, y0 - 12), cv2.FONT_HERSHEY_SIMPLEX,
                    0.60, (245, 245, 245), 1, cv2.LINE_AA)
        cv2.rectangle(img, (x0, y0), (x0 + width, y0 + height), (95, 105, 115), 2)

        ideal = (self.standoff_ideal_m - self.standoff_min_m) / max(0.1, self.standoff_max_m - self.standoff_min_m)
        ideal_x = x0 + int(clamp(ideal, 0.0, 1.0) * width)
        cv2.line(img, (ideal_x, y0 - 5), (ideal_x, y0 + height + 5),
                 (245, 245, 245), 2, cv2.LINE_AA)

        marker_x = x0 + int(bar_value * width)
        cv2.rectangle(img, (x0 + 2, y0 + 2), (marker_x, y0 + height - 2), color, -1)
        cv2.circle(img, (marker_x, y0 + height // 2), 8, color, -1, cv2.LINE_AA)
        cv2.putText(img, "near", (x0, y0 + height + 34), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (210, 210, 210), 1, cv2.LINE_AA)
        cv2.putText(img, "far", (x0 + width - 24, y0 + height + 34), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (210, 210, 210), 1, cv2.LINE_AA)

    def draw_3d_cue(self, img, color):
        h, w = img.shape[:2]
        dz = self.filtered_dz_m
        if dz is None or not math.isfinite(dz):
            z_label = "ALT: CHECK"
            z_dir = 0
        elif dz > self.z_deadband_m:
            z_label = "CLIMB +{:.1f} m".format(dz)
            z_dir = -1
        elif dz < -self.z_deadband_m:
            z_label = "DESCEND {:.1f} m".format(dz)
            z_dir = 1
        else:
            z_label = "LEVEL"
            z_dir = 0

        panel_w, panel_h = 210, 122
        x0, y0 = w - panel_w - 22, 92
        x1, y1 = w - 22, y0 + panel_h
        cv2.rectangle(img, (x0, y0), (x1, y1), (0, 0, 0), -1)
        cv2.rectangle(img, (x0, y0), (x1, y1), color, 2)
        cv2.putText(img, "3D CUE", (x0 + 14, y0 + 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.58, (245, 245, 245), 2, cv2.LINE_AA)
        cv2.putText(img, z_label, (x0 + 14, y0 + 60), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (245, 245, 245), 1, cv2.LINE_AA)

        cx = x0 + panel_w // 2
        cy = y0 + 92
        cv2.line(img, (cx - 70, cy), (cx + 70, cy), (110, 110, 110), 1, cv2.LINE_AA)
        cv2.line(img, (cx, cy - 26), (cx, cy + 26), (110, 110, 110), 1, cv2.LINE_AA)
        bearing = self.filtered_bearing_deg if self.filtered_bearing_deg is not None else 0.0
        bx = int(cx + clamp(bearing / 85.0, -1.0, 1.0) * 58.0)
        by = int(cy + z_dir * 22)
        cv2.circle(img, (bx, by), 7, color, -1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), 4, (245, 245, 245), 1, cv2.LINE_AA)

    def draw_hold_symbol(self, img, color):
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.circle(img, (cx, cy), 74, (0, 0, 0), 9, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), 72, color, 5, cv2.LINE_AA)
        cv2.line(img, (cx - 42, cy), (cx + 42, cy), color, 6, cv2.LINE_AA)
        self.draw_label(img, "HOLD / CHECK", (cx - 108, cy + 115), color, 0.85)

    def draw_bottom_prompt(self, img, prompt, color):
        h, w = img.shape[:2]
        cv2.rectangle(img, (0, h - 72), (w, h), (18, 20, 24), -1)
        cv2.rectangle(img, (0, h - 75), (w, h - 72), color, -1)
        cv2.putText(img, prompt, (24, h - 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.75, (245, 245, 245), 2, cv2.LINE_AA)

    def draw_numbers(self, img, color, status_age):
        h, w = img.shape[:2]
        fields = [
            ("intent", self.status.get("pilot_intent")),
            ("task", self.status.get("task_progress")),
            ("score", self.status.get("top_score")),
            ("coverage", self.status.get("coverage_gain")),
            ("risk", self.status.get("collision_risk")),
            ("unknown", self.status.get("path_unknown_ratio")),
            ("cand", self.status.get("candidate_count")),
            ("age", "{:.1f}".format(status_age) if math.isfinite(status_age) else "inf"),
        ]
        y = 116
        for key, value in fields:
            if value is None:
                continue
            cv2.putText(img, "{}: {}".format(key, value), (24, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (230, 230, 230), 1, cv2.LINE_AA)
            y += 22

    def draw_label(self, img, text, origin, color, scale):
        x, y = origin
        (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)
        cv2.rectangle(img, (x - 10, y - th - 10), (x + tw + 10, y + baseline + 10),
                      (0, 0, 0), -1)
        cv2.rectangle(img, (x - 10, y - th - 10), (x + tw + 10, y + baseline + 10),
                      color, 2)
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (245, 245, 245), 2, cv2.LINE_AA)

    def prompt_for(self, mode, health, reason, fresh):
        if not fresh:
            return "NO ASSIST DATA - HOLD AND CHECK"
        if mode == "EXPLORE":
            return "LOCAL RESCAN: keep green dot near center"
        if mode == "CONSERVATIVE":
            return "SLOW DOWN: short cautious movement only"
        if mode == "RETREAT_OR_HOLD":
            return "PULL BACK OR HOLD: keep structure in view"
        if mode == "RECOVER":
            return "RECOVER: small smooth motion only"
        if reason == "NO_PILOT_INTENT":
            return "SELECT INTENT: free mode shows health only"
        if reason == "PILOT_HOLD":
            return "HOLD: pilot requested stable hover/check"
        if reason:
            return "SUPPRESSED: {}".format(reason[:42])
        if health in ("CRITICAL", "ENDED", "STARTING"):
            return "HOLD: mapping evidence is not reliable"
        return "HOLD AND CHECK"

    def direction_label(self, bearing):
        if self.status.get("mode") == "RETREAT_OR_HOLD":
            return "RETREAT / INCREASE DISTANCE"
        if abs(bearing) <= self.bearing_deadband_deg:
            candidate = "HOLD HEADING / RESCAN"
        elif bearing >= self.strong_turn_threshold_deg:
            candidate = "MOVE LEFT"
        elif bearing <= -self.strong_turn_threshold_deg:
            candidate = "MOVE RIGHT"
        elif bearing >= self.slight_turn_threshold_deg:
            candidate = "SLIGHT LEFT"
        elif bearing <= -self.slight_turn_threshold_deg:
            candidate = "SLIGHT RIGHT"
        else:
            candidate = self.last_direction_label

        now = time.time()
        if (candidate != self.last_direction_label and
                now - self.last_label_change_wall < self.min_label_hold_s):
            return self.last_direction_label

        if candidate != self.last_direction_label:
            self.last_direction_label = candidate
            self.last_label_change_wall = now
        return self.last_direction_label

    def z_label(self, dz):
        if dz > self.z_deadband_m:
            return "CLIMB +{:.1f} m".format(dz)
        if dz < -self.z_deadband_m:
            return "DESCEND {:.1f} m".format(dz)
        return "LEVEL"

    def color_for(self, mode, health, fresh):
        if not fresh:
            return (120, 120, 120)
        # SUPPRESSED must override health-based color: even if health=NORMAL the
        # planner is not giving guidance, so green would mislead the pilot.
        if mode == "SUPPRESSED":
            return (120, 120, 120)
        if mode == "EXPLORE" or health == "NORMAL":
            return (60, 220, 80)
        if mode == "CONSERVATIVE" or health == "WATCH":
            return (0, 190, 255)
        if mode == "RETREAT_OR_HOLD" or health == "WARNING":
            return (0, 120, 255)
        if mode == "RECOVER" or health == "RECOVERING":
            return (255, 150, 60)
        return (40, 40, 230)


if __name__ == "__main__":
    rospy.init_node("fast_livo2_assist_video_overlay")
    AssistVideoOverlay()
    rospy.spin()
