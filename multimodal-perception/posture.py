"""姿态/动作规则分类器：根据关键点几何关系判断人体状态。

不依赖训练好的动作模型，只用关键点坐标算角度和比例，适合 MVP 快速验证。
输出：posture（站立/坐/蹲/跪/躺）+ actions（捂肚子/捂头/蜷缩/跌倒）+ 身体指标

关键设计：
1. 所有判定尽量用"角度/比例"这类尺度无关的指标，避免因人在画面中远近不同导致误判。
2. 所有关键点先做可见性过滤，不可靠的关键点（如被遮挡的髋部）不参与计算，
   避免 MediaPipe 在低可见性时输出的噪声坐标污染判定结果。
"""

import math
from collections import deque

# 关键点索引（与 MediaPipe Pose 一致）
NOSE = 0
LEFT_SHOULDER, RIGHT_SHOULDER = 11, 12
LEFT_ELBOW, RIGHT_ELBOW = 13, 14
LEFT_WRIST, RIGHT_WRIST = 15, 16
LEFT_HIP, RIGHT_HIP = 23, 24
LEFT_KNEE, RIGHT_KNEE = 25, 26
LEFT_ANKLE, RIGHT_ANKLE = 27, 28

# 规则阈值
VIS_THRESH = 0.3         # 关键点可见性阈值，低于该值视为不可靠
TORSO_LYING_DEG = 65     # 躯干与竖直方向夹角超过该值判定为躺（接近水平；原 50 太松，前倾/弯腰会误判）
TORSO_SEMI_LYING_DEG = 50  # 半躺辅助：躯干角超过该值 + 髋踝竖直差小（腿也接近平放），才视为躺（兼顾蜷缩侧卧）
SEMI_LYING_HIP_ANKLE = 0.25  # 半躺辅助：髋-踝竖直差/身高小于该值，说明腿也放平了（区别于直立前倾时腿仍竖直）
HIP_RATIO_SITTING = 0.52  # 髋部在身体中的高度比例超过该值判定为坐（髋部偏低）
KNEE_SITTING_DEG = 135   # 膝关节角度小于该值判定为坐（膝盖弯曲；正面坐时膝盖前伸、2D 里看起来没那么弯，需放宽）
SQUAT_HIP_ANKLE = 0.28   # 蹲姿：髋-踝竖直差/身高，小于该值=屁股贴近地面（跨视角稳定，侧身也成立）
# 说明：坐姿的髋明显高于脚踝（髋-踝竖直差普遍 > 0.3），蹲姿屁股贴近脚踝（该值 < 0.28）；
# 用"髋相对脚踝位置"这一尺度无关指标区分蹲/坐，比单一 y 坐标更能覆盖浅蹲/亚洲蹲。
SQUAT_HIP_BELOW_KNEE = 0.02  # 髋-膝竖直差/身高大于该值 = 髋已降到膝盖以下（深蹲强信号，不依赖脚踝）
KNEEL_HIP_KNEE = -0.18   # 跪：髋明显高于膝（大腿接近竖直），髋-膝竖直差/身高小于该值
KNEEL_KNEE_ANKLE = 0.18  # 跪：膝盖接近脚踝（小腿贴地/折叠），踝-膝竖直差/身高小于该值
KNEEL_HIP_RATIO_MAX = 0.60  # 跪：髋部仍需偏高（躯干直立，未沉到坐姿高度），否则判坐，避免坐姿被误判成跪
KNEE_SPLAY_RATIO = 1.35     # 蹲姿补充：膝盖外张（膝宽 > 髋宽 1.35 倍）配合髋下沉，识别浅蹲/亚洲蹲
MIN_BODY_HEIGHT = 0.25   # 身体在画面中占比低于该值视为"离太远"，结果不可靠
HAND_BELLY_FACTOR = 0.5   # 手腕到腹部距离 < 0.5×躯干长 → 捂肚子（相对尺度）
HAND_BELLY_T_MIN = 0.15   # 手腕在躯干上的投影位置下限（0=肩，1=髋）
HAND_BELLY_T_MAX = 0.8    # 上限：超过该值（靠近髋部）不算捂肚子，排除手放胯部
HAND_BELLY_LATERAL = 0.25  # 手腕距躯干线的垂直距离 < 0.25×躯干长（排除叉腰/手放身体两侧）
HAND_BELLY_LATERAL_LYING = 0.6  # 躺姿放宽：躺着时手臂绕到身前抱腹，手腕横向偏移大，用更宽的阈值
HAND_BELLY_ELBOW_BENT = 150  # 捂肚子时手肘明显弯曲（手收到身前）；站立手自然垂放时肘接近伸直(170°+)，靠此排除误判
BELLY_HOLD_FACTOR = 0.45  # 双手在腹部区域（宽松兜底）：手腕到腹部中心 < 0.45×躯干长即算，覆盖蹲姿下躯干短、手收在身前的"双手捂腹"
HAND_HEAD_FACTOR = 0.6   # 手腕到头部距离 < 0.6×躯干长 → 捂头（相对尺度）
LEG_FOLD_CURLED = 0.7    # 髋-膝距离/躯干长度，小于该值判定膝盖收向胸口（蜷缩主信号）
FALL_FRAMES = 5          # 连续躺姿帧数阈值（跌倒/倒地不起）
POSTURE_HOLD_FRAMES = 4  # 新姿态需连续出现该帧数才确认切换（消除单帧误判/光影带来的标签闪烁）
WRITHING_WINDOW = 12     # 蠕动检测时间窗帧数（约 0.4s@30fps）
WRITHING_MOTION = 0.04   # 窗口内平均每帧身体中心位移（相对躯干长）超过该值视为运动剧烈
WRITHING_FLIPS = 4       # 窗口内位移方向翻转次数达到该值视为来回摆动（区别于走路）
PACING_WINDOW = 30       # 踱步检测时间窗帧数（约 1s@30fps）
PACING_FLIPS = 6         # 窗口内身体中心左右方向翻转次数达到该值视为来回踱步（区别于单向走路）
PACING_RANGE = 0.02      # 窗口内身体中心水平位移总跨度超过该值，排除原地小抖动


def _visible(p):
    return p[3] >= VIS_THRESH


def _midpoint_visible(a, b):
    """返回可见关键点的中点；若都不可见返回 None。"""
    pts = [p for p in (a, b) if _visible(p)]
    if not pts:
        return None
    return (sum(p[0] for p in pts) / len(pts),
            sum(p[1] for p in pts) / len(pts))


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _angle_deg(a, b, c):
    """三点 a-b-c 在 b 处的夹角（度）。"""
    v1 = (a[0] - b[0], a[1] - b[1])
    v2 = (c[0] - b[0], c[1] - b[1])
    d = math.hypot(*v1) * math.hypot(*v2)
    if d == 0:
        return 0.0
    cos = (v1[0] * v2[0] + v1[1] * v2[1]) / d
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def _torso_angle_deg(mid_shoulder, mid_hip):
    """躯干与竖直方向的夹角：0=直立，90=水平躺平。"""
    dx = mid_shoulder[0] - mid_hip[0]
    dy = mid_shoulder[1] - mid_hip[1]
    return math.degrees(math.atan2(abs(dx), abs(dy)))


class PostureClassifier:
    def __init__(self, fall_frames=FALL_FRAMES):
        self._prev_posture = None
        self._lying_count = 0
        self._fall_frames = fall_frames
        self._posture_confirmed = None   # 已确认（去抖后）的姿态标签
        self._posture_cand = None        # 候选新姿态
        self._posture_cand_count = 0     # 候选新姿态连续出现帧数
        self._center_hist = deque(maxlen=WRITHING_WINDOW)
        self._disp_hist = deque(maxlen=WRITHING_WINDOW)
        self._dir_hist = deque(maxlen=WRITHING_WINDOW)
        self._pace_hist = deque(maxlen=PACING_WINDOW)

    def _smooth(self, raw):
        """姿态去抖：新标签需连续出现 POSTURE_HOLD_FRAMES 帧才切换，抑制单帧误判造成的闪烁。"""
        if raw == self._posture_confirmed:
            self._posture_cand = None
            self._posture_cand_count = 0
            return raw
        if raw == self._posture_cand:
            self._posture_cand_count += 1
        else:
            self._posture_cand = raw
            self._posture_cand_count = 1
        if self._posture_cand_count >= POSTURE_HOLD_FRAMES:
            self._posture_confirmed = self._posture_cand
            self._posture_cand = None
            self._posture_cand_count = 0
            return self._posture_confirmed
        return self._posture_confirmed if self._posture_confirmed is not None else raw

    def classify(self, landmarks):
        """输入 pose_landmarks，返回 (posture, actions, metrics)。"""
        lm = landmarks.landmark

        def pt(i):
            return (lm[i].x, lm[i].y, lm[i].z, lm[i].visibility)

        # 只使用可见性达标的关键点
        mid_shoulder = _midpoint_visible(pt(LEFT_SHOULDER), pt(RIGHT_SHOULDER))
        mid_hip = _midpoint_visible(pt(LEFT_HIP), pt(RIGHT_HIP))
        mid_knee = _midpoint_visible(pt(LEFT_KNEE), pt(RIGHT_KNEE))
        mid_ankle = _midpoint_visible(pt(LEFT_ANKLE), pt(RIGHT_ANKLE))

        # 身体竖直范围：只用躯干+腿部核心关键点，不用手/手腕/手肘。
        # 否则坐下时手抬起（或搭在腿上）会把"身体高度"撑大，导致髋部比例算错，坐姿被误判成站/蹲。
        _core_ids = (NOSE, LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP,
                     LEFT_KNEE, RIGHT_KNEE, LEFT_ANKLE, RIGHT_ANKLE)
        vis_pts = [pt(i)[:2] for i in _core_ids if _visible(pt(i))]
        if not vis_pts:
            return "unknown", [], {"no_keypoints": True, "far": True}
        body_top = min(p[1] for p in vis_pts)
        body_bottom = max(p[1] for p in vis_pts)
        body_height = body_bottom - body_top
        far = body_height < MIN_BODY_HEIGHT

        # 朝向：鼻子可见说明脸朝向摄像头（或侧面），不可见说明背对
        facing = "front" if _visible(pt(NOSE)) else "back"

        # 姿态：先判躺（躯干水平），再判蹲（髋部贴近膝盖+多指标投票），再判坐，否则站立
        posture = "unknown"
        torso_angle = None
        hip_ratio = None
        knee_angle = None
        hip_ankle_ratio = None
        knee_hip_width_ratio = None
        hip_knee_gap = None
        knee_ankle_ratio = None
        butt_near_feet = None
        hip_below_knee = None
        if mid_shoulder is not None and mid_hip is not None:
            torso_angle = _torso_angle_deg(mid_shoulder, mid_hip)
            # 髋-踝竖直差/身高（提前算，供躺姿"腿是否放平"的辅助判断使用）
            hip_ankle_ratio = ((mid_ankle[1] - mid_hip[1]) / body_height
                               if mid_ankle is not None and body_height > 0 else None)
            lie_full = torso_angle > TORSO_LYING_DEG
            lie_semi = (torso_angle > TORSO_SEMI_LYING_DEG
                        and hip_ankle_ratio is not None
                        and hip_ankle_ratio < SEMI_LYING_HIP_ANKLE)
            if lie_full or lie_semi:
                posture = "lying"
            else:
                # 膝盖、脚踝都不可见 → 下半身被画面裁切，髋部成了最低点，hip_ratio 会失真成 1，
                # 无法可靠区分坐/蹲/跪/站，统一判 unknown，避免"腿出框=坐姿"的误判。
                legs_visible = (mid_knee is not None or mid_ankle is not None)
                if not legs_visible:
                    posture = "unknown"
                else:
                    hip_ratio = (mid_hip[1] - body_top) / body_height if body_height > 0 else 0.5
                    hip_ratio = max(0.0, min(1.0, hip_ratio))
                    butt_near_feet = (hip_ankle_ratio is not None
                                      and hip_ankle_ratio < SQUAT_HIP_ANKLE)
                    # 膝关节角度（蹲/坐/跪判定都要用）
                    if mid_knee is not None and mid_ankle is not None:
                        knee_angle = _angle_deg(mid_hip, mid_knee, mid_ankle)
                    if mid_knee is not None and body_height > 0:
                        # 髋-膝竖直差/身高（负=髋高于膝，正=髋低于膝）
                        hip_knee_gap = (mid_hip[1] - mid_knee[1]) / body_height
                        lk, rk = pt(LEFT_KNEE), pt(RIGHT_KNEE)
                        lh, rh = pt(LEFT_HIP), pt(RIGHT_HIP)
                        if _visible(lk) and _visible(rk) and _visible(lh) and _visible(rh):
                            knee_width = abs(lk[0] - rk[0])
                            hip_width = abs(lh[0] - rh[0])
                            if hip_width > 0:
                                knee_hip_width_ratio = knee_width / hip_width
                    if mid_knee is not None and mid_ankle is not None and body_height > 0:
                        # 膝-踝竖直差/身高（跪姿时膝盖贴地、小腿折叠，该值趋近 0）
                        knee_ankle_ratio = (mid_ankle[1] - mid_knee[1]) / body_height

                    # 蹲/跪的独立信号（尽量少依赖脚踝，脚踝在蹲/跪时最容易被遮挡）
                    hip_below_knee = bool(hip_knee_gap is not None and hip_knee_gap > SQUAT_HIP_BELOW_KNEE)

                    # 跪：膝盖贴地（膝≈踝，小腿折叠）+ 髋明显高于膝（大腿接近竖直）+ 髋部仍偏高（躯干直立，未沉到坐姿高度）
                    kneeling = bool(
                        hip_knee_gap is not None and knee_ankle_ratio is not None
                        and hip_knee_gap < KNEEL_HIP_KNEE
                        and knee_ankle_ratio < KNEEL_KNEE_ANKLE
                        and hip_ratio is not None and hip_ratio < KNEEL_HIP_RATIO_MAX
                    )

                    # 蹲：三个信号任一命中即判蹲（跨视角更稳）：
                    #   1) 屁股贴近脚（髋≈踝，深蹲主信号，侧身也成立）
                    #   2) 髋已降到膝盖以下（深蹲强信号，不依赖脚踝，脚踝被遮挡时兜底）
                    #   3) 膝盖明显向外张开(膝宽>髋宽) + 髋部下沉 —— 浅蹲/亚洲蹲正面特征
                    #      （蹲下时膝盖外八字、髋尚未完全贴地；坐姿膝盖不会明显外扩，故可区分）
                    knees_splayed = bool(
                        knee_hip_width_ratio is not None
                        and knee_hip_width_ratio > KNEE_SPLAY_RATIO)
                    hip_lowered = bool(
                        hip_ratio is not None and hip_ratio > HIP_RATIO_SITTING)
                    squatting = False
                    if not kneeling:
                        squatting = (bool(butt_near_feet) or hip_below_knee
                                     or (knees_splayed and hip_lowered))

                    if kneeling:
                        posture = "kneeling"
                    elif squatting:
                        posture = "squatting"
                    elif hip_ratio > HIP_RATIO_SITTING and not butt_near_feet:
                        posture = "sitting"
                    elif knee_angle is not None and not butt_near_feet:
                        posture = "sitting" if knee_angle < KNEE_SITTING_DEG else "standing"
                    else:
                        posture = "standing"
        elif mid_hip is not None and mid_knee is not None and mid_ankle is not None:
            knee_angle = _angle_deg(mid_hip, mid_knee, mid_ankle)
            posture = "sitting" if knee_angle < KNEE_SITTING_DEG else "standing"

        # 动作
        actions = []
        hand_lateral = None
        hand_t = None
        torso_len = _dist(mid_shoulder, mid_hip) if mid_shoulder and mid_hip else 0.0
        if mid_shoulder is not None and mid_hip is not None and torso_len > 0:
            # 腹部中心：髋部向上 30% 处（下腹部，对应痛经/肠胃不适的捂肚子位置）
            belly = (mid_hip[0] + 0.3 * (mid_shoulder[0] - mid_hip[0]),
                     mid_hip[1] + 0.3 * (mid_shoulder[1] - mid_hip[1]))
            # 躯干向量（肩→髋）
            tvx = mid_hip[0] - mid_shoulder[0]
            tvy = mid_hip[1] - mid_shoulder[1]
            torso_len2 = torso_len * torso_len
            best_hand_dist = None
            for wrist, elbow, shoulder in (
                    (pt(LEFT_WRIST), pt(LEFT_ELBOW), pt(LEFT_SHOULDER)),
                    (pt(RIGHT_WRIST), pt(RIGHT_ELBOW), pt(RIGHT_SHOULDER))):
                if not _visible(wrist):
                    continue
                # 手腕在躯干上的投影位置 t（0=肩，1=髋）
                t = ((wrist[0] - mid_shoulder[0]) * tvx +
                     (wrist[1] - mid_shoulder[1]) * tvy) / torso_len2
                # 手腕到躯干线的垂直距离（横向偏移）
                lateral = abs((wrist[0] - mid_shoulder[0]) * tvy -
                              (wrist[1] - mid_shoulder[1]) * tvx) / torso_len
                if hand_t is None or _dist(wrist, belly) < best_hand_dist:
                    hand_t = t
                    hand_lateral = lateral / torso_len
                    best_hand_dist = _dist(wrist, belly)
                # 肘部弯曲约束：真按腹时手肘明显弯曲、手收到身前；站立自然垂放时肘接近伸直，靠它排除误判。
                # 躺姿时手臂绕到身前抱腹、姿态复杂、肘角不稳定，故不在躺姿启用此约束。
                elbow_bent = True
                if posture != "lying" and _visible(elbow) and _visible(shoulder):
                    elbow_bent = _angle_deg(shoulder, elbow, wrist) < HAND_BELLY_ELBOW_BENT
                # 捂肚子：手腕在躯干中段（排除手放胯部/垂放）、靠近躯干线（排除叉腰）、肘弯曲
                lateral_limit = (HAND_BELLY_LATERAL_LYING if posture == "lying"
                                 else HAND_BELLY_LATERAL)
                if (HAND_BELLY_T_MIN < t < HAND_BELLY_T_MAX and
                        lateral < lateral_limit * torso_len and
                        _dist(wrist, belly) < HAND_BELLY_FACTOR * torso_len and
                        elbow_bent):
                    actions.append("hand_on_belly")
                    break
            nose = pt(NOSE)
            if _visible(nose):
                head_hands = 0
                for wrist in (pt(LEFT_WRIST), pt(RIGHT_WRIST)):
                    if _visible(wrist) and _dist(wrist, nose) < HAND_HEAD_FACTOR * torso_len:
                        head_hands += 1
                # 双手都靠近头部才算"抱头"（不适/头晕的典型姿势），单手摸脸/托腮/擦汗不算，避免误报
                if head_hands >= 2:
                    actions.append("hand_on_head")

            # 双手都在腹部区域（宽松兜底组合信号）：蹲姿/坐姿下躯干短、严格的"捂肚子"
            # 常因横向偏移或肘角失配而漏判；这里只判"双手手腕是否都落在腹部附近"，
            # 作为"难受关怀"的触发依据，不依赖蹲/坐姿态分类结果。
            belly_hands = 0
            for wrist in (pt(LEFT_WRIST), pt(RIGHT_WRIST)):
                if _visible(wrist) and _dist(wrist, belly) < BELLY_HOLD_FACTOR * torso_len:
                    belly_hands += 1
            if belly_hands >= 2:
                actions.append("belly_hold")

        # 膝盖收起程度：髋-膝距离相对躯干长度（蜷缩时膝盖收向胸口，该比值明显变小）
        leg_fold = None
        if mid_hip is not None and mid_knee is not None and torso_len > 0:
            leg_fold = _dist(mid_hip, mid_knee) / torso_len
            if posture == "lying" and leg_fold < LEG_FOLD_CURLED:
                actions.append("curled_up")

        # 跌倒/倒地不起：连续躺姿达到阈值帧数
        if posture == "lying":
            self._lying_count += 1
        else:
            self._lying_count = 0
        if self._lying_count >= self._fall_frames:
            actions.append("fallen")
        self._prev_posture = posture

        # 床上打滚检测（仅躺姿）：身体中心在时间窗内高频、不规则地来回移动（疼痛翻滚的典型表现）。
        # 只认"躺姿 + 来回翻滚"；站立/坐着/蹲着时的晃动都不计入，避免走动/扭动被误判成打滚。
        # 非躺姿时清空历史轨迹，防止起身再躺下后被上一次的旧中心点误判成"打滚"。
        writhing = False
        writ_motion = None
        writ_flips = None
        if mid_hip is not None and torso_len > 0 and posture == "lying":
            self._center_hist.append(mid_hip)
            if len(self._center_hist) >= 2:
                dx = self._center_hist[-1][0] - self._center_hist[-2][0]
                dy = self._center_hist[-1][1] - self._center_hist[-2][1]
                self._dir_hist.append((dx, dy))
                self._disp_hist.append(math.hypot(dx, dy) / torso_len)
            if len(self._disp_hist) >= 4:
                writ_motion = sum(self._disp_hist) / len(self._disp_hist)
                flips = 0
                for i in range(1, len(self._dir_hist)):
                    if self._dir_hist[i][0] * self._dir_hist[i - 1][0] < 0:
                        flips += 1
                    if self._dir_hist[i][1] * self._dir_hist[i - 1][1] < 0:
                        flips += 1
                writ_flips = flips
                if writ_motion > WRITHING_MOTION and flips >= WRITHING_FLIPS:
                    writhing = True
                    actions.append("writhing")
        else:
            self._center_hist.clear()
            self._disp_hist.clear()
            self._dir_hist.clear()

        # 踱步检测：站立下身体中心左右来回移动（夜里来回踱步/坐立不安），区别于匀速单向走路
        if posture == "standing" and mid_hip is not None:
            self._pace_hist.append(mid_hip[0])
            if len(self._pace_hist) == PACING_WINDOW:
                xs = list(self._pace_hist)
                flips = 0
                for i in range(2, len(xs)):
                    d_prev = xs[i - 1] - xs[i - 2]
                    d_cur = xs[i] - xs[i - 1]
                    if d_prev * d_cur < 0 and d_cur != 0:
                        flips += 1
                if flips >= PACING_FLIPS and (max(xs) - min(xs)) >= PACING_RANGE:
                    actions.append("pacing")
        else:
            self._pace_hist.clear()

        # 身体指标
        metrics = {
            "torso_angle_deg": round(torso_angle, 1) if torso_angle is not None else None,
            "knee_angle_deg": round(knee_angle, 1) if knee_angle is not None else None,
            "hip_ratio": round(hip_ratio, 3) if hip_ratio is not None else None,
            "hip_ankle_ratio": round(hip_ankle_ratio, 3) if hip_ankle_ratio is not None else None,
            "hip_knee_gap": round(hip_knee_gap, 3) if hip_knee_gap is not None else None,
            "knee_ankle_ratio": round(knee_ankle_ratio, 3) if knee_ankle_ratio is not None else None,
            "knee_hip_width_ratio": round(knee_hip_width_ratio, 3) if knee_hip_width_ratio is not None else None,
            "body_height_ratio": round(body_height, 3),
            "center_x": round(mid_hip[0], 4) if mid_hip is not None else None,
            "center_y": round(mid_hip[1], 4) if mid_hip is not None else None,
            "leg_fold_ratio": round(leg_fold, 3) if leg_fold is not None else None,
            "hand_lateral_ratio": round(hand_lateral, 3) if hand_lateral is not None else None,
            "hand_t": round(hand_t, 3) if hand_t is not None else None,
            "writhing_motion": round(writ_motion, 4) if writ_motion is not None else None,
            "writhing_flips": writ_flips,
            "facing": facing,
            "far": far,
        }

        posture = self._smooth(posture)
        return posture, actions, metrics
