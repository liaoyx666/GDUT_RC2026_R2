#!/usr/bin/env python3
import rospy
import math
from std_srvs.srv import Empty
from std_msgs.msg import Float64, Bool

try:
    import tkinter as tk
    from tkinter import ttk
except Exception:
    # Fallback for environments without Tk (headless). Print instructions and exit.
    print("Tkinter not available. Please install python3-tk or use rostopic pub to command joints.")
    exit(0)


JOINTS = [
    ("joint1", "/arm_all/joint1_position_controller/command"),
    ("joint2", "/arm_all/joint2_position_controller/command"),
    ("joint3", "/arm_all/joint3_position_controller/command"),
    ("joint4", "/arm_all/joint4_position_controller/command"),
    ("joint5", "/arm_all/joint5_position_controller/command"),
    ("joint6", "/arm_all/joint6_position_controller/command"),
]


class SliderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Arm Joint Sliders (deg)")
        self.publishers = {}
        self.vars = {}
        # 真空吸盘命名空间前缀（例如 "/arm" 或 "/sunday"），用于服务与话题根路径
        self.vacuum_prefix = rospy.get_param("~vacuum_prefix", "/arm")

        # 参数改为角度制（GUI 显示/输入为度，发布前转换为弧度）
        # 优先读取 ~min_deg/~max_deg（度），若未提供则从 ~min/~max（弧度）转换
        param_min_deg = rospy.get_param("~min_deg", None)
        param_max_deg = rospy.get_param("~max_deg", None)
        if param_min_deg is None or param_max_deg is None:
            min_rad = rospy.get_param("~min", -math.pi)
            max_rad = rospy.get_param("~max", math.pi)
            self.min_deg = param_min_deg if param_min_deg is not None else math.degrees(min_rad)
            self.max_deg = param_max_deg if param_max_deg is not None else math.degrees(max_rad)
        else:
            self.min_deg = float(param_min_deg)
            self.max_deg = float(param_max_deg)
        self.step_deg = rospy.get_param("~step_deg", 1.0)  # ttk.Scale 无步进，仅用于文本输入时可参考

        main = ttk.Frame(root, padding=10)
        main.grid(sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        for idx, (name, topic) in enumerate(JOINTS):
            ttk.Label(main, text=name, width=8).grid(row=idx, column=0, sticky="w", padx=4, pady=4)
            var = tk.DoubleVar(value=0.0)
            scale = ttk.Scale(
                main,
                from_=self.min_deg,
                to=self.max_deg,
                orient=tk.HORIZONTAL,
                variable=var,
                command=lambda v, n=name: self.on_change(n),
            )
            scale.grid(row=idx, column=1, sticky="ew", padx=6)
            main.columnconfigure(1, weight=1)

            entry = ttk.Entry(main, width=8)
            entry.insert(0, "0.0")
            entry.grid(row=idx, column=2, padx=2)
            ttk.Label(main, text="deg").grid(row=idx, column=3, padx=2, sticky="w")

            # Bind entry to update slider (度)
            def make_setter(var_ref, scale_ref, entry_ref):
                def setter(event=None, joint=name):
                    try:
                        val_deg = float(entry_ref.get())
                    except ValueError:
                        return
                    val_deg = max(self.min_deg, min(self.max_deg, val_deg))
                    var_ref.set(val_deg)
                    scale_ref.configure(value=val_deg)
                    self.publish(joint, val_deg)
                return setter

            entry.bind('<Return>', make_setter(var, scale, entry))

            self.vars[name] = var
            self.publishers[name] = rospy.Publisher(topic, Float64, queue_size=10)

        # Reset button
        reset_btn = ttk.Button(main, text="Reset (0 deg)", command=self.reset_all)
        reset_btn.grid(row=len(JOINTS), column=0, columnspan=1, pady=8, sticky="w")

        # Vacuum gripper toggle buttons
        btns = ttk.Frame(main)
        btns.grid(row=len(JOINTS), column=1, columnspan=3, sticky="e")
        on_btn = ttk.Button(btns, text="All Vacuum ON", command=self.vacuum_all_on)
        off_btn = ttk.Button(btns, text="All Vacuum OFF", command=self.vacuum_all_off)
        on_btn.grid(row=0, column=0, padx=4)
        off_btn.grid(row=0, column=1, padx=4)

        # Prepare service proxies lazily
        self._vac_on = None
        self._vac_off = None
        self._vac_proxies = {}  # suffix -> (on_proxy, off_proxy)

        # Grasping status indicators
        status_frame = ttk.LabelFrame(main, text="Vacuum grasping status")
        status_frame.grid(row=len(JOINTS)+1, column=0, columnspan=4, sticky="ew", pady=(8, 0))
        status_frame.columnconfigure(1, weight=1)
        self._grasp_vars = {}    # suffix -> tk.StringVar("?"/"True"/"False")
        self._grasp_state = {}   # suffix -> latest bool or None
        self._grasp_topics = {}  # suffix -> topic name
        row = 0
        for suf in self._ns_suffixes():
            name = f"cup{suf if suf else '_0'}"
            topic = f"{self.vacuum_prefix}/vacuum_gripper{suf}/grasping{suf}"
            self._grasp_topics[suf] = topic
            ttk.Label(status_frame, text=name, width=10).grid(row=row, column=0, sticky="w", padx=4, pady=2)
            var = tk.StringVar(value="?")
            self._grasp_vars[suf] = var
            lbl = ttk.Label(status_frame, textvariable=var, width=8)
            lbl.grid(row=row, column=1, sticky="w")
            # subscriber
            rospy.Subscriber(topic, Bool, self._make_grasp_cb(suf), queue_size=10)
            row += 1

        # Periodic publish to hold positions (optional)
        self.timer_period = rospy.get_param("~hold_period", 0.2)
        self.root.after(int(self.timer_period * 1000), self.hold_publish)
        # Periodic status UI refresh (avoid Tk thread-safety issues)
        self.root.after(150, self._refresh_grasp_labels)

    def on_change(self, name):
        val_deg = self.vars[name].get()
        self.publish(name, val_deg)

    def publish(self, name, val_deg):
        # 将度转换为弧度再发布
        pub = self.publishers.get(name)
        if pub is not None:
            rad = math.radians(val_deg)
            pub.publish(Float64(rad))

    def reset_all(self):
        for name in self.vars:
            self.vars[name].set(0.0)
            self.publish(name, 0.0)

    def hold_publish(self):
        # republish current values to keep controllers latched to their setpoints
        for name in self.vars:
            self.publish(name, self.vars[name].get())
        self.root.after(int(self.timer_period * 1000), self.hold_publish)

    # Vacuum service helpers
    def _ensure_services(self):
        if self._vac_on is None or self._vac_off is None:
            try:
                base = f"{self.vacuum_prefix}/vacuum_gripper"
                rospy.wait_for_service(f'{base}/on', timeout=2.0)
                rospy.wait_for_service(f'{base}/off', timeout=2.0)
                self._vac_on = rospy.ServiceProxy(f'{base}/on', Empty)
                self._vac_off = rospy.ServiceProxy(f'{base}/off', Empty)
            except Exception as e:
                rospy.logwarn('Vacuum services not available: %s', e)
                self._vac_on = None
                self._vac_off = None

    def _ns_suffixes(self):
        return [""] + [f"_{i}" for i in range(1, 9)]

    def _ensure_all_services(self, timeout=0.8):
        # Prepare proxies for all 9 cups: /arm/vacuum_gripper{_i}/on|off
        for suf in self._ns_suffixes():
            if suf in self._vac_proxies and self._vac_proxies[suf] != (None, None):
                continue
            ns = f"{self.vacuum_prefix}/vacuum_gripper{suf}"
            on_srv = f"{ns}/on"
            off_srv = f"{ns}/off"
            try:
                rospy.wait_for_service(on_srv, timeout=timeout)
                rospy.wait_for_service(off_srv, timeout=timeout)
                on_proxy = rospy.ServiceProxy(on_srv, Empty)
                off_proxy = rospy.ServiceProxy(off_srv, Empty)
                self._vac_proxies[suf] = (on_proxy, off_proxy)
            except Exception as e:
                rospy.logwarn('Vacuum services not available for %s: %s', ns, e)
                self._vac_proxies[suf] = (None, None)

    # grasping callbacks and UI refresh
    def _make_grasp_cb(self, suf):
        def cb(msg):
            self._grasp_state[suf] = bool(msg.data)
        return cb

    def _refresh_grasp_labels(self):
        for suf in self._ns_suffixes():
            state = self._grasp_state.get(suf, None)
            if state is None:
                self._grasp_vars[suf].set("?")
            else:
                self._grasp_vars[suf].set("True" if state else "False")
        self.root.after(150, self._refresh_grasp_labels)

    def vacuum_on(self):
        self._ensure_services()
        if self._vac_on:
            try:
                self._vac_on()
                rospy.loginfo('Vacuum ON called')
            except Exception as e:
                rospy.logwarn('Vacuum ON failed: %s', e)

    def vacuum_off(self):
        self._ensure_services()
        if self._vac_off:
            try:
                self._vac_off()
                rospy.loginfo('Vacuum OFF called')
            except Exception as e:
                rospy.logwarn('Vacuum OFF failed: %s', e)

    def vacuum_all_on(self):
        self._ensure_all_services()
        for suf in self._ns_suffixes():
            ns = f"{self.vacuum_prefix}/vacuum_gripper{suf}"
            on_proxy, _ = self._vac_proxies.get(suf, (None, None))
            if on_proxy:
                try:
                    on_proxy()
                    rospy.loginfo('Vacuum ON: %s', ns)
                except Exception as e:
                    rospy.logwarn('Vacuum ON failed for %s: %s', ns, e)

    def vacuum_all_off(self):
        self._ensure_all_services()
        for suf in self._ns_suffixes():
            ns = f"{self.vacuum_prefix}/vacuum_gripper{suf}"
            _, off_proxy = self._vac_proxies.get(suf, (None, None))
            if off_proxy:
                try:
                    off_proxy()
                    rospy.loginfo('Vacuum OFF: %s', ns)
                except Exception as e:
                    rospy.logwarn('Vacuum OFF failed for %s: %s', ns, e)


def main():
    rospy.init_node("arm_joint_sliders", anonymous=True)
    root = tk.Tk()
    app = SliderApp(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()