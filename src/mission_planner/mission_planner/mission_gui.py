"""
mission_gui.py — GUI for the robot mission planner
Package : mission_planner

Run:  ros2 run mission_planner mission_gui
"""

import json
import os
import re
import threading
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


def _maps_dir() -> str:
    share = get_package_share_directory('my_local')
    ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    return os.path.join(ws_root, 'src', 'my_local', 'maps')


def _clean(name: str) -> str:
    """Strip trailing coordinate hints like '(0, -4)' from landmark names."""
    return re.sub(r'\s*\([^)]*\)\s*$', '', name).strip()


# ── ROS2 node ─────────────────────────────────────────────────────────

class MissionGUINode(Node):
    def __init__(self):
        super().__init__('mission_gui')
        self._pub        = self.create_publisher(String, '/mission_goal', 10)
        self._status_cb  = None
        self.create_subscription(String, '/mission_status', self._on_status, 10)

    def _on_status(self, msg: String):
        if self._status_cb:
            self._status_cb(msg.data)

    def send_goal(self, sequence: str):
        self._pub.publish(String(data=sequence))

    def set_status_callback(self, cb):
        self._status_cb = cb


# ── GUI ───────────────────────────────────────────────────────────────

class MissionGUI:

    BG       = '#1e1e2e'
    PANEL    = '#2a2a3e'
    DOCK_C   = '#2d6a4f'
    LM_C     = '#1a4a7a'
    ACCENT   = '#4dabf7'
    BTN_GO   = '#2f9e44'
    BTN_CLR  = '#c92a2a'
    BTN_CTRL = '#343a40'
    TEXT     = '#f0f0f0'
    SUBTEXT  = '#adb5bd'
    ROW_DEF  = '#181825'   # pending step
    ROW_CUR  = '#8a6000'   # current step  (dark yellow)
    ROW_DONE = '#1a4d2e'   # completed step (dark green)
    ROW_FAIL = '#6a1a1a'   # failed step    (dark red)
    FG_CUR   = '#ffd43b'
    FG_DONE  = '#69db7c'
    FG_FAIL  = '#ff8787'

    def __init__(self, root: tk.Tk, node: MissionGUINode,
                 landmarks: list[tuple[str, dict]]):
        self.root      = root
        self.node      = node
        self.landmarks = landmarks
        self.queue: list[tuple[int, str]] = []   # (1-based index, clean name)

        node.set_status_callback(self._on_status_thread)
        self._build()

    # ── layout ────────────────────────────────────────────────────────

    def _build(self):
        r = self.root
        r.title('Robot Mission Planner')
        r.configure(bg=self.BG)
        r.resizable(False, False)

        tk.Label(r, text='🤖  Robot Mission Planner',
                 font=tkfont.Font(family='Helvetica', size=16, weight='bold'),
                 bg=self.BG, fg=self.ACCENT
                 ).grid(row=0, column=0, columnspan=2, pady=(16, 6))

        self._build_left(r)
        self._build_right(r)
        self._build_status(r)

    def _build_left(self, r):
        f = tk.Frame(r, bg=self.PANEL, padx=14, pady=14)
        f.grid(row=1, column=0, sticky='ns', padx=(12, 6), pady=6)

        tk.Label(f, text='LANDMARKS',
                 font=('Helvetica', 10, 'bold'),
                 bg=self.PANEL, fg=self.SUBTEXT
                 ).pack(anchor='w', pady=(0, 10))

        for i, (name, _) in enumerate(self.landmarks, start=1):
            label = _clean(name)
            tk.Button(
                f,
                text=f'  {i}.  {label}   ＋',
                font=('Helvetica', 11),
                bg=self.LM_C, fg=self.TEXT,
                activebackground=self.ACCENT, activeforeground='#000',
                relief='flat', padx=14, pady=12,
                cursor='hand2', anchor='w',
                command=lambda n=i, l=label: self._add(n, l),
            ).pack(fill='x', pady=3)

    def _build_right(self, r):
        f = tk.Frame(r, bg=self.PANEL, padx=14, pady=14)
        f.grid(row=1, column=1, sticky='nsew', padx=(6, 12), pady=6)

        tk.Label(f, text='MISSION PATH',
                 font=('Helvetica', 10, 'bold'),
                 bg=self.PANEL, fg=self.SUBTEXT
                 ).pack(anchor='w', pady=(0, 6))

        # fixed dock — start
        tk.Label(f, text='🏠  Docking Station  (start)',
                 font=('Helvetica', 10, 'bold'),
                 bg=self.DOCK_C, fg=self.TEXT,
                 padx=12, pady=9, anchor='w'
                 ).pack(fill='x', pady=(0, 2))

        # step listbox
        lf = tk.Frame(f, bg=self.PANEL)
        lf.pack(fill='both', expand=True, pady=2)

        self._lb = tk.Listbox(
            lf, height=7, width=34,
            font=('Courier', 11),
            bg=self.ROW_DEF, fg=self.TEXT,
            selectbackground='#364fc7', selectforeground='white',
            relief='flat', bd=0, activestyle='none',
        )
        self._lb.pack(side='left', fill='both', expand=True)
        sb = tk.Scrollbar(lf, command=self._lb.yview)
        sb.pack(side='right', fill='y')
        self._lb.config(yscrollcommand=sb.set)

        self._hint = tk.Label(
            f, text='← Click a landmark to add it to the path',
            font=('Helvetica', 9, 'italic'),
            bg=self.PANEL, fg=self.SUBTEXT,
        )
        self._hint.pack(pady=2)

        # fixed dock — end
        self._dock_end = tk.Label(
            f, text='🏠  Docking Station  (end)',
            font=('Helvetica', 10, 'bold'),
            bg=self.DOCK_C, fg=self.TEXT,
            padx=12, pady=9, anchor='w',
        )
        self._dock_end.pack(fill='x', pady=(2, 8))

        # controls
        ctrl = tk.Frame(f, bg=self.PANEL)
        ctrl.pack(fill='x', pady=2)
        for txt, cmd in [('▲ Up', self._move_up),
                         ('▼ Down', self._move_down),
                         ('✕ Remove', self._remove)]:
            tk.Button(ctrl, text=txt, font=('Helvetica', 9),
                      bg=self.BTN_CTRL, fg=self.TEXT,
                      activebackground='#495057',
                      relief='flat', padx=8, pady=5, cursor='hand2',
                      command=cmd).pack(side='left', padx=2)

        tk.Button(f, text='▶   START MISSION',
                  font=('Helvetica', 13, 'bold'),
                  bg=self.BTN_GO, fg='white',
                  activebackground='#37b24d',
                  relief='flat', padx=12, pady=12, cursor='hand2',
                  command=self._start
                  ).pack(fill='x', pady=(10, 3))

        tk.Button(f, text='✕   CLEAR PATH',
                  font=('Helvetica', 10),
                  bg=self.BTN_CLR, fg='white',
                  activebackground='#e03131',
                  relief='flat', padx=12, pady=7, cursor='hand2',
                  command=self._clear
                  ).pack(fill='x', pady=3)

    def _build_status(self, r):
        f = tk.Frame(r, bg='#181825', pady=8)
        f.grid(row=2, column=0, columnspan=2, sticky='ew', padx=12, pady=(0, 10))

        self._status_var = tk.StringVar(value='IDLE — ready')
        self._status_lbl = tk.Label(
            f, textvariable=self._status_var,
            font=('Helvetica', 10, 'bold'),
            bg='#181825', fg=self.ACCENT,
            anchor='w', padx=12,
        )
        self._status_lbl.pack(fill='x')

    # ── queue operations ──────────────────────────────────────────────

    def _add(self, number: int, label: str):
        self.queue.append((number, label))
        self._refresh()

    def _remove(self):
        sel = self._lb.curselection()
        if sel:
            del self.queue[sel[0]]
            self._refresh()

    def _move_up(self):
        sel = self._lb.curselection()
        if sel and sel[0] > 0:
            i = sel[0]
            self.queue[i - 1], self.queue[i] = self.queue[i], self.queue[i - 1]
            self._refresh()
            self._lb.selection_set(i - 1)

    def _move_down(self):
        sel = self._lb.curselection()
        if sel and sel[0] < len(self.queue) - 1:
            i = sel[0]
            self.queue[i], self.queue[i + 1] = self.queue[i + 1], self.queue[i]
            self._refresh()
            self._lb.selection_set(i + 1)

    def _clear(self):
        self.queue.clear()
        self._dock_end.config(bg=self.DOCK_C)
        self._status_var.set('IDLE — ready')
        self._status_lbl.config(fg=self.ACCENT)
        self._refresh()

    def _refresh(self):
        self._lb.delete(0, 'end')
        for step, (_, label) in enumerate(self.queue, start=1):
            self._lb.insert('end', f'  Step {step:>2}.  {label}')
            self._lb.itemconfig(step - 1, bg=self.ROW_DEF, fg=self.TEXT)

        if self.queue:
            self._hint.pack_forget()
        else:
            self._hint.pack(pady=2)

    # ── mission control ───────────────────────────────────────────────

    def _start(self):
        if not self.queue:
            return
        # Reset all colours
        self._dock_end.config(bg=self.DOCK_C)
        for i in range(self._lb.size()):
            self._lb.itemconfig(i, bg=self.ROW_DEF, fg=self.TEXT)

        sequence = ','.join(str(num) for num, _ in self.queue)
        self.node.send_goal(sequence)
        self._status_var.set('Mission sent — waiting for robot…')
        self._status_lbl.config(fg=self.ACCENT)

    # ── status updates (called from ROS thread) ───────────────────────

    def _on_status_thread(self, status: str):
        self.root.after(0, lambda s=status: self._handle_status(s))

    def _handle_status(self, status: str):
        if status == 'DOCKED':
            self._status_var.set('✓  Mission complete — Docked')
            self._status_lbl.config(fg=self.FG_DONE)
            self._dock_end.config(bg='#1a4d2e', fg=self.FG_DONE)
            return

        # Parse:  KIND:step:total:label
        parts = status.split(':', 3)
        if len(parts) < 3 or parts[0] not in ('GOING', 'AT', 'FAILED'):
            return

        kind  = parts[0]
        step  = int(parts[1])
        total = int(parts[2])
        label = parts[3] if len(parts) > 3 else ''
        n_lm  = total - 1          # number of landmark steps (total excludes home)
        lb_i  = step - 1           # 0-based listbox index

        if kind == 'GOING':
            if lb_i < n_lm:        # navigating to a landmark
                self._status_var.set(
                    f'▶  Going to  "{label}"   (step {step} of {total})'
                )
                self._status_lbl.config(fg=self.FG_CUR)
                if lb_i < self._lb.size():
                    self._lb.itemconfig(lb_i, bg=self.ROW_CUR, fg=self.FG_CUR)
            else:                   # returning home
                self._status_var.set(
                    f'↩  Returning to dock   (step {step} of {total})'
                )
                self._status_lbl.config(fg='#ff8787')
                self._dock_end.config(bg=self.ROW_CUR, fg=self.FG_CUR)

        elif kind == 'AT':
            self._status_var.set(
                f'✓  At  "{label}"   (step {step} of {total}) — waiting 2 s'
            )
            self._status_lbl.config(fg=self.FG_DONE)
            if lb_i < self._lb.size():
                self._lb.itemconfig(lb_i, bg=self.ROW_DONE, fg=self.FG_DONE)

        elif kind == 'FAILED':
            self._status_var.set(
                f'✗  Failed at  "{label}"   (step {step} of {total})'
            )
            self._status_lbl.config(fg=self.FG_FAIL)
            if lb_i < n_lm and lb_i < self._lb.size():
                self._lb.itemconfig(lb_i, bg=self.ROW_FAIL, fg=self.FG_FAIL)


# ── entry point ───────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MissionGUINode()

    db_path = os.path.join(_maps_dir(), 'landmark_db.json')
    with open(db_path, 'r') as f:
        landmarks = list(json.load(f).items())

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    root = tk.Tk()
    MissionGUI(root, node, landmarks)
    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()