import tkinter as tk

root = Tk()

class DebuggerPanel:
    def __init__(self, root):
        self.win = tk.Toplevel(root)
        self.win.title("Debugger Panel")

        self.left  = tk.Frame(self.win)
        self.left.grid(row=0, column=0, padx=12, pady=12, sticky="nw")

        self.right = tk.Frame(self.win)
        self.right.grid(row=0, column=1, padx=12, pady=12, sticky="ne")


       # -------- LEFT COLUMN: current (c_*) --------
        tk.Label(self.left, text="c_posX:").grid(row=0, column=0, sticky="w")
        self.c_posX_value = tk.Label(self.left, text="0.0")
        self.c_posX_value.grid(row=0, column=1, sticky="w")

        tk.Label(self.left, text="c_posY:").grid(row=1, column=0, sticky="w")
        self.c_posY_value = tk.Label(self.left, text="0.0")
        self.c_posY_value.grid(row=1, column=1, sticky="w")

        tk.Label(self.left, text="c_theta:").grid(row=2, column=0, sticky="w")
        self.c_theta_value = tk.Label(self.left, text="0.0")
        self.c_theta_value.grid(row=2, column=1, sticky="w")

        tk.Label(self.left, text="c_vix:").grid(row=3, column=0, sticky="w")
        self.c_vix_value = tk.Label(self.left, text="0.0")
        self.c_vix_value.grid(row=3, column=1, sticky="w")

        tk.Label(self.left, text="c_viy:").grid(row=4, column=0, sticky="w")
        self.c_viy_value = tk.Label(self.left, text="0.0")
        self.c_viy_value.grid(row=4, column=1, sticky="w")

        tk.Label(self.left, text="c_wi:").grid(row=5, column=0, sticky="w")
        self.c_wi_value = tk.Label(self.left, text="0.0")
        self.c_wi_value.grid(row=5, column=1, sticky="w")

        tk.Label(self.left, text="c_v:").grid(row=6, column=0, sticky="w")
        self.c_v_value = tk.Label(self.left, text="0.0")
        self.c_v_value.grid(row=6, column=1, sticky="w")

        tk.Label(self.left, text="c_w:").grid(row=7, column=0, sticky="w")
        self.c_w_value = tk.Label(self.left, text="0.0")
        self.c_w_value.grid(row=7, column=1, sticky="w")

        # -------- RIGHT COLUMN: destination (d_*) --------
        tk.Label(self.right, text="d_posX:").grid(row=0, column=0, sticky="w")
        self.d_posX_value = tk.Label(self.right, text="0.0")
        self.d_posX_value.grid(row=0, column=1, sticky="w")

        tk.Label(self.right, text="d_posY:").grid(row=1, column=0, sticky="w")
        self.d_posY_value = tk.Label(self.right, text="0.0")
        self.d_posY_value.grid(row=1, column=1, sticky="w")

        tk.Label(self.right, text="d_theta:").grid(row=2, column=0, sticky="w")
        self.d_theta_value = tk.Label(self.right, text="0.0")
        self.d_theta_value.grid(row=2, column=1, sticky="w")


root.mainloop()