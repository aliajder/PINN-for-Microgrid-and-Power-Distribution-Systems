import os

os.environ.setdefault("DDE_BACKEND", "pytorch")

import datetime
import re

import deepxde as dde
import numpy as np
import pandas as pd

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
HD_FROM_NAME = re.compile(r"H(?P<H>[0-9]+(?:\.[0-9]+)?)D(?P<D>[0-9]+(?:\.[0-9]+)?)", re.I)


def resolve_data_path(path):
    if path is None:
        return path
    if os.path.isabs(path) and os.path.exists(path):
        return path
    candidates = [
        path,
        os.path.join(os.getcwd(), path),
        os.path.join(REPO_ROOT, path),
        os.path.join(REPO_ROOT, "data", "synchronous-machine", os.path.basename(path)),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return os.path.abspath(candidate)
    raise FileNotFoundError("Could not find data file: %s" % path)


def parse_true_hd(path):
    name = os.path.basename(path)
    match = HD_FROM_NAME.search(name)
    if not match:
        return None, None
    return float(match.group("H")), float(match.group("D"))


def last_short_name(column):
    return str(column).split("|")[-1]


class SYNC:
    def __init__(self, path=None, date=None, index=None, **kwargs):
        self.st = 0.0
        self.et = 1.0
        self.num_domain = 0
        self.num_boundary = 0
        self.num_test = None
        self.prompt_on_dat_conflict = False
        self.output_dir = os.path.join(REPO_ROOT, "results")
        self.save_model_dir = os.path.join(REPO_ROOT, "save_model")

        self.net = dde.nn.FNN([1] + [100] * 3 + [3], "swish", "Glorot uniform")
        self.method = "adam"
        self.lr = 0.0001
        self.period = 200
        self.iterations = 20000

        self.path = path
        self.date = date
        self.index = index

        if date is None:
            now = datetime.datetime.now()
            self.date = now.strftime("%m%d")

        self.set_variable()
        self.__dict__.update(kwargs)
        self.iterations = int(self.iterations)

        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(self.save_model_dir, exist_ok=True)

        if index is None:
            self.fnamevar_path = os.path.join(self.output_dir, "SYNC_variables_" + self.date + ".dat")
        else:
            self.fnamevar_path = os.path.join(
                self.output_dir, "SYNC_variables_" + self.date + "_" + str(index) + ".dat"
            )
        self.check_dat_file()

        self.path = resolve_data_path(path)
        input_data = pd.read_csv(self.path)
        step_time = input_data.Time[1] - input_data.Time[0]
        input_data.Time -= step_time * int(input_data.shape[0] * self.st)
        input_data = input_data[int(input_data.shape[0] * self.st) : int(input_data.shape[0] * self.et)]
        self.input_data = input_data
        self.rename_sync_csv()

        H = dde.Variable(self.init_H)
        D = dde.Variable(self.init_D)
        self.variable_list = [H, D]

        def SYNC_ode(x, y):
            Tm, w, Te = y[:, 0:1], y[:, 1:2], y[:, 2:3]
            dw_dt = dde.grad.jacobian(y, x, i=1)
            return [Tm - Te - 2 * H * dw_dt - D * (w - 1)]

        self.pi_ode = SYNC_ode

    def check_dat_file(self):
        if not os.path.exists(self.fnamevar_path):
            return
        if not self.prompt_on_dat_conflict:
            return
        new_index = input(
            "The dat file already exists, please enter a new index to modify, or press Enter to continue"
        )
        if new_index != "":
            self.index = new_index
            self.fnamevar_path = os.path.join(
                self.output_dir, "SYNC_variables_" + self.date + "_" + str(new_index) + ".dat"
            )

    def rename_sync_csv(self):
        shorts = {last_short_name(col): col for col in self.input_data.columns}

        def pick(*names):
            for name in names:
                if name in shorts:
                    return shorts[name]
            return None

        time_col = pick("Time")
        tm_col = pick("Tm")
        w_col = pick("w_sync", "WPU", "TESTSPDOUT", "w")
        te_col = pick("Te_sync", "TESTTELECT", "Te")
        missing = [label for label, col in (("Time", time_col), ("Tm", tm_col), ("w", w_col), ("Te", te_col)) if col is None]
        if missing:
            raise KeyError("Missing columns %s in %s" % (missing, self.path))

        renamed = self.input_data[[time_col, tm_col, w_col, te_col]].copy()
        renamed.columns = ["Time", "Tm", "w", "Te"]
        renamed.set_index("Time", inplace=True)
        self.input_data = renamed

        true_H, true_D = parse_true_hd(self.path)
        self.true_H = true_H
        self.true_D = true_D
        self.real_value_list = [true_H, true_D]

    def set_variable(self):
        self.init_H = 1.0
        self.init_D = 1.0

    def build(self, transform=None, **kwargs):
        if transform in ("input", "all"):
            self.net = dde.nn.FNN([6] + [100] * 3 + [3], "tanh", "Glorot uniform")
        self.__dict__.update(kwargs)
        input_data = self.input_data
        x = input_data.index.to_numpy()
        geom = dde.geometry.TimeDomain(0, x[-1])
        y = input_data.to_numpy()
        observe_t = x.reshape(-1, 1)
        n = y.shape[0]

        def boundary(_, on_initial):
            return on_initial

        observe_y0 = dde.icbc.PointSetBC(observe_t, y[:, 0:1], component=0)
        observe_y1 = dde.icbc.PointSetBC(observe_t, y[:, 1:2], component=1)
        observe_y2 = dde.icbc.PointSetBC(observe_t, y[:, 2:3], component=2)
        ic0 = dde.icbc.IC(geom, lambda X: y[0, 0], boundary, component=0)
        ic1 = dde.icbc.IC(geom, lambda X: y[0, 1], boundary, component=1)
        ic2 = dde.icbc.IC(geom, lambda X: y[0, 2], boundary, component=2)

        data = dde.data.PDE(
            geom,
            self.pi_ode,
            [observe_y0, observe_y1, observe_y2, ic0, ic1, ic2],
            anchors=observe_t,
        )

        if transform is not None:
            from deepxde.backend import torch

            tran_in = transform in ("input", "all")
            tran_out = transform in ("output", "all")

            def feature_transform(t):
                t = 0.01 * t
                return torch.concat(
                    (
                        t,
                        torch.sin(t),
                        torch.sin(2 * t),
                        torch.sin(3 * t),
                        torch.sin(4 * t),
                        torch.sin(5 * t),
                    ),
                    axis=1,
                )

            data_t = observe_t
            data_y = y

            def output_transform(t, y_nn):
                idx = n - 1
                k = (data_y[idx] - data_y[0]) / (data_t[idx] - data_t[0])
                b = (data_t[idx] * data_y[0] - data_t[0] * data_y[idx]) / (data_t[idx] - data_t[0])
                linear = torch.as_tensor(k) * t + torch.as_tensor(b)
                factor = torch.tanh(t) * torch.tanh(idx - t)
                return linear + factor * torch.as_tensor([1.0, 1.0, 1.0]) * y_nn

            if tran_in:
                self.net.apply_feature_transform(feature_transform)
            if tran_out:
                self.net.apply_output_transform(output_transform)

        model = dde.Model(data, self.net)
        model.compile(self.method, lr=self.lr, external_trainable_variables=self.variable_list)
        self.model = model
        self.x = x
        self.y = y
        self.observe_t = observe_t

    def train(self):
        os.makedirs(os.path.dirname(self.fnamevar_path) or ".", exist_ok=True)
        os.makedirs(self.save_model_dir, exist_ok=True)
        variable = dde.callbacks.VariableValue(self.variable_list, period=self.period, filename=self.fnamevar_path)
        if self.index is None:
            check_point_path = os.path.join(self.save_model_dir, "model_" + self.date + ".ckpt")
        else:
            check_point_path = os.path.join(
                self.save_model_dir, "model_" + self.date + "_" + str(self.index) + ".ckpt"
            )
        checker = dde.callbacks.ModelCheckpoint(check_point_path, save_better_only=True, period=50000)
        self.model.train(iterations=self.iterations, callbacks=[variable, checker])

    def read_variable_history(self, load_fnamevar_path=None):
        path = load_fnamevar_path or self.fnamevar_path
        lines = open(path, "r").readlines()
        chat = np.array(
            [
                np.fromstring(
                    min(re.findall(re.escape("[") + "(.*?)" + re.escape("]"), line), key=len),
                    sep=",",
                )
                for line in lines
            ]
        )
        return chat

    def latest_variables(self, load_fnamevar_path=None):
        chat = self.read_variable_history(load_fnamevar_path)
        return float(chat[-1, 0]), float(chat[-1, 1])

    def load_model(self, model_path, load_fnamevar_path=None):
        if load_fnamevar_path is None:
            try:
                self.H, self.D = self.latest_variables(self.fnamevar_path)
                print("Load variable values from " + self.fnamevar_path)
            except OSError:
                pass
        else:
            self.init_H, self.init_D = self.latest_variables(load_fnamevar_path)

        H = dde.Variable(self.init_H)
        D = dde.Variable(self.init_D)
        self.variable_list = [H, D]
        self.model.compile(self.method, lr=self.lr, external_trainable_variables=self.variable_list)
        self.model.restore(model_path, verbose=1)


if __name__ == "__main__":
    input_data_path = os.path.join(REPO_ROOT, "data", "synchronous-machine", "927testforTe2.csv")
    microgird = SYNC(path=input_data_path, st=0.2, et=0.5, index="TransAll")
    microgird.build(
        transform="all",
        net=dde.nn.FNN([6] + [50] * 3 + [3], "swish", "Glorot uniform"),
        lr=1e-3,
        iterations=4000,
    )
    microgird.train()
